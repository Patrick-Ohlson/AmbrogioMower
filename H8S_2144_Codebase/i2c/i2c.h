/*
 * i2c.h - Software bit-banged I2C driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_i2c.md
 *
 * The original firmware uses function pointer tables (i2c_bus1_vtable,
 * i2c_bus2_vtable) to indirect I2C pin access, allowing the same
 * protocol code to work with different physical pin configurations.
 *
 * This reimplementation provides two I2C bus contexts:
 *   Bus 1 (i2c_context1 @ RAM 0xFFE096): RTC + EEPROM — SCL=P8.6, SDA=P9.7
 *     Split-port: SCL and SDA on different ports, separate DDR shadows.
 *     Vtable at ROM 0x1A270, init via startvars copy from ROM 0x1A9DB.
 *   Bus 2 (i2c_context2 @ RAM 0xFFE09C): Accelerometer — SCL=PB.7, SDA=PB.6
 *     Same-port: Both pins on Port B (open-drain). Shared DDR shadow.
 *     PB.ODR (0xFFFFBC) = data/open-drain control (0=low, 1=release)
 *     PB.DDR (0xFFFFBE) = direction (1=output, 0=input/float)
 *     PIN readback via 0xFFFFBD (H8S DDR read returns PIN)
 *
 * Bus 1 pin assignments (confirmed by vtable dispatch analysis):
 *   Vtable slot → dispatch function → pin function:
 *   [+0x00] i2c_ClockHigh → i2c_bus1_ReleaseSCL  (0x122A8): P8_DDR &= ~0x40
 *   [+0x04] i2c_ClockLow  → i2c_bus1_DriveSCL_Low(0x122D0): P8_DR &= 0xBF; P8_DDR |= 0x40
 *   [+0x08] i2c_DataHigh  → i2c_bus1_ReleaseSDA  (0x12306): P9_DDR &= ~0x80
 *   [+0x0C] i2c_DataLow   → i2c_bus1_DriveSDA_Low(0x1232E): P9_DR &= 0x7F; P9_DDR |= 0x80
 *   [+0x10] stretching    → i2c_bus1_ReadSCL     (0x12364): (P8_DR >> 6) & 1
 *   [+0x14] i2c_ReadSDA   → i2c_bus1_ReadSDA     (0x12374): P9_DR >> 7
 *
 * Bus 2 pin assignments (confirmed by vtable dispatch analysis):
 *   [+0x00] ReleaseSCL  → i2c_bus2_ReleaseSCL  (0x12384): DDR &= ~0x80; ODR |= 0x80
 *   [+0x04] DriveSCL    → i2c_bus2_DriveSCL_Low(0x123B4): ODR &= ~0x80; DDR |= 0x80
 *   [+0x08] ReleaseSDA  → i2c_bus2_ReleaseSDA  (0x123E8): DDR &= ~0x40; ODR |= 0x40
 *   [+0x0C] DriveSDA    → i2c_bus2_DriveSDA_Low(0x12418): ODR &= ~0x40; DDR |= 0x40
 *   [+0x10] ReadSCL     → i2c_bus2_ReadSCL     (0x1244C): (PIN >> 7) & 1
 *   [+0x14] ReadSDA     → i2c_bus2_ReadSDA     (0x1245C): (PIN >> 6) & 1
 *
 * Shadow register integration:
 *   DDR shadows are POINTERS to global shadow variables (motor.h),
 *   ensuring a single authoritative copy per DDR register.
 *   DR shadows are also POINTERS — Bus 1 SCL uses shadow_p9_dr
 *   (shares P9 with motor IN2/blade bits, EMI-vulnerable).
 *   Bus 2 PB.ODR is readable and doesn't share motor pins, so
 *   DR shadows are NULL → safe to do direct RMW on hardware.
 *
 * Clock stretching:
 *   Uses GetSystemCounter() for timeout (matches original firmware's
 *   i2c_ClockHigh at 0x15E96). Timeout = 10 ticks (~100ms).
 *
 * Original firmware functions:
 *   i2c_Start       (0x14882) -> i2c_Start()
 *   i2c_Stop        (0x148A4) -> i2c_Stop()
 *   i2c_ReadByte    (0x148DA) -> i2c_ReadByte()
 *   i2c_WriteByte   (0x1495E) -> i2c_WriteByte()
 *   i2c_ReadMulti   (0x149C2) -> i2c_ReadMulti()
 *   i2c_WriteMulti  (0x149F4) -> i2c_WriteMulti()
 *   i2c_ClockHigh   (0x15E96) -> (internal, uses GetSystemCounter)
 *   i2c_ClockLow    (0x15EF2) -> (internal)
 *   i2c_DataHigh    (0x15F00) -> (internal)
 *   i2c_DataLow     (0x15F0E) -> (internal)
 *   i2c_ReadSDA     (0x15F1C) -> (internal)
 *   i2c_InitBus     (0xA884)  -> i2c_Init()
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>

/* ---- I2C bus identifiers ---- */
#define I2C_BUS1    0       /* RTC + EEPROM bus (i2c_bus1_vtable @ 0x1A270) */
#define I2C_BUS2    1       /* Accelerometer bus (i2c_bus2_vtable @ 0x1A28C) */
#define I2C_BUS_COUNT 2

/* ---- I2C bus context ---- */
/*
 * Each I2C bus has its own SDA/SCL pin assignments.
 *
 * Shadow register design:
 *   DDR and DR shadow fields are POINTERS to external shadow variables
 *   (typically the globals in motor.h: shadow_p8_ddr, shadow_p9_ddr,
 *   shadow_p8_dr, shadow_p9_dr).
 *
 *   This ensures a SINGLE authoritative shadow per hardware register,
 *   shared between the I2C driver and motor/misc code.  No desync.
 *
 *   DDR shadows (ddr_shadow): REQUIRED — H8S DDR is write-only.
 *     All DDR modifications go through the pointed-to shadow.
 *     For same-port buses (Bus 2), both SCL and SDA DDR shadow
 *     pointers point to the SAME variable — automatic sync.
 *
 *   DR shadows (dr_shadow): OPTIONAL — set to NULL if the port's
 *     DR register is safe for direct RMW (no shared EMI-critical bits).
 *     Bus 1 SDA (P9.DR) shares motor direction pins → shadow required.
 *     Bus 2 (PB.ODR) is readable and not motor-critical → NULL OK.
 */
typedef struct {
    /* SDA pin (data line) */
    volatile uint8_t *sda_ddr;      /* SDA port DDR register (write-only!) */
    volatile uint8_t *sda_dr;       /* SDA port Data Register (hardware)   */
    volatile uint8_t *sda_pin;      /* SDA port Pin input Register         */
    uint8_t sda_mask;               /* Bit mask for SDA pin                */
    uint8_t *sda_ddr_shadow;        /* -> external DDR shadow variable     */
    uint8_t *sda_dr_shadow;         /* -> external DR shadow, or NULL      */

    /* SCL pin (clock line) */
    volatile uint8_t *scl_ddr;      /* SCL port DDR register (write-only!) */
    volatile uint8_t *scl_dr;       /* SCL port Data Register (hardware)   */
    volatile uint8_t *scl_pin;      /* SCL port Pin input Register         */
    uint8_t scl_mask;               /* Bit mask for SCL pin                */
    uint8_t *scl_ddr_shadow;        /* -> external DDR shadow variable     */
    uint8_t *scl_dr_shadow;         /* -> external DR shadow, or NULL      */

    /* Bus state */
    uint8_t error;                  /* Bus error flag                      */
} i2c_bus_t;

/* ---- Clock stretching timeout (system ticks, 1 tick = ~10ms) ---- */
#define I2C_CLOCK_STRETCH_TICKS  10     /* ~100ms, matches original firmware */

/* ---- Public API ---- */

void i2c_Init(uint8_t bus, const i2c_bus_t *ctx);
int i2c_Start(uint8_t bus);
void i2c_Stop(uint8_t bus);
int i2c_WriteByte(uint8_t bus, uint8_t byte);
uint8_t i2c_ReadByte(uint8_t bus, uint8_t nack);
int i2c_ReadMulti(uint8_t bus, uint8_t *buf, uint8_t count);
int i2c_WriteMulti(uint8_t bus, const uint8_t *buf, uint8_t count);
uint8_t i2c_GetError(uint8_t bus);
void i2c_ClearError(uint8_t bus);

#endif /* I2C_H */
