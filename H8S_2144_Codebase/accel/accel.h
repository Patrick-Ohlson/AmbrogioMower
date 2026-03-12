/*
 * accel.h - Accelerometer/tilt sensor driver (LIS302DL / LIS3DH)
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_i2c.md
 *
 * Supports two sensor variants with auto-detection:
 *   LIS302DL (STMicro) - I2C addr 0x1D (8-bit: 0x3A/0x3B), SDO=1
 *   LIS3DH   (STMicro) - I2C addr 0x19 (8-bit: 0x32/0x33), SA0=1
 *
 * NOTE: The original firmware labels the LIS302DL functions as "mma7455".
 * Hardware testing confirmed the chip is an LIS302DL:
 *   - WHO_AM_I (reg 0x0F) = 0x3B (MMA7455L would be 0x55)
 *   - Firmware register addresses (0x20, 0x27, 0x29, 0x2B, 0x2D)
 *     match the LIS302DL register map exactly
 *   - I2C address 0x1D with SDO=1 matches LIS302DL pinout
 *
 * Original firmware functions:
 *   accel_Init         (0xBC7A) -> accel_Init()
 *   accel_Poll         (0xBCDA) -> accel_Read()
 *   accel_FilterXYZ    (0xBA44) -> accel_Read() internal
 *   accel_PackVector   (0x157C8) -> accel_GetTilt()
 *   accel_CalcTiltAngle(0x157D4) -> accel_GetTilt()
 *   mma7455_Probe      (0x14B3E) -> accel_Init() internal [actually LIS302DL]
 *   lis3dh_Init        (0x14E32) -> accel_Init() internal
 */

#ifndef ACCEL_H
#define ACCEL_H

#include <stdint.h>

/* ---- Sensor type (set by accel_Init) ---- */
#define ACCEL_NONE      0
#define ACCEL_LIS302DL  1       /* Firmware calls this "MMA7455L" but it's LIS302DL */
#define ACCEL_LIS3DH    2

/* Keep legacy name for compatibility */
#define ACCEL_MMA7455L  ACCEL_LIS302DL

/* ---- LIS302DL I2C addresses ---- */
/* SDO=1 on L200 board: addr = 0x1D (7-bit) = 0x3A/0x3B (8-bit)
 * Firmware: mma7455_ReadReg/WriteReg hardcode 0x3A/0x3B
 * Hardware confirmed: ACK at 0x3A, WHO_AM_I at 0x0F = 0x3B */
#define LIS302DL_ADDR_W     0x3A    /* 7-bit address 0x1D, write */
#define LIS302DL_ADDR_R     0x3B    /* 7-bit address 0x1D, read  */

/* Legacy aliases */
#define MMA7455_ADDR_W  LIS302DL_ADDR_W
#define MMA7455_ADDR_R  LIS302DL_ADDR_R

/* ---- LIS302DL registers ---- */
/* Register map verified against Ghidra decompilation and hardware test.
 * Firmware register usage matches LIS302DL datasheet exactly:
 *   mma7455_WriteReg: hardcodes reg 0x20 (CTRL_REG1)
 *   mma7455_PollAndRead: reads reg 0x27 (STATUS_REG), checks bit 3
 *   mma7455_ReadAxes: reads regs 0x29, 0x2B, 0x2D (OUT_X/Y/Z) */
#define LIS302DL_REG_WHO_AM_I   0x0F  /* WHO_AM_I (reads 0x3B) */
#define LIS302DL_REG_CTRL_REG1  0x20  /* Power, data rate, axes enable */
#define LIS302DL_REG_CTRL_REG2  0x21  /* High-pass filter config */
#define LIS302DL_REG_CTRL_REG3  0x22  /* Interrupt pin config */
#define LIS302DL_REG_STATUS     0x27  /* Status register (bit 3 = ZYXDA) */
#define LIS302DL_REG_OUT_X      0x29  /* X-axis output (8-bit signed) */
#define LIS302DL_REG_OUT_Y      0x2B  /* Y-axis output (8-bit signed) */
#define LIS302DL_REG_OUT_Z      0x2D  /* Z-axis output (8-bit signed) */

/* LIS302DL CTRL_REG1 value from firmware (mma7455_Probe writes 0x47):
 * Bit 7: DR=0 (100Hz), Bit 6: PD=1 (active mode),
 * Bit 5-4: FS=0/STP=0 (+/-2g, no self-test),
 * Bit 2-0: Zen=1, Yen=1, Xen=1 */
#define LIS302DL_CTRL1_ACTIVE   0x47  /* Active, 100Hz, +/-2g, XYZ on */

/* LIS302DL WHO_AM_I expected value */
#define LIS302DL_WHOAMI_VALUE   0x3B

/* LIS302DL STATUS_REG bit masks */
#define LIS302DL_STATUS_ZYXDA   0x08  /* New data available on all axes */

/* ---- LIS3DH I2C addresses ---- */
/* SA0=1 on L200 board: addr = 0x19 (7-bit) = 0x32/0x33 (8-bit)
 * Firmware: lis3dh_Init(ctx, 0x32) at Startup (0x20AE) */
#define LIS3DH_ADDR_W      0x32    /* 7-bit address 0x19, write */
#define LIS3DH_ADDR_R      0x33    /* 7-bit address 0x19, read  */

/* ---- LIS3DH registers ---- */
#define LIS3DH_REG_WHO_AM_I     0x0F  /* WHO_AM_I (should read 0x33) */
#define LIS3DH_REG_CTRL_REG1    0x20  /* Data rate, axes enable */
#define LIS3DH_REG_CTRL_REG2    0x21  /* High-pass filter config */
#define LIS3DH_REG_CTRL_REG3    0x22  /* Interrupt config */
#define LIS3DH_REG_CTRL_REG4    0x23  /* BDU, full scale, self-test */
#define LIS3DH_REG_CTRL_REG5    0x24  /* FIFO, interrupt latch */
#define LIS3DH_REG_CTRL_REG6    0x25  /* Click/interrupt on PAD2 */
#define LIS3DH_REG_OUT_X_L      0x28  /* X-axis output low byte */
#define LIS3DH_REG_INT1_CFG     0x30  /* Interrupt 1 config */
#define LIS3DH_REG_INT1_THS     0x32  /* Interrupt 1 threshold */
#define LIS3DH_REG_INT1_DURATION 0x33 /* Interrupt 1 duration */
#define LIS3DH_REG_INT2_CFG     0x34  /* Interrupt 2 config */
#define LIS3DH_REG_INT2_THS     0x36  /* Interrupt 2 threshold */
#define LIS3DH_REG_INT2_DURATION 0x37 /* Interrupt 2 duration */
#define LIS3DH_REG_CLICK_CFG    0x38  /* Click config */

/* LIS3DH auto-increment bit for multi-byte reads */
#define LIS3DH_AUTO_INCREMENT   0x80

/* LIS3DH WHO_AM_I expected value */
#define LIS3DH_WHOAMI_VALUE     0x33

/* ---- Raw axis data ---- */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} accel_raw_t;

/* ---- Filtered axis data (32-bit IIR accumulators) ---- */
typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} accel_filtered_t;

/* ---- Tilt angles in degrees ---- */
typedef struct {
    int16_t tilt1;      /* pitch: atan2(X, Z) — tilt around Y-axis */
    int16_t tilt2;      /* roll:  atan2(Y, Z) — tilt around X-axis */
} accel_tilt_t;

/* ---- Public API ---- */

/*
 * accel_Init - Auto-detect and initialize accelerometer
 *
 * Probes for LIS302DL first (writes CTRL_REG1, ACK = present).
 * If not found, tries LIS3DH initialization (config registers).
 *
 * @return  Sensor type: ACCEL_LIS302DL, ACCEL_LIS3DH, or ACCEL_NONE
 *
 * Firmware: accel_Init (0xBC7A)
 */
uint8_t accel_Init(void);

/*
 * accel_Read - Read and filter accelerometer axes
 *
 * Reads raw XYZ from the detected sensor and applies a 15/16+1/16
 * IIR low-pass filter. Axis signs are adjusted per sensor mounting:
 *   LIS302DL: X and Y negated
 *   LIS3DH:   Y and X swapped and negated
 *
 * @param raw  Optional: raw axis data before filtering (NULL to skip)
 * @return     0 on success, -1 if no sensor or read error
 *
 * Firmware: accel_Poll (0xBCDA) -> accel_ReadMMA7455/accel_ReadLIS3DH
 */
int accel_Read(accel_raw_t *raw);

/*
 * accel_GetFiltered - Get current IIR-filtered axis values
 *
 * Returns the 32-bit filtered accumulators (fixed-point, scaled by 16).
 *
 * @param f  Pointer to accel_filtered_t to fill
 *
 * Firmware: dwAccelFilterX/Y/Z (0xFFE9E0-0xFFE9E8)
 */
void accel_GetFiltered(accel_filtered_t *f);

/*
 * accel_GetTilt - Compute tilt angles from filtered data
 *
 * Converts filtered accelerometer data to tilt angles in degrees
 * using fixed-point atan2 approximation.
 *
 * @param tilt  Pointer to accel_tilt_t to fill (degrees, signed)
 *
 * Firmware: accel_PackVector (0x157C8) + accel_CalcTiltAngle (0x157D4)
 */
void accel_GetTilt(accel_tilt_t *tilt);

/*
 * accel_GetType - Return detected sensor type
 *
 * @return  ACCEL_NONE, ACCEL_LIS302DL, or ACCEL_LIS3DH
 */
uint8_t accel_GetType(void);

/* ---- Low-level register access (for diagnostics/tests) ---- */

/*
 * accel_WriteReg - Write a single register on the accelerometer bus
 *
 * Generic I2C register write on Bus 2. Caller supplies device address.
 *
 * @param dev_addr_w  8-bit I2C write address (e.g. LIS302DL_ADDR_W)
 * @param reg         Register address
 * @param val         Value to write
 * @return            0 on success, -1 on NACK/error
 */
int accel_WriteReg(uint8_t dev_addr_w, uint8_t reg, uint8_t val);

/*
 * accel_ReadReg - Read a single register on the accelerometer bus
 *
 * Generic I2C register read on Bus 2. Caller supplies device addresses.
 *
 * @param dev_addr_w  8-bit I2C write address
 * @param dev_addr_r  8-bit I2C read address
 * @param reg         Register address
 * @param val         Pointer to store read value
 * @return            0 on success, -1 on NACK/error
 */
int accel_ReadReg(uint8_t dev_addr_w, uint8_t dev_addr_r,
                  uint8_t reg, uint8_t *val);

#endif /* ACCEL_H */
