/*
 * i2c.c - Software bit-banged I2C driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_i2c.md
 *
 * Shadow register integration (verified against Ghidra RE):
 *   DDR: All DDR writes go through shadow pointers (H8S DDR is write-only).
 *   DR:  DriveLow clears the pin bit via shadow pointer if available,
 *         then writes the full shadow to hardware. Prevents RMW hazard
 *         on P9.DR (SDA line, shares motor direction P9.0/P9.1 and blade P9.5).
 *         If dr_shadow is NULL (Bus 2 PB.ODR), direct RMW is used.
 *
 * Clock stretching: uses GetSystemCounter() with 10-tick (~100ms) timeout,
 *   matching the original firmware's i2c_ClockHigh (0x15E96).
 *
 * Pin control pattern (from Ghidra i2c_bus1 decompilation):
 *   DriveLow:  DR &= ~mask (via shadow), then DDR |= mask (output)
 *   Release:   DDR &= ~mask (input, pull-up brings HIGH)
 *   ReadPin:   return (*pin >> bit_pos) & 1
 */

#include "i2c.h"
#include "../timer/timer.h"

/* ---- Bus context storage ---- */
static i2c_bus_t bus_ctx[I2C_BUS_COUNT];

/* ---- Timing delay ----
 * I2C standard mode (100 kHz): SCL HIGH min 4.0 µs, LOW min 4.7 µs.
 * At ~18 MHz, each loop iteration takes ~8 cycles = 0.44 µs.
 * 25 iterations = ~11 µs — safe margin for 100 kHz I2C.
 *
 * NOTE: The original firmware has NO explicit delays in its
 * pin-level functions. Timing comes from the function pointer
 * dispatch overhead (~30-50 cycles per indirect call). Our
 * direct pin access is faster, so we need explicit delays.
 */
static void i2c_delay(void)
{
    volatile uint8_t i;
    for (i = 0; i < 25; i++)
        ;
}

/* ---- Pin-level: DR write (shadow-safe) ----
 *
 * If a DR shadow pointer is provided, modify the shadow and write
 * the full shadow to hardware.  This avoids reading the hardware DR
 * register (which could return corrupted data during motor EMI).
 *
 * If dr_shadow is NULL, do a direct RMW on hardware (safe for ports
 * like PB.ODR that are readable and don't share motor-critical bits).
 */
static void i2c_dr_clear_bit(volatile uint8_t *dr, uint8_t *dr_shadow, uint8_t mask)
{
    if (dr_shadow != (uint8_t *)0) {
        *dr_shadow &= ~mask;
        *dr = *dr_shadow;
    } else {
        *dr &= ~mask;
    }
}

/* ---- Pin-level functions ---- */

/*
 * Release SCL (set DDR to input — pull-up brings HIGH).
 * Implements clock stretching: polls SCL with GetSystemCounter() timeout.
 *
 * Original firmware i2c_ClockHigh (0x15E96):
 *   ReleaseSCL();
 *   if (!error) {
 *       start = GetSystemCounter();
 *       do { if (ReadSCL()) break; } while (GetSystemCounter()-start < 10);
 *   }
 *   error = (ReadSCL() == 0);
 */
static void i2c_clock_high(i2c_bus_t *ctx)
{
    uint32_t start;

    /* Release SCL: DDR bit to input (0) = pull-up brings high */
    *ctx->scl_ddr_shadow &= ~ctx->scl_mask;
    *ctx->scl_ddr = *ctx->scl_ddr_shadow;
    i2c_delay();

    /* Clock stretching: wait for slave to release SCL.
     * Uses GetSystemCounter() — matches original firmware (0x15E96).
     * Timeout = 10 ticks (~100ms). */
    if (!(ctx->error)) {
        start = GetSystemCounter();
        while (!(*ctx->scl_pin & ctx->scl_mask)) {
            if ((GetSystemCounter() - start) >= I2C_CLOCK_STRETCH_TICKS)
                break;
        }
    }

    /* Set error flag if SCL is still low */
    if (!(*ctx->scl_pin & ctx->scl_mask)) {
        ctx->error |= 0x10;
    }
}

/*
 * Pull SCL low (set as output driving low).
 *
 * Original firmware i2c_bus1_DriveSCL_Low (0x122D0):
 *   P8_DR &= 0xBF;  bShadow_P8_DDR |= 0x40;  P8_DDR = shadow;
 */
static void i2c_clock_low(i2c_bus_t *ctx)
{
    /* DR = 0 first (clear bit before enabling output) — via shadow */
    i2c_dr_clear_bit(ctx->scl_dr, ctx->scl_dr_shadow, ctx->scl_mask);
    /* DDR = 1 (output drives low) */
    *ctx->scl_ddr_shadow |= ctx->scl_mask;
    *ctx->scl_ddr = *ctx->scl_ddr_shadow;
    i2c_delay();
}

/*
 * Release SDA (set DDR to input — pull-up brings HIGH).
 *
 * Original firmware i2c_bus1_ReleaseSDA (0x12306):
 *   bShadow_P9_DDR &= ~0x80;  P9_DDR = shadow;
 */
static void i2c_data_high(i2c_bus_t *ctx)
{
    *ctx->sda_ddr_shadow &= ~ctx->sda_mask;
    *ctx->sda_ddr = *ctx->sda_ddr_shadow;
    i2c_delay();
}

/*
 * Pull SDA low (set as output driving low).
 *
 * Original firmware i2c_bus1_DriveSDA_Low (0x1232E):
 *   P9_DR &= 0x7F;  bShadow_P9_DDR |= 0x80;  P9_DDR = shadow;
 */
static void i2c_data_low(i2c_bus_t *ctx)
{
    /* DR = 0 first (clear bit before enabling output) — via shadow */
    i2c_dr_clear_bit(ctx->sda_dr, ctx->sda_dr_shadow, ctx->sda_mask);
    /* DDR = 1 (output drives low) */
    *ctx->sda_ddr_shadow |= ctx->sda_mask;
    *ctx->sda_ddr = *ctx->sda_ddr_shadow;
    i2c_delay();
}

/*
 * Read SDA pin state.
 *
 * Original firmware i2c_bus1_ReadSDA (0x12374):
 *   return (P9_DR >> 7);
 */
static uint8_t i2c_read_sda(i2c_bus_t *ctx)
{
    /* Ensure SDA is released (input) before reading */
    *ctx->sda_ddr_shadow &= ~ctx->sda_mask;
    *ctx->sda_ddr = *ctx->sda_ddr_shadow;
    i2c_delay();
    return (*ctx->sda_pin & ctx->sda_mask) ? 1 : 0;
}

/* ==================================================================
 * Public API
 * ================================================================== */

void i2c_Init(uint8_t bus, const i2c_bus_t *ctx)
{
    if (bus >= I2C_BUS_COUNT)
        return;

    bus_ctx[bus] = *ctx;
    bus_ctx[bus].error = 0;

    /* Release both lines (idle state: SDA=high, SCL=high) */
    i2c_data_high(&bus_ctx[bus]);
    i2c_clock_high(&bus_ctx[bus]);
}

/*
 * i2c_Start - Generate START condition
 *
 * START: SDA goes low while SCL is high.
 *
 * Original firmware i2c_Start (0x14882):
 *   DataHigh -> ClockHigh -> DataLow -> ClockLow
 */
int i2c_Start(uint8_t bus)
{
    i2c_bus_t *ctx;

    if (bus >= I2C_BUS_COUNT)
        return -1;

    ctx = &bus_ctx[bus];

    /* Ensure SDA is high before START */
    i2c_data_high(ctx);
    i2c_clock_high(ctx);
    i2c_delay();

    /* START: pull SDA low while SCL is high */
    i2c_data_low(ctx);
    i2c_delay();

    /* Pull SCL low to begin clock */
    i2c_clock_low(ctx);

    return 0;
}

/*
 * i2c_Stop - Generate STOP condition
 *
 * STOP: SDA goes high while SCL is high.
 * Retries up to 17 times (0x11) if SDA doesn't release.
 *
 * Original firmware i2c_Stop (0x148A4):
 *   Loop 0x11 times: ClockLow -> DataLow -> ClockHigh -> DataHigh
 *   Break if ReadSDA == 1 (SDA released)
 */
void i2c_Stop(uint8_t bus)
{
    i2c_bus_t *ctx;
    uint8_t retry;

    if (bus >= I2C_BUS_COUNT)
        return;

    ctx = &bus_ctx[bus];

    for (retry = 0; retry < 17; retry++) {
        i2c_clock_low(ctx);
        i2c_data_low(ctx);
        i2c_clock_high(ctx);
        i2c_data_high(ctx);

        /* Verify SDA is actually high (no bus contention) */
        if (i2c_read_sda(ctx))
            return;     /* Success */
    }

    /* Failed to release bus after 17 retries */
    ctx->error |= 0x01;
}

/*
 * i2c_WriteByte - Write 8 bits MSB-first, check ACK
 *
 * Original firmware i2c_WriteByte (0x1495E):
 *   mask=0x80, 8 iterations, shift right, ACK = (ReadSDA == 0)
 */
int i2c_WriteByte(uint8_t bus, uint8_t byte)
{
    i2c_bus_t *ctx;
    uint8_t i;
    uint8_t ack;

    if (bus >= I2C_BUS_COUNT)
        return 0;

    ctx = &bus_ctx[bus];

    /* Shift out 8 bits MSB-first */
    for (i = 0; i < 8; i++) {
        if (byte & 0x80)
            i2c_data_high(ctx);
        else
            i2c_data_low(ctx);

        i2c_clock_high(ctx);
        i2c_clock_low(ctx);

        byte <<= 1;
    }

    /* Release SDA for ACK read */
    i2c_data_high(ctx);

    /* Clock ACK bit */
    i2c_clock_high(ctx);
    ack = i2c_read_sda(ctx);   /* 0 = ACK, 1 = NACK */
    i2c_clock_low(ctx);

    return ack ? 0 : 1;    /* Return 1=ACK, 0=NACK */
}

/*
 * i2c_ReadByte - Read 8 bits MSB-first, send ACK/NACK
 *
 * Original firmware i2c_ReadByte (0x148DA):
 *   DataHigh, mask=0x80, 8 iterations, OR in bit, ACK/NACK at end
 */
uint8_t i2c_ReadByte(uint8_t bus, uint8_t nack)
{
    i2c_bus_t *ctx;
    uint8_t i;
    uint8_t byte = 0;

    if (bus >= I2C_BUS_COUNT)
        return 0xFF;

    ctx = &bus_ctx[bus];

    /* Release SDA for slave to drive */
    i2c_data_high(ctx);

    /* Shift in 8 bits MSB-first */
    for (i = 0; i < 8; i++) {
        byte <<= 1;

        i2c_clock_high(ctx);

        if (i2c_read_sda(ctx))
            byte |= 1;

        i2c_clock_low(ctx);
    }

    /* Send ACK (SDA low) or NACK (SDA high) */
    if (nack)
        i2c_data_high(ctx);
    else
        i2c_data_low(ctx);

    i2c_clock_high(ctx);
    i2c_clock_low(ctx);

    /* Release SDA */
    i2c_data_high(ctx);

    return byte;
}

/*
 * i2c_ReadMulti - Read N bytes with auto ACK/NACK
 *
 * Firmware: i2c_ReadMulti (0x149C2)
 */
int i2c_ReadMulti(uint8_t bus, uint8_t *buf, uint8_t count)
{
    uint8_t i;

    for (i = 0; i < count; i++)
        buf[i] = i2c_ReadByte(bus, (i == count - 1));  /* NACK on last byte */

    return 0;
}

/*
 * i2c_WriteMulti - Write N bytes, check ACK after each
 *
 * Firmware: i2c_WriteMulti (0x149F4)
 */
int i2c_WriteMulti(uint8_t bus, const uint8_t *buf, uint8_t count)
{
    uint8_t i;

    for (i = 0; i < count; i++) {
        if (!i2c_WriteByte(bus, buf[i]))
            return 0;   /* NACK — abort */
    }

    return 1;   /* All ACKed */
}

uint8_t i2c_GetError(uint8_t bus)
{
    if (bus >= I2C_BUS_COUNT)
        return 0xFF;

    return bus_ctx[bus].error;
}

void i2c_ClearError(uint8_t bus)
{
    if (bus >= I2C_BUS_COUNT)
        return;

    bus_ctx[bus].error = 0;
}
