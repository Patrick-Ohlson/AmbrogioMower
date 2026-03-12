/*
 * rtc.c - DS1307 RTC driver via software I2C
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_i2c.md
 *
 * The original firmware uses a function pointer table (ptrBladeJumpTable1)
 * for I2C pin control, allowing the same protocol code to work with
 * different physical pin configurations. This reimplementation uses
 * the i2c/ bit-bang driver module on Bus 1 (shared with EEPROM).
 *
 * DS1307 registers store time/date in BCD format. All public API uses
 * binary (non-BCD) values.
 *
 * Initialization (ds1307_Init, 0x14488):
 *   Original firmware calls ds1307_Init(ctx, 5) from i2c_InitBus (0xA884).
 *   param_2=5 dispatches through a jump table (ds1307_Init_SQW_JumpTable,
 *   0x1A4F4) to select DS1307 control register value 0x13 (SQW on, 32.768kHz).
 *   This enables the SQW output pin as a 32.768kHz clock source.
 *   Jump table recovered from Ghidra: 6 cases mapping param_2 → ctrl reg value:
 *     0→0x00, 1→0x80, 2→0x10, 3→0x11, 4→0x12, 5→0x13
 */

#include "rtc.h"
#include "../i2c/i2c.h"

/* RTC and EEPROM share I2C Bus 1 (firmware: ptrBladeJumpTable1) */
#define RTC_I2C_BUS  I2C_BUS1

/* ---- BCD conversion helpers ---- */

static uint8_t bcd_to_bin(uint8_t bcd)
{
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

static uint8_t bin_to_bcd(uint8_t bin)
{
    return ((bin / 10) << 4) | (bin % 10);
}

/* ---- Internal I2C transaction helpers ---- */

/*
 * Set DS1307 register pointer to 'reg', then read 'count' bytes.
 * Returns 0 on success, -1 on NACK.
 *
 * Protocol: START -> 0xD0 -> reg -> rSTART -> 0xD1 -> read N -> STOP
 *
 * Firmware: ds1307_ReadTime (0x1410C), ds1307_ReadRAM (0x141DC)
 */
static int ds1307_read(uint8_t reg, uint8_t *buf, uint8_t count)
{
    uint8_t i;

    i2c_Start(RTC_I2C_BUS);
    if (!i2c_WriteByte(RTC_I2C_BUS, DS1307_ADDR_W)) { i2c_Stop(RTC_I2C_BUS); return -1; }
    if (!i2c_WriteByte(RTC_I2C_BUS, reg))            { i2c_Stop(RTC_I2C_BUS); return -1; }

    i2c_Start(RTC_I2C_BUS);    /* repeated start */
    if (!i2c_WriteByte(RTC_I2C_BUS, DS1307_ADDR_R)) { i2c_Stop(RTC_I2C_BUS); return -1; }

    for (i = 0; i < count; i++) {
        buf[i] = i2c_ReadByte(RTC_I2C_BUS, i == count - 1);    /* NACK on last byte */
    }
    i2c_Stop(RTC_I2C_BUS);
    return 0;
}

/*
 * Set DS1307 register pointer to 'reg', then write 'count' bytes.
 * Returns 0 on success, -1 on NACK.
 *
 * Protocol: START -> 0xD0 -> reg -> write N -> STOP
 *
 * Firmware: ds1307_WriteTime (0x1428A), ds1307_WriteRAM (0x143F2)
 */
static int ds1307_write(uint8_t reg, const uint8_t *buf, uint8_t count)
{
    uint8_t i;

    i2c_Start(RTC_I2C_BUS);
    if (!i2c_WriteByte(RTC_I2C_BUS, DS1307_ADDR_W)) { i2c_Stop(RTC_I2C_BUS); return -1; }
    if (!i2c_WriteByte(RTC_I2C_BUS, reg))            { i2c_Stop(RTC_I2C_BUS); return -1; }

    for (i = 0; i < count; i++) {
        if (!i2c_WriteByte(RTC_I2C_BUS, buf[i]))     { i2c_Stop(RTC_I2C_BUS); return -1; }
    }
    i2c_Stop(RTC_I2C_BUS);
    return 0;
}

/* ==================================================================
 * Public API
 * ================================================================== */

/*
 * rtc_Init - Initialize I2C bus and DS1307
 *
 * Firmware: i2c_InitBus (0xA884) -> ds1307_Init (0x14488) with param=5
 *
 * Two operations:
 *   1. Write control register 0x07 with SQW mode value.
 *      Original firmware uses param_2=5 → ctrl=0x13 (SQW on, 32.768kHz).
 *      Jump table at 0x1A4F4 recovered:
 *        case 0: 0x00 (SQW off, OUT=0)
 *        case 1: 0x80 (SQW off, OUT=1)
 *        case 2: 0x10 (SQW on, 1Hz)
 *        case 3: 0x11 (SQW on, 4.096kHz)
 *        case 4: 0x12 (SQW on, 8.192kHz)
 *        case 5: 0x13 (SQW on, 32.768kHz) <-- firmware default
 *
 *   2. Clear Clock Halt (CH) bit 7 in seconds register if set,
 *      to start the oscillator.
 */
void rtc_Init(void)
{
    uint8_t val;
    uint8_t sec;

    /* Step 1: Configure SQW output (firmware default: 0x13 = 32.768kHz)
     * Firmware: ds1307_Init case 5 → writes reg 0x07 = 0x13 */
    val = DS1307_CTRL_SQW_32K;
    ds1307_write(DS1307_REG_CONTROL, &val, 1);

    /* Step 2: Clear Clock Halt bit if set
     * The CH bit is bit 7 of the seconds register. If set, the
     * DS1307 oscillator is stopped. Clear it to start timekeeping. */
    if (ds1307_read(DS1307_REG_SECONDS, &sec, 1) == 0) {
        if (sec & 0x80) {
            sec &= 0x7F;
            ds1307_write(DS1307_REG_SECONDS, &sec, 1);
        }
    }
}

/*
 * rtc_GetTime - Read time from DS1307 (BCD -> binary)
 *
 * Firmware: rtc_GetTime (0xA974) -> ds1307_ReadTime (0x1410C)
 * BCD decode:
 *   seconds: mask out CH bit 7, then (reg>>3 & 0xE)*5 + (reg & 0xF)
 *   minutes: (reg>>4)*10 + (reg & 0xF)
 *   hours:   (reg>>4)*10 + (reg & 0xF)  (24-hour mode)
 */
int rtc_GetTime(rtc_time_t *t)
{
    uint8_t buf[3];

    if (ds1307_read(DS1307_REG_SECONDS, buf, 3) != 0)
        return -1;

    t->seconds = bcd_to_bin(buf[0] & 0x7F);    /* mask CH bit */
    t->minutes = bcd_to_bin(buf[1]);
    t->hours   = bcd_to_bin(buf[2] & 0x3F);    /* mask 12/24 bit */
    return 0;
}

/*
 * rtc_SetTime - Write time to DS1307 (binary -> BCD)
 *
 * Firmware: ds1307_WriteTime (0x1428A)
 */
int rtc_SetTime(const rtc_time_t *t)
{
    uint8_t buf[3];

    buf[0] = bin_to_bcd(t->seconds);   /* CH bit cleared = oscillator running */
    buf[1] = bin_to_bcd(t->minutes);
    buf[2] = bin_to_bcd(t->hours);     /* 24-hour mode (bit 6 = 0) */

    return ds1307_write(DS1307_REG_SECONDS, buf, 3);
}

/*
 * rtc_GetDate - Read date from DS1307 (BCD -> binary)
 *
 * Reads registers 4-6: date(day-of-month), month, year
 *
 * Firmware: rtc_GetDate (called in Startup)
 */
int rtc_GetDate(rtc_date_t *d)
{
    uint8_t buf[3];

    if (ds1307_read(DS1307_REG_DATE, buf, 3) != 0)
        return -1;

    d->day   = bcd_to_bin(buf[0]);
    d->month = bcd_to_bin(buf[1]);
    d->year  = bcd_to_bin(buf[2]);
    return 0;
}

/*
 * rtc_SetDate - Write date to DS1307 (binary -> BCD)
 *
 * Firmware: ds1307_WriteDate (0x1433E)
 */
int rtc_SetDate(const rtc_date_t *d)
{
    uint8_t buf[3];

    buf[0] = bin_to_bcd(d->day);
    buf[1] = bin_to_bcd(d->month);
    buf[2] = bin_to_bcd(d->year);

    return ds1307_write(DS1307_REG_DATE, buf, 3);
}

/*
 * rtc_ReadRAM - Read from DS1307 battery-backed SRAM
 *
 * Firmware: rtc_ReadRAM (0xA9EE) -> ds1307_ReadRAM (0x141DC)
 * Assert: address + size <= DS1307_RAM_SIZE
 */
int rtc_ReadRAM(uint8_t addr, uint8_t *buf, uint8_t size)
{
    if ((uint16_t)addr + size > DS1307_RAM_SIZE)
        return -1;

    return ds1307_read(DS1307_RAM_START + addr, buf, size);
}

/*
 * rtc_WriteRAM - Write to DS1307 battery-backed SRAM
 *
 * Firmware: rtc_WriteRAM (0xA9C0) -> ds1307_WriteRAM (0x143F2)
 * Assert: address + size <= DS1307_RAM_SIZE
 */
int rtc_WriteRAM(uint8_t addr, const uint8_t *buf, uint8_t size)
{
    if ((uint16_t)addr + size > DS1307_RAM_SIZE)
        return -1;

    return ds1307_write(DS1307_RAM_START + addr, buf, size);
}
