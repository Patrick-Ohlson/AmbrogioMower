/*
 * rtc.h - DS1307 RTC driver via software I2C
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_i2c.md
 *
 * Hardware: Dallas/Maxim DS1307 Real-Time Clock
 * Interface: Software bit-banged I2C (via function pointer table)
 * I2C Address: 0xD0 (write), 0xD1 (read)
 *
 * Original firmware functions (Ghidra-verified):
 *   i2c_InitBus        (0xA884)  -> rtc_Init()  [sets ptrI2C_Context1, calls ds1307_Init(ctx,5)]
 *   ds1307_Init        (0x14488) -> rtc_Init() internal [SQW jump table at 0x1A4F4]
 *   ds1307_ReadDate    (0x14056) -> rtc_GetDate() internal
 *   ds1307_ReadTime    (0x1410C) -> rtc_GetTime() internal
 *   ds1307_ReadRAM     (0x141DC) -> rtc_ReadRAM() internal
 *   ds1307_WriteTime   (0x1428A) -> rtc_SetTime() internal
 *   ds1307_WriteDate   (0x1433E) -> rtc_SetDate() internal
 *   ds1307_WriteRAM    (0x143F2) -> rtc_WriteRAM() internal
 *   rtc_SetTime        (0xA8AA)  -> rtc_SetTime() wrapper [copies 3B struct, calls ds1307_WriteTime]
 *   rtc_SetDate        (0xA8EA)  -> rtc_SetDate() wrapper [copies 3B struct, calls ds1307_WriteDate]
 *   rtc_GetDate        (0xA92A)  -> rtc_GetDate() wrapper
 *   rtc_GetTime        (0xA974)  -> rtc_GetTime() wrapper
 *   rtc_WriteRAM       (0xA9C0)  -> rtc_WriteRAM() wrapper
 *   rtc_ReadRAM        (0xA9EE)  -> rtc_ReadRAM() wrapper
 */

#ifndef RTC_H
#define RTC_H

#include <stdint.h>

/* ---- DS1307 I2C addresses ---- */
#define DS1307_ADDR_W       0xD0
#define DS1307_ADDR_R       0xD1

/* ---- DS1307 register map ---- */
#define DS1307_REG_SECONDS  0x00
#define DS1307_REG_MINUTES  0x01
#define DS1307_REG_HOURS    0x02
#define DS1307_REG_DAY      0x03    /* day of week (1-7) */
#define DS1307_REG_DATE     0x04    /* day of month (1-31) */
#define DS1307_REG_MONTH    0x05
#define DS1307_REG_YEAR     0x06
#define DS1307_REG_CONTROL  0x07
#define DS1307_RAM_START    0x08    /* battery-backed SRAM starts here */
#define DS1307_RAM_END      0x3F    /* 56 bytes: 0x08-0x3F */
#define DS1307_RAM_SIZE     56

/* ---- DS1307 control register (0x07) values ----
 *
 * Recovered from ds1307_Init jump table at 0x1A4F4 (6 entries).
 * Firmware uses DS1307_CTRL_SQW_32K (case 5) for initialization.
 *
 * Control register bits:
 *   Bit 7:   OUT   — Output level when SQWE=0
 *   Bit 4:   SQWE  — Square Wave Enable
 *   Bits 1:0 RS1:RS0 — Rate Select
 */
#define DS1307_CTRL_OFF_LOW     0x00    /* SQW off, OUT=0 (case 0) */
#define DS1307_CTRL_OFF_HIGH    0x80    /* SQW off, OUT=1 (case 1) */
#define DS1307_CTRL_SQW_1HZ    0x10    /* SQW on, 1 Hz (case 2) */
#define DS1307_CTRL_SQW_4K     0x11    /* SQW on, 4.096 kHz (case 3) */
#define DS1307_CTRL_SQW_8K     0x12    /* SQW on, 8.192 kHz (case 4) */
#define DS1307_CTRL_SQW_32K    0x13    /* SQW on, 32.768 kHz (case 5) — firmware default */

/* ---- Time/date structures ---- */

/*
 * Time in binary format (not BCD).
 * Firmware stores at rtcTimeBuf (0xFFE938): hours, minutes, seconds
 */
typedef struct {
    uint8_t hours;      /* 0-23 */
    uint8_t minutes;    /* 0-59 */
    uint8_t seconds;    /* 0-59 */
} rtc_time_t;

/*
 * Date in binary format (not BCD).
 * Firmware stores at rtcDateBuf (0xFFE93B): year, month, day
 */
typedef struct {
    uint8_t year;       /* 0-99 (20xx) */
    uint8_t month;      /* 1-12 */
    uint8_t day;        /* 1-31 */
} rtc_date_t;

/* ---- Public API ---- */

/*
 * rtc_Init - Initialize DS1307 RTC
 *
 * Configures the DS1307 control register for SQW output (32.768kHz,
 * matching original firmware case 5) and ensures the oscillator is running
 * by clearing the Clock Halt bit if set.
 *
 * I2C bus must be initialized before calling this function.
 *
 * Firmware: i2c_InitBus (0xA884) -> ds1307_Init (0x14488, param_2=5)
 *   ds1307_Init uses jump table at 0x1A4F4 to select ctrl reg value.
 */
void rtc_Init(void);

/*
 * rtc_GetTime - Read current time from DS1307
 *
 * Reads seconds/minutes/hours registers (BCD) and converts to binary.
 * Masks out Clock Halt (CH) bit 7 from seconds register.
 *
 * @param t  Pointer to rtc_time_t to fill
 * @return   0 on success, -1 on I2C error
 *
 * Firmware: rtc_GetTime (0xA974) -> ds1307_ReadTime (0x1410C)
 */
int rtc_GetTime(rtc_time_t *t);

/*
 * rtc_SetTime - Write time to DS1307
 *
 * Converts binary to BCD and writes hours/minutes/seconds registers.
 *
 * @param t  Pointer to rtc_time_t with time to set
 * @return   0 on success, -1 on I2C error
 *
 * Firmware: ds1307_WriteTime (0x1428A)
 */
int rtc_SetTime(const rtc_time_t *t);

/*
 * rtc_GetDate - Read current date from DS1307
 *
 * Reads date/month/year registers (BCD) and converts to binary.
 *
 * @param d  Pointer to rtc_date_t to fill
 * @return   0 on success, -1 on I2C error
 *
 * Firmware: rtc_GetDate (called in Startup)
 */
int rtc_GetDate(rtc_date_t *d);

/*
 * rtc_SetDate - Write date to DS1307
 *
 * Converts binary to BCD and writes date/month/year registers.
 *
 * @param d  Pointer to rtc_date_t with date to set
 * @return   0 on success, -1 on I2C error
 *
 * Firmware: ds1307_WriteDate (0x1433E)
 */
int rtc_SetDate(const rtc_date_t *d);

/*
 * rtc_ReadRAM - Read bytes from DS1307 battery-backed SRAM
 *
 * SRAM range: 0x08-0x3F (56 bytes). Address is 0-based offset
 * into the SRAM area (automatically offset by +0x08).
 *
 * @param addr   SRAM offset (0-55)
 * @param buf    Destination buffer
 * @param size   Number of bytes to read
 * @return       0 on success, -1 on error or out-of-range
 *
 * Firmware: rtc_ReadRAM (0xA9EE) -> ds1307_ReadRAM (0x141DC)
 */
int rtc_ReadRAM(uint8_t addr, uint8_t *buf, uint8_t size);

/*
 * rtc_WriteRAM - Write bytes to DS1307 battery-backed SRAM
 *
 * @param addr   SRAM offset (0-55)
 * @param buf    Source buffer
 * @param size   Number of bytes to write
 * @return       0 on success, -1 on error or out-of-range
 *
 * Firmware: rtc_WriteRAM (0xA9C0) -> ds1307_WriteRAM (0x143F2)
 */
int rtc_WriteRAM(uint8_t addr, const uint8_t *buf, uint8_t size);

#endif /* RTC_H */
