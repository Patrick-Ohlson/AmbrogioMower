# RTC Driver — rtc

## Module Summary

Dallas/Maxim DS1307 real-time clock driver over software bit-banged I2C.
Provides time/date read/write, SQW output configuration, and access to the
56-byte battery-backed SRAM.

| Property | Value |
|----------|-------|
| Device | DS1307 Real-Time Clock |
| Interface | Software I2C (bit-banged) |
| I2C Address | 0xD0 (write), 0xD1 (read) |
| SRAM | 56 bytes battery-backed (0x08-0x3F) |
| SQW Output | 32.768 kHz (firmware default, case 5) |
| Source doc | `STATES/state_i2c.md` (sections 1-2) |

## Files

| File | Description |
|------|-------------|
| `rtc.h` | Public API header with time/date structs and DS1307 constants |
| `rtc.c` | Implementation |
| `rtc_doc.md` | This file |

## How to Include

```c
#include "rtc/rtc.h"
```

```makefile
SRCS += rtc/rtc.c
```

### Dependencies

| Dependency | Required For |
|------------|-------------|
| `<stdint.h>` | `uint8_t` types |
| `i2c/i2c.h` | `i2c_Start()`, `i2c_Stop()`, `i2c_WriteByte()`, `i2c_ReadByte()` |

**Bus assignment:** This module uses `I2C_BUS1` (shared with EEPROM).
The original firmware routes RTC traffic through `ptrI2C_Context1`
(formerly ptrBladeJumpTable1). The `i2c/` module must be initialized
for Bus 1 before calling any RTC functions.

```makefile
SRCS += i2c/i2c.c rtc/rtc.c
```

## API Reference

### Types

```c
typedef struct {
    uint8_t hours;      /* 0-23 */
    uint8_t minutes;    /* 0-59 */
    uint8_t seconds;    /* 0-59 */
} rtc_time_t;

typedef struct {
    uint8_t year;       /* 0-99 (20xx) */
    uint8_t month;      /* 1-12 */
    uint8_t day;        /* 1-31 */
} rtc_date_t;
```

### Constants

```c
/* DS1307 control register values (from jump table at 0x1A4F4) */
#define DS1307_CTRL_OFF_LOW     0x00    /* SQW off, OUT=0 (case 0) */
#define DS1307_CTRL_OFF_HIGH    0x80    /* SQW off, OUT=1 (case 1) */
#define DS1307_CTRL_SQW_1HZ    0x10    /* SQW on, 1 Hz (case 2) */
#define DS1307_CTRL_SQW_4K     0x11    /* SQW on, 4.096 kHz (case 3) */
#define DS1307_CTRL_SQW_8K     0x12    /* SQW on, 8.192 kHz (case 4) */
#define DS1307_CTRL_SQW_32K    0x13    /* SQW on, 32.768 kHz (case 5) -- firmware default */
```

### `void rtc_Init(void)`

Initialize DS1307. Two operations:
1. Write control register 0x07 with SQW mode (0x13 = 32.768kHz, matching
   original firmware case 5).
2. Clear the Clock Halt (CH) bit if set to start the oscillator.

**Firmware:** `i2c_InitBus` (0xA884) -> `ds1307_Init` (0x14488) with param_2=5.
ds1307_Init dispatches through jump table at 0x1A4F4 to select ctrl value.

### `int rtc_GetTime(rtc_time_t *t)`

Read current time. Returns 0 on success, -1 on I2C error.

### `int rtc_SetTime(const rtc_time_t *t)`

Write time to DS1307. Returns 0 on success, -1 on I2C error.

### `int rtc_GetDate(rtc_date_t *d)`

Read current date. Returns 0 on success, -1 on I2C error.

### `int rtc_SetDate(const rtc_date_t *d)`

Write date to DS1307. Returns 0 on success, -1 on I2C error.

### `int rtc_ReadRAM(uint8_t addr, uint8_t *buf, uint8_t size)`

Read from battery-backed SRAM. Addr is 0-based offset (0-55).
Returns -1 if addr+size exceeds 56 bytes.

### `int rtc_WriteRAM(uint8_t addr, const uint8_t *buf, uint8_t size)`

Write to battery-backed SRAM. Same range check as read.

## DS1307 Register Map

```
+------+----------+-----------------------------------+
| 0x00 | Seconds  | BCD, bit 7 = Clock Halt (CH)     |
| 0x01 | Minutes  | BCD (0-59)                        |
| 0x02 | Hours    | BCD, 24-hour mode (bit 6 = 0)    |
| 0x03 | Day      | Day of week (1-7, not used)      |
| 0x04 | Date     | Day of month BCD (1-31)          |
| 0x05 | Month    | BCD (1-12)                        |
| 0x06 | Year     | BCD (00-99)                       |
| 0x07 | Control  | SQW output config (see below)    |
| 0x08 | RAM[0]   | Battery-backed SRAM              |
|  ..  |   ..     |   56 bytes total                  |
| 0x3F | RAM[55]  |                                   |
+------+----------+-----------------------------------+
```

### Control Register (0x07) — SQW/OUT Configuration

Recovered from ds1307_Init jump table at 0x1A4F4 (6 x 32-bit entries).
The `jmp @er2` at 0x144AA caused Ghidra to report UNRECOVERED_JUMPTABLE.

```
Bit 7:   OUT   — Output level when SQWE=0
Bit 4:   SQWE  — Square Wave Enable (1=on)
Bits 1:0 RS1:RS0 — Rate Select:
  00 = 1 Hz
  01 = 4.096 kHz
  10 = 8.192 kHz
  11 = 32.768 kHz
```

| Case | Value | OUT | SQWE | RS | Frequency |
|------|-------|-----|------|----|-----------|
| 0 | 0x00 | 0 | 0 | -- | SQW off, pin low |
| 1 | 0x80 | 1 | 0 | -- | SQW off, pin high |
| 2 | 0x10 | 0 | 1 | 00 | 1 Hz |
| 3 | 0x11 | 0 | 1 | 01 | 4.096 kHz |
| 4 | 0x12 | 0 | 1 | 10 | 8.192 kHz |
| 5 | 0x13 | 0 | 1 | 11 | **32.768 kHz** (firmware default) |

## Firmware Traceability

### Low-Level DS1307 Functions

| Our Function | Firmware Name | Address | Notes |
|-------------|---------------|---------|-------|
| `rtc_Init` (internal) | `ds1307_Init` | 0x14488 | SQW config via jump table 0x1A4F4 |
| `rtc_GetDate` (internal) | `ds1307_ReadDate` | 0x14056 | Reads regs 4-6, BCD->binary |
| `rtc_GetTime` (internal) | `ds1307_ReadTime` | 0x1410C | Reads regs 0-2, BCD->binary, masks CH |
| `rtc_ReadRAM` (internal) | `ds1307_ReadRAM` | 0x141DC | Assert: addr+size <= 56 |
| `rtc_SetTime` (internal) | `ds1307_WriteTime` | 0x1428A | Binary->BCD, writes regs 0-2 |
| `rtc_SetDate` (internal) | `ds1307_WriteDate` | 0x1433E | Binary->BCD, writes regs 4-6 |
| `rtc_WriteRAM` (internal) | `ds1307_WriteRAM` | 0x143F2 | Assert: addr+size <= 56 |

### Application-Level Wrappers

| Our Function | Firmware Name | Address | Error Handling |
|-------------|---------------|---------|----------------|
| `rtc_Init` | `i2c_InitBus` | 0xA884 | Prints "CLOCK ERROR", sets ptrI2C_Context1 |
| `rtc_SetTime` | `rtc_SetTime` | 0xA8AA | Copies 3B struct to stack, prints debug on error |
| `rtc_SetDate` | `rtc_SetDate` | 0xA8EA | Copies 3B struct to stack, prints debug on error |
| `rtc_GetDate` | `rtc_GetDate` | 0xA92A | Prints "CLOCK ERROR" |
| `rtc_GetTime` | `rtc_GetTime` | 0xA974 | Prints "CLOCK ERROR" |
| `rtc_WriteRAM` | `rtc_WriteRAM` | 0xA9C0 | Prints "CLOCK ERROR" |
| `rtc_ReadRAM` | `rtc_ReadRAM` | 0xA9EE | Prints "CLOCK ERROR" |

## L200 SRAM Usage

The firmware stores persistent data in DS1307 SRAM (56 bytes at 0x08-0x3F):

| Offset | Size | Purpose |
|--------|------|---------|
| 0-1 | 2 bytes | Mow time accumulator (word_FFEBBC backup) |
| 2 | 1 byte | Checksum (verified during config_InitFromEEPROM at 0x40AE) |

The config startup sequence (`config_InitFromEEPROM`) reads RTC RAM at
offsets 0 and 2 (two separate reads) and verifies the checksum matches
before applying the data.

## Implementation Notes

### BCD Conversion
The DS1307 stores all time/date values in Binary-Coded Decimal (BCD).
Our driver converts between BCD and binary at the API boundary:
- `bcd_to_bin()`: standard `(bcd >> 4) * 10 + (bcd & 0x0F)`
- `bin_to_bcd()`: standard `((bin / 10) << 4) | (bin % 10)`

The original firmware uses a slightly different seconds decode:
`(reg >> 3 & 0xE) * 5 + (reg & 0xF)` — mathematically equivalent
but avoids the division.

### Clock Halt Bit
Bit 7 of the seconds register (0x00) is the Clock Halt (CH) flag.
When set, the DS1307 oscillator is stopped. `rtc_Init()` clears this
bit to ensure the clock runs. `rtc_GetTime()` masks it out when reading.

### Error Handling
All public functions return -1 on I2C error. The original firmware
wrappers print "CLOCK ERROR" via DebugPrint on failure but continue
execution — RTC errors are logged but not fatal.

## Usage Example

```c
#include "rtc/rtc.h"

void read_clock(void)
{
    rtc_time_t t;
    rtc_date_t d;

    rtc_Init();           /* Configure SQW=32.768kHz, clear CH bit */
    rtc_GetTime(&t);
    rtc_GetDate(&d);
    /* t.hours, t.minutes, t.seconds now contain binary values */
    /* d.year, d.month, d.day now contain binary values */
}
```

## Test Coverage

Tests in `tests/test_rtc.c` (RTC_Test):

| # | Test | What it validates |
|---|------|-------------------|
| 1 | I2C bus error check | i2c_GetError(BUS1) == 0 |
| 2 | Read time | rtc_GetTime succeeds, range check H<24, M<60, S<60 |
| 3 | Read date | rtc_GetDate succeeds, range check 1<=M<=12, 1<=D<=31 |
| 4 | Clock running | Two reads 2s apart differ (oscillator active) |
| 5 | SRAM write/read | Write 0xDEADBEEF, read back, verify match |
| 6 | SetTime/GetTime roundtrip | Write known time, read back, verify match |
| 7 | SetDate/GetDate roundtrip | Write known date, read back, verify match |
| 8 | Control register | Read back reg 0x07, verify SQW config value |
| 9 | RAM boundary check | rtc_ReadRAM with addr+size>56 returns -1 |
