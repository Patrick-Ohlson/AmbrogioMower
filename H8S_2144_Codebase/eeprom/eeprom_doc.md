# EEPROM Driver — eeprom

## Module Summary

24Cxx series I2C EEPROM driver with page-boundary write handling, ACK polling,
and configuration system with dual-copy validation and checksum verification.

| Property | Value |
|----------|-------|
| Device | 24Cxx series I2C EEPROM (likely 24C16+) |
| I2C Address | 0xA0-0xA7 (bits A2:A0 from address bits 10:8) |
| Page Size | 16 bytes (write boundary) |
| Block Size | 256 bytes (device address boundary) |
| Config Layout | Dual copies at 0x000 and 0x400 |
| Interface | Software I2C (shared bus with RTC) |
| Source doc | `STATES/state_i2c.md` (sections 3-4) |

## Files

| File | Description |
|------|-------------|
| `eeprom.h` | Public API header with config constants |
| `eeprom.c` | Implementation (read/write/config validation) |
| `eeprom_doc.md` | This file |

## How to Include

```c
#include "eeprom/eeprom.h"
```

```makefile
SRCS += eeprom/eeprom.c
```

### Dependencies

| Dependency | Required For |
|------------|-------------|
| `<stdint.h>` | `uint8_t`, `uint16_t` types |
| `i2c/i2c.h` | `i2c_Start()`, `i2c_Stop()`, `i2c_WriteByte()`, `i2c_ReadByte()` |

**Bus assignment:** This module uses `I2C_BUS1` (shared with RTC).
The original firmware routes EEPROM traffic through `ptrBladeJumpTable1`.
The `i2c/` module must be initialized for Bus 1 before calling any EEPROM
functions.

```makefile
SRCS += i2c/i2c.c eeprom/eeprom.c
```

## API Reference

### Low-Level I/O

#### `int eeprom_Poll(uint16_t addr)`

ACK polling — waits for internal write cycle to complete (~5ms typical).
Repeatedly sends START + device address until EEPROM ACKs.

#### `int eeprom_Read(uint16_t addr, uint8_t *buf, uint16_t size)`

Read N bytes from EEPROM. Handles 256-byte block crossing automatically.
Returns 1 on success, 0 on failure.

#### `int eeprom_Write(uint16_t addr, const uint8_t *buf, uint16_t size)`

Write N bytes to EEPROM. Splits writes at 16-byte page boundaries.
Performs ACK polling before each page write. Returns 1 on success, 0 on failure.

### Wrapper Functions

#### `int eeprom_ReadBlock(uint16_t addr, uint8_t *buf, uint16_t size)`

Read with error reporting. Original firmware prints "EEPROM ERROR" on failure.

#### `int eeprom_WriteBlock(uint16_t addr, const uint8_t *buf, uint16_t size)`

Write with error reporting.

### Configuration System

#### `uint8_t eeprom_ConfigChecksum(uint8_t init, const uint8_t *data, uint16_t size)`

Additive checksum with one's complement: `~(init + sum(data[0..size-1]))`.
Used for both header and data validation.

#### `int eeprom_ConfigValidate(uint16_t base_addr, uint8_t *buf)`

Validates a config copy at the given EEPROM base address:

| Return | Meaning |
|--------|---------|
| 0 | OK — config valid, data in buf |
| 1 | Header checksum bad |
| 2 | Version byte mismatch |
| 3 | Size byte mismatch |
| 4 | Data checksum bad |
| 5 | Factory default marker |

## EEPROM Memory Layout

```
+--------+-----------------------------------+
| 0x0000 | Primary config header (3 bytes)   |
|        |   +0: version (0x01)              |
|        |   +1: size code (0x0B)            |
|        |   +2: header checksum             |
+--------+-----------------------------------+
| 0x0003 | Config data (0xB0 = 176 bytes)    |
|        |   RAM mirror: 0xFFEB60-0xFFEC0F   |
+--------+-----------------------------------+
| 0x00B3 | Data checksum (1 byte)            |
+--------+-----------------------------------+
| 0x00B4 | Statistics block                  |
+--------+-----------------------------------+
| 0x0400 | Backup config header (3 bytes)    |
+--------+-----------------------------------+
| 0x0403 | Backup config data (0xB0 bytes)   |
+--------+-----------------------------------+
| 0x04B3 | Backup data checksum              |
+--------+-----------------------------------+
```

## Startup Sequence

The L200 firmware loads config at startup (config_InitFromEEPROM, 0x40AE):

1. Try **backup** config at 0x400 first
2. If backup fails (non-zero, non-5): try **primary** at 0x000
3. If both fail or either returns 5: call factory defaults
4. Read DS1307 RAM for persistent battery type data
5. Apply battery type voltage thresholds

## Firmware Traceability

| Our Function | Firmware Name | Address |
|-------------|---------------|---------|
| `eeprom_Poll` | `eeprom_Poll` | 0x151B2 |
| `eeprom_Read` | `eeprom_Read` | 0x151F8 |
| `eeprom_Write` | `eeprom_Write` | 0x152D8 |
| `eeprom_ReadBlock` | `eeprom_ReadBlock` / `checkEEprom` | 0xB51A |
| `eeprom_WriteBlock` | `eeprom_WriteBlock` | 0xB548 |
| `eeprom_ConfigChecksum` | `config_Checksum` | 0x3EC8 |
| `eeprom_ConfigValidate` | `config_Validate` / `fCheckEprom` | 0x4002 |

## Not Included

| Feature | Reason | Location |
|---------|--------|----------|
| `config_InitFromEEPROM` (full startup load) | Application-level orchestration | `state_i2c.md` §4.4 |
| `config_SaveBothCopies` (save to both copies) | Application-level | `state_i2c.md` §4.5 |
| `config_SaveStats` (periodic stats save) | Application-level | `state_i2c.md` §4.5 |
| `config_FactoryDefaults` | Application-level | `state_i2c.md` §4.6 |
| `config_ApplyBatteryType` | Application-level | `state_i2c.md` §4.7 |
| Config RAM structure decode (0xFFEB60) | Application-level | `state_i2c.md` §4.8 |

## Usage Example

```c
#include "eeprom/eeprom.h"

void load_config(void)
{
    uint8_t config_buf[0xB0];
    int result;

    /* Try backup copy first (matches firmware behavior) */
    result = eeprom_ConfigValidate(EEPROM_CONFIG_BACKUP, config_buf);

    if (result != EEPROM_CFG_OK && result != EEPROM_CFG_FACTORY) {
        /* Try primary copy */
        result = eeprom_ConfigValidate(EEPROM_CONFIG_PRIMARY, config_buf);
    }

    if (result == EEPROM_CFG_OK) {
        /* config_buf now contains valid 176-byte config data */
    } else {
        /* Need factory defaults */
    }
}

void save_byte(void)
{
    uint8_t val = 0x42;
    eeprom_Write(0x66, &val, 1);    /* Write battery type 'B' */
}
```
