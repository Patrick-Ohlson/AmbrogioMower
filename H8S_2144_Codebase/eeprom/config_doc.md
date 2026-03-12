# EEPROM Configuration Manager — config

## Module Summary

Typed C structure mapping the L200's 0xB0-byte EEPROM configuration block,
with factory defaults, dual-copy load/save, and checksum validation. The
struct can be overlaid directly on the RAM config area (0xFFEB60–0xFFEC0F)
or used as a standalone copy for load/modify/save workflows.

| Property | Value |
|----------|-------|
| Config size | 0xB0 (176) bytes |
| RAM location | 0xFFEB60 — 0xFFEC0F |
| EEPROM primary | Offset 0x000 (header) + 0x003 (data) + 0x0B3 (checksum) |
| EEPROM backup | Offset 0x400 (header) + 0x403 (data) + 0x4B3 (checksum) |
| Header | 3 bytes: version (0x01), size code (0x0B), header checksum |
| Checksum | Additive + one's complement (via `eeprom_ConfigChecksum`) |
| Packing | `__attribute__((packed))` — no padding, exact byte layout |
| Source doc | `STATES/state_i2c.md` section 4.4–4.8 |

## Files

| File | Description |
|------|-------------|
| `config.h` | Config struct, bit flag macros, API declarations |
| `config.c` | Factory defaults, load/save implementation |
| `config_doc.md` | This file |

## How to Include

```c
#include "eeprom/config.h"
```

```makefile
SRCS += eeprom/config.c
```

### Dependencies

| Dependency | Required For |
|------------|-------------|
| `<stdint.h>` | Fixed-width types |
| `<string.h>` | `memcpy` in `config_LoadDefaults()` |
| `eeprom/eeprom.h` | EEPROM read/write and checksum functions |
| `i2c/i2c.h` | (indirect, via eeprom module) |

## Config Structure Overview

The `mower_config_t` struct maps every byte of the 0xB0 config block:

```
Offset  Size  Field                   Section
------  ----  ----------------------  ---------------------------
+0x00   32B   Schedule & Area Config  schedule_base, area pct/flags, compass, timers
+0x20   48B   Extended Schedule       schedule_table[40], date, area_priority[4]
+0x50   20B   Runtime Statistics      work time, password, speed, charge counters
+0x64    3B   Config Parameters       max_follow_wire, noise_level, battery_type
+0x67   16B   (reserved)
+0x77    6B   Temperature Max Stats   motor & driver peak temperatures
+0x7D   13B   Error Counters          per-error-type saturating counters
+0x8A   33B   Event Timestamps        11 × packed_date_t (3 bytes each)
+0xAB    4B   Config Bit Flags        config_bits, feature_flags, ext_feature/config
+0xAF    1B   (reserved)
        ----
        176B = 0xB0 total
```

## API Reference

### Types

```c
/* 3-byte packed date (matches DS1307 date format) */
typedef struct __attribute__((packed)) {
    uint16_t date_word;     /* High part of packed date */
    uint8_t  date_byte;     /* Low part / extra byte */
} packed_date_t;

/* Main config: 0xB0 bytes, exact EEPROM layout */
typedef struct __attribute__((packed)) {
    /* ... 176 bytes of fields — see config.h for full layout ... */
} mower_config_t;
```

### Constants

```c
/* Factory default instance (const, in ROM) */
extern const mower_config_t CONFIG_FACTORY_DEFAULTS;
```

### `void config_LoadDefaults(mower_config_t *cfg)`

Copy factory defaults into a config struct. Equivalent to firmware's
`config_FactoryDefaults` / `fSetParameterToDefault` (0x1844):
1. Zeros entire 0xB0-byte block
2. Sets non-zero defaults (charge_zone, timers, battery type, flag bits)

### `int config_Load(mower_config_t *cfg)`

Load and validate config from EEPROM with dual-copy fallback.

| Return | Meaning |
|--------|---------|
| 0 | Loaded from primary EEPROM copy |
| 1 | Primary corrupt, loaded from backup |
| -1 | Both corrupt — factory defaults loaded |

Firmware: `config_InitFromEEPROM` (0x40AE)

### `int config_Save(const mower_config_t *cfg)`

Write config to both EEPROM copies with header and checksums.
Writes backup first (0x400), then primary (0x000).

Returns 1 on success, 0 on EEPROM write failure.

Firmware: `config_SaveBothCopies` (0x4120)

## Config Bit Flag Macros

### bConfigBits (+0xAB) — `config_bits`

| Macro | Value | Meaning |
|-------|-------|---------|
| `CFG_RAIN_MASK` | 0xC0 | Bits 7:6 mask |
| `CFG_RAIN_DISABLED` | 0x00 | Rain sensor off |
| `CFG_RAIN_PAUSE` | 0x40 | Pause on rain |
| `CFG_RAIN_ENABLED` | 0x80 | Rain sensor on (default) |
| `CFG_COMPASS_MASK` | 0x30 | Bits 5:4 mask |
| `CFG_COMPASS_DISABLED` | 0x00 | Compass off |
| `CFG_COMPASS_ENABLED` | 0x20 | Compass on |

### featureFlags (+0xAC) — `feature_flags`

| Macro | Value | Meaning |
|-------|-------|---------|
| `CFG_FEAT_FORCE_CHARGE` | 0x10 | Force charge mode |
| `CFG_FEAT_HI_BATT_SEL` | 0x08 | Alt battery threshold |
| `CFG_FEAT_ZONE_TRACK` | 0x02 | Charge zone tracking |

### extFeatureFlags (+0xAD) — `ext_feature_flags`

| Macro | Value | Meaning |
|-------|-------|---------|
| `CFG_EXT_SPIRAL` | 0x04 | Spiral mow pattern |
| `CFG_EXT_RAPID_RETURN` | 0x02 | Fast return to charger |
| `CFG_EXT_SAFETY_HANDLE` | 0x01 | Safety handle required |

### extConfigFlags2 (+0xAE) — `ext_config_flags2`

| Macro | Value | Meaning |
|-------|-------|---------|
| `CFG_EXT2_BORDER_BLADE` | 0x40 | Border blade enable |
| `CFG_EXT2_CLOSED_AREA` | 0x20 | Closed area work mode |

### Area Flags (per-area byte)

| Macro | Value | Meaning |
|-------|-------|---------|
| `AREA_FLAG_DIRECTION` | 0x80 | Reverse mow direction |
| `AREA_FLAG_BOUNCE` | 0x40 | Bounce on perimeter wire |

## Factory Defaults

| Field | Offset | Default | Description |
|-------|--------|---------|-------------|
| `charge_zone` | +0x12 | 0x7F (127) | Charge zone boundary |
| `timer_param1` | +0x13 | 0x08 (8) | Timer parameter 1 |
| `timer_param2` | +0x14 | 0x1E (30) | Timer parameter 2 |
| `timer_param3` | +0x19 | 0x16 (22) | Timer parameter 3 |
| `timer_param4` | +0x1A | 0x1E (30) | Timer parameter 4 |
| `max_follow_wire` | +0x64 | 0x0D (13) | Max wire follow distance |
| `noise_level` | +0x65 | 0x1C (28) | Noise threshold |
| `battery_type` | +0x66 | 0x42 ('B') | Battery type B |
| `config_bits` | +0xAB | 0x81 | Rain sensor + bit0 |
| `feature_flags` | +0xAC | 0x03 | Zone track + bit0 |
| `ext_feature_flags` | +0xAD | 0x00 | None |
| `ext_config_flags2` | +0xAE | 0x60 | Border blade + closed area |

All other fields default to 0.

## EEPROM Layout Per Copy

```
+0x00  version byte (0x01)
+0x01  size code (0x0B)
+0x02  header_chk = ~(0 + version + size)
+0x03  config data (0xB0 bytes = mower_config_t)
+0xB3  data_chk = ~(0 + sum of config data)
```

## Firmware Traceability

| Our Function | Firmware Name | Address |
|-------------|---------------|---------|
| `config_LoadDefaults` | `config_FactoryDefaults` / `SetStartConfig` | 0x1844 |
| `config_Load` | `config_InitFromEEPROM` / `InitAndGetConfigErrors` | 0x40AE |
| `config_Save` | `config_SaveBothCopies` / `doEepromConfig` | 0x4120 |
| (internal write) | `config_SaveToEEPROM` / `tm_sub_3F1E` | 0x3F1E |
| — | `config_PrepareRead` (RTC RAM backup) | 0x4138 |
| — | `config_ApplyBatteryType` | 0x3F94 |

## Not Included

| Feature | Reason | Location |
|---------|--------|----------|
| `config_ApplyBatteryType` | Depends on voltage threshold RAM vars | Firmware 0x3F94 |
| `config_PrepareRead` (RTC backup) | Writes mow_time_accum to DS1307 SRAM | Firmware 0x4138 |
| `config_SaveStats` (periodic) | Application-level, calls PrepareRead | Firmware 0x4138 |
| Language table apply | Separate localization module | `tm_set_Languages` |
| Factory default marker (error 5) | Detection logic not fully documented | `config_Validate` |

## Usage Example

```c
#include "eeprom/config.h"
#include "eeprom/eeprom.h"

mower_config_t cfg;

void startup(void)
{
    int result = config_Load(&cfg);

    if (result == -1) {
        /* Both EEPROM copies corrupt — running on factory defaults */
        config_Save(&cfg);   /* Write fresh defaults to EEPROM */
    }

    /* Adjust a setting */
    cfg.noise_level = 0x20;
    cfg.config_bits = (cfg.config_bits & ~CFG_RAIN_MASK) | CFG_RAIN_PAUSE;

    /* Save to both EEPROM copies */
    config_Save(&cfg);
}
```
