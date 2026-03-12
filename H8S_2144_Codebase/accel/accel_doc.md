# Accelerometer/Tilt Sensor Driver — accel

## Module Summary

3-axis I2C accelerometer driver with auto-detection for two sensor variants.
Provides raw reading, IIR filtering, and tilt angle computation for the
mower's tilt/lift safety system.

| Property | Value |
|----------|-------|
| Sensor A | MMA7455L (Freescale/NXP), I2C 0x1D |
| Sensor B | LIS3DH (STMicro), I2C address from context |
| Interface | Software I2C (bit-banged, separate bus from RTC/EEPROM) |
| Filter | 15/16 IIR low-pass on all 3 axes |
| Source doc | `STATES/state_i2c.md` (section 6) |

## Files

| File | Description |
|------|-------------|
| `accel.h` | Public API header with axis/tilt structs |
| `accel.c` | Implementation (both sensor variants) |
| `accel_doc.md` | This file |

## How to Include

```c
#include "accel/accel.h"
```

```makefile
SRCS += accel/accel.c
```

### Dependencies

| Dependency | Required For |
|------------|-------------|
| `<stdint.h>` | `uint8_t`, `int16_t`, `int32_t` types |
| `i2c/i2c.h` | `i2c_Start()`, `i2c_Stop()`, `i2c_WriteByte()`, `i2c_ReadByte()` |

**Bus assignment:** This module uses `I2C_BUS2` — a **separate bus** from
the RTC/EEPROM (which share Bus 1). The original firmware routes
accelerometer traffic through `ptrBladeJumpTable2` with potentially
different SDA/SCL pins. The `i2c/` module must be initialized for Bus 2
before calling `accel_Init()`.

```makefile
SRCS += i2c/i2c.c accel/accel.c
```

## API Reference

### `uint8_t accel_Init(void)`

Auto-detect sensor: probes MMA7455L first (WHO_AM_I at reg 0x47),
then tries LIS3DH init. Returns `ACCEL_MMA7455L`, `ACCEL_LIS3DH`,
or `ACCEL_NONE`.

### `int accel_Read(accel_raw_t *raw)`

Read sensor and update IIR filter. Optionally returns raw axis data
before filtering. Axis orientation is corrected per sensor variant:
- MMA7455L: X and Y negated (inverted mounting)
- LIS3DH: X/Y swapped and negated

### `void accel_GetFiltered(accel_filtered_t *f)`

Get current 32-bit IIR-filtered axis accumulators (fixed-point, x16 scale).

### `void accel_GetTilt(accel_tilt_t *tilt)`

Compute tilt angles in degrees from filtered data using atan2 approximation.

### `uint8_t accel_GetType(void)`

Returns the detected sensor type constant.

## Tilt Safety System

The L200 firmware reads the accelerometer every 50 ticks in the idle function
and classifies tilt severity:

| Condition | Threshold | Counter Cap | Response |
|-----------|-----------|-------------|----------|
| Normal | Both axes < 31 deg | decrement to 0 | Clear tilt flag |
| Moderate | One axis >= 31 deg | increment to 20 | Stop navigation |
| Severe | Both axes >= 31 deg | increment to 12 | Faster safety stop |

EEPROM calibration offsets at config +0x56 and +0x58 are subtracted
from raw tilt angles for zero-point correction.

## Firmware Traceability

| Our Function | Firmware Name | Address |
|-------------|---------------|---------|
| `accel_Init` | `accel_Init` | 0xBC7A |
| `accel_Read` | `accel_Poll` → `accel_ReadMMA7455`/`accel_ReadLIS3DH` | 0xBCDA → 0xBC38/0xBB44 |
| (filter) | `accel_FilterXYZ` | 0xBA44 |
| `accel_GetTilt` | `accel_PackVector` + `accel_CalcTiltAngle` | 0x157C8 + 0x157D4 |
| (MMA probe) | `mma7455_Probe` | 0x14B3E |
| (MMA read) | `mma7455_ReadAxes` | 0x14A80 |
| (LIS init) | `lis3dh_Init` | 0x14E32 |
| (LIS read) | `lis3dh_ReadXYZ_Raw` | 0x14D1A |

## Not Included

| Feature | Reason | Location |
|---------|--------|----------|
| Speed estimation (LIS3DH Z-axis IIR) | Application-level | `idleFunc` in MainMower |
| Tilt counter & recovery timer | Application-level safety logic | `idleFunc` |
| EEPROM calibration offsets | Handled by config system | `eeprom/` module |

## Usage Example

```c
#include "accel/accel.h"

void check_tilt(void)
{
    accel_tilt_t tilt;
    uint8_t type;

    type = accel_Init();
    if (type == ACCEL_NONE)
        return;     /* no sensor found */

    accel_Read(NULL);           /* read + filter */
    accel_GetTilt(&tilt);

    if (tilt.tilt1 > 31 || tilt.tilt1 < -31)
        /* tilt detected on axis 1 */;
}
```
