# I2C Bit-Bang Driver — i2c

## Module Summary

Software bit-banged I2C master driver supporting multiple independent bus
contexts. Provides all protocol primitives needed by the RTC, EEPROM, and
accelerometer driver modules.

| Property | Value |
|----------|-------|
| Mode | Master only, single-master bus |
| Speed | ~100 kHz (standard mode, depends on delay calibration) |
| Bus Count | 2 independent buses |
| Bus 1 | RTC (DS1307) + EEPROM (24Cxx) — firmware: ptrBladeJumpTable1 |
| Bus 2 | Accelerometer (MMA7455L / LIS3DH) — firmware: ptrBladeJumpTable2 |
| Features | Clock stretching support (~10ms timeout) |
| Source doc | `STATES/state_i2c.md` (section 1) |

## Files

| File | Description |
|------|-------------|
| `i2c.h` | Public API header with bus context struct |
| `i2c.c` | Implementation (bit-bang protocol) |
| `i2c_doc.md` | This file |

## How to Include

```c
#include "i2c/i2c.h"
```

```makefile
SRCS += i2c/i2c.c
```

### Dependencies

| Dependency | Required For |
|------------|-------------|
| `<stdint.h>` | `uint8_t` types |
| GPIO registers | Port DDR/DR/PIN access via context struct |

**Integration:** The `rtc`, `eeprom`, and `accel` modules all include
`i2c/i2c.h` and call the bus-aware API directly:
- **RTC + EEPROM** use `I2C_BUS1` (firmware: `ptrBladeJumpTable1`)
- **Accelerometer** uses `I2C_BUS2` (firmware: `ptrBladeJumpTable2`)

Initialize both buses via `i2c_Init()` before using any I2C peripheral.

## API Reference

### Types

```c
typedef struct {
    volatile uint8_t *ddr;      /* Port Data Direction Register */
    volatile uint8_t *dr;       /* Port Data Register */
    volatile uint8_t *pin;      /* Port Pin input Register */
    uint8_t sda_mask;           /* Bit mask for SDA pin */
    uint8_t scl_mask;           /* Bit mask for SCL pin */
    uint8_t error;              /* Bus error flag */
} i2c_bus_t;
```

### `void i2c_Init(uint8_t bus, const i2c_bus_t *ctx)`

Initialize a bus context. Copies the context, clears errors, and releases
SDA/SCL to idle (high).

### `int i2c_Start(uint8_t bus)`

Generate START condition (SDA goes low while SCL is high). Also handles
repeated-start if called while bus is active.

### `void i2c_Stop(uint8_t bus)`

Generate STOP condition (SDA goes high while SCL is high). Retries up to
17 times if SDA doesn't release due to bus contention.

### `int i2c_WriteByte(uint8_t bus, uint8_t byte)`

Send one byte MSB-first. Returns 1 if slave ACKs, 0 if NACK.

### `uint8_t i2c_ReadByte(uint8_t bus, uint8_t nack)`

Read one byte MSB-first. Set `nack=0` to send ACK (more bytes follow),
`nack!=0` to send NACK (last byte in transfer).

### `int i2c_ReadMulti(uint8_t bus, uint8_t *buf, uint8_t count)`

Read N bytes into buffer. Sends ACK after each byte except the last (NACK).

### `int i2c_WriteMulti(uint8_t bus, const uint8_t *buf, uint8_t count)`

Write N bytes from buffer. Checks ACK after each byte. Returns 1 if all
ACKed, 0 on NACK.

### `uint8_t i2c_GetError(uint8_t bus)` / `void i2c_ClearError(uint8_t bus)`

Check/clear bus error flags (e.g. clock stretching timeout = bit 4).

## Bit-Bang Protocol

### Pin Control Method

The H8S/2144 ports use a DDR (Data Direction Register) to control pin
direction. For open-drain I2C:

| Action | DDR | DR | Effect |
|--------|-----|-----|--------|
| Drive low | 1 (output) | 0 | Pin driven low |
| Release (high) | 0 (input) | — | External pull-up pulls high |
| Read | 0 (input) | — | Read PIN register |

### Clock Stretching

`i2c_clock_high()` releases SCL then polls the pin for up to ~10ms.
If a slave holds SCL low (stretch), the master waits. On timeout,
`error` bit 4 (`0x10`) is set.

## Firmware Traceability

| Our Function | Firmware Name | Address |
|-------------|---------------|---------|
| `i2c_Init` | `i2c_InitBus` (partial) | 0xA884 |
| `i2c_Start` | `i2c_Start` | 0x14882 |
| `i2c_Stop` | `i2c_Stop` | 0x148A4 |
| `i2c_WriteByte` | `i2c_WriteByte` | 0x1495E |
| `i2c_ReadByte` | `i2c_ReadByte` | 0x148DA |
| `i2c_ReadMulti` | `i2c_ReadMulti` | 0x149C2 |
| `i2c_WriteMulti` | `i2c_WriteMulti` | 0x149F4 |
| (clock high) | `i2c_ClockHigh` | 0x15E96 |
| (clock low) | `i2c_ClockLow` | 0x15EF2 |
| (data high) | `i2c_DataHigh` | 0x15F00 |
| (data low) | `i2c_DataLow` | 0x15F0E |
| (read SDA) | `i2c_ReadSDA` | 0x15F1C |

## Not Included

| Feature | Reason | Location |
|---------|--------|----------|
| Function pointer table dispatch | Replaced by context struct | Original: `func_0x00001980` |
| GetSystemCounter for clock stretching timeout | Uses simple loop counter instead | `timer10ms` module |
| Wheel encoder I2C (Port 9 / SCI1/SCI2) | Separate bus, different protocol | `STATES/state_i2c.md` section 5 |
| CRC-8 protected I2C command protocol | Higher-level layer for motor controllers | `STATES/state_i2c.md` section 1.4 |

### CRC-8 Protocol Layer (firmware only)

The original firmware includes a CRC-8 protected command protocol built on
top of these I2C primitives, used for communication with external motor
controller boards. This layer provides:

- **CRC-8 integrity** via `crc8Update` (0x15B30) + 256-byte lookup table at ROM 0x1A516 (polynomial 0x5E)
- **Command bytes**: 0x40=read, 0x48=write8, 0x49=write16, 0x4A=write32
- **Transaction wrapper** `i2cTransactWithCRC` (0x10C28): write command + CRC, read response + verify CRC

This is an application-level protocol not needed for our clean-room driver
(which directly drives L6203 motors without external controllers). See
`STATES/state_i2c.md` section 1.4 for full protocol documentation.

## Usage Example

```c
#include "i2c/i2c.h"
#include "../iodefine.h"

void setup_i2c(void)
{
    /* Configure Bus 1 for RTC + EEPROM
     * Actual SDA/SCL pin assignments are board-dependent.
     * Example: Port A, SDA=PA.x, SCL=PA.y */
    i2c_bus_t bus1;
    bus1.ddr = &PA.EQU.DDR;
    bus1.dr  = (volatile uint8_t *)&PA.ODR.BYTE;
    bus1.pin = &PA.EQU.PIN.BYTE;
    bus1.sda_mask = 0x01;   /* Example: PA.0 */
    bus1.scl_mask = 0x02;   /* Example: PA.1 */

    i2c_Init(I2C_BUS1, &bus1);

    /* Now talk to DS1307 */
    i2c_Start(I2C_BUS1);
    i2c_WriteByte(I2C_BUS1, 0xD0);   /* DS1307 write addr */
    i2c_WriteByte(I2C_BUS1, 0x00);   /* Register 0 */
    i2c_Stop(I2C_BUS1);
}
```
