# LCD Driver — lcd

## Module Summary

HD44780-compatible character LCD driver using 4-bit parallel GPIO on Port 3.
Clean-room reimplementation derived from Ghidra reverse engineering of the
Ambrogio/Husqvarna L200 main board firmware (H8S/2144).

| Property | Value |
|----------|-------|
| Display | 16×2 HD44780 character LCD |
| Interface | 4-bit parallel GPIO (Port 3) |
| MCU Port | P3.0=RS, P3.1=RW, P3.2=E, P3.4–P3.7=D4–D7 |
| Source doc | `STATES/state_Display.md` |

## Files

| File | Description |
|------|-------------|
| `lcd.h` | Public API header |
| `lcd.c` | Implementation |
| `lcd_doc.md` | This file |

## How to Include

### In your C source file:

```c
#include "lcd/lcd.h"
```

### In the Makefile:

Add to your source list and include path:

```makefile
SRCS += lcd/lcd.c
```

### Dependencies

| Dependency | Path | Required For |
|------------|------|-------------|
| `iodefine.h` | `../iodefine.h` | Port 3 register definitions (`P3.DDR`, `P3.DR`) |
| `<stdio.h>` | toolchain | `vsnprintf()` in `lcd_Print()` |
| `<stdarg.h>` | toolchain | variadic args in `lcd_Print()` |
| `<string.h>` | toolchain | (currently unused, reserved) |
| `<stdint.h>` | toolchain | `uint8_t` types |

No other project modules are required. The LCD driver is standalone.

## API Reference

### `void lcd_Init(void)`

Full initialization: configures Port 3 DDR, sends HD44780 init sequence
(4-bit mode, 2-line, 5×8 font, display on, clear). Call once at startup
before any other lcd function.

### `void lcd_Clear(void)`

Clear display and return cursor to home position.

### `void lcd_SetRow(uint8_t row)`

Set DDRAM write address to the beginning of `row` (0–3, normally 0–1).

### `void lcd_WriteCommand(uint8_t cmd)`

Send a raw HD44780 command byte. Waits for busy flag first.

### `void lcd_WriteString(const char *str)`

Write a null-terminated string at the current cursor position. Does not
pad or truncate — use `lcd_Print()` for formatted row output.

### `void lcd_Print(uint8_t row, const char *fmt, ...)`

Printf-style formatted write to a full row. Always writes exactly 16
characters (pads short strings with spaces, truncates long ones).
Overwrites the entire row.

```c
lcd_Print(0, "Hello World");
lcd_Print(1, "Batt: %04u mV", voltage);
```

## Hardware Wiring

```
 H8S/2144 Port 3              HD44780 LCD
 +-----------------+          +-----------+
 | P3.0 (RS)  ───────────────→ RS        |
 | P3.1 (RW)  ───────────────→ RW        |
 | P3.2 (E)   ───────────────→ E         |
 | P3.3       (unused, input) |           |
 | P3.4 (D4)  ←─────────────→ D4        |
 | P3.5 (D5)  ←─────────────→ D5        |
 | P3.6 (D6)  ←─────────────→ D6        |
 | P3.7 (D7)  ←─────────────→ D7 / BF   |
 +-----------------+          +-----------+

 P3DDR = 0xF7 (write mode)   P3DDR = 0x07 (read/BF mode)
```

## Firmware Traceability

Every function maps to an original firmware address:

| Our Function | Firmware Name | Address |
|-------------|---------------|---------|
| `lcd_Init` | `lcd_FullInit` → `lcd_InitSequence` | 0xB0F4 → 0x14858 |
| `lcd_Clear` | `lcd_Clear` → `lcd_ClearDisplay` | 0xB0FC → 0x147B4 |
| `lcd_SetRow` | `lcd_SetRowAddress` | 0x147BC |
| `lcd_WriteCommand` | `lcd_WriteCommand` | 0x14782 |
| `lcd_WriteString` | `lcd_WriteString` | 0x147F8 |
| `lcd_Print` | `tm_DisplayWrite` → `lcd_FormatAndWrite` | 0xB168 → 0xB104 |
| (static) `lcd_SetRS` | `lcd_SetRS` | 0x146E6 |
| (static) `lcd_SetRW` | `lcd_SetRW` | 0x146EE |
| (static) `lcd_SetE` | `lcd_SetE` | 0x14728 |
| (static) `lcd_WriteNibble` | `lcd_WriteNibble` | 0x14748 |
| (static) `lcd_ReadNibble` | `lcd_ReadNibble` | 0x14768 |
| (static) `lcd_CheckBusy` | `lcd_CheckBusyFlag` | 0x146F6 |

## Not Included

These related subsystems are documented in `state_Display.md` but
intentionally excluded from this driver module:

| Feature | Reason | Future Module |
|---------|--------|---------------|
| Localization (`lcd_LocalizeString`) | Application-level, not hardware | `localization/` |
| Keyboard matrix scan | Separate hardware (Port 1 / Port 8) | `keyboard/` |
| Periodic display update | Application logic, not driver | application code |
| Auto-shutoff / backlight (P4.5) | Separate GPIO, power management | `power/` |

## Usage Example

```c
#include "lcd/lcd.h"

void startup_display(void)
{
    lcd_Init();
    lcd_Clear();
    lcd_Print(0, "2010 sw:%04d", sw_version);
    lcd_Print(1, "HW 2.0 S/N:%05ld", serial_number);
}
```
