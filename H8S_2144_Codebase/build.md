# Build Instructions — L200 TestFirmware

## Toolchain

**KPIT GNUH8 v12.02** — H8S/2600 cross-compiler for Renesas H8S/2144

| Tool | Binary | Purpose |
|------|--------|---------|
| Compiler | `h8300-elf-gcc` | C compiler, H8S/2600 mode |
| Objcopy | `h8300-elf-objcopy` | ELF to flat binary conversion |

Toolchain location: `F:\Dropbox\Robotic\GNUH8\h8300-elf\`

## Quick Build (command line)

```bash
cd "F:/Dropbox/Robotic/GNUH8/2021_GNUH8_Mower/BASE_L200_2013_REMOTE - Claude/STATES/TestFirmware"
PATH="/F/Dropbox/Robotic/GNUH8/h8300-elf/bin:$PATH" make rom
```

Or use `make` then `h8300-elf-objcopy -O binary main.out main.bin`.

## Compiler Flags

| Flag | Purpose |
|------|---------|
| `-ms` | H8S CPU mode (16/32-bit H8S instruction set) |
| `-ms2600` | H8S/2600 variant (includes H8S/2144) |
| `-nostartfiles` | No default C runtime startup (we provide start.S) |
| `-save-temps` | Keep `.i` (preprocessed) and `.s` (assembly) files |
| `-O0` | No optimization (readable disassembly/debugging) |
| `-ggdb` | GDB debug info |
| `-DROMSTART` | Define `ROMSTART` macro for conditional compilation |

## Source Files

All sources are listed in the `Makefile` in both OBJS and SOURCES variables.
The gcc command line must match.

### Core / Startup
| File | Purpose |
|------|---------|
| `start.S` | Startup assembly (vector table jump, .data copy, .bss clear) |
| `vects.c` | Interrupt vector table |
| `sci.c` | SCI1 UART driver (polled TX/RX) |
| `hwinit.c` | Hardware init (port DDR/PCR/DR setup) |

### Drivers (each in own subdirectory)
| Directory | Files | Purpose |
|-----------|-------|---------|
| `timer/` | `timer.c` | Software timer linked list, GetSystemCounter() |
| `lcd/` | `lcd.c` | HD44780 LCD (4-bit, Port 3), lcd_Print() |
| `misc/` | `misc.c` | LED, backlight, auto-shutoff |
| `i2c/` | `i2c.c` | Bit-banged I2C (Bus 1 + Bus 2) |
| `rtc/` | `rtc.c` | DS1307 RTC (I2C Bus 1) |
| `motor/` | `motor.c` | L6203 wheel+blade motors, shadow registers |
| `steering/` | `steering.c` | High-level direction API |
| `utils/` | `utils.c` | delay_ms, print_hex*, FRT helpers |
| `eeprom/` | `eeprom.c`, `config.c` | 24C16 EEPROM + config parser |
| `accel/` | `accel.c` | LIS302DL accelerometer (I2C Bus 2) |
| `keyboard/` | `keyboard.c` | 5-key matrix keypad (P8 cols, P1 rows) |
| `battery/` | `battery.c` | ADC ch0/ch2, IIR filter, charge relay |
| `wire/` | `wire.c` | Boundary wire signal detection (P2.2) |
| `bump/` | `bump.c` | Bump/lift sensor debounce (P6.0) |

### Test Modules
| File | Menu Key | Purpose |
|------|----------|---------|
| `tests/test_motor.c` | 1 | Wheel motor test |
| `tests/test_drive.c` | 2 | Blade motor test |
| `tests/test_timer.c` | 3 | Timer demo |
| `tests/test_sci.c` | 4 | SCI/UART demo |
| `tests/test_lcd.c` | 5 | LCD test |
| `tests/test_memory.c` | 6 | .data/.bss memory test |
| `tests/test_misc.c` | 7 | Misc GPIO test |
| `tests/test_rtc.c` | 8 | RTC test |
| `tests/test_i2c.c` | 9 | I2C diagnostic |
| `tests/test_steering.c` | A | Steering test |
| `tests/demo.c` | B | Demo mode |
| `tests/test_eeprom.c` | C | EEPROM test |
| `tests/test_accel.c` | D | Accelerometer test |
| `tests/test_keyboard.c` | E | Keyboard test |
| `tests/test_battery.c` | F | Battery/charger ADC test |
| `tests/test_wire.c` | G | Wire signal detection test |
| `tests/test_bump.c` | H | Bump sensor test |

### Other
| File | Purpose |
|------|---------|
| `main.c` | Main entry, test menu, UART setup |
| `main.ld` | Linker script (ROM 0x00000-0x1FFFF, RAM 0xFFE080+) |
| `version.h` | Auto-generated build date + revision |

## Build Outputs

| File | Format | Description |
|------|--------|-------------|
| `main.out` | ELF | Linked executable (for Ghidra/debugger) |
| `main.bin` | Raw binary | Flat binary for flash programming (131,069 bytes) |
| `main.map` | Text | Linker map with section addresses |

## Adding a New Module — Checklist

When adding a new driver or test module, update **ALL THREE** of these:

### 1. Makefile — OBJS variable
Add the `.o` file:
```makefile
OBJS = ... \
       newmodule/newmodule.o \
       tests/test_newmodule.o \
       ...
```

### 2. Makefile — SOURCES variable
Add the `.c` file (same order):
```makefile
SOURCES = ... \
          newmodule/newmodule.c \
          tests/test_newmodule.c \
          ...
```

### 3. main.c — Three changes needed
1. **Include headers** (near top, before `tests/demo.h`):
   ```c
   #include "newmodule/newmodule.h"
   #include "tests/test_newmodule.h"
   ```
2. **Switch case** in `Test_Menu()` (after last case, before `default:`):
   ```c
   case 'X':
   case 'x': NewModule_Test();
              break;
   ```
3. **Menu display** in `String_Output()` (before the closing `*****` line):
   ```c
   SendString((unsigned char *)"      *   X. NewModule Test *");
   SendString((unsigned char *)"\r\n");
   SendString((unsigned char *)"\r\n");
   ```

### 4. Verify
Build and confirm zero errors + correct binary size:
```bash
PATH="/F/Dropbox/Robotic/GNUH8/h8300-elf/bin:$PATH" make rom
wc -c main.bin
```

## Notes

- Bare-metal H8S code — no OS, minimal libc
- `-save-temps` generates `.i` and `.s` intermediates for inspection
- Binary must be exactly 131,069 bytes (128KB ROM + 1 sentinel byte)
- Shadow registers in `motor.h` are shared across modules (keyboard, I2C, etc.)
