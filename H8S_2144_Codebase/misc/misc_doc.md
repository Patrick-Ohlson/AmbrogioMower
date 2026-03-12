# Misc Hardware Control — misc

## Module Summary

Simple utility functions for status LED, LCD backlight/power control,
auto-shutoff countdown, and DAC output for blade motor braking.

| Property | Value |
|----------|-------|
| Status LED | P9.5 (toggle via XOR 0x20) |
| Backlight / Power | P4.5 (set = on, clear = off) |
| Motor Outputs | P2[2:0] (cleared at init) |
| DAC | DA_DADR0 (0xFF = brake, lower = motor speed) |
| Auto-Shutoff | 3000-tick countdown, P4.5 toggle pattern |
| Source doc | `STATES/state_mower_init.md`, `STATES/state_Display.md` §6.7 |

## Files

| File | Description |
|------|-------------|
| `misc.h` | Public API header |
| `misc.c` | Implementation |
| `misc_doc.md` | This file |

## How to Include

```c
#include "misc/misc.h"
```

```makefile
SRCS += misc/misc.c
```

### Dependencies

| Dependency | Required For |
|------------|-------------|
| `<stdint.h>` | `uint8_t`, `uint16_t` types |
| `iodefine.h` | P2, P4, P9, PA, DA register access |

## API Reference

### `void misc_InitPorts(void)`

Initialize GPIO ports to safe defaults (fDoPorts1 equivalent):
- P2 bits 2:0 cleared (motor outputs low)
- PA.0 open-drain mode
- DA_DADR0 = 0xFF (blade motor brake)
- P4.5 set (power enable / backlight on)

### `void misc_BlinkLed(void)`

Toggle the status LED on P9.5 (XOR toggle).

### `void misc_SetLed(uint8_t on)`

Set the status LED to a specific state (on or off).

### `void misc_BacklightOn(void)` / `void misc_BacklightOff(void)`

Control the LCD backlight / power enable via P4.5.

### `void misc_ShutoffReset(void)`

Reset the auto-shutoff counter to 3000 ticks. Call on any keypress or
user activity.

### `uint8_t misc_ShutoffTick(void)`

Advance the auto-shutoff countdown. Returns 1 when in shutoff (power-save)
mode, 0 when still counting down. When shutoff activates, toggles P4.5
in the following pattern:

| Phase | Action |
|-------|--------|
| 0x7E (126) | P4.5 on |
| 0x88 (136) | P4.5 off |
| 0x9C (156) | P4.5 on |
| 0xA6 (166) | P4.5 off, wrap to start |

### `void misc_SetDAC(uint8_t value)`

Set DAC channel 0 (DA_DADR0). 0xFF = blade motor brake/off. Lower values
for motor speed control.

## Hardware Pin Map

```
P9.5  ─── Status LED (active high, toggle)
P4.5  ─── LCD Backlight / Power Enable
P2.0  ─── Motor driver output (cleared at init)
P2.1  ─── Motor driver output (cleared at init)
P2.2  ─── Motor driver output (cleared at init)
PA.0  ─── Open-drain configuration
DA0   ─── Blade motor DAC (0xFF = brake)
```

## Firmware Traceability

| Our Function | Firmware Name | Address/Location |
|-------------|---------------|-----------------|
| `misc_InitPorts` | `fDoPorts1` | Startup sequence |
| `misc_BlinkLed` | `BlinkLed` | Various (P9_DR ^= 0x20) |
| `misc_BacklightOn` | — | P4_DR \|= 0x20 (Startup) |
| `misc_BacklightOff` | — | P4_DR &= ~0x20 (shutoff) |
| `misc_ShutoffTick` | wKeyCounter decrement | state_Display.md §6.7 |
| `misc_SetDAC` | DA_DADR0 assignment | fDoPorts1 |

## Not Included

| Feature | Reason | Location |
|---------|--------|----------|
| Full keypad scanning | Separate keypad module | `state_Display.md` §6 |
| Motor driver PWM control | Separate motor module | `state_mower_init.md` |
| Charge control logic | Application-level | `state_mower_init.md` |
| Watchdog configuration | Startup-only, one-time | `OldMain` |
| Board variant detection | Startup-only | `CheckBoard` |

## Usage Example

```c
#include "misc/misc.h"

void main_init(void)
{
    misc_InitPorts();       /* Safe defaults for all ports */
    misc_BacklightOn();     /* LCD backlight on */
}

void main_loop(void)
{
    /* Blink LED every iteration */
    misc_BlinkLed();

    /* Auto-shutoff: call each scan tick */
    if (misc_ShutoffTick()) {
        /* In power-save mode */
    }

    /* On keypress: */
    misc_ShutoffReset();
}
```
