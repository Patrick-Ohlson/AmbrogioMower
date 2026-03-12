/*
 * misc.h - Miscellaneous hardware control (LED, backlight, shutoff)
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_mower_init.md
 * and STATES/state_Display.md
 *
 * Simple utility functions for:
 *   - Status LED toggle (P9.5)
 *   - LCD backlight / power control (P4.5)
 *   - Auto-shutoff countdown (wKeyCounter mechanism)
 *   - DAC output for blade motor brake (DA_DADR0)
 *
 * Original firmware functions:
 *   BlinkLed          (Startup)    -> misc_BlinkLed()
 *   fDoPorts1         (Startup)    -> misc_InitPorts()
 *   (auto-shutoff)    (state_Display.md §6.7) -> misc_ShutoffTick()
 */

#ifndef MISC_H
#define MISC_H

#include <stdint.h>

/* ---- Auto-shutoff constants ---- */
#define MISC_SHUTOFF_RELOAD     3000    /* Tick count before power-save (3000 scans) */

/* ---- Public API ---- */

/*
 * misc_InitPorts - Initialize GPIO ports for basic operation
 *
 * Sets up the minimal port configuration from fDoPorts1:
 *   P2_DR &= 0xF8  (clear motor driver outputs)
 *   PA_ODR |= 0x01  (PA.0 open-drain)
 *   DA_DADR0 = 0xFF (DAC max = blade motor brake)
 *   P4_DR |= 0x20   (P4.5 power enable)
 *
 * Firmware: fDoPorts1 (Startup sequence)
 */
void misc_InitPorts(void);

/*
 * misc_BlinkLed - Toggle the status LED on P9.5
 *
 * XOR toggles P9 bit 5 each time it is called.
 *
 * Firmware: BlinkLed (P9_DR ^= 0x20)
 */
void misc_BlinkLed(void);

/*
 * misc_SetLed - Set the status LED to a specific state
 *
 * @param on  Non-zero = LED on, 0 = LED off
 */
void misc_SetLed(uint8_t on);

/*
 * misc_BacklightOn - Turn on LCD backlight / power enable
 *
 * Sets P4.5 high.
 *
 * Firmware: P4_DR |= 0x20 (Startup, shutoff recovery)
 */
void misc_BacklightOn(void);

/*
 * misc_BacklightOff - Turn off LCD backlight / power enable
 *
 * Clears P4.5 low.
 *
 * Firmware: P4_DR &= ~0x20 (shutoff sequence)
 */
void misc_BacklightOff(void);

/*
 * misc_ShutoffReset - Reset the auto-shutoff countdown
 *
 * Call this on any keypress or user activity to reset the
 * shutoff counter to MISC_SHUTOFF_RELOAD (3000).
 *
 * Firmware: wKeyCounter = 3000 (on keypress)
 */
void misc_ShutoffReset(void);

/*
 * misc_ShutoffTick - Advance the auto-shutoff countdown by one tick
 *
 * Decrements the shutoff counter. When it reaches 0, enters a
 * power-save cycle that toggles P4.5 in a timed pattern:
 *   count 0x7E (126): P4.5 on
 *   count 0x88 (136): P4.5 off
 *   count 0x9C (156): P4.5 on
 *   count 0xA6 (166): wrap to 0x8000, P4.5 off
 *
 * @return  1 if shutoff is active (counter at 0), 0 otherwise
 *
 * Firmware: wKeyCounter decrement + P4.5 toggle (state_Display.md §6.7)
 */
uint8_t misc_ShutoffTick(void);

/*
 * misc_SetDAC - Set DAC channel 0 output
 *
 * Controls the blade motor DAC output. 0xFF = max (brake/off),
 * lower values = motor speed control.
 *
 * @param value  DAC output value (0x00-0xFF)
 *
 * Firmware: DA_DADR0 = value
 */
void misc_SetDAC(uint8_t value);

#endif /* MISC_H */
