/*
 * misc.c - Miscellaneous hardware control (LED, backlight, shutoff)
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_mower_init.md
 * and STATES/state_Display.md
 *
 * Hardware assignments:
 *   P9.5 — Status LED (toggle via XOR 0x20)
 *   P4.5 — LCD backlight / power enable
 *   P2[2:0] — Motor driver outputs (cleared at init)
 *   PA.0 — Open-drain configuration
 *   DA_DADR0 — Blade motor DAC (0xFF = brake)
 */

#include "misc.h"
#include "../iodefine.h"
#include "../motor/motor.h"     /* shadow_p2_dr, shadow_p4_dr, shadow_p9_dr */

/* ---- Auto-shutoff state ---- */
static uint16_t shutoff_counter = MISC_SHUTOFF_RELOAD;
static uint16_t shutoff_phase = 0;     /* Power-save cycle phase counter */

/* ==================================================================
 * Public API
 * ================================================================== */

/*
 * misc_InitPorts - GPIO port initialization
 *
 * Matches fDoPorts1 from the startup sequence:
 *   P2_DR &= 0xF8  — clear bits 2:0 (motor driver outputs low)
 *   PA_ODR |= 0x01  — PA.0 open-drain mode
 *   DA_DADR0 = 0xFF — DAC0 output max (blade motor brake / off)
 *   P4_DR |= 0x20   — Set P4.5 (power enable / backlight on)
 */
void misc_InitPorts(void)
{
    shadow_p2_dr &= 0xF8;      /* Clear P2 bits 2:0 — motor outputs low */
    P2.DR.BYTE = shadow_p2_dr;
    PA.ODR.BYTE |= 0x01;       /* PA.0 open-drain */
    DA.DADR0 = 0xFF;            /* DAC0 = max (blade motor brake) */
    shadow_p4_dr |= 0x20;      /* P4.5 = power enable / backlight on */
    P4.DR.BYTE = shadow_p4_dr;
}

/*
 * misc_BlinkLed - Toggle status LED on P9.5
 *
 * Firmware: BlinkLed — XOR toggle P9_DR bit 5
 */
void misc_BlinkLed(void)
{
    shadow_p9_dr ^= 0x20;
    P9.DR.BYTE = shadow_p9_dr;
}

void misc_SetLed(uint8_t on)
{
    if (on)
        shadow_p9_dr |= 0x20;
    else
        shadow_p9_dr &= (uint8_t)~0x20;
    P9.DR.BYTE = shadow_p9_dr;
}

void misc_BacklightOn(void)
{
    shadow_p4_dr |= 0x20;
    P4.DR.BYTE = shadow_p4_dr;
}

void misc_BacklightOff(void)
{
    shadow_p4_dr &= (uint8_t)~0x20;
    P4.DR.BYTE = shadow_p4_dr;
}

void misc_ShutoffReset(void)
{
    shutoff_counter = MISC_SHUTOFF_RELOAD;
    shutoff_phase = 0;
}

/*
 * misc_ShutoffTick - Auto-shutoff countdown tick
 *
 * From state_Display.md §6.7:
 *   wKeyCounter starts at 3000 and decrements each scan.
 *   At 0: enters power-save cycle with P4.5 toggling pattern:
 *     phase 0x7E (126): P4.5 on
 *     phase 0x88 (136): P4.5 off
 *     phase 0x9C (156): P4.5 on
 *     phase 0xA6 (166): wrap to 0x8000, P4.5 off
 */
uint8_t misc_ShutoffTick(void)
{
    if (shutoff_counter > 0) {
        shutoff_counter--;
        return 0;   /* Not in shutoff mode */
    }

    /* In power-save mode — toggle P4.5 on timed pattern */
    shutoff_phase++;

    if (shutoff_phase == 0x7E) {
        shadow_p4_dr |= 0x20;        /* On */
        P4.DR.BYTE = shadow_p4_dr;
    }
    else if (shutoff_phase == 0x88) {
        shadow_p4_dr &= (uint8_t)~0x20;  /* Off */
        P4.DR.BYTE = shadow_p4_dr;
    }
    else if (shutoff_phase == 0x9C) {
        shadow_p4_dr |= 0x20;        /* On */
        P4.DR.BYTE = shadow_p4_dr;
    }
    else if (shutoff_phase == 0xA6) {
        shadow_p4_dr &= (uint8_t)~0x20;  /* Off */
        P4.DR.BYTE = shadow_p4_dr;
        shutoff_phase = 0;          /* Wrap (firmware sets to 0x8000, we restart) */
    }

    return 1;   /* In shutoff mode */
}

void misc_SetDAC(uint8_t value)
{
    DA.DADR0 = value;
}
