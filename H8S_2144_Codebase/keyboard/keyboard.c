/*
 * keyboard.c - 5-key matrix keypad driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * See keyboard.h for full hardware description and scan table.
 *
 * Port access uses shadow registers (from motor.h) to avoid
 * read-modify-write hazards on H8S write-only DDR registers.
 *
 * P8 lower nibble [3:0] = column outputs (shared with motor DDR shadow)
 * P8 upper nibble [7:4] = I2C Bus 1 pins — MUST be preserved
 * P1 bits [4:0] = row inputs (active-low, external pull-ups)
 */

#include "keyboard.h"
#include "../iodefine.h"
#include "../motor/motor.h"     /* shadow_p8_ddr, shadow_p8_dr */
#include "../timer/timer.h"     /* GetSystemCounter() */

/* ---- Module state ---- */

/* Raw 32-bit scan result from last kbd_Scan() call */
static uint32_t last_scan;

/* Debounce state */
static uint8_t  prev_key;       /* Previous scan key code */
static uint8_t  debounce_cnt;   /* Consecutive matching scan count */
static uint8_t  key_accepted;   /* 1 = key already reported (edge trigger) */

/* ---- Internal helpers ---- */

/*
 * scan_to_key - Map 32-bit scan result to key code
 *
 * Only one key at a time is recognized. If multiple bits are set
 * (ghosting or multi-press), returns KEY_NONE.
 */
static uint8_t scan_to_key(uint32_t scan)
{
    switch (scan) {
    case KBD_SCAN_ENTER: return KEY_ENTER;
    case KBD_SCAN_DOWN:  return KEY_DOWN;
    case KBD_SCAN_HOME:  return KEY_HOME;
    case KBD_SCAN_PAUSE: return KEY_PAUSE;
    case KBD_SCAN_UP:    return KEY_UP;
    default:             return KEY_NONE;
    }
}

/* Brief NOP delay for column settle time after DDR change.
 * H8S needs a few clock cycles for the pin driver to stabilize. */
static void kbd_settle(void)
{
    __asm__ volatile ("nop");
    __asm__ volatile ("nop");
    __asm__ volatile ("nop");
    __asm__ volatile ("nop");
}

/* ---- Public API ---- */

void kbd_Init(void)
{
    last_scan    = 0;
    prev_key     = KEY_NONE;
    debounce_cnt = 0;
    key_accepted = 0;
}

uint8_t kbd_Scan(void)
{
    uint32_t result = 0;
    uint8_t  rows;
    uint8_t  col;
    uint8_t  saved_upper;

    /* Save the upper nibble of P8 DDR shadow (I2C Bus 1 pins) */
    saved_upper = shadow_p8_ddr & 0xF0;

    /* Ensure P8 DR lower nibble is 0 so DDR=1 drives LOW */
    shadow_p8_dr = shadow_p8_dr & 0xF0;
    P8.DR.BYTE = shadow_p8_dr;

    /* Scan each column: set one DDR bit at a time to drive that column low */
    for (col = 0; col < 4; col++) {
        /* Drive only column 'col' low: DDR bit = 1, DR bit = 0 */
        shadow_p8_ddr = saved_upper | (uint8_t)(1 << col);
        P8.DDR = shadow_p8_ddr;

        kbd_settle();

        /* Read rows: active-low, invert and mask bits [4:0] */
        rows = (~P1.DR.BYTE) & 0x1F;

        /* Pack row bits into 32-bit result using shift-left accumulate
         * (matches original firmware: uVar3 = ~bVar2 & 0x1f | uVar3 << 8).
         * First column scanned (col 0) ends up in MSB [31:24],
         * last column scanned (col 3) ends up in LSB [7:0]. */
        result = (result << 8) | (uint32_t)rows;
    }

    /* Restore P8 DDR: lower nibble = 0x00 (all columns high-Z after scan).
     * Matches original firmware: bShadow_P8_DDR &= 0xF0.
     * Prevents phantom current draw through the matrix. */
    shadow_p8_ddr = saved_upper;
    P8.DDR = shadow_p8_ddr;

    /* Mask to valid key positions only */
    result &= KBD_SCAN_MASK;

    last_scan = result;
    return scan_to_key(result);
}

uint8_t kbd_GetKey(void)
{
    uint8_t key = kbd_Scan();

    if (key == KEY_NONE) {
        /* No key pressed — reset debounce state */
        prev_key     = KEY_NONE;
        debounce_cnt = 0;
        key_accepted = 0;
        return KEY_NONE;
    }

    if (key == prev_key) {
        /* Same key as before */
        if (key_accepted) {
            /* Already reported this press — wait for release */
            return KEY_NONE;
        }
        if (debounce_cnt < KBD_DEBOUNCE_COUNT) {
            debounce_cnt++;
        }
        if (debounce_cnt >= KBD_DEBOUNCE_COUNT) {
            /* Debounce threshold reached — accept key (edge trigger) */
            key_accepted = 1;
            return key;
        }
    } else {
        /* Different key — restart debounce */
        prev_key     = key;
        debounce_cnt = 1;
        key_accepted = 0;
    }

    return KEY_NONE;
}

uint8_t kbd_WaitKey(uint16_t timeout_ticks)
{
    uint32_t start = GetSystemCounter();
    uint8_t  key;

    while (1) {
        key = kbd_GetKey();
        if (key != KEY_NONE)
            return key;

        /* Check timeout (0 = wait forever) */
        if (timeout_ticks != 0) {
            if ((GetSystemCounter() - start) >= timeout_ticks)
                return KEY_NONE;
        }
    }
}

uint32_t kbd_GetLastScan(void)
{
    return last_scan;
}
