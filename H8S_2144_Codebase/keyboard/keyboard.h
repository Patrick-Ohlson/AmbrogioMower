/*
 * keyboard.h - 5-key matrix keypad driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation of keyboardstuff() at 0x1ACD8
 * and state_Display.md section 6 keyboard analysis.
 *
 * Hardware: 4-column x 5-row GPIO matrix
 *   Columns (active-low output): P8 bits [3:0] via DDR shadow
 *     DDR=1 + DR=0 drives column LOW; DDR=0 = high-Z (pull-up idle)
 *   Rows (input): P1 bits [4:0] — active-low, pull-ups via PCR=0x5F
 *     Read as: ~P1.DR.BYTE & 0x1F
 *
 * Scan method (matches original firmware):
 *   For each column 0-3:
 *     1. Set shadow_p8_ddr = (upper nibble) | (1 << col)
 *     2. Write shadow to P8.DDR hardware
 *     3. Brief settle delay (few NOPs)
 *     4. Read rows: (~P1.DR.BYTE) & 0x1F
 *     5. Shift-left accumulate: result = (result << 8) | rows
 *   Restore shadow_p8_ddr lower nibble to 0x00 (high-Z) after scan.
 *   Mask result with 0x1C100010 (valid key bit positions).
 *
 * 32-bit scan result byte layout (shift-left accumulate):
 *   Bits [31:24] = Column 0 (P8.0) — first scanned, shifted left 3x
 *   Bits [23:16] = Column 1 (P8.1) — shifted left 2x
 *   Bits [15:8]  = Column 2 (P8.2) — shifted left 1x
 *   Bits [7:0]   = Column 3 (P8.3) — last scanned, stays in LSB
 *
 * 5 physical keys on L200 keypad:
 *
 *   Key   | Code | Scan Result  | Col/Row
 *   ------+------+--------------+---------
 *   ENTER | 'E'  | 0x00000010   | P8.3 / P1.4  (col 3 = LSB byte)
 *   DOWN  | 'D'  | 0x00100000   | P8.1 / P1.4  (col 1 = byte 2)
 *   HOME  | 'H'  | 0x04000000   | P8.0 / P1.2  (col 0 = MSB byte)
 *   PAUSE | 'P'  | 0x08000000   | P8.0 / P1.3  (col 0 = MSB byte)
 *   UP    | 'U'  | 0x10000000   | P8.0 / P1.4  (col 0 = MSB byte)
 *
 * Debounce: 6 consecutive identical non-zero scans before key accepted
 * (matches original firmware counter threshold).
 *
 * Original firmware functions:
 *   keyboardstuff   (0x1ACD8) -> kbd_Scan() + kbd_GetKey()
 *   Auto-shutoff counter at 3000 (30 seconds @ 10ms tick)
 */

#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <stdint.h>

/* ---- Key codes ---- */
#define KEY_NONE    0       /* No key pressed */
#define KEY_ENTER   'E'     /* 0x45 — Enter/Select key */
#define KEY_DOWN    'D'     /* 0x44 — Down arrow key */
#define KEY_HOME    'H'     /* 0x48 — Home key */
#define KEY_PAUSE   'P'     /* 0x50 — Pause key */
#define KEY_UP      'U'     /* 0x55 — Up arrow key */

/* ---- Scan result bit masks (verified against Ghidra keyboardstuff 0x1ACD8) ---- */
#define KBD_SCAN_ENTER  0x00000010UL    /* P8.3 col (byte 0), P1.4 row */
#define KBD_SCAN_DOWN   0x00100000UL    /* P8.1 col (byte 2), P1.4 row */
#define KBD_SCAN_HOME   0x04000000UL    /* P8.0 col (byte 3), P1.2 row */
#define KBD_SCAN_PAUSE  0x08000000UL    /* P8.0 col (byte 3), P1.3 row */
#define KBD_SCAN_UP     0x10000000UL    /* P8.0 col (byte 3), P1.4 row */

/* Valid key mask — OR of all scan results */
#define KBD_SCAN_MASK   0x1C100010UL

/* ---- Debounce threshold ---- */
#define KBD_DEBOUNCE_COUNT  6   /* Consecutive matching scans to accept */

/* ---- Public API ---- */

/*
 * kbd_Init - Reset keyboard module state
 *
 * Clears debounce counters and previous scan state.
 * Call once at startup (P8/P1 GPIO already configured by hwinit).
 */
void kbd_Init(void);

/*
 * kbd_Scan - Perform a single matrix scan
 *
 * Scans all 4 columns, reads rows, and maps the 32-bit scan result
 * to a key code. Does NOT debounce — returns instantaneous key state.
 *
 * Uses shadow_p8_ddr from motor.h for EMI-safe port access.
 * Restores shadow_p8_ddr to original value after scan.
 *
 * @return  Key code (KEY_ENTER/DOWN/HOME/PAUSE/UP) or KEY_NONE
 *
 * Also stores the raw 32-bit scan result accessible via kbd_GetLastScan().
 */
uint8_t kbd_Scan(void);

/*
 * kbd_GetKey - Scan with 6-count debounce
 *
 * Calls kbd_Scan() and compares with previous scan result.
 * If the same non-zero key is seen for KBD_DEBOUNCE_COUNT consecutive
 * calls, returns the key code once (edge-triggered: returns KEY_NONE
 * until key is released and pressed again).
 *
 * @return  Debounced key code or KEY_NONE
 */
uint8_t kbd_GetKey(void);

/*
 * kbd_WaitKey - Blocking wait for a debounced keypress
 *
 * Polls kbd_GetKey() in a loop. Returns when a key is accepted
 * or the timeout expires.
 *
 * @param timeout_ticks  Timeout in system ticks (10ms each), 0 = wait forever
 * @return  Debounced key code or KEY_NONE on timeout
 */
uint8_t kbd_WaitKey(uint16_t timeout_ticks);

/*
 * kbd_GetLastScan - Get the raw 32-bit scan result from last kbd_Scan()
 *
 * Useful for diagnostics — shows which matrix intersection was active.
 *
 * @return  Raw 32-bit scan result (masked with KBD_SCAN_MASK)
 */
uint32_t kbd_GetLastScan(void);

#endif /* KEYBOARD_H */
