/*============================================================================
 * config.h — Build-time configuration for TestFirmware
 *
 * Toggle these defines to control startup and hardware behaviour.
 * Included by main.c, lcd.c, sci.c, and demo_config.h.
 *============================================================================*/

#ifndef CONFIG_H
#define CONFIG_H

/* ---- Autostart Configuration ---- */

/* AUTOSTART_ENABLE:
 *   1 = show countdown on LCD, then auto-launch AUTOSTART_MODULE
 *   0 = go straight to menu (no countdown)
 */
#define AUTOSTART_ENABLE    1

/* AUTOSTART_MODULE:
 *   Menu key of the module to auto-launch when countdown expires.
 *   Examples: 'B' = Demo Mode, 'I' = AO Framework, '6' = Memory Test
 */
#define AUTOSTART_MODULE    'K'

/* AUTOSTART_SECONDS:
 *   Countdown duration before auto-launch (seconds).
 *   Only used when AUTOSTART_ENABLE = 1.
 */
#define AUTOSTART_SECONDS   0

/* ---- Simulation Mode ---- */

/* SIMULATION:
 *   1 = disable LCD and UART hardware access for Trace32 Simulator.
 *       lcd_* functions become no-ops, UART init/TX/RX are skipped.
 *       AUTOSTART_MODULE launches immediately (no menu, no countdown).
 *   0 = normal hardware operation
 */
#define SIMULATION          0

#endif /* CONFIG_H */
