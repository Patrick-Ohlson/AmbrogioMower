/*============================================================================
 * error.c — Lightweight error reporting for bare-metal H8S
 *
 * Firmware convention (unk_FFFF00):
 *   CheckWatchdog checks strlen(0xFFFF00). If non-zero it displays
 *   "HALT:" on LCD line 1 and scrolls the error text on line 2.
 *   RealBoot clears byte[0] to 0 at power-on.
 *
 * This module replicates that pattern in portable C.
 *
 * TestFirmware mode: By default, error_set() prints the error via UART
 * and returns (allowing tests to detect and report errors without halting).
 * Define AO_HALT_ON_ERROR to restore the original halt behaviour.
 *============================================================================*/

#include "error.h"
#include "../sci.h"              /* SendString / PutChar for UART output */

/*--- The error buffer itself ---*/
volatile char g_error_str[ERROR_STR_SIZE];

/*--------------------------------------------------------------------------
 * error_clear — Zero the error flag byte
 *
 * Firmware equivalent: RealBoot / sub_1216C writes 0 to unk_FFFF00.
 *--------------------------------------------------------------------------*/
void error_clear(void)
{
    g_error_str[0] = '\0';
}

/*--------------------------------------------------------------------------
 * error_set — Copy message into error buffer
 *
 * Simple byte-copy, no library dependency (avoids pulling in strcpy/
 * strncpy and their newlib baggage).
 *
 * In TestFirmware mode (default): prints "ASSERT: <msg>\r\n" via UART
 * and returns so the test can detect the error and continue.
 *
 * With AO_HALT_ON_ERROR defined: halts in an infinite loop (matches
 * the firmware CheckWatchdog convention).
 *--------------------------------------------------------------------------*/
void error_set(const char *msg)
{
    uint16_t i;

    if (msg == (const char *)0) {
        g_error_str[0] = '?';
        g_error_str[1] = '\0';
    } else {
        for (i = 0; i < (ERROR_STR_SIZE - 1) && msg[i] != '\0'; i++) {
            g_error_str[i] = msg[i];
        }
        g_error_str[i] = '\0';
    }

    /* Print to UART so the test runner can see it */
    SendString((unsigned char *)"ASSERT: ");
    SendString((unsigned char *)g_error_str);
    SendString((unsigned char *)"\r\n");

#ifdef AO_HALT_ON_ERROR
    /* Halt — firmware enters infinite CheckWatchdog display loop */
    while (1) {}
#endif
}
