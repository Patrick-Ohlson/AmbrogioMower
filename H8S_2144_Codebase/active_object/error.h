/*============================================================================
 * error.h — Lightweight error reporting for bare-metal H8S
 *
 * Matches the L200 firmware convention:
 *   Address 0xFFFF00 holds a null-terminated ASCII error string.
 *   If byte[0] != 0, CheckWatchdog displays "HALT:" on LCD line 1
 *   and scrolls the error string across LCD line 2.
 *
 * For the test framework we place the error buffer in normal RAM
 * (linker will assign). On the real board, the linker script can
 * pin g_error_str to 0xFFFF00 if desired.
 *
 * Trace32:  Data.String D:V.ADDRESS(g_error_str)
 *           Var.View g_error_str
 *============================================================================*/

#ifndef ERROR_H
#define ERROR_H

#include <stdint.h>

/*--- Error string buffer (firmware: unk_FFFF00) ---
 *
 * 64 bytes matches the firmware's scrolling display which reads
 * strlen() of the whole buffer.  CheckWatchdog scrolls in 16-char
 * chunks so any length works; 64 gives room for two full LCD
 * lines of context.
 *
 * byte[0] == 0  → no error
 * byte[0] != 0  → error, string is the message
 */
#define ERROR_STR_SIZE  64

extern volatile char g_error_str[ERROR_STR_SIZE];

/*--- API ---*/

/**
 * error_set — Record an error and halt (matches firmware HALT convention)
 * @msg: Short ASCII message (will be truncated to ERROR_STR_SIZE-1)
 *
 * Copies msg into g_error_str then enters infinite loop.
 * On real board: CheckWatchdog will detect the non-zero byte
 * and display "HALT: <msg>" on the LCD.
 * In Trace32: set breakpoint on error_set, or inspect g_error_str.
 */
void error_set(const char *msg);

/**
 * error_clear — Clear the error flag (called at boot)
 *
 * Firmware equivalent: RealBoot writes 0 to unk_FFFF00.
 */
void error_clear(void);

/**
 * error_active — Check if an error is latched
 * Returns: 1 if error string is non-empty, 0 otherwise
 */
static inline int error_active(void)
{
    return (g_error_str[0] != '\0');
}

/*============================================================================
 * ASSERT macro — calls error_set with a message then traps
 *
 * Include this header instead of defining ASSERT locally.
 * In NDEBUG builds the condition is still not evaluated (zero cost).
 *============================================================================*/
#ifndef NDEBUG
  #define ASSERT(cond, msg) \
      do { if (!(cond)) { error_set(msg); } } while(0)
#else
  #define ASSERT(cond, msg) ((void)0)
#endif

#endif /* ERROR_H */
