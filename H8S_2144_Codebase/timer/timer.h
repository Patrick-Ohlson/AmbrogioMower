/*
 * timer.h - System tick timer and software timer framework
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra RE of original firmware.
 *
 * Hardware: 8-bit Timer Y (TMRY) at 0xFFFFF0
 *   Compare Match A interrupt -> vector 72 (CMIAY_8BitXY)
 *   Prescaler: phi/2048, TCORA=98  -> ~10.04ms period
 *
 * Architecture (matches production firmware):
 *   ISR _CMIAY_8BitXY (0x0A72):
 *     1. fUpdateSystemCounter(1) — increment SystemCounter,
 *        walk software timer list, fire expired timers
 *     2. ao_Scheduler() — dispatch AO events (stub for now)
 *
 *   Software timers are a doubly-linked list of sw_timer_t objects.
 *   Each has a countdown, callback, and optional auto-reload.
 *
 * Firmware traceability:
 *   _CMIAY_8BitXY       @ 0x0A72
 *   fUpdateSystemCounter @ 0x16B54
 *   GetSystemCounter     @ 0x16BA4
 *   timer_Init           @ 0x16AB0  (software timer init)
 *   timer_Set            @ 0x16AC0  (arm/disarm software timer)
 *   timer_Fire           @ 0x16A5E  (fire callback, manage list)
 */

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

/* ---- Tick period ---- */
#define TIMER_TICK_MS   10      /* ~10ms per tick */

/* ---- Critical section macros ----
 * Matches production firmware pattern:
 *   stc.w  ccr, @(0x2,er7)   ; save CCR to stack
 *   orc    #0x80, ccr         ; set I bit -> disable IRQs
 *   ...hardware access...
 *   ldc.w  @(0x2,er7), ccr   ; restore CCR (re-enables IRQs
 *                             ;   only if they were enabled before)
 *
 * Usage:
 *   unsigned char _ccr;
 *   ENTER_CRITICAL(_ccr);
 *   ...access shared data...
 *   EXIT_CRITICAL(_ccr);
 */
#define ENTER_CRITICAL(saved_ccr) do { \
    __asm__ volatile ("stc ccr,%0l" : "=r"(saved_ccr)); \
    __asm__ volatile ("orc #0x80,ccr"); \
} while (0)

#define EXIT_CRITICAL(saved_ccr) do { \
    __asm__ volatile ("ldc %0l,ccr" :: "r"(saved_ccr)); \
} while (0)

/* ---- Software timer structure ----
 * Matches production firmware layout at offsets:
 *   +0x00: prev      (linked list)
 *   +0x04: next      (linked list)
 *   +0x08: interval  (reload value set by timer_Set)
 *   +0x0C: callback  (function pointer set by timer_Init)
 *   +0x10: countdown (current ticks remaining)
 *   +0x14: repeat    (0 = one-shot, nonzero = auto-reload)
 */
typedef struct timer_s {
    struct timer_s *prev;
    struct timer_s *next;
    uint16_t        interval;       /* ticks between fires */
    void          (*callback)(struct timer_s *);
    uint16_t        countdown;      /* ticks remaining */
    uint16_t        repeat;         /* auto-reload value (0=one-shot) */
} sw_timer_t;

/* ---- Hardware init ---- */

/*
 * timer_HW_Init - Initialize TMRY hardware and enable tick interrupt
 *
 * Enables TMRY clock, configures ~10ms compare-match A interrupt.
 * Resets SystemCounter to 0. Call once at startup.
 *
 * Firmware equivalent: TMRY setup in SetupHW_Q (0x121A4)
 */
void timer_HW_Init(void);

/* ---- System counter ---- */

/*
 * GetSystemCounter - Read the global system tick counter
 *
 * Increments by 1 every ~10ms. Wraps at 0xFFFFFFFF.
 *
 * Firmware equivalent: GetSystemCounter (0x16BA4)
 */
uint32_t GetSystemCounter(void);

/* ---- Software timer API ---- */

/*
 * timer_Init - Initialize a software timer object
 *
 * Sets callback, clears countdown. Does NOT arm the timer.
 * Call timer_Set() to arm.
 *
 * Firmware equivalent: timer_Init (0x16AB0)
 */
void timer_Init(sw_timer_t *t, void (*callback)(sw_timer_t *));

/*
 * timer_Set - Arm or disarm a software timer
 *
 * @param t         Timer object
 * @param ticks     Countdown in ticks (0 = disarm)
 * @param repeat    Non-zero = auto-reload after fire
 *
 * Firmware equivalent: timer_Set (0x16AC0)
 */
void timer_Set(sw_timer_t *t, uint16_t ticks, uint8_t repeat);

/*
 * INT_CMIAY_8BitXY - TMRY Compare Match A ISR (vector 72)
 *
 * Calls fUpdateSystemCounter(1) then ao_Scheduler() (stub).
 *
 * Firmware equivalent: _CMIAY_8BitXY (0x0A72)
 */
void INT_CMIAY_8BitXY(void);

/* ---- Diagnostics ---- */

/* ao_Scheduler call count — incremented by ISR each tick.
 * Exposed for test verification that ISR + scheduler are running. */
extern volatile uint32_t ao_scheduler_count;

#endif /* TIMER_H */
