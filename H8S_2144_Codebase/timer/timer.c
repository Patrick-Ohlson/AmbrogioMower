/*
 * timer.c - System tick timer and software timer framework
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 *
 * TMRY register map (base 0xFFFFF0):
 *   TCR   = 0xFFFFF0  Timer Control Register
 *   TCSR  = 0xFFFFF1  Timer Control/Status Register
 *   TCORA = 0xFFFFF2  Time Constant Register A
 *   TCORB = 0xFFFFF3  Time Constant Register B
 *   TCNT  = 0xFFFFF4  Timer Counter
 *   TISR  = 0xFFFFF5  Timer Input Select Register
 *
 * TCR = 0x4B = 0100_1011:
 *   CMIEA=1 (compare A interrupt), CCLR=01 (clear on TCORA),
 *   CKS=011 (phi/2048)
 * TCORA = 98 -> period = 98 * 2048 / 20MHz = ~10.04ms
 *
 * Production firmware ISR flow (_CMIAY_8BitXY @ 0x0A72):
 *   bCMIAY_8BitXY_FLAG = 1
 *   fUpdateSystemCounter(1)   <- walks timer list, increments counter
 *   KMIMR &= 0xBF             <- clear keyboard IRQ mask bit
 *   bCMIAY_8BitXY_FLAG = 0
 *   ao_Scheduler()            <- dispatch AO events
 */

#include "timer.h"
#include "../iodefine.h"
#include "../misc/misc.h"

/* ==================================================================
 * Globals
 * ================================================================== */

/* System tick counter — matches _SystemCounter in production firmware */
static volatile uint32_t SystemCounter;

/* Software timer linked list head — matches g_timer_list_head */
static sw_timer_t *g_timer_list_head;

/* ISR re-entrancy flag — matches bCMIAY_8BitXY_FLAG */
static volatile uint8_t bCMIAY_8BitXY_FLAG;

/* LED heartbeat counter (toggle every ~1s = 100 ticks) */
static volatile uint8_t led_counter;
#define LED_TOGGLE_TICKS  100

/* ao_Scheduler call counter — for diagnostics/testing */
volatile uint32_t ao_scheduler_count;

/* ==================================================================
 * Internal: fUpdateSystemCounter
 *
 * Walks the software timer list, decrementing each countdown.
 * Fires timers that reach zero. Increments SystemCounter.
 *
 * Firmware equivalent: fUpdateSystemCounter (0x16B54)
 * ================================================================== */
static void fUpdateSystemCounter(uint16_t n)
{
    sw_timer_t *t = g_timer_list_head;
    sw_timer_t *next;

    while (t != (sw_timer_t *)0) {
        next = t->next;
        if (n < t->countdown) {
            /* Still counting down */
            t->countdown -= n;
        }
        else {
            /* Timer expired — fire callback */
            if (t->callback != (void *)0) {
                t->callback(t);
            }
            /* Reload or remove from list */
            if (t->repeat != 0) {
                t->countdown = t->interval;
            }
            else {
                /* One-shot: unlink from list */
                if (t->prev != (sw_timer_t *)0) {
                    t->prev->next = t->next;
                }
                else {
                    g_timer_list_head = t->next;
                }
                if (t->next != (sw_timer_t *)0) {
                    t->next->prev = t->prev;
                }
                t->countdown = 0;
            }
        }
        t = next;
    }

    SystemCounter += n;
}

/* ==================================================================
 * Internal: ao_Scheduler (stub)
 *
 * Placeholder for the Active Object event dispatcher.
 * Production firmware dispatches queued AO events here (0x1688A).
 * Will be implemented when AO framework is added.
 * ================================================================== */
static void ao_Scheduler(void)
{
    ao_scheduler_count++;
    /* TODO: AO event dispatch loop */
}

/* ==================================================================
 * Public API
 * ================================================================== */

void timer_HW_Init(void)
{
    /* Enable TMRY module clock (clear MSTPCR._TMRY) */
    MSTPCR.BIT._TMRY = 0;

    /* Ensure TMRY registers are accessible (SYSCR.HIE = 0) */
    SYSCR.BIT.HIE = 0;

    /* Stop timer during configuration */
    TMRY.TCSR.BYTE = 0x00;
    TMRY.TISR.BYTE = 0xFE;     /* External clock/reset disabled */
    TMRY.TCNT      = 0;
    TMRY.TCORA     = 98;       /* ~10ms period */

    /* Reset state */
    SystemCounter      = 0;
    g_timer_list_head  = (sw_timer_t *)0;
    bCMIAY_8BitXY_FLAG = 0;
    led_counter        = 0;
    ao_scheduler_count = 0;

    /* Start: CMIEA=1, CCLR=01 (TCORA clear), CKS=011 (phi/2048) */
    TMRY.TCR.BYTE = 0x4B;

    /* Enable global interrupts (clear I bit in CCR) */
    {
        unsigned char ccr;
        __asm__ volatile ("stc ccr,%0l" : "=r"(ccr));
        ccr &= ~0x80;  /* Clear I bit (bit 7) */
        __asm__ volatile ("ldc %0l,ccr" :: "r"(ccr));
    }
}

uint32_t GetSystemCounter(void)
{
    unsigned char ccr;
    uint32_t val;
    ENTER_CRITICAL(ccr);
    val = SystemCounter;
    EXIT_CRITICAL(ccr);
    return val;
}

void timer_Init(sw_timer_t *t, void (*callback)(sw_timer_t *))
{
    t->prev      = (sw_timer_t *)0;
    t->next      = (sw_timer_t *)0;
    t->interval  = 0;
    t->callback  = callback;
    t->countdown = 0;
    t->repeat    = 0;
}

void timer_Set(sw_timer_t *t, uint16_t ticks, uint8_t repeat)
{
    unsigned char ccr;

    ENTER_CRITICAL(ccr);   /* ISR walks this list — protect it */

    if (ticks == 0) {
        /* Disarm: remove from list if armed */
        if (t->countdown != 0) {
            if (t->prev != (sw_timer_t *)0) {
                t->prev->next = t->next;
            }
            else {
                g_timer_list_head = t->next;
            }
            if (t->next != (sw_timer_t *)0) {
                t->next->prev = t->prev;
            }
            t->countdown = 0;
        }
    }
    else {
        /* Arm: insert at list head if not already armed */
        if (t->countdown == 0) {
            t->prev = (sw_timer_t *)0;
            t->next = g_timer_list_head;
            if (g_timer_list_head != (sw_timer_t *)0) {
                g_timer_list_head->prev = t;
            }
            g_timer_list_head = t;
        }
        t->interval  = ticks;
        t->countdown = ticks;
        t->repeat    = repeat ? ticks : 0;
    }

    EXIT_CRITICAL(ccr);
}

/* ==================================================================
 * ISR: TMRY Compare Match A — vector 72
 *
 * Matches production firmware _CMIAY_8BitXY (0x0A72):
 *   1. Set re-entrancy flag
 *   2. fUpdateSystemCounter(1) — tick software timers + counter
 *   3. Clear re-entrancy flag
 *   4. ao_Scheduler() — dispatch AO events (stub)
 *   5. LED heartbeat — toggle every ~1 second
 * ================================================================== */
void INT_CMIAY_8BitXY(void) __attribute__((interrupt_handler));
void INT_CMIAY_8BitXY(void)
{
    bCMIAY_8BitXY_FLAG = 1;

    fUpdateSystemCounter(1);

    bCMIAY_8BitXY_FLAG = 0;

    ao_Scheduler();

    /* Clear compare match A flag */
    while (TMRY.TCSR.BIT.CMFA != 0)
        TMRY.TCSR.BIT.CMFA = 0;

    /* LED heartbeat: toggle every ~1 second */
    led_counter++;
    if (led_counter >= LED_TOGGLE_TICKS) {
        led_counter = 0;
        misc_SetLed(!(P9.DR.BYTE & 0x20));
    }
}
