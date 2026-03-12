/*
 * test_timer.c — System timer / TMRY diagnostic test
 *
 * Verifies:
 *   1. TMRY hardware registers after timer_HW_Init()
 *   2. ISR is firing (SystemCounter increments)
 *   3. ao_Scheduler stub is called (ao_scheduler_count increments)
 *   4. Software timer callback fires
 *   5. LED heartbeat visible (toggles every ~1s)
 *   6. Live counter display (~5 seconds)
 */

#include "test_timer.h"
#include "../iodefine.h"
#include "../micros.h"
#include "../sci.h"
#include "../utils/utils.h"
#include "../timer/timer.h"
#include "../misc/misc.h"

/* Software timer test callback counter */
static volatile uint16_t sw_timer_fired;

static void test_sw_callback(sw_timer_t *t)
{
    sw_timer_fired++;
}

void Timer_Test(void)
{
    uint16_t pass = 0, fail = 0;
    uint32_t t0, t1, sched0, sched1;
    uint32_t i;
    sw_timer_t test_tmr;

    SendString((unsigned char *)"\r\n");
    SendString((unsigned char *)"========================================\r\n");
    SendString((unsigned char *)"  System Timer (TMRY) Diagnostic\r\n");
    SendString((unsigned char *)"========================================\r\n");

    /* ---- 1. TMRY register check ---- */
    SendString((unsigned char *)"--- 1. TMRY registers ---\r\n");

    SendString((unsigned char *)"  MSTPCR._TMRY = ");
    print_hex8(MSTPCR.BIT._TMRY);
    if (MSTPCR.BIT._TMRY == 0) {
        SendString((unsigned char *)" [PASS] clock enabled\r\n"); pass++;
    } else {
        SendString((unsigned char *)" [FAIL] clock stopped\r\n"); fail++;
    }

    SendString((unsigned char *)"  TMRY.TCR  = 0x");
    print_hex8(TMRY.TCR.BYTE);
    if (TMRY.TCR.BYTE == 0x4B) {
        SendString((unsigned char *)" [PASS] CMIEA=1 CCLR=01 CKS=011\r\n"); pass++;
    } else {
        SendString((unsigned char *)" [FAIL] expect 0x4B\r\n"); fail++;
    }

    SendString((unsigned char *)"  TMRY.TCORA= ");
    print_hex8(TMRY.TCORA);
    if (TMRY.TCORA == 98) {
        SendString((unsigned char *)" [PASS] period=98\r\n"); pass++;
    } else {
        SendString((unsigned char *)" [FAIL] expect 98\r\n"); fail++;
    }

    /* Check global interrupts enabled (I bit = 0 in CCR) */
    {
        unsigned char ccr_val;
        __asm__ volatile ("stc ccr,%0l" : "=r"(ccr_val));
        SendString((unsigned char *)"  CCR = 0x");
        print_hex8(ccr_val);
        if ((ccr_val & 0x80) == 0) {
            SendString((unsigned char *)" [PASS] I=0 (IRQ enabled)\r\n"); pass++;
        } else {
            SendString((unsigned char *)" [FAIL] I=1 (IRQ masked!)\r\n"); fail++;
        }
    }

    /* ---- 2. ISR firing check ---- */
    SendString((unsigned char *)"--- 2. ISR firing (wait ~500ms) ---\r\n");

    t0 = GetSystemCounter();
    sched0 = ao_scheduler_count;

    SendString((unsigned char *)"  Counter before: ");
    print_hex32(t0);
    SendString((unsigned char *)"\r\n");

    /* Busy-wait ~500ms using a loop (not timer-based, since we're testing the timer) */
    for (i = 0; i < 500000UL; i++)
        ;

    t1 = GetSystemCounter();
    sched1 = ao_scheduler_count;

    SendString((unsigned char *)"  Counter after:  ");
    print_hex32(t1);
    SendString((unsigned char *)"\r\n");

    SendString((unsigned char *)"  Delta ticks:    ");
    print_hex32(t1 - t0);
    if (t1 > t0) {
        SendString((unsigned char *)" [PASS] ISR is firing\r\n"); pass++;
    } else {
        SendString((unsigned char *)" [FAIL] counter did not increment!\r\n"); fail++;
    }

    /* ---- 3. ao_Scheduler hit check ---- */
    SendString((unsigned char *)"--- 3. ao_Scheduler stub ---\r\n");

    SendString((unsigned char *)"  Calls before: ");
    print_hex32(sched0);
    SendString((unsigned char *)"  after: ");
    print_hex32(sched1);
    if (sched1 > sched0) {
        SendString((unsigned char *)" [PASS]\r\n"); pass++;
    } else {
        SendString((unsigned char *)" [FAIL] scheduler not called\r\n"); fail++;
    }

    /* ---- 4. Software timer callback ---- */
    SendString((unsigned char *)"--- 4. Software timer (one-shot 5 ticks) ---\r\n");

    sw_timer_fired = 0;
    timer_Init(&test_tmr, test_sw_callback);
    timer_Set(&test_tmr, 5, 0);  /* one-shot, 5 ticks = ~50ms */

    SendString((unsigned char *)"  Armed, waiting ~200ms...\r\n");
    t0 = GetSystemCounter();
    while ((GetSystemCounter() - t0) < 20)   /* wait ~200ms */
        ;

    SendString((unsigned char *)"  Fired count: ");
    print_hex16(sw_timer_fired);
    if (sw_timer_fired == 1) {
        SendString((unsigned char *)" [PASS] callback fired once\r\n"); pass++;
    } else if (sw_timer_fired > 1) {
        SendString((unsigned char *)" [FAIL] fired multiple times\r\n"); fail++;
    } else {
        SendString((unsigned char *)" [FAIL] callback never fired\r\n"); fail++;
    }

    /* ---- 5. Software timer repeating ---- */
    SendString((unsigned char *)"--- 5. Software timer (repeating 10 ticks) ---\r\n");

    sw_timer_fired = 0;
    timer_Init(&test_tmr, test_sw_callback);
    timer_Set(&test_tmr, 10, 1);  /* repeat, 10 ticks = ~100ms */

    t0 = GetSystemCounter();
    while ((GetSystemCounter() - t0) < 50)   /* wait ~500ms */
        ;
    timer_Set(&test_tmr, 0, 0);  /* disarm */

    SendString((unsigned char *)"  Fired count in ~500ms: ");
    print_hex16(sw_timer_fired);
    if (sw_timer_fired >= 4 && sw_timer_fired <= 6) {
        SendString((unsigned char *)" [PASS] ~5 fires\r\n"); pass++;
    } else {
        SendString((unsigned char *)" [FAIL] expected 4-6\r\n"); fail++;
    }

    /* ---- 6. Live counter + LED (5 seconds) ---- */
    SendString((unsigned char *)"--- 6. Live display (~5s, watch LED blink) ---\r\n");

    t0 = GetSystemCounter();
    while ((GetSystemCounter() - t0) < 500) {  /* 500 ticks = ~5s */
        /* Print counter every ~1 second */
        t1 = GetSystemCounter();
        SendString((unsigned char *)"  tick=");
        print_hex32(t1);
        SendString((unsigned char *)" sched=");
        print_hex32(ao_scheduler_count);
        SendString((unsigned char *)" LED=");
        print_hex8((P9.DR.BYTE >> 5) & 1);
        SendString((unsigned char *)"\r\n");

        /* Wait ~1 second */
        while ((GetSystemCounter() - t1) < 100)
            ;
    }

    /* ---- Summary ---- */
    SendString((unsigned char *)"========================================\r\n");
    SendString((unsigned char *)"  Results: ");
    print_hex16(pass);
    SendString((unsigned char *)" passed, ");
    print_hex16(fail);
    SendString((unsigned char *)" failed\r\n");
    if (fail == 0) {
        SendString((unsigned char *)"  ALL TESTS PASSED\r\n");
    } else {
        SendString((unsigned char *)"  *** FAILURES DETECTED ***\r\n");
    }
}
