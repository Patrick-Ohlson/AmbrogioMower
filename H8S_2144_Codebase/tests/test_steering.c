/* test_steering.c — Steering driver test
 *
 * Tests the high-level steering module:  state machine, GPIO pin
 * verification, timer-based timed movements, convenience commands,
 * and live hardware movement.
 *
 * IMPORTANT: Sections 1-9 use a TEST VTABLE that replaces power_on
 * and set_speed with no-ops.  Direction functions (LeftForward, etc.)
 * are the real motor.h functions so GPIO pins can be verified, but
 * the L6203 H-bridge power is never enabled (P4.0 stays low) and
 * no PWMX duty is written.  This prevents the motors from physically
 * spinning and avoids the current draw that disrupts SCI1/UART.
 *
 * Section 10 is a live movement test that uses the REAL motor backend;
 * it is gated behind a UART 'Y' confirm.
 *
 * Depends on: motor driver (motor_Init), timer driver (timer_HW_Init),
 *             UART (SendString/PutChar/GetChar).
 */
#include "test_steering.h"
#include "../iodefine.h"
#include "../sci.h"
#include "../motor/motor.h"
#include "../steering/steering.h"
#include "../utils/utils.h"
#include "../timer/timer.h"

/* ====================================================================
 * Test motor vtable — GPIO-only, fully isolated from motor hardware
 *
 * Direction functions set ONLY GPIO pins (P2/P9) for verification.
 * They do NOT touch:
 *   - FRT.TIER  (encoder capture interrupts — may crash if no ISR)
 *   - STCR      (PWMX bus enable — interferes with SCI1)
 *   - PWMX DADRA/DADRB (speed registers)
 *   - P4.0      (H-bridge power)
 *
 * This prevents ALL motor-related side effects during static tests.
 * ==================================================================== */

static void nop_void(void) { /* intentionally empty */ }
static void nop_speed(uint8_t s) { (void)s; }

/* GPIO-only direction: set P2 (IN1) and P9 (IN2) without FRT/STCR.
 * Uses shadow registers so motor.c shadow stays in sync.  */
static void test_left_forward(void)  { shadow_p2_dr |=  MOTOR_P2_LEFT_IN1;
                                        P2.DR.BYTE = shadow_p2_dr;
                                        shadow_p9_dr &= (uint8_t)~MOTOR_P9_LEFT_IN2;
                                        P9.DR.BYTE = shadow_p9_dr; }
static void test_left_reverse(void)  { shadow_p2_dr |=  MOTOR_P2_LEFT_IN1;
                                        P2.DR.BYTE = shadow_p2_dr;
                                        shadow_p9_dr |=  MOTOR_P9_LEFT_IN2;
                                        P9.DR.BYTE = shadow_p9_dr; }
static void test_left_disable(void)  { shadow_p2_dr &= (uint8_t)~MOTOR_P2_LEFT_IN1;
                                        P2.DR.BYTE = shadow_p2_dr;
                                        shadow_p9_dr &= (uint8_t)~MOTOR_P9_LEFT_IN2;
                                        P9.DR.BYTE = shadow_p9_dr; }
static void test_right_forward(void) { shadow_p2_dr |=  MOTOR_P2_RIGHT_IN1;
                                        P2.DR.BYTE = shadow_p2_dr;
                                        shadow_p9_dr &= (uint8_t)~MOTOR_P9_RIGHT_IN2;
                                        P9.DR.BYTE = shadow_p9_dr; }
static void test_right_reverse(void) { shadow_p2_dr |=  MOTOR_P2_RIGHT_IN1;
                                        P2.DR.BYTE = shadow_p2_dr;
                                        shadow_p9_dr |=  MOTOR_P9_RIGHT_IN2;
                                        P9.DR.BYTE = shadow_p9_dr; }
static void test_right_disable(void) { shadow_p2_dr &= (uint8_t)~MOTOR_P2_RIGHT_IN1;
                                        P2.DR.BYTE = shadow_p2_dr;
                                        shadow_p9_dr &= (uint8_t)~MOTOR_P9_RIGHT_IN2;
                                        P9.DR.BYTE = shadow_p9_dr; }
static void test_stop_all(void)      { test_left_disable(); test_right_disable(); }

static const steer_motor_ops_t test_motor_ops = {
    nop_void,               /* power_on       — H-bridge stays off      */
    nop_void,               /* power_off      — no-op                   */
    test_left_forward,      /* left_forward   — GPIO only: P2.6=1 P9.0=0*/
    test_left_reverse,      /* left_reverse   — GPIO only: P2.6=1 P9.0=1*/
    test_left_disable,      /* left_disable   — GPIO only: clears pins  */
    nop_speed,              /* left_set_speed — no PWMX write           */
    test_right_forward,     /* right_forward  — GPIO only: P2.5=1 P9.1=0*/
    test_right_reverse,     /* right_reverse  — GPIO only: P2.5=1 P9.1=1*/
    test_right_disable,     /* right_disable  — GPIO only: clears pins  */
    nop_speed,              /* right_set_speed— no PWMX write           */
    test_stop_all           /* stop_all       — GPIO cleanup only       */
};

/* ---- NOP-based motor delay with periodic SCI + bus recovery ----
 *
 * Uses NOP busy-loop (same as delay_ms) — NOT GetSystemCounter().
 * Motor EMI can disrupt the TMRY ISR, causing GetSystemCounter to
 * stop incrementing and the delay to loop forever.  NOP counting
 * has zero peripheral dependency and cannot hang from EMI.
 *
 * Periodically:
 *   - Clears SCI1 error flags (ORER/FER/PER)
 *   - Restores STCR from shadow via motor_RecoverBus() in case
 *     EMI corrupted it (prevents FLSHE flash remap crash)
 */
static void motor_safe_delay(uint32_t ms)
{
    volatile uint32_t i;
    uint32_t loops = 925UL * ms;

    for (i = 0; i < loops; i++) {
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        /* Every 1024 iterations (~1ms), recover SCI + STCR */
        if ((i & 0x3FF) == 0) {
            if (SCI1.SSR.BYTE & (SSR_ORER | SSR_FER | SSR_PER)) {
                SCI1.SSR.BYTE &= (unsigned char)~(SSR_ORER | SSR_FER | SSR_PER);
            }
            motor_RecoverBus();
        }
    }
}

/* ---- Full SCI1 + bus recovery after motor EMI ----
 *
 * EMI can corrupt:
 *   1. SSR (error flags stuck)
 *   2. SCR (TE/RE cleared → SCI1 dead)
 *   3. STCR (FLSHE set → flash remap → crash, or ICKS0 cleared)
 *
 * This function recovers all three:
 *   - SSR: clears ORER/FER/PER error flags
 *   - SCR: force-restores TE|RE (TIE left for PutChar to re-enable)
 *   - STCR: restored from motor.c shadow via motor_RecoverBus()
 */
static void motor_sci_recover(void)
{
    SCI1.SSR.BYTE &= (unsigned char)~(SSR_ORER | SSR_FER | SSR_PER);
    SCI1.SCR.BYTE = SCR_TE | SCR_RE;
    motor_RecoverBus();   /* restore STCR from shadow (IICE + ICKS0) */
}

/* ---- Callback test helper ---- */
static volatile uint8_t  g_cb_fired;
static volatile uint8_t  g_cb_dir;
static volatile steer_state_t g_cb_state;

static void test_done_callback(steer_state_t state, uint8_t dir)
{
    g_cb_fired++;
    g_cb_state = state;
    g_cb_dir   = dir;
}

void Steering_Test(void)
{
    uint8_t pass_count = 0;
    uint8_t fail_count = 0;
    uint8_t reg_val;
    steer_status_t sts;
    uint16_t est;
    char ch;

    SendString((unsigned char *)"\r\n========================================\r\n");
    SendString((unsigned char *)"  Steering Driver Test\r\n");
    SendString((unsigned char *)"========================================\r\n");

    /* Make sure motor hardware is initialised */
    motor_Init();


    /* --- Section 1: steer_Init() and idle state --- */
    SendString((unsigned char *)"\r\n--- 1. steer_Init() defaults ---\r\n");

    steer_Init(&test_motor_ops, 0, 0);   /* test vtable, default cfg, no cb */

    if (steer_GetState() == STEER_STATE_IDLE) {
        SendString((unsigned char *)"  state=IDLE      [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  state!=IDLE     [FAIL]\r\n");
        fail_count++;
    }

    if (steer_IsMoving() == 0) {
        SendString((unsigned char *)"  IsMoving=0      [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  IsMoving!=0     [FAIL]\r\n");
        fail_count++;
    }

    steer_GetStatus(&sts);
    if (sts.direction == STEER_DIR_STOP && sts.timed == 0) {
        SendString((unsigned char *)"  dir=S timed=0   [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  status mismatch [FAIL]\r\n");
        fail_count++;
    }


    /* --- Section 2: steer_Forward() GPIO pins --- */
    SendString((unsigned char *)"\r\n--- 2. steer_Forward() ---\r\n");

    steer_Forward(0x40, 0);     /* speed=0x40, continuous (no timer) */

    /* Both wheels forward: P2.6=1 P9.0=0 (left), P2.5=1 P9.1=0 (right) */
    reg_val = P2.DR.BYTE;
    SendString((unsigned char *)"  P2.DR=0x");
    print_hex8(reg_val);
    if ((reg_val & MOTOR_P2_LEFT_IN1) && (reg_val & MOTOR_P2_RIGHT_IN1)) {
        SendString((unsigned char *)" [PASS] both IN1 high\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)" [FAIL] IN1 pins\r\n");
        fail_count++;
    }

    reg_val = P9.DR.BYTE;
    SendString((unsigned char *)"  P9.DR=0x");
    print_hex8(reg_val);
    if ((reg_val & (MOTOR_P9_LEFT_IN2 | MOTOR_P9_RIGHT_IN2)) == 0x00) {
        SendString((unsigned char *)" [PASS] both IN2 low\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)" [FAIL] IN2 pins\r\n");
        fail_count++;
    }

    if (steer_GetState() == STEER_STATE_FORWARD) {
        SendString((unsigned char *)"  state=FORWARD   [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  state!=FORWARD  [FAIL]\r\n");
        fail_count++;
    }

    steer_Stop();


    /* --- Section 3: steer_Reverse() GPIO pins --- */
    SendString((unsigned char *)"\r\n--- 3. steer_Reverse() ---\r\n");

    steer_Reverse(0x40, 0);

    /* Both wheels reverse: P2.6=1 P9.0=1 (left), P2.5=1 P9.1=1 (right) */
    reg_val = P2.DR.BYTE;
    if ((reg_val & MOTOR_P2_LEFT_IN1) && (reg_val & MOTOR_P2_RIGHT_IN1)) {
        SendString((unsigned char *)"  IN1 pins high   [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  IN1 pins wrong  [FAIL]\r\n");
        fail_count++;
    }

    reg_val = P9.DR.BYTE;
    if ((reg_val & MOTOR_P9_LEFT_IN2) && (reg_val & MOTOR_P9_RIGHT_IN2)) {
        SendString((unsigned char *)"  IN2 pins high   [PASS] (reverse)\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  IN2 pins wrong  [FAIL]\r\n");
        fail_count++;
    }

    if (steer_GetState() == STEER_STATE_REVERSE) {
        SendString((unsigned char *)"  state=REVERSE   [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  state!=REVERSE  [FAIL]\r\n");
        fail_count++;
    }

    steer_Stop();


    /* --- Section 4: steer_TurnLeft() GPIO pins --- */
    SendString((unsigned char *)"\r\n--- 4. steer_TurnLeft() ---\r\n");

    steer_TurnLeft(0x40, 0);

    /* Turn left: left=REV (P2.6=1,P9.0=1), right=FWD (P2.5=1,P9.1=0) */
    if ((P2.DR.BYTE & MOTOR_P2_LEFT_IN1) &&
         (P9.DR.BYTE & MOTOR_P9_LEFT_IN2)) {
        SendString((unsigned char *)"  L wheel REV     [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  L wheel wrong   [FAIL]\r\n");
        fail_count++;
    }

    if ((P2.DR.BYTE & MOTOR_P2_RIGHT_IN1) &&
        !(P9.DR.BYTE & MOTOR_P9_RIGHT_IN2)) {
        SendString((unsigned char *)"  R wheel FWD     [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  R wheel wrong   [FAIL]\r\n");
        fail_count++;
    }

    if (steer_GetState() == STEER_STATE_TURN_LEFT) {
        SendString((unsigned char *)"  state=TURN_LEFT [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  state wrong     [FAIL]\r\n");
        fail_count++;
    }

    steer_Stop();


    /* --- Section 5: steer_TurnRight() GPIO pins --- */
    SendString((unsigned char *)"\r\n--- 5. steer_TurnRight() ---\r\n");

    steer_TurnRight(0x40, 0);

    /* Turn right: left=FWD (P2.6=1,P9.0=0), right=REV (P2.5=1,P9.1=1) */
    if ((P2.DR.BYTE & MOTOR_P2_LEFT_IN1) &&
        !(P9.DR.BYTE & MOTOR_P9_LEFT_IN2)) {
        SendString((unsigned char *)"  L wheel FWD     [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  L wheel wrong   [FAIL]\r\n");
        fail_count++;
    }

    if ((P2.DR.BYTE & MOTOR_P2_RIGHT_IN1) &&
         (P9.DR.BYTE & MOTOR_P9_RIGHT_IN2)) {
        SendString((unsigned char *)"  R wheel REV     [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  R wheel wrong   [FAIL]\r\n");
        fail_count++;
    }

    if (steer_GetState() == STEER_STATE_TURN_RIGHT) {
        SendString((unsigned char *)"  state=TURN_RIGHT[PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  state wrong     [FAIL]\r\n");
        fail_count++;
    }

    steer_Stop();


    /* --- Section 6: steer_Stop() verification --- */
    SendString((unsigned char *)"\r\n--- 6. steer_Stop() cleanup ---\r\n");

    steer_Forward(0x40, 0);
    steer_Stop();

    /* All direction pins should be clear */
    if (!(P2.DR.BYTE & MOTOR_P2_LEFT_IN1) &&
        !(P2.DR.BYTE & MOTOR_P2_RIGHT_IN1)) {
        SendString((unsigned char *)"  IN1 clear       [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  IN1 not clear   [FAIL]\r\n");
        fail_count++;
    }

    if (!(P9.DR.BYTE & MOTOR_P9_LEFT_IN2) &&
        !(P9.DR.BYTE & MOTOR_P9_RIGHT_IN2)) {
        SendString((unsigned char *)"  IN2 clear       [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  IN2 not clear   [FAIL]\r\n");
        fail_count++;
    }

    if (steer_GetState() == STEER_STATE_IDLE && steer_IsMoving() == 0) {
        SendString((unsigned char *)"  state=IDLE      [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  state wrong     [FAIL]\r\n");
        fail_count++;
    }


    /* --- Section 7: Command override (fwd -> turn) --- */
    SendString((unsigned char *)"\r\n--- 7. Command override ---\r\n");

    steer_Forward(0x40, 0);
    /* Immediately override with a left turn */
    steer_TurnLeft(0x40, 0);

    if (steer_GetState() == STEER_STATE_TURN_LEFT) {
        SendString((unsigned char *)"  Fwd->TurnL ok   [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  Override failed  [FAIL]\r\n");
        fail_count++;
    }

    /* Verify left wheel is reverse (not still forward) */
    if ((P2.DR.BYTE & MOTOR_P2_LEFT_IN1) &&
         (P9.DR.BYTE & MOTOR_P9_LEFT_IN2)) {
        SendString((unsigned char *)"  L wheel=REV ok  [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  L wheel wrong   [FAIL]\r\n");
        fail_count++;
    }

    steer_Stop();


    /* --- Section 8: EstimateTurnTicks math --- */
    SendString((unsigned char *)"\r\n--- 8. EstimateTurnTicks() ---\r\n");

    /* With defaults: ticks_90deg = 150 */
    est = steer_EstimateTurnTicks(90);
    SendString((unsigned char *)"  90 deg  = ");
    print_hex16(est);
    if (est == 150) {
        SendString((unsigned char *)" [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)" [FAIL expect 150]\r\n");
        fail_count++;
    }

    est = steer_EstimateTurnTicks(45);
    SendString((unsigned char *)"  45 deg  = ");
    print_hex16(est);
    if (est == 75) {
        SendString((unsigned char *)" [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)" [FAIL expect 75]\r\n");
        fail_count++;
    }

    est = steer_EstimateTurnTicks(180);
    SendString((unsigned char *)"  180 deg = ");
    print_hex16(est);
    if (est == 300) {
        SendString((unsigned char *)" [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)" [FAIL expect 300]\r\n");
        fail_count++;
    }

    est = steer_EstimateTurnTicks(0);
    SendString((unsigned char *)"  0 deg   = ");
    print_hex16(est);
    if (est == 0) {
        SendString((unsigned char *)" [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)" [FAIL expect 0]\r\n");
        fail_count++;
    }


    /* --- Section 9: Timed movement with callback --- */
    SendString((unsigned char *)"\r\n--- 9. Timed movement + callback ---\r\n");
    SendString((unsigned char *)"  Requires timer_HW_Init() (TMRY running).\r\n");

    /* Re-init with test vtable + our test callback */
    g_cb_fired = 0;
    g_cb_dir   = 0;
    g_cb_state = STEER_STATE_IDLE;
    steer_Init(&test_motor_ops, 0, test_done_callback);

    /* Forward for 10 ticks (~100 ms) — test vtable: no motor spin */
    steer_Forward(0x40, 10);

    steer_GetStatus(&sts);
    if (sts.timed == 1) {
        SendString((unsigned char *)"  timer armed     [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  timer not armed [FAIL]\r\n");
        fail_count++;
    }

    /* Wait ~250 ms for the timer to expire (10 ticks * 10ms + margin) */
    {
        uint32_t start = GetSystemCounter();
        while ((GetSystemCounter() - start) < 25) {
            /* spin — ~250 ms */
        }
    }

    if (g_cb_fired >= 1) {
        SendString((unsigned char *)"  callback fired  [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  callback missed [FAIL]\r\n");
        fail_count++;
    }

    if (g_cb_dir == STEER_DIR_FORWARD && g_cb_state == STEER_STATE_FORWARD) {
        SendString((unsigned char *)"  cb dir=F st=FWD [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  cb args wrong   [FAIL]\r\n");
        fail_count++;
    }

    if (steer_GetState() == STEER_STATE_IDLE) {
        SendString((unsigned char *)"  back to IDLE    [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  not IDLE after  [FAIL]\r\n");
        fail_count++;
    }


    /* --- Section 10: Live motor diagnostic (optional) --- */
    SendString((unsigned char *)"\r\n--- 10. Live motor diagnostic ---\r\n");
    SendString((unsigned char *)"  Progressive test to find safe operating point.\r\n");
    SendString((unsigned char *)"  LIFT MOWER OFF GROUND!\r\n");
    SendString((unsigned char *)"  Press 'Y' to run, other key to skip: ");

    /* Drain stale RX bytes (terminal may have sent CR/LF after menu key).
     * Flush TX first so prompt is visible, then wait 50ms for trailing
     * bytes to arrive, then drain them. */
    sci1_tx_flush();
    {
        uint32_t start = GetSystemCounter();
        while ((GetSystemCounter() - start) < 5) {
            if (SCI1.SSR.BYTE & (SSR_ORER | SSR_FER | SSR_PER))
                SCI1.SSR.BYTE &= (unsigned char)~(SSR_ORER | SSR_FER | SSR_PER);
            if (SCI1.SSR.BYTE & SSR_RDRF) {
                (void)SCI1.RDR;
                SCI1.SSR.BYTE &= (unsigned char)~SSR_RDRF;
            }
        }
    }

    ch = GetChar();
    PutChar(ch);
    SendString((unsigned char *)"\r\n");

    if (ch == 'Y' || ch == 'y') {
        /*
         * Progressive motor diagnostic — isolates which step crashes
         * the MCU when real motors draw current.
         *
         * Step A: motor_PowerOn() only (no dir, no speed -> no spin)
         * Step B: add direction pins (still no speed -> no spin)
         * Step C: low speed 0x08 brief pulse (50ms)
         * Step D: medium speed 0x20 brief pulse (200ms)
         * Step E: target speed 0x40 for 1 second
         *
         * EMI protection pattern for each step:
         *   1. sci1_tx_flush() — drain TX so TXI ISR is idle
         *   2. Motor operation with motor_safe_delay (NOP-based,
         *      periodically clears SCI errors)
         *   3. motor_PowerOff + NOP cooldown
         *   4. motor_sci_recover() — clear SSR errors AND
         *      force-restore SCR (TE|RE) in case EMI corrupted it
         *
         * motor_safe_delay uses NOP counting, NOT GetSystemCounter,
         * because motor EMI can disrupt the TMRY ISR.
         */

        steer_Init(0, 0, 0);   /* real brushed backend */
        motor_safe_delay(500);

        /* Step A: power bus only */
        SendString((unsigned char *)"  A. PowerOn only...");
        sci1_tx_flush();
        motor_PowerOn();
        motor_safe_delay(200);
        motor_PowerOff();
        motor_safe_delay(50);
        motor_sci_recover();
        SendString((unsigned char *)" OK\r\n");

        /* Step B: power + direction (no speed = no spin)
         *
         * motor_LeftForward/RightForward enable FRT encoder capture
         * interrupts (TIER bits 5,7).  We suppress TIER immediately
         * after since we don't use encoder feedback.  Sub-step markers
         * (1-6) help identify exactly where a hang occurs.
         */
        SendString((unsigned char *)"  B. Power+Dir...");
        sci1_tx_flush();
        PutChar('1');
        motor_LeftForward();
        motor_KillEncoderIRQs();     /* kill encoder IRQs via shadow */
        PutChar('2');
        motor_RightForward();
        motor_KillEncoderIRQs();     /* kill encoder IRQs via shadow */
        PutChar('3');
        sci1_tx_flush();              /* drain markers before power on */
        motor_PowerOn();
        motor_safe_delay(200);
        motor_PowerOff();
        motor_LeftDisable();
        motor_RightDisable();
        motor_safe_delay(50);
        motor_sci_recover();
        PutChar('4');
        PutChar('5');
        PutChar('6');
        SendString((unsigned char *)" OK\r\n");

        /* Step C: low speed 0x08 for 50ms */
        SendString((unsigned char *)"  C. Speed 0x08 50ms...");
        sci1_tx_flush();
        motor_LeftForward();
        motor_RightForward();
        motor_KillEncoderIRQs();     /* suppress encoder IRQs via shadow */
        motor_LeftSetSpeed(0x08);
        motor_RightSetSpeed(0x08);
        motor_PowerOn();
        motor_safe_delay(50);
        motor_PowerOff();
        motor_LeftDisable();
        motor_RightDisable();
        motor_safe_delay(100);
        motor_sci_recover();
        SendString((unsigned char *)" OK\r\n");

        /* Step D: speed 0x20 for 200ms */
        SendString((unsigned char *)"  D. Speed 0x20 200ms...");
        sci1_tx_flush();
        motor_LeftForward();
        motor_RightForward();
        motor_KillEncoderIRQs();
        motor_LeftSetSpeed(0x20);
        motor_RightSetSpeed(0x20);
        motor_PowerOn();
        motor_safe_delay(200);
        motor_PowerOff();
        motor_LeftDisable();
        motor_RightDisable();
        motor_safe_delay(100);
        motor_sci_recover();
        SendString((unsigned char *)" OK\r\n");

        /* Step E: speed 0x40 — progressive duration.
         * D passed at 0x20/200ms.  E ramps duration at 0x40 to find
         * how long the MCU can sustain motor current before SCI fails
         * or the MCU hangs.  Sub-step markers E1-E4.
         *
         * EMI protection: sci1_tx_flush before each burst ensures
         * TXI ISR is idle.  motor_sci_recover after each burst
         * restores SCR in case EMI cleared TE/RE. */
        SendString((unsigned char *)"  E. Speed 0x40...");
        sci1_tx_flush();
        motor_KillEncoderIRQs();
        /* E1: 100ms burst */
        motor_LeftForward();
        motor_RightForward();
        motor_KillEncoderIRQs();
        motor_LeftSetSpeed(0x40);
        motor_RightSetSpeed(0x40);
        motor_PowerOn();
        motor_safe_delay(100);
        motor_PowerOff();
        motor_safe_delay(50);
        motor_sci_recover();
        PutChar('1');

        /* E2: 250ms burst */
        sci1_tx_flush();
        motor_PowerOn();
        motor_safe_delay(250);
        motor_PowerOff();
        motor_safe_delay(50);
        motor_sci_recover();
        PutChar('2');

        /* E3: 500ms burst */
        sci1_tx_flush();
        motor_PowerOn();
        motor_safe_delay(500);
        motor_PowerOff();
        motor_safe_delay(50);
        motor_sci_recover();
        PutChar('3');

        /* E4: 1000ms burst */
        sci1_tx_flush();
        motor_PowerOn();
        motor_safe_delay(1000);
        motor_PowerOff();
        motor_safe_delay(100);
        motor_LeftDisable();
        motor_RightDisable();
        motor_sci_recover();
        PutChar('4');
        SendString((unsigned char *)" OK\r\n");

        motor_StopAll();
        SendString((unsigned char *)"  All steps OK    [PASS]\r\n");
        pass_count++;
    } else {
        SendString((unsigned char *)"  Live test skipped\r\n");
    }


    /* --- Cleanup --- */
    steer_Stop();
    motor_StopAll();

    /* --- Summary --- */
    SendString((unsigned char *)"\r\n========================================\r\n");
    SendString((unsigned char *)"  Results: ");
    print_hex8(pass_count);
    SendString((unsigned char *)" passed, ");
    print_hex8(fail_count);
    SendString((unsigned char *)" failed\r\n");
    if (fail_count == 0)
        SendString((unsigned char *)"  ALL TESTS PASSED\r\n");
    else
        SendString((unsigned char *)"  *** FAILURES DETECTED ***\r\n");
    SendString((unsigned char *)"========================================\r\n");
}
