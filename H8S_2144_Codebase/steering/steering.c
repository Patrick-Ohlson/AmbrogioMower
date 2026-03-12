/*
 * steering.c - High-level steering/movement driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 *
 * See steering.h for full API documentation.
 */

#include "steering.h"
#include "../motor/motor.h"
#include "../timer/timer.h"

/* ====================================================================
 * Default brushed-motor backend vtable
 *
 * Wraps the existing motor/motor.h functions for the L6203 H-bridge.
 * A future brushless driver would define its own steer_motor_ops_t.
 * ==================================================================== */

const steer_motor_ops_t steer_motor_ops_brushed = {
    motor_PowerOn,          /* power_on       */
    motor_PowerOff,         /* power_off      */
    motor_LeftForward,      /* left_forward   */
    motor_LeftReverse,      /* left_reverse   */
    motor_LeftDisable,      /* left_disable   */
    motor_LeftSetSpeed,     /* left_set_speed */
    motor_RightForward,     /* right_forward  */
    motor_RightReverse,     /* right_reverse  */
    motor_RightDisable,     /* right_disable  */
    motor_RightSetSpeed,    /* right_set_speed*/
    motor_StopAll           /* stop_all       */
};

/* ====================================================================
 * Module-private state
 * ==================================================================== */

static const steer_motor_ops_t *g_ops;          /* active motor backend   */
static steer_config_t           g_cfg;          /* active configuration   */
static steer_done_cb_t          g_done_cb;      /* user completion cb     */

static volatile steer_state_t   g_state;        /* current movement state */
static volatile uint8_t         g_dir;          /* current direction char */
static volatile uint8_t         g_speed_l;      /* current left speed     */
static volatile uint8_t         g_speed_r;      /* current right speed    */
static volatile uint8_t         g_timed;        /* 1 = timer armed        */

static sw_timer_t               g_timer;        /* software timer         */

/* ====================================================================
 * Internal: timer expiry callback  (runs in ISR context)
 *
 * Stops motors, transitions to IDLE, then fires the user callback.
 * The user callback may safely issue a new steer_* command (chaining).
 * ==================================================================== */

static void steer_timer_expired(sw_timer_t *t)
{
    steer_state_t old_state = g_state;
    uint8_t       old_dir   = g_dir;

    (void)t;   /* unused — required by sw_timer callback signature */

    /* Disable both wheels through the vtable */
    g_ops->left_disable();
    g_ops->right_disable();

    g_state   = STEER_STATE_IDLE;
    g_dir     = STEER_DIR_STOP;
    g_speed_l = 0;
    g_speed_r = 0;
    g_timed   = 0;

    /* Fire the user's completion callback (still in ISR context) */
    if (g_done_cb != (steer_done_cb_t)0) {
        g_done_cb(old_state, old_dir);
    }
}

/* ====================================================================
 * Internal: cancel any active movement
 *
 * Disarms the software timer and disables motors.
 * Does NOT fire the completion callback.
 * Caller must hold a critical section.
 * ==================================================================== */

static void steer_cancel_active(void)
{
    /* Disarm timer if running */
    if (g_timed) {
        timer_Set(&g_timer, 0, 0);     /* ticks=0 disarms */
        g_timed = 0;
    }

    /* Disable motors if we are moving */
    if (g_state != STEER_STATE_IDLE) {
        g_ops->left_disable();
        g_ops->right_disable();
        g_state   = STEER_STATE_IDLE;
        g_dir     = STEER_DIR_STOP;
        g_speed_l = 0;
        g_speed_r = 0;
    }
}

/* ====================================================================
 * Internal: apply a new movement
 *
 * Cancels any active movement, configures motors for the requested
 * state, and optionally arms the software timer.
 * ==================================================================== */

static void steer_apply(steer_state_t new_state, uint8_t dir,
                         uint8_t speed_l, uint8_t speed_r,
                         uint16_t ticks)
{
    unsigned char ccr;

    ENTER_CRITICAL(ccr);

    /* Cancel any previous movement first */
    steer_cancel_active();

    /* Ensure motor power is on */
    g_ops->power_on();

    /* --- Set left wheel direction --- */
    switch (new_state) {
    case STEER_STATE_FORWARD:
    case STEER_STATE_TURN_RIGHT:    /* left fwd, right rev */
        g_ops->left_forward();
        break;
    case STEER_STATE_REVERSE:
    case STEER_STATE_TURN_LEFT:     /* left rev, right fwd */
        g_ops->left_reverse();
        break;
    default:
        g_ops->left_disable();
        break;
    }

    /* --- Set right wheel direction --- */
    switch (new_state) {
    case STEER_STATE_FORWARD:
    case STEER_STATE_TURN_LEFT:     /* left rev, right fwd */
        g_ops->right_forward();
        break;
    case STEER_STATE_REVERSE:
    case STEER_STATE_TURN_RIGHT:    /* left fwd, right rev */
        g_ops->right_reverse();
        break;
    default:
        g_ops->right_disable();
        break;
    }

    /* --- Set wheel speeds --- */
    g_ops->left_set_speed(speed_l);
    g_ops->right_set_speed(speed_r);

    /* --- Update internal state --- */
    g_state   = new_state;
    g_dir     = dir;
    g_speed_l = speed_l;
    g_speed_r = speed_r;

    /* --- Arm timer if a duration was specified --- */
    if (ticks > 0) {
        /* Clamp to failsafe maximum if configured */
        if (g_cfg.max_move_ticks > 0 && ticks > g_cfg.max_move_ticks) {
            ticks = g_cfg.max_move_ticks;
        }
        timer_Set(&g_timer, ticks, 0);     /* one-shot */
        g_timed = 1;
    } else {
        g_timed = 0;
    }

    EXIT_CRITICAL(ccr);
}

/* ====================================================================
 * Public API — Initialisation
 * ==================================================================== */

void steer_Init(const steer_motor_ops_t *ops,
                const steer_config_t    *cfg,
                steer_done_cb_t          done_cb)
{
    /* Select motor backend */
    if (ops != (const steer_motor_ops_t *)0) {
        g_ops = ops;
    } else {
        g_ops = &steer_motor_ops_brushed;
    }

    /* Load configuration */
    if (cfg != (const steer_config_t *)0) {
        g_cfg = *cfg;
    } else {
        g_cfg.wheel_diameter_mm = STEER_DEFAULT_WHEEL_DIA;
        g_cfg.wheelbase_mm      = STEER_DEFAULT_WHEELBASE;
        g_cfg.speed_forward     = STEER_DEFAULT_SPEED_FWD;
        g_cfg.speed_reverse     = STEER_DEFAULT_SPEED_REV;
        g_cfg.speed_turn        = STEER_DEFAULT_SPEED_TURN;
        g_cfg.ticks_90deg       = STEER_DEFAULT_TICKS_90;
        g_cfg.ticks_180deg      = STEER_DEFAULT_TICKS_180;
        g_cfg.max_move_ticks    = STEER_DEFAULT_MAX_TICKS;
    }

    g_done_cb = done_cb;

    /* Reset internal state */
    g_state   = STEER_STATE_IDLE;
    g_dir     = STEER_DIR_STOP;
    g_speed_l = 0;
    g_speed_r = 0;
    g_timed   = 0;

    /* Initialise the software timer (not armed yet) */
    timer_Init(&g_timer, steer_timer_expired);
}

void steer_SetConfig(const steer_config_t *cfg)
{
    if (cfg != (const steer_config_t *)0) {
        unsigned char ccr;
        ENTER_CRITICAL(ccr);
        g_cfg = *cfg;
        EXIT_CRITICAL(ccr);
    }
}

void steer_SetCallback(steer_done_cb_t done_cb)
{
    unsigned char ccr;
    ENTER_CRITICAL(ccr);
    g_done_cb = done_cb;
    EXIT_CRITICAL(ccr);
}

/* ====================================================================
 * Public API — Movement Commands
 * ==================================================================== */

void steer_Forward(uint8_t speed, uint16_t ticks)
{
    if (speed == 0) speed = g_cfg.speed_forward;
    steer_apply(STEER_STATE_FORWARD, STEER_DIR_FORWARD,
                speed, speed, ticks);
}

void steer_Reverse(uint8_t speed, uint16_t ticks)
{
    if (speed == 0) speed = g_cfg.speed_reverse;
    steer_apply(STEER_STATE_REVERSE, STEER_DIR_BACK,
                speed, speed, ticks);
}

void steer_TurnLeft(uint8_t speed, uint16_t ticks)
{
    if (speed == 0) speed = g_cfg.speed_turn;
    steer_apply(STEER_STATE_TURN_LEFT, STEER_DIR_LEFT,
                speed, speed, ticks);
}

void steer_TurnRight(uint8_t speed, uint16_t ticks)
{
    if (speed == 0) speed = g_cfg.speed_turn;
    steer_apply(STEER_STATE_TURN_RIGHT, STEER_DIR_RIGHT,
                speed, speed, ticks);
}

void steer_Stop(void)
{
    unsigned char ccr;
    ENTER_CRITICAL(ccr);
    steer_cancel_active();
    EXIT_CRITICAL(ccr);
    /* Note: steer_Stop() does NOT fire the completion callback. */
}

/* ====================================================================
 * Convenience — pre-configured angle turns
 * ==================================================================== */

void steer_TurnLeft90(void)
{
    steer_TurnLeft(0, g_cfg.ticks_90deg);
}

void steer_TurnRight90(void)
{
    steer_TurnRight(0, g_cfg.ticks_90deg);
}

void steer_TurnLeft180(void)
{
    steer_TurnLeft(0, g_cfg.ticks_180deg);
}

void steer_TurnRight180(void)
{
    steer_TurnRight(0, g_cfg.ticks_180deg);
}

/* ====================================================================
 * Public API — Status
 * ==================================================================== */

steer_state_t steer_GetState(void)
{
    return g_state;
}

void steer_GetStatus(steer_status_t *status)
{
    unsigned char ccr;
    ENTER_CRITICAL(ccr);
    status->state          = g_state;
    status->direction      = g_dir;
    status->speed_left     = g_speed_l;
    status->speed_right    = g_speed_r;
    status->timed          = g_timed;
    status->ticks_remaining = g_timed ? g_timer.countdown : 0;
    EXIT_CRITICAL(ccr);
}

uint8_t steer_IsMoving(void)
{
    return (g_state != STEER_STATE_IDLE) ? 1 : 0;
}

/* ====================================================================
 * Turn-Time Estimation
 *
 * Linear scaling from the calibrated 90-degree constant:
 *   ticks = (angle_deg * ticks_90deg) / 90
 *
 * 32-bit intermediate avoids overflow (e.g. 360 * 300 = 108 000).
 * Only called at command time, not from ISR.
 * ==================================================================== */

uint16_t steer_EstimateTurnTicks(uint16_t angle_deg)
{
    if (angle_deg == 0) return 0;
    return (uint16_t)(((uint32_t)angle_deg * g_cfg.ticks_90deg) / 90);
}
