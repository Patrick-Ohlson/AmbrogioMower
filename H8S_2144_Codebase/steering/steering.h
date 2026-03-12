/*
 * steering.h - High-level steering/movement driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 *
 * Provides directional movement commands (forward, back, left, right,
 * stop) with optional timer-based duration.  Abstracts the low-level
 * motor driver through a function-pointer table so a future brushless
 * backend can replace the current brushed L6203 driver.
 *
 * Original firmware steering functions (Italian naming, for reference):
 *   setSteeringDirection      (0x5BEA) -> steer_TurnLeft / steer_TurnRight
 *   executeSteeringManeuver   (0x5A40) -> steer_TurnLeft(speed, ticks)
 *   forwardMoveWithSteering   (0x5C50) -> steer_Forward
 *   stopAndWait               (0x5830) -> steer_Stop
 *   loopSteeringUntilDone     (0x18AC) -> chained via completion callback
 *
 * Direction convention (English, clean break from Italian original):
 *   'F' = Forward       (original had no single char)
 *   'B' = Back           "
 *   'R' = Right          (original: 'D' Destra)
 *   'L' = Left           (original: 'S' Sinistra)
 *   'S' = Stop           (original: 'N' Nessuno)
 *
 * Timer integration:
 *   Uses sw_timer_t from timer/timer.h.  1 tick = ~10 ms.
 *   100 ticks = 1 second, 500 ticks = 5 seconds.
 *
 * Thread safety:
 *   Timed-movement callbacks fire from the TMRY ISR context.
 *   Shared state is protected by ENTER_CRITICAL / EXIT_CRITICAL.
 */

#ifndef STEERING_H
#define STEERING_H

#include <stdint.h>
#include "../timer/timer.h"

/* ====================================================================
 * Direction Constants
 * ==================================================================== */

#define STEER_DIR_FORWARD   'F'
#define STEER_DIR_BACK      'B'
#define STEER_DIR_RIGHT     'R'
#define STEER_DIR_LEFT      'L'
#define STEER_DIR_STOP      'S'

/* ====================================================================
 * Movement States
 * ==================================================================== */

typedef enum {
    STEER_STATE_IDLE = 0,       /* Motors disabled, no active movement  */
    STEER_STATE_FORWARD,        /* Both wheels forward                  */
    STEER_STATE_REVERSE,        /* Both wheels reverse                  */
    STEER_STATE_TURN_LEFT,      /* Spot turn left (L rev, R fwd)        */
    STEER_STATE_TURN_RIGHT,     /* Spot turn right (L fwd, R rev)       */
    STEER_STATE_ARC_LEFT,       /* Arc left (L slow, R fast) -- future  */
    STEER_STATE_ARC_RIGHT,      /* Arc right (L fast, R slow) -- future */
    STEER_STATE_STOPPING        /* Transitional: motors disabling       */
} steer_state_t;

/* ====================================================================
 * Motor Backend Abstraction (vtable)
 *
 * Allows swapping the brushed L6203 backend for a future brushless
 * driver without changing the steering logic.  The default vtable
 * wraps the existing motor/motor.h functions.
 * ==================================================================== */

typedef struct {
    /* Power control */
    void (*power_on)(void);
    void (*power_off)(void);

    /* Left wheel */
    void (*left_forward)(void);
    void (*left_reverse)(void);
    void (*left_disable)(void);
    void (*left_set_speed)(uint8_t speed);

    /* Right wheel */
    void (*right_forward)(void);
    void (*right_reverse)(void);
    void (*right_disable)(void);
    void (*right_set_speed)(uint8_t speed);

    /* Combined stop */
    void (*stop_all)(void);
} steer_motor_ops_t;

/* ====================================================================
 * Physical Configuration
 *
 * All dimensions in millimetres.  Integer maths only.
 *
 * L200 approximate dimensions (measured from hardware):
 *   Wheel diameter:     ~210 mm (8.3 in)
 *   Wheelbase:          ~450 mm (17.7 in, axle centre-to-centre)
 *   Wheel circumference: ~660 mm  (pi * 210)
 *
 * Turn-time estimation (without encoders):
 *   For a spot turn (one wheel fwd, other rev), each wheel traces
 *   an arc of radius = wheelbase / 2.  For 90 degrees:
 *     arc = (90/360) * pi * wheelbase  = pi * 450 / 4  ~= 353 mm
 *   ticks_90deg is a tunable constant calibrated on real hardware.
 *
 * Speed mapping (8-bit PWMX for our direct 6203 drive):
 *   0x60  (~96)  = slow / manoeuvre
 *   0x80  (~128) = medium / normal mowing
 *   0xB0  (~176) = fast forward
 * ==================================================================== */

typedef struct {
    /* Physical dimensions (mm) */
    uint16_t wheel_diameter_mm;     /* Wheel outer diameter             */
    uint16_t wheelbase_mm;          /* Axle-to-axle distance            */

    /* Default speeds (0-255, 8-bit PWMX duty) */
    uint8_t  speed_forward;         /* Default forward speed            */
    uint8_t  speed_reverse;         /* Default reverse speed            */
    uint8_t  speed_turn;            /* Default spot-turn speed          */

    /* Turn-time calibration (ticks, 1 tick ~= 10 ms) */
    uint16_t ticks_90deg;           /* Ticks for a 90-degree spot turn  */
    uint16_t ticks_180deg;          /* Ticks for a 180-degree spot turn */

    /* Safety */
    uint16_t max_move_ticks;        /* Maximum movement duration (0 = no limit) */
} steer_config_t;

/* Default values — CALIBRATE ticks_90deg / ticks_180deg on hardware! */
#define STEER_DEFAULT_WHEEL_DIA     210     /* mm                       */
#define STEER_DEFAULT_WHEELBASE     450     /* mm                       */
#define STEER_DEFAULT_SPEED_FWD     200    /* ~50 % duty               */
#define STEER_DEFAULT_SPEED_REV     200   /* ~37 % duty               */
#define STEER_DEFAULT_SPEED_TURN    0x60    /* ~37 % duty               */
#define STEER_DEFAULT_TICKS_90      150     /* ~1.5 s  — CALIBRATE!     */
#define STEER_DEFAULT_TICKS_180     300     /* ~3.0 s  — CALIBRATE!     */
#define STEER_DEFAULT_MAX_TICKS     3000    /* 30 s absolute failsafe   */

/* ====================================================================
 * Completion Callback
 *
 * Called when a timed movement expires.  Fires from TMRY ISR context
 * (inside fUpdateSystemCounter -> sw_timer callback).
 *
 * IMPORTANT: keep the callback short.  No blocking, no printf.
 * It IS safe to call steer_Stop() or start a new movement from the
 * callback (enables movement chaining).
 *
 * @param state   The movement state that just completed
 * @param dir     The direction char ('F','B','R','L') that completed
 * ==================================================================== */

typedef void (*steer_done_cb_t)(steer_state_t state, uint8_t dir);

/* ====================================================================
 * Runtime Status (read-only query)
 * ==================================================================== */

typedef struct {
    steer_state_t state;            /* Current movement state           */
    uint8_t       direction;        /* Current direction char           */
    uint8_t       speed_left;       /* Current left-wheel speed         */
    uint8_t       speed_right;      /* Current right-wheel speed        */
    uint16_t      ticks_remaining;  /* Ticks left (0 if untimed)        */
    uint8_t       timed;            /* 1 = timer active, 0 = continuous */
} steer_status_t;

/* ====================================================================
 * Public API — Initialisation
 * ==================================================================== */

/*
 * steer_Init - Initialise the steering module.
 *
 * Sets up internal state, stores the motor backend vtable and the
 * physical configuration.  Does NOT call motor_Init() — the caller
 * must initialise the motor driver separately beforehand.
 *
 * @param ops     Motor backend vtable (NULL = use default brushed L6203)
 * @param cfg     Physical configuration (NULL = use defaults)
 * @param done_cb Completion callback (NULL = no callback)
 *
 * Call once at startup, after motor_Init() and timer_HW_Init().
 */
void steer_Init(const steer_motor_ops_t *ops,
                const steer_config_t    *cfg,
                steer_done_cb_t          done_cb);

/*
 * steer_SetConfig - Update configuration at runtime.
 * Takes effect on the next movement command.
 */
void steer_SetConfig(const steer_config_t *cfg);

/*
 * steer_SetCallback - Change the completion callback.
 */
void steer_SetCallback(steer_done_cb_t done_cb);

/* ====================================================================
 * Public API — Movement Commands
 *
 * All commands are NON-BLOCKING.  They configure motor direction/speed
 * and optionally arm a software timer.  When the timer expires, the
 * motors stop and the completion callback fires.
 *
 * speed = 0  → use the relevant config default.
 * ticks = 0  → move continuously until steer_Stop().
 *
 * If a new command is issued while a previous movement is active, the
 * old one is cancelled (motors stopped, timer disarmed) before the
 * new command takes effect.
 * ==================================================================== */

void steer_Forward(uint8_t speed, uint16_t ticks);
void steer_Reverse(uint8_t speed, uint16_t ticks);
void steer_TurnLeft(uint8_t speed, uint16_t ticks);
void steer_TurnRight(uint8_t speed, uint16_t ticks);

/*
 * steer_Stop - Immediately stop all wheel motors.
 * Disarms any active timer.  Does NOT fire the completion callback.
 */
void steer_Stop(void);

/* ====================================================================
 * Convenience — pre-configured angle turns (use config defaults)
 * ==================================================================== */

void steer_TurnLeft90(void);
void steer_TurnRight90(void);
void steer_TurnLeft180(void);
void steer_TurnRight180(void);

/* ====================================================================
 * Public API — Status
 * ==================================================================== */

steer_state_t steer_GetState(void);
void          steer_GetStatus(steer_status_t *status);
uint8_t       steer_IsMoving(void);

/* ====================================================================
 * Turn-Time Estimation
 *
 * Linear scaling from the calibrated ticks_90deg value:
 *   ticks = (angle_deg * ticks_90deg) / 90
 *
 * This is an approximation.  Calibrate ticks_90deg on real hardware.
 *
 * @param angle_deg   Turn angle in degrees (1-360)
 * @return            Estimated ticks (0 if angle is 0)
 * ==================================================================== */

uint16_t steer_EstimateTurnTicks(uint16_t angle_deg);

/* ====================================================================
 * Default Motor Backend
 *
 * Pre-built vtable that wraps motor/motor.h functions for the
 * brushed L6203 driver.  Pass to steer_Init(), or pass NULL.
 * ==================================================================== */

extern const steer_motor_ops_t steer_motor_ops_brushed;

#endif /* STEERING_H */
