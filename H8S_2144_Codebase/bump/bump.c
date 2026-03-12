/*
 * bump.c - Bump / Lift Sensor Driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_MainMower.md
 * and STATES/state_MotorControl.md
 *
 * Hardware: Port 6, bit 0 (P6.0) — active-high lift/bump sensor input.
 * The sensor is debounced by counting consecutive high readings at a
 * rate-limited interval (~16 system ticks). After 4+ consecutive highs
 * the sensor is considered triggered.
 *
 * Bump classification uses the motor error direction set by the wheel
 * motor error checker (checkWheelMotorErrors, 0x60EC):
 *   - Left wheel error (dir=1) → front bump (0x0A)
 *   - Right wheel error (dir=2) → rear bump (0x0B)
 *   - No error (dir=0) → no directional bump (0x00)
 *
 * Original firmware functions:
 *   pollLiftSensorPort6  (0x1528) -> bump_Poll()
 *   sub_156C             (0x156C) -> bump_IsTriggered()
 *   detectBumpCollision  (0x6FD4) -> bump_Detect()
 *   setPostBumpState     (0x7542) -> bump_SetPostState()
 */

#include "bump.h"
#include "../iodefine.h"

/* ---- Hardware defines ---- */

/* P6.0 is the bump/lift sensor input (active high).
 * Read via P6.DR.BYTE bit 0.
 * Firmware: pollLiftSensorPort6 reads P6_DR at 0x1528 */
#define BUMP_SENSOR_BIT     0x01    /* P6.DR bit 0 mask */

/* Minimum tick interval between debounce polls.
 * Firmware: pollLiftSensorPort6 checks (counter2 - dword_FFE854) > 0x10
 * 0x10 = 16 ticks */
#define BUMP_POLL_INTERVAL  0x10

/* Debounce threshold: sensor is triggered when debounce_count >= 4.
 * The firmware checks (dword_FFE858 >> 2) > 0, which is equivalent
 * to dword_FFE858 >= 4.
 * Firmware: sub_156C (0x156C) */
#define BUMP_DEBOUNCE_SHIFT 2       /* Right shift for threshold check */

/* ---- Module state ---- */

/* Debounce counter: consecutive high readings from P6.0.
 * Incremented each poll when P6.0 is high, reset to 0 when low.
 * Firmware: dword_FFE858 */
static uint32_t debounce_count = 0;

/* System tick of last debounce poll (for rate limiting).
 * Firmware: dword_FFE854 */
static uint32_t last_poll_tick = 0;

/* Motor error direction set by wheel motor error checker.
 *   0 = no direction, 1 = left wheel (front), 2 = right wheel (rear)
 * Firmware: word_FFE940 */
static uint16_t motor_dir = 0;

/* Bump active flag: 1 = bump currently in progress (sensor held high).
 * Set when bump first detected, cleared on release.
 * Firmware: word_FFE942 */
static uint16_t active = 0;

/* System tick when bump first triggered (for timeout/duration tracking).
 * Firmware: dword_FFE944 */
static uint32_t trigger_tick = 0;

/* Raw contact flag: set during active bump detection.
 * Firmware: byte_FFE8BE */
static uint8_t contact = 0;

/* Post-bump recovery state: records direction after bump maneuver.
 *   2 = was moving forward, 3 = was reversing
 * Firmware: byte_FFE97D */
static uint8_t post_state = 0;

/* System tick when post-bump state was recorded.
 * Firmware: dword_FFE97E */
static uint32_t post_tick = 0;

/* ====================================================================
 * Public API
 * ==================================================================== */

void bump_Init(void)
{
    debounce_count = 0;
    last_poll_tick = 0;
    motor_dir = 0;
    active = 0;
    trigger_tick = 0;
    contact = 0;
    post_state = 0;
    post_tick = 0;
}

/*
 * bump_Poll - Poll the lift/bump sensor with debounce
 *
 * Firmware: pollLiftSensorPort6 (0x1528)
 *
 * Pseudocode from Ghidra decompilation:
 *   if (counter2 - dword_FFE854 > 0x10) {
 *       dword_FFE854 = counter2;
 *       if (P6_DR & 0x01) {
 *           dword_FFE858++;       // consecutive high count
 *       } else {
 *           dword_FFE858 = 0;     // reset on low
 *       }
 *   }
 *
 * The rate limiting ensures we don't sample faster than ~16 ticks,
 * which provides noise immunity on the mechanical switch input.
 */
void bump_Poll(uint32_t sys_tick)
{
    /* Rate limit: only poll every BUMP_POLL_INTERVAL ticks.
     * Uses unsigned subtraction for tick counter wraparound safety. */
    if ((sys_tick - last_poll_tick) <= BUMP_POLL_INTERVAL)
        return;

    last_poll_tick = sys_tick;

    /* Read P6.0: active high = bumped/lifted */
    if (P6.DR.BYTE & BUMP_SENSOR_BIT) {
        /* Sensor high — increment consecutive-high counter */
        debounce_count++;
    } else {
        /* Sensor low — reset debounce counter */
        debounce_count = 0;
    }
}

/*
 * bump_IsTriggered - Check if bump sensor is triggered (debounced)
 *
 * Firmware: sub_156C (0x156C)
 *
 * Ghidra decompilation:
 *   return (dword_FFE858 >> 2) > 0;
 *
 * This is equivalent to debounce_count >= 4, meaning we need at least
 * 4 consecutive high readings (at ~16 tick intervals = ~64 ticks total
 * debounce time) before declaring the sensor triggered.
 */
uint8_t bump_IsTriggered(void)
{
    return (debounce_count >> BUMP_DEBOUNCE_SHIFT) > 0 ? 1 : 0;
}

/*
 * bump_Detect - Run bump collision detection logic
 *
 * Firmware: detectBumpCollision (0x6FD4)
 *
 * Ghidra decompilation (simplified):
 *
 *   uint8_t result = 0;
 *   if (sub_156C() != 0) {                     // sensor triggered?
 *       byte_FFE8BE = 1;                        // contact = true
 *       if (word_FFE942 == 0) {                 // not already active?
 *           dword_FFE944 = counter2;            // record trigger time
 *           word_FFE942 = 1;                    // mark active
 *       }
 *       result = 1;                             // BUMP_TRIGGERED
 *   } else {
 *       byte_FFE8BE = 0;                        // contact = false
 *       if (word_FFE942 != 0) {                 // was active? (released)
 *           word_FFE942 = 0;                    // clear active
 *           if (word_FFE940 == 1) {             // left motor error?
 *               result = 0x0A;                  // BUMP_FRONT
 *           } else if (word_FFE940 == 2) {      // right motor error?
 *               result = 0x0B;                  // BUMP_REAR
 *           }
 *           word_FFE940 = 0;                    // clear motor direction
 *       }
 *   }
 *   return result;
 *
 * State transitions:
 *   Idle + triggered      → active=1, return BUMP_TRIGGERED (1)
 *   Active + triggered    → active=1, return BUMP_TRIGGERED (1)
 *   Active + released     → active=0, classify by motor_dir:
 *                            dir=1 → BUMP_FRONT (0x0A)
 *                            dir=2 → BUMP_REAR  (0x0B)
 *                            dir=0 → BUMP_NONE  (0)
 *   Idle + released       → no change, return BUMP_NONE (0)
 */
uint8_t bump_Detect(void)
{
    uint8_t result = BUMP_NONE;

    if (bump_IsTriggered()) {
        /* Sensor is triggered (debounced) — bump in progress */
        contact = 1;

        if (active == 0) {
            /* First detection: record trigger timestamp, set active */
            trigger_tick = last_poll_tick;
            active = 1;
        }

        result = BUMP_TRIGGERED;

    } else {
        /* Sensor released */
        contact = 0;

        if (active != 0) {
            /* Bump was active — now released. Classify by motor direction. */
            active = 0;

            if (motor_dir == BUMP_DIR_LEFT) {
                /* Left wheel motor error → front bump */
                result = BUMP_FRONT;
            } else if (motor_dir == BUMP_DIR_RIGHT) {
                /* Right wheel motor error → rear bump */
                result = BUMP_REAR;
            }
            /* else motor_dir == 0: no directional info → result stays BUMP_NONE */

            /* Clear motor direction after classification.
             * The direction is consumed once per bump event. */
            motor_dir = BUMP_DIR_NONE;
        }
    }

    return result;
}

/*
 * bump_SetMotorDir - Set motor error direction for bump classification
 *
 * Called by checkWheelMotorErrors (0x60EC) when a motor stall or
 * overcurrent condition is detected on a wheel motor.
 *
 * Firmware: writes directly to word_FFE940
 */
void bump_SetMotorDir(uint16_t dir)
{
    motor_dir = dir;
}

/*
 * bump_SetPostState - Record post-bump recovery direction
 *
 * Firmware: setPostBumpState (0x7542)
 *
 * Ghidra decompilation:
 *   if (was_forward) {
 *       byte_FFE97D = 2;          // BUMP_POST_FORWARD
 *   } else {
 *       byte_FFE97D = 3;          // BUMP_POST_REVERSE
 *   }
 *   dword_FFE97E = counter2;      // timestamp
 *
 * Called by the main mowing loop after executing a bump recovery
 * maneuver. The post-state tells subsequent motion planning which
 * direction the mower was travelling when the bump occurred.
 */
void bump_SetPostState(uint8_t was_forward)
{
    if (was_forward) {
        post_state = BUMP_POST_FORWARD;     /* 2 — was going forward */
    } else {
        post_state = BUMP_POST_REVERSE;     /* 3 — was reversing */
    }

    /* Record the timestamp of this post-bump state.
     * We use last_poll_tick as an approximation of the current tick.
     * In the firmware, this reads counter2 directly. */
    post_tick = last_poll_tick;
}

void bump_GetState(bump_state_t *state)
{
    state->debounce_count = debounce_count;
    state->last_poll_tick = last_poll_tick;
    state->motor_dir      = motor_dir;
    state->active         = active;
    state->trigger_tick   = trigger_tick;
    state->contact        = contact;
    state->post_state     = post_state;
    state->post_tick      = post_tick;
}

uint8_t bump_GetContact(void)
{
    return contact;
}

/*
 * bump_GetReverseSpeed - Map bump code to recovery reverse speed
 *
 * The main mowing loop (state_MainMower.md) selects a reverse motor
 * speed based on the bump type returned by detectBumpCollision:
 *
 *   0x0A (front) → speed 0x1D5 (469) — moderate reverse
 *   0x0B (rear)  → speed 0x2C0 (704) — fast reverse
 *   other        → speed 0xEA  (234) — slow reverse (side bump)
 *
 * Returns 0 if no bump (BUMP_NONE), since no recovery is needed.
 */
uint16_t bump_GetReverseSpeed(uint8_t bump_code)
{
    switch (bump_code) {
    case BUMP_FRONT:
        return BUMP_SPEED_FRONT;    /* 0x1D5 = 469 */
    case BUMP_REAR:
        return BUMP_SPEED_REAR;     /* 0x2C0 = 704 */
    case BUMP_TRIGGERED:
        return BUMP_SPEED_SIDE;     /* 0xEA = 234 (default/side) */
    case BUMP_NONE:
    default:
        return 0;                   /* No bump — no recovery speed */
    }
}
