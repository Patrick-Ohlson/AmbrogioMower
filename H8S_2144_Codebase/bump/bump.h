/*
 * bump.h - Bump / Lift Sensor Driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_MainMower.md
 * and STATES/state_MotorControl.md
 *
 * The L200 has a lift/bump sensor on Port 6, bit 0 (P6.0). The sensor
 * is an active-high digital input: high = lifted/bumped, low = normal.
 *
 * The driver debounces the raw pin by counting consecutive high readings
 * at a rate-limited interval (~16 system ticks). After 4+ consecutive
 * highs the sensor is considered triggered.
 *
 * Bump collision detection also tracks motor error direction to classify
 * bumps as front, rear, or side — selecting different reverse speeds
 * for the recovery maneuver.
 *
 * Original firmware functions:
 *   pollLiftSensorPort6  (0x1528) -> bump_Poll()
 *   sub_156C             (0x156C) -> bump_IsTriggered()
 *   detectBumpCollision  (0x6FD4) -> bump_Detect()
 *   setPostBumpState     (0x7542) -> bump_SetPostState()
 */

#ifndef BUMP_H
#define BUMP_H

#include <stdint.h>

/* ====================================================================
 * Bump Detection Return Codes
 *
 * detectBumpCollision() returns a code indicating bump type:
 *
 *   0    — No bump detected
 *   1    — Sensor triggered (debounce in progress, first detection)
 *   0x0A — Front bump (motor direction was forward / left-wheel error)
 *   0x0B — Rear bump (motor direction was reverse / right-wheel error)
 *
 * The return code selects reverse speed in the main mowing loop:
 *   0x0A (front) → speed 0x1D5 (469) — moderate reverse
 *   0x0B (rear)  → speed 0x2C0 (704) — fast reverse
 *   other        → speed 0xEA  (234) — slow reverse (side bump)
 *
 * Firmware: detectBumpCollision (0x6FD4)
 * ==================================================================== */

#define BUMP_NONE           0       /* No bump detected */
#define BUMP_TRIGGERED      1       /* Sensor triggered (active contact) */
#define BUMP_FRONT          0x0A    /* Front bump — forward collision */
#define BUMP_REAR           0x0B    /* Rear bump — reverse collision */

/* Reverse speeds for recovery maneuver after bump
 * Firmware: state_MotorControl.md motor speed reference table */
#define BUMP_SPEED_FRONT    0x01D5  /* 469 — moderate reverse */
#define BUMP_SPEED_REAR     0x02C0  /* 704 — fast reverse */
#define BUMP_SPEED_SIDE     0x00EA  /* 234 — slow reverse (default) */

/* ====================================================================
 * Motor Direction Codes
 *
 * The wheel motor error checker (checkWheelMotorErrors, 0x60EC) sets
 * a direction word that detectBumpCollision uses to classify the bump:
 *
 *   0 — No motor error direction recorded
 *   1 — Left wheel motor error → maps to BUMP_FRONT (0x0A)
 *   2 — Right wheel motor error → maps to BUMP_REAR (0x0B)
 *
 * Firmware: word_FFE940
 * ==================================================================== */

#define BUMP_DIR_NONE       0
#define BUMP_DIR_LEFT       1       /* Left wheel error → front bump */
#define BUMP_DIR_RIGHT      2       /* Right wheel error → rear bump */

/* ====================================================================
 * Post-Bump State Codes
 *
 * After a bump recovery maneuver, setPostBumpState() records the
 * recovery direction for subsequent motion planning.
 *
 * Firmware: byte_FFE97D via setPostBumpState (0x7542)
 * ==================================================================== */

#define BUMP_POST_FORWARD   2       /* Post-bump: was moving forward */
#define BUMP_POST_REVERSE   3       /* Post-bump: was moving in reverse */

/* ====================================================================
 * Types
 * ==================================================================== */

/* Bump detection state — holds all internal counters and flags */
typedef struct {
    uint32_t debounce_count;    /* Consecutive high readings from P6.0
                                 * Firmware: dword_FFE858 */
    uint32_t last_poll_tick;    /* System tick of last debounce poll
                                 * Firmware: dword_FFE854 */
    uint16_t motor_dir;         /* Motor error direction (0/1/2)
                                 * Set by checkWheelMotorErrors
                                 * Firmware: word_FFE940 */
    uint16_t active;            /* 1 = bump currently active (sensor held)
                                 * Firmware: word_FFE942 */
    uint32_t trigger_tick;      /* Tick when bump first triggered
                                 * Firmware: dword_FFE944 */
    uint8_t  contact;           /* 1 = physical contact sensed (raw)
                                 * Firmware: byte_FFE8BE */
    uint8_t  post_state;        /* Post-bump recovery state (2 or 3)
                                 * Firmware: byte_FFE97D */
    uint32_t post_tick;         /* Tick when post-bump state was set
                                 * Firmware: dword_FFE97E */
} bump_state_t;

/* ====================================================================
 * Public API
 * ==================================================================== */

/*
 * bump_Init - Reset bump detection state
 *
 * Clears debounce counter, motor direction, and all flags.
 * Call once at startup.
 */
void bump_Init(void);

/*
 * bump_Poll - Poll the lift/bump sensor with debounce
 *
 * Reads P6.0 at a rate-limited interval (~16 system ticks between reads).
 * If P6.0 is high, increments a consecutive-high counter.
 * If P6.0 is low, resets the counter to zero.
 *
 * Call this every main loop tick. The debounce counter is used by
 * bump_IsTriggered() and bump_Detect().
 *
 * @param sys_tick  Current system tick counter (from GetSystemCounter)
 *
 * Firmware: pollLiftSensorPort6 (0x1528)
 */
void bump_Poll(uint32_t sys_tick);

/*
 * bump_IsTriggered - Check if bump sensor is triggered (debounced)
 *
 * The sensor is considered triggered after 4+ consecutive high readings
 * (debounce_count >> 2 > 0).
 *
 * @return  1 if triggered, 0 if not
 *
 * Firmware: sub_156C (0x156C) — returns (dword_FFE858 >> 2 > 0)
 */
uint8_t bump_IsTriggered(void);

/*
 * bump_Detect - Run bump collision detection logic
 *
 * Checks the debounced sensor state and motor error direction to
 * classify the bump type. Returns a code used by the main mowing
 * loop to select reverse speed and recovery maneuver.
 *
 * State machine:
 *   - If sensor triggered and not already active: record timestamp,
 *     set active=1, return BUMP_TRIGGERED (1)
 *   - If sensor triggered and already active: return BUMP_TRIGGERED (1)
 *   - If sensor released and motor_dir was 1: return BUMP_FRONT (0x0A)
 *   - If sensor released and motor_dir was 2: return BUMP_REAR (0x0B)
 *   - If sensor released and motor_dir was 0: return BUMP_NONE (0)
 *
 * @return  BUMP_NONE, BUMP_TRIGGERED, BUMP_FRONT, or BUMP_REAR
 *
 * Firmware: detectBumpCollision (0x6FD4)
 */
uint8_t bump_Detect(void);

/*
 * bump_SetMotorDir - Set motor error direction for bump classification
 *
 * Called by the wheel motor error checker when a motor stall or
 * overcurrent is detected. The direction determines whether the
 * next bump release is classified as front or rear.
 *
 * @param dir  BUMP_DIR_NONE (0), BUMP_DIR_LEFT (1), or BUMP_DIR_RIGHT (2)
 *
 * Firmware: written by checkWheelMotorErrors (0x60EC) to word_FFE940
 */
void bump_SetMotorDir(uint16_t dir);

/*
 * bump_SetPostState - Record post-bump recovery direction
 *
 * Called after a bump recovery maneuver to record which direction
 * the mower was moving. Used by subsequent motion planning.
 *
 * @param was_forward  1 = mower was going forward, 0 = was reversing
 *
 * Firmware: setPostBumpState (0x7542)
 */
void bump_SetPostState(uint8_t was_forward);

/*
 * bump_GetState - Get full bump detection state (for diagnostics)
 *
 * @param state  Pointer to bump_state_t to populate
 */
void bump_GetState(bump_state_t *state);

/*
 * bump_GetContact - Check raw contact flag
 *
 * Returns the raw contact flag set during active bump detection.
 *
 * @return  1 if contact sensed, 0 if clear
 *
 * Firmware: byte_FFE8BE
 */
uint8_t bump_GetContact(void);

/*
 * bump_GetReverseSpeed - Get recommended reverse speed for bump type
 *
 * Convenience function: maps a bump_Detect() return code to the
 * appropriate motor reverse speed for recovery.
 *
 * @param bump_code  Return value from bump_Detect()
 * @return  Motor speed: BUMP_SPEED_FRONT, BUMP_SPEED_REAR,
 *          BUMP_SPEED_SIDE, or 0 if no bump
 */
uint16_t bump_GetReverseSpeed(uint8_t bump_code);

#endif /* BUMP_H */
