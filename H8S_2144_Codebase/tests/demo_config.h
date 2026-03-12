/*
 * demo_config.h — Demo mode sequence configuration
 *
 * Defines the command types, step struct, and default demo sequence.
 * Edit the demo_sequence[] table to change what the demo does.
 * The runner (demo.c) reads this table and executes each step,
 * printing the current command on the LCD.
 *
 * Timing:
 *   seconds field = duration in whole seconds
 *   For turns, seconds=0 means auto-timed from angle via
 *   steer_EstimateTurnTicks().
 */

#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H

#include <stdint.h>
#include "../config.h"

/* ---- Auto-start delay — sourced from config.h ---- */
#define DEMO_AUTOSTART_SECONDS   AUTOSTART_SECONDS

/* ---- Command types ---- */
typedef enum {
    DEMO_CMD_END = 0,       /* End of sequence (sentinel)                    */
    DEMO_CMD_BLADE_ON,      /* param = speed 0-1000, seconds = run time      */
    DEMO_CMD_BLADE_OFF,     /* Blade off, seconds = pause after              */
    DEMO_CMD_FORWARD,       /* param = speed 0-255 (0=default), seconds      */
    DEMO_CMD_REVERSE,       /* param = speed 0-255 (0=default), seconds      */
    DEMO_CMD_TURN_LEFT,     /* param = angle degrees, seconds=0 = auto-timed */
    DEMO_CMD_TURN_RIGHT,    /* param = angle degrees, seconds=0 = auto-timed */
    DEMO_CMD_STOP,          /* Stop motors, seconds = pause duration          */
    DEMO_CMD_DELAY,         /* Do nothing, seconds = wait time               */
    DEMO_CMD_SHUTOFF,       /* Power down — end of demo                      */
} demo_cmd_t;

/* ---- Single step in the demo sequence ---- */
typedef struct {
    uint8_t  cmd;           /* demo_cmd_t command                            */
    uint16_t param;         /* speed (wheel 0-255, blade 0-1000) or angle°   */
    uint8_t  seconds;       /* duration in seconds (0 = auto for turns)      */
} demo_step_t;

/* ---- Default demo sequence ----
 *
 * Edit this table to change the demo behaviour.
 * Terminate with DEMO_CMD_END.
 */
static const demo_step_t demo_sequence[] = {
    /*  Command              Param   Sec   Description                       */
    { DEMO_CMD_BLADE_ON,      500,    5 }, /* Blade 50% for 5s               */
    { DEMO_CMD_BLADE_OFF,       0,    1 }, /* Blade off, 1s settle           */
    { DEMO_CMD_FORWARD,         0,    2 }, /* Forward 2s (default speed)     */
    { DEMO_CMD_STOP,            0,    1 }, /* Stop, 1s pause                 */
    { DEMO_CMD_REVERSE,         0,    2 }, /* Reverse 2s                     */
    { DEMO_CMD_STOP,            0,    1 }, /* Stop, 1s pause                 */
    { DEMO_CMD_TURN_LEFT,      90,    0 }, /* 90° left (auto-timed)          */
    { DEMO_CMD_STOP,            0,    1 }, /* Stop, 1s pause                 */
    { DEMO_CMD_TURN_RIGHT,     90,    0 }, /* 90° right (auto-timed)         */
    { DEMO_CMD_STOP,            0,    1 }, /* Stop, 1s pause                 */
    { DEMO_CMD_TURN_LEFT,     180,    0 }, /* 180° left (auto-timed)         */
    { DEMO_CMD_STOP,            0,    1 }, /* Stop, 1s pause                 */
    { DEMO_CMD_SHUTOFF,         0,    0 }, /* Power down                     */
    { DEMO_CMD_END,             0,    0 }, /* Sentinel                       */
};

#define DEMO_SEQUENCE_LEN  (sizeof(demo_sequence) / sizeof(demo_sequence[0]))

#endif /* DEMO_CONFIG_H */
