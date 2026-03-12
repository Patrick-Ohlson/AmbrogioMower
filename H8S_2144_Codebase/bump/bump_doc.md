# Bump / Lift Sensor Driver — bump

## Module Summary

Debounced digital input driver for the L200's bump/lift sensor on P6.0, with
collision classification using motor error direction. The driver polls the sensor,
applies debounce filtering, detects bump events, classifies them by direction
(front/rear/side), and provides recommended reverse speeds for recovery maneuvers.

| Property | Value |
|----------|-------|
| Sensor pin | P6.0 (Port 6, bit 0) — active high |
| Debounce method | Consecutive-high counter, rate-limited (~16 tick interval) |
| Trigger threshold | 4+ consecutive high readings (debounce_count >> 2 > 0) |
| Bump classification | Motor error direction: left=front, right=rear |
| Source docs | `STATES/state_MainMower.md`, `STATES/state_MotorControl.md` |

## Files

| File | Description |
|------|-------------|
| `bump.h` | Types, constants, API declarations |
| `bump.c` | Implementation (poll, debounce, detect, classify) |
| `bump_doc.md` | This file |
| `tests/test_bump.c` | Interactive test (menu key H) |

## How to Include

```c
#include "bump/bump.h"
```

```makefile
SRCS += bump/bump.c
```

### Dependencies

| Dependency | Required For |
|------------|-------------|
| `<stdint.h>` | Fixed-width types |
| `iodefine.h` | P6.DR register access |

## Architecture

### Debounce Pipeline

```
 P6.0 (raw pin)
   │
   ▼
 bump_Poll(tick)          Rate-limited: every 0x10 (16) ticks
   │                      Reads P6.DR.BYTE & 0x01
   │
   ├─ pin HIGH ──▶ debounce_count++
   │
   └─ pin LOW  ──▶ debounce_count = 0
                   │
                   ▼
 bump_IsTriggered()       Threshold: debounce_count >> 2 > 0
   │                      (equivalent to count >= 4)
   │
   ├─ count < 4 ──▶ return 0 (not triggered)
   │
   └─ count >= 4 ─▶ return 1 (triggered)
```

### Bump Detection State Machine

```
 bump_Detect()
   │
   ├─ Sensor TRIGGERED ────────────────────────┐
   │    │                                       │
   │    ├─ active == 0 (first detection)        │
   │    │    ├─ record trigger_tick              │
   │    │    ├─ set active = 1                   │
   │    │    ├─ set contact = 1                  │
   │    │    └─ return BUMP_TRIGGERED (1)        │
   │    │                                       │
   │    └─ active == 1 (still held)             │
   │         ├─ set contact = 1                  │
   │         └─ return BUMP_TRIGGERED (1)        │
   │                                            │
   └─ Sensor RELEASED ─────────────────────────┐
        │                                       │
        ├─ active == 0 (was idle)               │
        │    ├─ set contact = 0                  │
        │    └─ return BUMP_NONE (0)             │
        │                                       │
        └─ active == 1 (was bumping, now released)
             ├─ set active = 0                   │
             ├─ set contact = 0                  │
             ├─ classify by motor_dir:           │
             │    ├─ dir=1 (left)  → BUMP_FRONT (0x0A)
             │    ├─ dir=2 (right) → BUMP_REAR  (0x0B)
             │    └─ dir=0 (none)  → BUMP_NONE  (0)
             ├─ clear motor_dir = 0              │
             └─ return classified code           │
```

### Recovery Speed Selection

The main mowing loop uses the bump code to select reverse motor speed:

| Bump Code | Value | Reverse Speed | Decimal | Description |
|-----------|-------|---------------|---------|-------------|
| `BUMP_FRONT` | 0x0A | 0x01D5 | 469 | Moderate reverse (front collision) |
| `BUMP_REAR` | 0x0B | 0x02C0 | 704 | Fast reverse (rear collision) |
| `BUMP_TRIGGERED` | 0x01 | 0x00EA | 234 | Slow reverse (side/default) |
| `BUMP_NONE` | 0x00 | 0 | 0 | No recovery needed |

## API Reference

### `void bump_Init(void)`

Reset all bump detection state. Clears debounce counter, motor direction,
active flag, contact flag, and post-bump state. Call once at startup.

### `void bump_Poll(uint32_t sys_tick)`

Poll the bump/lift sensor with rate-limited debounce.

- Rate limit: minimum 0x10 (16) ticks between samples
- On high reading: increments consecutive-high counter
- On low reading: resets counter to zero

Call every main loop tick. The counter is consumed by `bump_IsTriggered()`.

Firmware: `pollLiftSensorPort6` (0x1528)

### `uint8_t bump_IsTriggered(void)`

Check if sensor is debounce-triggered.

Returns 1 if `debounce_count >= 4` (i.e., `debounce_count >> 2 > 0`).

Firmware: `sub_156C` (0x156C)

### `uint8_t bump_Detect(void)`

Run the full bump collision detection state machine. Returns a code
indicating bump type, used by the main mowing loop for recovery.

| Return | Meaning |
|--------|---------|
| `BUMP_NONE` (0) | No bump / sensor released with no motor error |
| `BUMP_TRIGGERED` (1) | Sensor currently triggered (bump in progress) |
| `BUMP_FRONT` (0x0A) | Front bump (left motor error direction) |
| `BUMP_REAR` (0x0B) | Rear bump (right motor error direction) |

Firmware: `detectBumpCollision` (0x6FD4)

### `void bump_SetMotorDir(uint16_t dir)`

Set the motor error direction used for bump classification.

Called by the wheel motor error checker when a stall or overcurrent
is detected. The direction is consumed (cleared to 0) when the bump
is released and classified.

| dir | Meaning | Maps to |
|-----|---------|---------|
| 0 | No error direction | BUMP_NONE |
| 1 | Left wheel error | BUMP_FRONT (0x0A) |
| 2 | Right wheel error | BUMP_REAR (0x0B) |

Firmware: written by `checkWheelMotorErrors` (0x60EC) to `word_FFE940`

### `void bump_SetPostState(uint8_t was_forward)`

Record the mower's travel direction after a bump recovery maneuver.

| was_forward | post_state | Meaning |
|-------------|------------|---------|
| 1 (true) | 2 | Was moving forward when bumped |
| 0 (false) | 3 | Was reversing when bumped |

Also records the current tick as the post-bump timestamp.

Firmware: `setPostBumpState` (0x7542) writes `byte_FFE97D` and `dword_FFE97E`

### `void bump_GetState(bump_state_t *state)`

Copy all internal bump detection state into a `bump_state_t` struct
for diagnostics or external inspection.

### `uint8_t bump_GetContact(void)`

Returns the raw contact flag: 1 = physical contact sensed (sensor
currently triggered or was triggered this cycle), 0 = clear.

Firmware: `byte_FFE8BE`

### `uint16_t bump_GetReverseSpeed(uint8_t bump_code)`

Convenience function: maps a `bump_Detect()` return code to the
motor reverse speed for recovery. Returns 0 for `BUMP_NONE`.

## RAM Variable Map

| Variable | RAM Address | Size | Description |
|----------|------------|------|-------------|
| `debounce_count` | 0xFFE858 | dword | Consecutive high readings from P6.0 |
| `last_poll_tick` | 0xFFE854 | dword | Tick of last debounce poll |
| `motor_dir` | 0xFFE940 | word | Motor error direction (0/1/2) |
| `active` | 0xFFE942 | word | Bump active flag (0/1) |
| `trigger_tick` | 0xFFE944 | dword | Tick when bump first triggered |
| `contact` | 0xFFE8BE | byte | Raw contact sensed flag |
| `post_state` | 0xFFE97D | byte | Post-bump recovery state (2/3) |
| `post_tick` | 0xFFE97E | dword | Tick when post-state was set |

## Firmware Traceability

| Our Function | Firmware Name | Address |
|-------------|---------------|---------|
| `bump_Init` | (startup clear) | — |
| `bump_Poll` | `pollLiftSensorPort6` | 0x1528 |
| `bump_IsTriggered` | `sub_156C` | 0x156C |
| `bump_Detect` | `detectBumpCollision` | 0x6FD4 |
| `bump_SetMotorDir` | (written by `checkWheelMotorErrors`) | 0x60EC |
| `bump_SetPostState` | `setPostBumpState` | 0x7542 |
| `bump_GetContact` | (reads `byte_FFE8BE`) | — |
| `bump_GetReverseSpeed` | (inline in main mower loop) | — |

## Not Included

| Feature | Reason | Location |
|---------|--------|----------|
| `checkWheelMotorErrors` | Motor control module — sets motor_dir externally | 0x60EC |
| Bump recovery maneuver | Application-level motor control logic | Main mower state machine |
| Tilt/lift angle detection | Separate accelerometer-based system | accel module |
| Motor current monitoring | Motor control subsystem | state_MotorControl |

## Usage Example

```c
#include "bump/bump.h"

/* In startup */
bump_Init();

/* In main loop (called every tick) */
void main_loop(uint32_t tick)
{
    /* Poll sensor with debounce */
    bump_Poll(tick);

    /* Check for bump event */
    uint8_t bump = bump_Detect();

    if (bump == BUMP_TRIGGERED) {
        /* Bump in progress — sensor still held. Wait for release. */
    }
    else if (bump == BUMP_FRONT || bump == BUMP_REAR) {
        /* Bump released — execute recovery maneuver */
        uint16_t speed = bump_GetReverseSpeed(bump);
        motor_SetReverse(speed);    /* (motor module call) */

        /* Record which direction we were going */
        bump_SetPostState(bump == BUMP_FRONT ? 1 : 0);
    }
}

/* Called by wheel motor error checker */
void on_motor_error(uint8_t which_wheel)
{
    /* which_wheel: 1=left, 2=right */
    bump_SetMotorDir(which_wheel);
}
```
