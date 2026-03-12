# Boundary Wire Signal Detection — wire

## Module Summary

Low-level driver for the L200's buried boundary wire inductive sensor.
Provides a 3-phase polled state machine that controls the P2.2 detection
pin, reads the signal hardware register, and decodes the result into
left/right signal flags. Also tracks border signal quality statistics
over a 100-sample sliding window.

| Property | Value |
|----------|-------|
| Sensor type | Inductive (buried boundary wire) |
| Control pin | P2.2 (Port 2, bit 2) — toggle enable/read |
| Signal register | byte_FFEC90 (2-bit index into lookup table) |
| Signal codes | H=right detect, I=left detect, N=no right, O=no left |
| Poll rate | Once per main loop tick (3 ticks per full cycle) |
| Stats window | 100 samples, gated by 0x45C tick minimum interval |
| Source doc | `STATES/state_WireFollow.md`, `STATES/state_MainMower.md` |

## Files

| File | Description |
|------|-------------|
| `wire.h` | Public API header with signal constants and types |
| `wire.c` | Implementation (detection state machine, statistics) |
| `wire_doc.md` | This file |
| `tests/test_wire.c` | Interactive test (menu key G) |

## How to Include

```c
#include "wire/wire.h"
```

```makefile
SRCS += wire/wire.c
```

### Dependencies

| Dependency | Required For |
|------------|-------------|
| `<stdint.h>` | Fixed-width types |
| `iodefine.h` | P2 register access (`P2.DR.BYTE`) |
| `motor/motor.h` | `shadow_p2_dr` — shared P2 shadow register |

**Important**: P2.2 is shared with the blade motor driver (MOTOR_P2_BLADE_IN1 = 0x04).
All P2 writes **must** go through `shadow_p2_dr` to avoid EMI corruption of motor
direction bits (P2.5, P2.6). The wire module sets/clears bit 2 via the shadow, then
writes the full shadow byte to `P2.DR.BYTE`.

## Signal Detection Principle

The L200 uses an inductive sensor to detect the AC signal in a buried
perimeter wire. The sensor has two channels (left and right) to determine
which side of the wire the mower is on.

### Hardware Interface

```
P2.2 ──── Signal detector enable/trigger
           1 = enable detector (charge sensing coil)
           0 = trigger read (latch result)

byte_FFEC90 ── Signal result register (read-only)
               2-bit value decoded via lookup table:
               0 → 'O' (no signal, left)
               1 → 'N' (no signal, right)
               2 → 'I' (detected, left)
               3 → 'H' (detected, right)
```

### 3-Phase State Machine

```
 Phase 0              Phase 1              Phase 2
┌─────────┐         ┌─────────┐         ┌──────────┐
│ P2.2 = 1│────────▶│ P2.2 = 0│────────▶│ Read reg │──┐
│ Enable  │ 1 tick  │ Trigger │ 1 tick  │ Decode   │  │
│ Clear   │         │ read    │         │ Set flags│  │
│ flags   │         │         │         │          │  │
└─────────┘         └─────────┘         └──────────┘  │
     ▲                                                 │
     └─────────────────────────────────────────────────┘
                         1 tick
```

Total cycle: 3 main-loop ticks for one complete signal reading.

## API Reference

### Types

```c
/* Signal flags from one poll cycle */
typedef struct {
    uint8_t sig1;       /* Right sensor: 1 = wire detected */
    uint8_t sig2;       /* Combined boundary indicator */
    uint8_t sig3;       /* Left sensor: 1 = wire detected */
    uint8_t sig4;       /* Detector sync/enabled flag */
    uint8_t raw_byte;   /* Raw signal byte: H/I/N/O */
} wire_flags_t;

/* Border signal statistics */
typedef struct {
    uint8_t quality;        /* 0-100 signal quality (0xFF = not yet computed) */
    uint8_t sample_count;   /* Current sample counter (0-99) */
    uint8_t hit_count;      /* Current window hit counter */
    uint8_t outside;        /* 1 = likely outside boundary */
} wire_stats_t;
```

### Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `WIRE_SIG_H` | 0x48 | Right sensor detects wire |
| `WIRE_SIG_I` | 0x49 | Left sensor detects wire |
| `WIRE_SIG_N` | 0x4E | No signal, right sensor |
| `WIRE_SIG_O` | 0x4F | No signal, left sensor |
| `WIRE_FLAG_SIG1` | 0x01 | Right sensor flag bit |
| `WIRE_FLAG_SIG2` | 0x02 | Boundary indicator bit |
| `WIRE_FLAG_SIG3` | 0x04 | Left sensor flag bit |
| `WIRE_FLAG_SIG4` | 0x08 | Sync/enabled flag bit |
| `WIRE_QUALITY_NONE` | 0xFF | Quality not yet computed |
| `WIRE_STRENGTH_MIN` | 0x81 | Min signal for wire follow |

### `void wire_Init(void)`

Reset signal detection state machine and clear all flags and statistics.
Call once at startup.

### `void wire_Poll(void)`

Run one phase of the 3-phase signal detection. Call once per main loop tick.
After 3 consecutive calls, the signal flags are updated with fresh data.

Firmware: `CheckSignalState` (0x1C45E)

### `void wire_GetFlags(wire_flags_t *flags)`

Read the current signal flags from the last completed decode cycle.

### `uint8_t wire_GetRawByte(void)`

Get the raw signal byte (H/I/N/O) from the last decode. Returns 0 if
no decode has occurred yet.

### `uint16_t wire_GetQuality(void)`

Get the signal quality metric used by the main mowing loop. Values > 300
trigger signal loss recovery in `MainMower`.

**Note**: In the original firmware this reads `wMotorControlSpeed` which is
computed by the motor control subsystem. This driver provides the interface;
the actual metric depends on integration with the motor control module.

Firmware: `getSignalQuality` (0x9FAE)

### `void wire_UpdateStats(uint8_t zone_tracking, uint32_t tick)`

Accumulate border signal statistics. Call each main loop tick.

- Samples are rate-limited to minimum 0x45C tick intervals
- After 100 samples, quality and outside-boundary flag are updated
- `zone_tracking` parameter: pass 1 if config `featureFlags` bit 1 is set

Firmware: `updateBorderSignalStats` (0x42FE)

### `void wire_GetStats(wire_stats_t *stats)` / `uint8_t wire_IsOutside(void)`

Read statistics or just the outside-boundary flag.

## Signal Decode Truth Table

| Signal Byte | ASCII | SIG1 (right) | SIG2 (boundary) | SIG3 (left) |
|-------------|-------|:------------:|:----------------:|:-----------:|
| 0x48 | 'H' | 1 | 0 | — |
| 0x49 | 'I' | 0 | 1 | 1 |
| 0x4E | 'N' | 0 | 0 | — |
| 0x4F | 'O' | 0 | 1 | 0 |

## RAM Variables (Firmware)

| Address | Ghidra Name | Size | Description |
|---------|-------------|------|-------------|
| 0xFFE956 | bSigRight | byte | Right sensor flag |
| 0xFFE957 | bSigBoundary | byte | Boundary indicator |
| 0xFFE958 | bSigLeft | byte | Left sensor flag |
| 0xFFE959 | bSigSync | byte | Sync/enable flag |
| 0xFFE97C | bSignalPhase | byte | State machine phase (0/1/2) |
| 0xFFE8A4 | bBorderSampleCount | byte | Border stats sample counter |
| 0xFFE8A5 | bBorderMissCount | byte | Border stats miss counter |
| 0xFFE89E | bBorderQualityFlag | byte | Border quality (0=good, 1=bad) |
| 0xFFE8A0 | dwBorderLastSampleTick | dword | Border stats last sample timestamp |
| 0xFFEC10 | wSignalStrength | word | Composite signal quality |
| 0xFFEC90 | (sig reg) | byte | Signal hardware register |

## Firmware Traceability

| Our Function | Firmware Name | Address |
|-------------|---------------|---------|
| `wire_Init` | (startup init) | Various |
| `wire_Poll` | `CheckSignalState` | 0x1C45E |
| (internal) `get_signal_byte` | `GetSignalByte` | 0x1C444 |
| `wire_GetQuality` | `getSignalQuality` | 0x9FAE |
| `wire_UpdateStats` | `updateBorderSignalStats` | 0x42FE |
| — | `doPeriodicSignalTick` | 0x1C408 |
| — | `checkHSMSignalState` | 0x14B6 |
| — | `borderDetectPulse` | 0x7076 |
| — | `borderPulse_jumpTable` (6 cases) | 0x18D44 |

## Not Included

| Feature | Reason | Location |
|---------|--------|----------|
| Wire follow state machine | Application-level logic, uses active objects | `wireFollowStateMachine` (0x5EBE) |
| borderDetectPulse | 6-phase P2.7 pulse generator, uses assembly jump table | `borderDetectPulse` (0x7076), jump table `borderPulse_jumpTable` (0x18D44), cases: `borderPulse_case0_init`–`borderPulse_case5_gap2`. See `state_WireFollow.md` |
| checkBorderStatus / borderRecovery | Higher-level recovery logic | 0x5DDE / 0x5CAE |
| Motor control speed (quality metric) | Separate motor module | `wMotorControlSpeed` |
| Active object event posting | AO framework dependency | `ao_post_wire_event_u8` |
| SetNoBorder / no-border mode | Application-level mode setting | 0x1C5A2 |

## Usage Example

```c
#include "wire/wire.h"

wire_flags_t wflags;
wire_stats_t wstats;

void startup(void)
{
    wire_Init();
}

void main_loop_tick(uint32_t tick)
{
    /* Poll signal detection (call every tick) */
    wire_Poll();

    /* Read current flags */
    wire_GetFlags(&wflags);

    if (wflags.sig1) {
        /* Right sensor detects boundary wire */
    }
    if (wflags.sig3) {
        /* Left sensor detects boundary wire */
    }

    /* Update statistics (zone_tracking from config) */
    wire_UpdateStats(cfg.feature_flags & CFG_FEAT_ZONE_TRACK ? 1 : 0, tick);

    /* Check if outside boundary */
    if (wire_IsOutside()) {
        /* Signal quality degraded — may be outside boundary */
    }
}
```
