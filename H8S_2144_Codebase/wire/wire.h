/*
 * wire.h - Boundary Wire Signal Detection Driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_WireFollow.md
 *
 * The L200 uses an inductive sensor to detect a buried boundary wire.
 * The sensor is controlled via P2.2 (enable/read toggle) and returns
 * a 2-bit signal code read from a hardware register, decoded into
 * ASCII status bytes H/I/N/O indicating left/right wire detection.
 *
 * Signal detection is a 3-phase process called once per main loop tick:
 *   Phase 0: Enable detector (P2.2 = 1)
 *   Phase 1: Trigger read    (P2.2 = 0)
 *   Phase 2: Decode result   (read signal register, set flags)
 *
 * Original firmware functions (unpatched _X versions preferred):
 *   CheckSignal_X          (0x46E0)  -> wire_CheckSync() [startup sync]
 *   GetSignalByte_X        (0x46CA)  -> (internal) [reads P2.DR bits 0-1]
 *   CheckSignalState       (0x1C45E) -> wire_Poll() [no _X version exists]
 *   getSignalQuality       (0x9FAE)  -> wire_GetQuality()
 *   updateBorderSignalStats(0x42FE)  -> wire_UpdateStats()
 *
 * Patched vs unpatched:
 *   GetSignalByte  (0x1C444) — patched, reads RAM 0xFFEC90 (static 0x4E)
 *   GetSignalByte_X(0x46CA)  — unpatched, reads P2.DR (real hardware)
 *   CheckSignal    (0x1C458) — patched stub, always returns 1
 *   CheckSignal_X  (0x46E0)  — unpatched, 2-pass sync verify
 */

#ifndef WIRE_H
#define WIRE_H

#include <stdint.h>

/* ====================================================================
 * Signal Byte Constants
 *
 * GetSignalByte_X() reads P2.DR bits 0-1 and indexes into a lookup
 * table at ROM 0x18A59: "ONIH"
 *
 *   Index 0 → 'O' (0x4F) — No signal, left sensor
 *   Index 1 → 'N' (0x4E) — No signal, right sensor
 *   Index 2 → 'I' (0x49) — Signal detected, left sensor
 *   Index 3 → 'H' (0x48) — Signal detected, right sensor
 *
 * Firmware: GetSignalByte_X (0x46CA) reads P2.DR (0xFFFFB3)
 *   The patched GetSignalByte (0x1C444) reads RAM 0xFFEC90 instead.
 * ==================================================================== */

#define WIRE_SIG_H      0x48    /* 'H' — wire detected by right sensor */
#define WIRE_SIG_I      0x49    /* 'I' — wire detected by left sensor */
#define WIRE_SIG_N      0x4E    /* 'N' — no signal, right sensor */
#define WIRE_SIG_O      0x4F    /* 'O' — no signal, left sensor */

/* ====================================================================
 * Signal Flag Bits
 *
 * The signal detection state machine sets 4 boolean flags per cycle.
 * These are used by the wire follow state machine and border detection.
 *
 * Firmware RAM locations:
 *   bSIG1 = 0xFFE956 — Right sensor signal present
 *   bSIG2 = 0xFFE957 — Combined signal status (bump/boundary)
 *   bSIG3 = 0xFFE958 — Left sensor signal present
 *   bSIG4 = 0xFFE959 — Signal detector enabled / sync flag
 *
 * In CheckSignalState:
 *   'H' (0x48) → SIG1=1,             SIG2=0
 *   'I' (0x49) → SIG3=1,   SIG2=1,   SIG1=0
 *   'N' (0x4E) → SIG1=0,             SIG2=0
 *   'O' (0x4F) → SIG3=0,   SIG2=1,   SIG1=0
 *
 * Firmware: CheckSignalState (0x1C45E)
 * ==================================================================== */

/* Bit positions for wire_flags_t packed representation */
#define WIRE_FLAG_SIG1      0x01    /* Right sensor: wire detected */
#define WIRE_FLAG_SIG2      0x02    /* Combined: boundary/bump indicator */
#define WIRE_FLAG_SIG3      0x04    /* Left sensor: wire detected */
#define WIRE_FLAG_SIG4      0x08    /* Detector enabled / sync active */

/* ====================================================================
 * Border Signal Statistics
 *
 * updateBorderSignalStats() samples the signal 100 times, counting
 * how many indicate "inside boundary". If charge zone tracking is
 * enabled (featureFlags bit 1) and quality < 95/100, the mower
 * considers itself potentially outside the boundary.
 *
 * Firmware: updateBorderSignalStats (0x42FE)
 * ==================================================================== */

#define WIRE_STATS_WINDOW   100     /* Samples before quality update */
#define WIRE_STATS_THRESHOLD 0x5F   /* 95 — boundary quality threshold */

/* Signal quality value indicating no signal / not initialised */
#define WIRE_QUALITY_NONE   0xFF

/* Minimum signal strength for wire follow (must exceed this) */
#define WIRE_STRENGTH_MIN   0x81

/* ====================================================================
 * Sync Check Timing Constants
 *
 * CheckSignal_X (0x46E0) uses:
 *   fWaitUntil(0x70) = 112 ticks settle time (~1.12s at 10ms/tick)
 *   counter2 loop for 7 ticks verify window (~70ms)
 *
 * Firmware: CheckSignal_X (0x46E0)
 * ==================================================================== */

#define WIRE_SYNC_SETTLE    0x70    /* 112 ticks — detector settle time */
#define WIRE_SYNC_VERIFY    7       /* 7 ticks — signal verify window */

/* ====================================================================
 * Types
 * ==================================================================== */

/* Wire signal flags — result of one poll cycle */
typedef struct {
    uint8_t sig1;       /* Right sensor: 1 = wire detected, 0 = no signal */
    uint8_t sig2;       /* Combined boundary indicator */
    uint8_t sig3;       /* Left sensor: 1 = wire detected, 0 = no signal */
    uint8_t sig4;       /* Detector enabled / sync flag */
    uint8_t raw_byte;   /* Raw signal byte (H/I/N/O) from last decode */
} wire_flags_t;

/* Border signal statistics */
typedef struct {
    uint8_t quality;        /* Signal quality (0-100): hits in last 100 samples
                             * 0xFF = not yet computed (need 100 samples first) */
    uint8_t sample_count;   /* Current sample count (0-99, resets at 100) */
    uint8_t hit_count;      /* Signal-present count within current window */
    uint8_t outside;        /* 1 = likely outside boundary (quality < threshold) */
} wire_stats_t;

/* ====================================================================
 * Public API
 * ==================================================================== */

/*
 * wire_Init - Reset signal detection state machine
 *
 * Clears all signal flags, resets the 3-phase state machine,
 * and initialises border statistics.
 *
 * Call once at startup before entering the main loop.
 */
void wire_Init(void);

/*
 * wire_CheckSync - Verify wire signal detector hardware
 *
 * Performs a 2-pass sync check at startup. Each pass:
 *   1. Enables detector (P2.2 = 1), waits WIRE_SYNC_SETTLE ticks
 *   2. Polls signal for WIRE_SYNC_VERIFY ticks — expects all 'H'
 *   3. Disables detector (P2.2 = 0), waits WIRE_SYNC_SETTLE ticks
 *   4. Polls signal for WIRE_SYNC_VERIFY ticks — expects all 'N'
 *
 * If pass 1 succeeds, returns immediately. If it fails, tries pass 2.
 * Called by MainMower at startup; failure → "SYNC ERROR" stop.
 *
 * BLOCKING: takes ~2.4 seconds (2 × 112 tick settle per pass).
 *
 * @return  1 = sync OK, 0 = both passes failed
 *
 * Firmware: CheckSignal_X (0x46E0) — the unpatched version.
 *   The patched CheckSignal (0x1C458) is a stub returning 1.
 */
uint8_t wire_CheckSync(void);

/*
 * wire_Poll - Run one cycle of the 3-phase signal detection
 *
 * Must be called once per main loop tick (from DoMotorState_AndSignal).
 * Each call advances the state machine by one phase:
 *
 *   Phase 0 → Phase 1: Set P2.2 high (enable detector), clear flags
 *   Phase 1 → Phase 2: Set P2.2 low (trigger read)
 *   Phase 2 → Phase 0: Read signal byte, decode into flags
 *
 * After phase 2 completes, the signal flags are valid and can be
 * read with wire_GetFlags().
 *
 * Firmware: CheckSignalState (0x1C45E)
 */
void wire_Poll(void);

/*
 * wire_GetFlags - Get current signal flags
 *
 * Returns the signal flags from the most recent completed poll cycle.
 *
 * @param flags  Pointer to wire_flags_t to populate
 */
void wire_GetFlags(wire_flags_t *flags);

/*
 * wire_GetRawByte - Get raw signal byte from last decode
 *
 * Returns: WIRE_SIG_H, WIRE_SIG_I, WIRE_SIG_N, or WIRE_SIG_O
 *          0 if no decode has occurred yet.
 */
uint8_t wire_GetRawByte(void);

/*
 * wire_GetQuality - Get border signal quality metric
 *
 * Returns the signal quality value used by the main mowing loop
 * for signal loss detection. Values > 300 trigger recovery mode.
 *
 * Note: In the original firmware this reads wMotorControlSpeed
 * (0xFFE082) which is computed elsewhere. This function provides
 * the interface; the actual value depends on motor control state.
 *
 * @return  Signal quality (0 = good, higher = worse)
 *
 * Firmware: getSignalQuality (0x9FAE)
 */
uint16_t wire_GetQuality(void);

/*
 * wire_UpdateStats - Update border signal statistics
 *
 * Call periodically (each main loop tick, gated by tick counter).
 * Samples the current signal state and accumulates statistics over
 * a 100-sample window.
 *
 * After 100 samples:
 *   - quality is updated (0-100, count of "inside" samples)
 *   - outside flag is set if quality < WIRE_STATS_THRESHOLD (95)
 *     and charge zone tracking is enabled (featureFlags bit 1)
 *   - counters reset for next window
 *
 * @param zone_tracking  1 = charge zone tracking enabled (featureFlags bit 1),
 *                       0 = disabled (outside flag always cleared)
 * @param tick           Current system tick counter (for interval gating)
 *
 * Firmware: updateBorderSignalStats (0x42FE)
 */
void wire_UpdateStats(uint8_t zone_tracking, uint32_t tick);

/*
 * wire_GetStats - Get current border signal statistics
 *
 * @param stats  Pointer to wire_stats_t to populate
 */
void wire_GetStats(wire_stats_t *stats);

/*
 * wire_IsOutside - Check if mower is likely outside boundary
 *
 * Convenience function: returns the outside flag from statistics.
 *
 * @return  1 = likely outside boundary, 0 = inside or not enough data
 */
uint8_t wire_IsOutside(void);

#endif /* WIRE_H */
