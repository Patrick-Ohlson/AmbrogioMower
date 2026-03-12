/*
 * wire.c - Boundary Wire Signal Detection Driver
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_WireFollow.md
 *
 * The boundary wire system uses an inductive sensor controlled by P2.2.
 * Signal detection is a 3-phase state machine polled from the main loop:
 *
 *   Phase 0: Enable detector    — P2.DR |= 0x04, clear flags, advance to 1
 *   Phase 1: Trigger read       — P2.DR &= ~0x04, advance to 2
 *   Phase 2: Decode signal byte — read hardware register, set SIG flags
 *
 * The signal hardware register (byte_FFEC90) provides a 2-bit index
 * into a lookup table "ONIH" yielding ASCII codes:
 *   'H' (0x48) = wire detected, right sensor
 *   'I' (0x49) = wire detected, left sensor
 *   'N' (0x4E) = no signal, right sensor
 *   'O' (0x4F) = no signal, left sensor
 *
 * Original firmware functions (unpatched _X versions):
 *   CheckSignal_X          (0x46E0) — startup sync (2-pass verify)
 *   GetSignalByte_X        (0x46CA) — read P2.DR bits 0-1, lookup ONIH
 *   CheckSignalState       (0x1C45E) — 3-phase poll (patched, no _X exists)
 *   getSignalQuality       (0x9FAE)
 *   updateBorderSignalStats(0x42FE)
 *
 * Related (not implemented here — application-level):
 *   borderDetectPulse      (0x7076)  — 6-phase P2.7 pulse state machine
 *     Jump table: borderPulse_jumpTable (0x18D44), 6 cases:
 *       borderPulse_case0_init   (0x70C8)  Init: clear P2.7
 *       borderPulse_case1_rest   (0x70DA)  Rest: wait 310 ticks
 *       borderPulse_case2_pulse1 (0x711E)  Pulse 1: set P2.7, 5 ticks
 *       borderPulse_case3_gap1   (0x715C)  Gap 1: clear P2.7, 7 ticks
 *       borderPulse_case4_pulse2 (0x719E)  Pulse 2: set P2.7, 10 ticks
 *       borderPulse_case5_gap2   (0x71D8)  Gap 2: clear P2.7, 7 ticks
 */

#include "wire.h"
#include "../iodefine.h"
#include "../motor/motor.h"     /* shadow_p2_dr — P2 shared with blade motor */
#include "../timer/timer.h"     /* GetSystemCounter */

/* ---- Hardware defines ---- */

/* P2.2 controls the boundary wire signal detector.
 * Set high to enable, set low to trigger a read.
 * Firmware: P2_DR bit2 toggle in CheckSignalState (0x1C45E) */
#define WIRE_DETECT_BIT     0x04    /* P2.DR bit 2 mask */

/* Signal byte lookup table: index 0-3 → 'O','N','I','H'
 * Firmware: ROM table at 0x18A59 */
static const uint8_t sig_lookup[4] = { 'O', 'N', 'I', 'H' };

/* Minimum tick interval between border signal samples.
 * Firmware: updateBorderSignalStats checks (counter2 - dword_FFE8A0) > 0x45C
 * 0x45C = 1116 ticks */
#define WIRE_STATS_INTERVAL 0x045C

/* ---- Module state ---- */

/* 3-phase state machine: 0 = idle/enable, 1 = triggered, 2 = decode */
static uint8_t poll_phase = 0;

/* Signal flags from last completed decode cycle
 * Firmware: bSIG1 (0xFFE956), bSIG2 (0xFFE957),
 *           bSIG3 (0xFFE958), bSIG4 (0xFFE959) */
static uint8_t sig1 = 0;       /* Right sensor signal present */
static uint8_t sig2 = 0;       /* Combined boundary indicator */
static uint8_t sig3 = 0;       /* Left sensor signal present */
static uint8_t sig4 = 0;       /* Detector enabled / sync flag */

/* Raw signal byte from last GetSignalByte decode */
static uint8_t last_sig_byte = 0;

/* Border signal quality statistics
 * Firmware: MemStart (quality), byte_FFE8A4 (sample count),
 *           byte_FFE8A5 (hit count), byte_FFE89E (outside flag) */
static uint8_t stat_quality = WIRE_QUALITY_NONE;
static uint8_t stat_sample_count = 0;
static uint8_t stat_hit_count = 0;
static uint8_t stat_outside = 0;

/* Timestamp of last statistics sample
 * Firmware: dword_FFE8A0 */
static uint32_t stat_last_tick = 0;

/* ---- Internal: GetSignalByte ----
 *
 * Read P2.DR bits 0-1 and convert to ASCII signal byte via lookup.
 *
 * Firmware: GetSignalByte_X (0x46CA) — the UNPATCHED version.
 *   Reads P2.DR (0xFFFFB3), masks with 0x03, indexes into ROM "ONIH"
 *   table at 0x18A59.  Returns 'O','N','I', or 'H'.
 *
 *   The patched GetSignalByte (0x1C444) reads from RAM byte_FFEC90
 *   instead — Ghidra decompiles it as "return 0x4E" because the RAM
 *   value was constant during static analysis.  We use the unpatched
 *   version which reads the real P2 hardware register.
 */
static uint8_t get_signal_byte(void)
{
    uint8_t idx = P2.DR.BYTE & 0x03;
    return sig_lookup[idx];
}

/* ====================================================================
 * Public API
 * ==================================================================== */

void wire_Init(void)
{
    /* Reset state machine */
    poll_phase = 0;

    /* Clear all signal flags */
    sig1 = 0;
    sig2 = 0;
    sig3 = 0;
    sig4 = 0;
    last_sig_byte = 0;

    /* Reset statistics */
    stat_quality = WIRE_QUALITY_NONE;
    stat_sample_count = 0;
    stat_hit_count = 0;
    stat_outside = 0;
    stat_last_tick = 0;

    /* Configure P2.0-P2.1 as inputs for wire signal detection.
     *
     * P2.DDR bits 0-1 must be 0 (input) so the wire detector's
     * output can be read on these pins.  hwinit/motor_Init set
     * P2.DDR = 0xE7 (bits 0,1 = output) which blocks the signal.
     *
     * The original firmware's emergency stop (fDoPorts1, 0x12BA8)
     * does the same: byte_FFE094 &= 0xFC → P2.DDR (clears bits 0,1).
     * This confirms P2.0-P2.1 must be inputs for wire signal reading.
     */
    shadow_p2_ddr &= (uint8_t)~0x03;   /* Clear DDR bits 0,1 → input */
    P2.DDR = shadow_p2_ddr;
}

/*
 * wire_CheckSync - Verify wire signal detector is working
 *
 * Firmware: CheckSignal_X (0x46E0) — the UNPATCHED version.
 *   The patched CheckSignal (0x1C458) is a stub that always returns 1.
 *
 * Protocol (2-pass):
 *   1. Enable detector (P2.2 = 1), wait settle_ticks
 *      Poll for verify_ticks: expect ALL reads = 'H' (wire detected)
 *   2. Disable detector (P2.2 = 0), wait settle_ticks
 *      Poll for verify_ticks: expect ALL reads = 'N' (no signal)
 *   3. If pass 1 fails, try once more (pass 2)
 *
 * Original uses fWaitUntil(0x70) for settle (112 ticks = ~1.12s)
 * and polls counter2 for 7 ticks (~70ms) during verify.
 *
 * Returns 1 if sync succeeded, 0 if both passes failed.
 */
uint8_t wire_CheckSync(void)
{
    uint8_t pass;
    uint8_t ok;
    uint32_t start;

    for (pass = 0; pass < 2; pass++) {
        ok = 1;

        /* Phase A: Enable detector — P2.2 high, wait 0x70 ticks */
        shadow_p2_dr |= WIRE_DETECT_BIT;
        P2.DR.BYTE = shadow_p2_dr;

        start = GetSystemCounter();
        while ((GetSystemCounter() - start) < WIRE_SYNC_SETTLE)
            ;

        /* Verify: poll for WIRE_SYNC_VERIFY ticks, expect all 'H' */
        start = GetSystemCounter();
        while ((GetSystemCounter() - start) < WIRE_SYNC_VERIFY) {
            if (get_signal_byte() != WIRE_SIG_H) {
                ok = 0;
            }
        }

        /* Phase B: Disable detector — P2.2 low, wait 0x70 ticks */
        shadow_p2_dr &= (uint8_t)~WIRE_DETECT_BIT;
        P2.DR.BYTE = shadow_p2_dr;

        start = GetSystemCounter();
        while ((GetSystemCounter() - start) < WIRE_SYNC_SETTLE)
            ;

        /* Verify: poll for WIRE_SYNC_VERIFY ticks, expect all 'N' */
        start = GetSystemCounter();
        while ((GetSystemCounter() - start) < WIRE_SYNC_VERIFY) {
            if (get_signal_byte() != WIRE_SIG_N) {
                ok = 0;
            }
        }

        /* Pass 1 succeeded — return immediately */
        if (pass == 0 && ok) {
            return 1;
        }
    }

    /* Return pass 2 result */
    return ok;
}

/*
 * wire_Poll - 3-phase signal detection state machine
 *
 * Firmware: CheckSignalState (0x1C45E / no _X version exists)
 *
 * Phase 0 (enable):
 *   - P2.DR |= 0x04 (set P2.2 high — enable detector)
 *   - Clear SIG1-SIG4 flags
 *   - Advance to phase 1
 *
 * Phase 1 (trigger):
 *   - P2.DR &= ~0x04 (set P2.2 low — trigger read)
 *   - Advance to phase 2
 *
 * Phase 2 (decode):
 *   - Call get_signal_byte() to read hardware
 *   - Decode into SIG flags based on the ASCII byte:
 *     'H' → SIG1=1
 *     'I' → SIG3=1, SIG2=1, SIG1=0
 *     'N' → SIG1=0, SIG2=0
 *     'O' → SIG3=0, SIG2=1, SIG1=0
 *   - Reset to phase 0
 */
void wire_Poll(void)
{
    uint8_t sig_byte;

    switch (poll_phase) {

    case 0:
        /* Phase 0: Enable detector — set P2.2 high
         * Must use shadow_p2_dr because P2 is shared with motor direction
         * pins (P2.5 right wheel, P2.6 left wheel, P2.2 blade IN1).
         * Direct read-modify-write risks EMI-corrupted reads. */
        shadow_p2_dr |= WIRE_DETECT_BIT;
        P2.DR.BYTE = shadow_p2_dr;

        /* Clear all flags for fresh cycle */
        sig4 = 0;
        sig1 = 0;
        sig2 = 0;
        sig3 = 0;

        poll_phase = 1;
        break;

    case 1:
        /* Phase 1: Trigger read — set P2.2 low */
        shadow_p2_dr &= (uint8_t)~WIRE_DETECT_BIT;
        P2.DR.BYTE = shadow_p2_dr;

        poll_phase = 2;
        break;

    case 2:
        /* Phase 2: Decode signal byte into flags */
        sig_byte = get_signal_byte();
        last_sig_byte = sig_byte;

        switch (sig_byte) {
        case WIRE_SIG_H:
            /* 'H' — Right sensor detects wire */
            sig1 = 1;
            sig2 = 0;
            break;

        case WIRE_SIG_I:
            /* 'I' — Left sensor detects wire */
            sig3 = 1;
            sig2 = 1;
            sig1 = 0;
            break;

        case WIRE_SIG_N:
            /* 'N' — No signal, right sensor */
            sig1 = 0;
            sig2 = 0;
            break;

        case WIRE_SIG_O:
            /* 'O' — No signal, left sensor */
            sig3 = 0;
            sig2 = 1;
            sig1 = 0;
            break;

        default:
            /* Unknown signal byte — treat as no signal */
            sig1 = 0;
            sig2 = 0;
            sig3 = 0;
            break;
        }

        /* Cycle complete — return to phase 0 */
        poll_phase = 0;
        break;

    default:
        /* Invalid state — reset */
        poll_phase = 0;
        break;
    }
}

void wire_GetFlags(wire_flags_t *flags)
{
    flags->sig1 = sig1;
    flags->sig2 = sig2;
    flags->sig3 = sig3;
    flags->sig4 = sig4;
    flags->raw_byte = last_sig_byte;
}

uint8_t wire_GetRawByte(void)
{
    return last_sig_byte;
}

uint16_t wire_GetQuality(void)
{
    /* In the original firmware, getSignalQuality() (0x9FAE) returns
     * wMotorControlSpeed (0xFFE082). This value is computed by the
     * motor control subsystem and reflects signal quality indirectly.
     *
     * For this driver module, we expose the interface but the actual
     * quality metric depends on the motor control state which is
     * outside this module's scope.
     *
     * TODO: Wire this up to the motor control module when available.
     * For now, return 0 (good signal) as a safe default. */
    return 0;
}

/*
 * wire_UpdateStats - Accumulate border signal statistics
 *
 * Firmware: updateBorderSignalStats (0x42FE)
 *
 * Called each main loop tick. Gated by a minimum interval (0x45C ticks)
 * between samples. After 100 samples:
 *   - quality = number of "signal present" hits out of 100
 *   - If zone_tracking is enabled and quality < 95 → outside = 1
 *   - Counters reset for next 100-sample window
 *
 * The firmware uses tm_sub_14B6() to check the signal state. That
 * function returns 0 = no signal, 2 = signal present (via sensor
 * comparison). We approximate this by checking sig1 (right sensor
 * detected wire). If sig1 was NOT set to 2 (the "confirmed" value),
 * the hit counter increments.
 */
void wire_UpdateStats(uint8_t zone_tracking, uint32_t tick)
{
    uint8_t hits;

    /* Skip if quality not yet requested (0xFF = uninitialised) */
    if (stat_quality == WIRE_QUALITY_NONE && stat_sample_count == 0) {
        /* First call — start accumulating. Set quality to 0xFF
         * to indicate "collecting samples" */
    }

    /* Rate-limit: minimum 0x45C ticks between samples */
    if ((tick - stat_last_tick) <= WIRE_STATS_INTERVAL)
        return;

    stat_last_tick = tick;

    /* Sample: check if wire is currently detected.
     * Firmware checks tm_sub_14B6() return value — if != 2,
     * it counts as a "miss" (increment hit counter for non-detections).
     * The logic is inverted: hit_count counts non-signal samples,
     * and quality represents "boundary confidence". */
    if (sig1 == 0 && sig3 == 0) {
        /* No wire signal on either sensor — count as miss */
        stat_hit_count++;
    }

    stat_sample_count++;

    /* After 100 samples, compute quality and reset */
    if (stat_sample_count >= WIRE_STATS_WINDOW) {
        hits = stat_hit_count;

        /* Quality = number of "inside boundary" detections
         * (100 - miss count) — higher is better */
        stat_quality = WIRE_STATS_WINDOW - hits;

        /* Reset counters for next window */
        stat_sample_count = 0;
        stat_hit_count = 0;

        /* Update outside-boundary flag */
        if (zone_tracking) {
            stat_outside = (hits >= (WIRE_STATS_WINDOW - WIRE_STATS_THRESHOLD)) ? 1 : 0;
        } else {
            stat_outside = 0;
        }
    }
}

void wire_GetStats(wire_stats_t *stats)
{
    stats->quality = stat_quality;
    stats->sample_count = stat_sample_count;
    stats->hit_count = stat_hit_count;
    stats->outside = stat_outside;
}

uint8_t wire_IsOutside(void)
{
    return stat_outside;
}
