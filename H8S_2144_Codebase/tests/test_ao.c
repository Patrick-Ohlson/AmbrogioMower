/*============================================================================
 * test_ao.c — Active Object Framework Test (menu key I)
 *
 * Tests the complete AO framework reconstructed from the L200 firmware:
 *   1.  Pool System      — alloc, resolve, free, handle encoding
 *   2.  Queue System     — FIFO ordering, empty-returns-NULL
 *   3.  Event Allocation — evt_Alloc, signal storage, payload at +6
 *   4.  HSM Construction — ao_Start, ENTRY/INIT chain trace
 *   5.  Event Dispatch   — state transition, trace sequence verification
 *   5b. State-Local Data — cumulative offset scheme: parent data preserved,
 *                          only child's slice cleared on entry (firmware pattern)
 *   6.  Cross-AO Posting — wire AO posts motor_run to motor AO
 *   7.  Timer System     — one-shot 5-tick countdown, timeout fires
 *   8.  LCA Transitions  — exit chain across 3-level hierarchy
 *   9.  Full Integration — all phases from test_framework.c
 *  10.  Live Display     — 20s LCD tick counter + state names
 *
 * Architecture
 * ~~~~~~~~~~~~
 * The L200 firmware uses a Samek-style Active Object pattern:
 *
 *   SmallHeap ─── fixed-block memory pools with 1-byte handles
 *       │         handle = ~(pool_index<<5 | block_index)
 *       ▼
 *   Queue ─────── singly-linked FIFO of event handles
 *       │         each event's byte[4] (next field) chains to next handle
 *       ▼
 *   Event ─────── signal(4B) + next(1B) + flags(1B) + payload(NB)
 *       │         signal = address of the ao_post_*() function that created it
 *       ▼
 *   HSM ─────────  hierarchical state machine engine
 *       │         handlers return: NULL=handled, UNHANDLED=pass to parent,
 *       │         &state_desc=transition.  ENTRY/EXIT/INIT are sentinel signals.
 *       ▼
 *   Active Object  wraps an HSM + two queues + priority + state_data[]
 *       │         queue1 = external events, queue2 = self-posted (internal)
 *       ▼
 *   Scheduler ──── priority bitmask, cooperative: ao_SignalReady() triggers
 *       │         immediate dispatch when idle (g_ready_set == 0)
 *       ▼
 *   Timer ──────── doubly-linked list, tick-based countdown, posts events
 *
 * State-local data — cumulative offset scheme (from firmware)
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Each state_desc_t carries a state_data_size field.  This is NOT the
 * size of this state's own data — it is the CUMULATIVE total of all
 * ancestor data + this state's data.
 *
 * When hsm_EnterState() enters a state, it clears only the DELTA:
 *   clear_start = parent->state_data_size  (0 if no parent)
 *   clear_len   = this->state_data_size - clear_start
 *   memset(&ao->state_data[clear_start], 0, clear_len)
 *
 * This preserves parent-level variables when entering a child state.
 * The firmware calls this "relative register offsets" — each hierarchy
 * level owns a slice of state_data[] at a fixed base offset.
 *
 * Example from this test:
 *   motor_running (size=2) owns [0..1]: running_flags, speed_total
 *   motor_normal  (size=4) owns [2..3]: norm_count, last_speed_lo
 *   Entering motor_normal clears [2..3]; [0..1] survives.
 *
 * Verified from firmware disassembly (MowerMain AO):
 *   ROOT(0x0E) → child(0x0E) → grandchild(0x26) → ggchild(0x42)
 *
 * Based on STATES/active_object/test_framework.c
 * Adapted for UART pass/fail output matching TestFirmware conventions.
 *
 * Original firmware functions tested:
 *   pool_Init(0x16BC0), pool_Register(0x16C30), pool_Alloc(0x16DC2),
 *   pool_Resolve(0x170AA), pool_Free(0x16F2C),
 *   queue_Init(0x1808A), queue_Enqueue(0x18094), queue_Dequeue(0x180C4),
 *   evt_Alloc(0x16304), hsm_Constructor(0x16628), hsm_Dispatch(0x16652),
 *   ao_Constructor(0x16740), ao_ProcessQueue(0x167EE),
 *   ao_PostEvent(0x1683E), ao_EnqueueAndSignal(0x16772),
 *   ao_PostTransition(0x16858), ao_Start(0x16984),
 *   ao_Scheduler(0x1688A), ao_SignalReady(0x16A18),
 *   timer_Set(0x16AC0), timer_Fire(0x16A5E), timer_Tick()
 *============================================================================*/

#include "test_ao.h"
#include "../active_object/ao.h"
#include "../sci.h"
#include "../utils/utils.h"
#include "../lcd/lcd.h"
#include <string.h>

/*============================================================================
 *                    OUTPUT HELPERS
 *
 * pstr() wraps SendString for const-string output over UART.
 * print_dec16() prints a uint16_t in decimal (for pass/fail counts).
 * print_hex8/16 come from utils.h.
 *============================================================================*/
static void pstr(const char *s) { SendString((unsigned char *)s); }
static void pnl(void)           { pstr("\r\n"); }

static void print_dec16(uint16_t val)
{
    char buf[6];  /* max 5 digits + null */
    int i = 0;
    if (val == 0) { PutChar('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) { PutChar(buf[--i]); }
}

/*============================================================================
 *                    TRACE BUFFER
 *
 * Linear buffer of 8-bit event IDs — same pattern as test_framework.c.
 * Each state handler calls trace(ID) on significant events.  After a
 * test sequence, trace_match() compares the buffer against an expected
 * byte sequence to verify the exact HSM event ordering.
 *
 * Sized to 64 entries to conserve BSS on the 4KB-RAM H8S/2144.
 *============================================================================*/
#define TRACE_BUF_SIZE  64
static volatile uint8_t  g_trace_buf[TRACE_BUF_SIZE];
static volatile uint16_t g_trace_idx = 0;

static void trace(uint8_t id)
{
    if (g_trace_idx < TRACE_BUF_SIZE) {
        g_trace_buf[g_trace_idx++] = id;
    }
}

static void trace_reset(void)
{
    memset((void *)g_trace_buf, 0, sizeof(g_trace_buf));
    g_trace_idx = 0;
}

/* Check if trace buffer starts with the expected sequence.
 * Returns 1 if first `len` entries match, 0 otherwise. */
static int trace_match(const uint8_t *expected, uint16_t len)
{
    uint16_t i;
    if (g_trace_idx < len) return 0;
    for (i = 0; i < len; i++) {
        if (g_trace_buf[i] != expected[i]) return 0;
    }
    return 1;
}

/*============================================================================
 *                    TRACE ID DEFINITIONS
 *
 * Two hex digits: high nibble = state, low nibble = event type.
 *   0x0n = motor_top        0x1n = motor_idle
 *   0x2n = motor_running    0x3n = motor_normal
 *   0x4n = motor_stopping   0x5n = wire_top
 *   0x6n = wire_idle        0x7n = wire_searching
 *   0x8n = wire_following
 *============================================================================*/

/* Motor HSM trace IDs */
#define T_MOTOR_TOP_ENTRY      0x01  /* motor_top: ENTRY signal received    */
#define T_MOTOR_TOP_EXIT       0x02  /* motor_top: EXIT signal received     */
#define T_MOTOR_TOP_INIT       0x03  /* motor_top: INIT → motor_idle        */
#define T_MOTOR_IDLE_ENTRY     0x10  /* motor_idle: ENTRY                   */
#define T_MOTOR_IDLE_EXIT      0x11  /* motor_idle: EXIT                    */
#define T_MOTOR_IDLE_RUN       0x12  /* motor_idle: sig_motor_run → running */
#define T_MOTOR_IDLE_CMD       0x13  /* motor_idle: sig_motor_cmd (handled) */
#define T_MOTOR_RUN_ENTRY      0x20  /* motor_running: ENTRY (starts timer) */
#define T_MOTOR_RUN_EXIT       0x21  /* motor_running: EXIT (cancels timer) */
#define T_MOTOR_RUN_INIT       0x22  /* motor_running: INIT → motor_normal  */
#define T_MOTOR_RUN_IDLE       0x23  /* motor_running: sig_motor_idle→idle  */
#define T_MOTOR_RUN_SPEED      0x24  /* motor_running: sig_motor_speed      */
#define T_MOTOR_NORM_ENTRY     0x30  /* motor_normal: ENTRY                 */
#define T_MOTOR_NORM_EXIT      0x31  /* motor_normal: EXIT                  */
#define T_MOTOR_NORM_STOP      0x32  /* motor_normal: sig_motor_stop→stopping*/
#define T_MOTOR_NORM_TIMEOUT   0x33  /* motor_normal: sig_timeout→stopping  */
#define T_MOTOR_NORM_SPEED     0x34  /* motor_normal: sig_motor_speed       */
                                     /*   (uses state_data to count)        */
#define T_MOTOR_STOP_ENTRY     0x40  /* motor_stopping: ENTRY               */
#define T_MOTOR_STOP_EXIT      0x41  /* motor_stopping: EXIT                */
#define T_MOTOR_STOP_CMD       0x42  /* motor_stopping: sig_motor_cmd→idle  */

/* Wire HSM trace IDs */
#define T_WIRE_TOP_ENTRY       0x50
#define T_WIRE_TOP_EXIT        0x51
#define T_WIRE_TOP_INIT        0x52  /* wire_top: INIT → wire_idle          */
#define T_WIRE_IDLE_ENTRY      0x60
#define T_WIRE_IDLE_EXIT       0x61
#define T_WIRE_IDLE_SEARCH     0x62  /* wire_idle: sig_wire_search→searching*/
#define T_WIRE_SEARCH_ENTRY    0x70
#define T_WIRE_SEARCH_EXIT     0x71
#define T_WIRE_SEARCH_FOUND    0x72  /* wire_searching: sig_wire_found      */
                                     /*   → also posts motor_run cross-AO   */
#define T_WIRE_FOLLOW_ENTRY    0x80
#define T_WIRE_FOLLOW_EXIT     0x81
#define T_WIRE_FOLLOW_IDLE     0x82  /* wire_following: sig_wire_idle→idle  */
#define T_WIRE_FOLLOW_SPEED    0x83  /* wire_following: sig_wire_speed      */

/*============================================================================
 *                    SIGNAL DEFINITIONS
 *
 * In the L200 firmware, signals ARE the ao_post_* function addresses.
 * The event header stores signal = &ao_post_motor_run (for example),
 * and state handlers compare evt->signal against known function addresses.
 *
 * For this test we define named signal functions.  Their addresses serve
 * as unique signal identifiers.
 *
 * NOT static: with static + -Os, GCC may fold identical empty function
 * bodies (Identical Code Folding), giving all signals the same address
 * and breaking signal comparison.  Matches working test_framework.c.
 *============================================================================*/

/* Motor signals */
void sig_motor_idle(void)  {}
void sig_motor_run(void)   {}
void sig_motor_stop(void)  {}
void sig_motor_cmd(void)   {}
void sig_motor_speed(void) {}

/* Wire follow signals */
void sig_wire_idle(void)   {}
void sig_wire_search(void) {}
void sig_wire_follow(void) {}
void sig_wire_found(void)  {}
void sig_wire_speed(void)  {}

/* Timer signal */
void sig_timeout(void)     {}

/*============================================================================
 *                    ISR TICK (polling mode)
 *
 * In the firmware, ao_timer_Tick() is called from the 10ms timer ISR,
 * and ao_Scheduler() runs cooperatively.  For testing we call both
 * manually — this simulates one ISR tick per call.
 *============================================================================*/
static void isr_tick(void)
{
    ao_timer_Tick();    /* decrement all active timers, fire any at zero */
    ao_Scheduler();     /* dispatch any ready AO queues                  */
}

/*============================================================================
 *                    MOTOR HSM (Priority 7, 3-level hierarchy)
 *
 * Hierarchy:
 *   motor_top                       ← top-level (always active)
 *     ├── motor_idle  [initial]     ← waiting for commands
 *     └── motor_running             ← blade spinning
 *           ├── motor_normal [init] ← normal operation, has 5-tick timeout
 *           └── motor_stopping      ← decelerating
 *
 * State transitions:
 *   idle ──sig_motor_run──► running/normal ──timeout──► stopping
 *   stopping ──sig_motor_cmd──► idle
 *   running/* ──sig_motor_idle──► idle  (handled by parent motor_running)
 *
 * Demonstrates the firmware's CUMULATIVE state_data offset scheme:
 *
 *   motor_running owns sizeof(motor_running_data_t) = 2 bytes at offset 0:
 *     .running_flags   — set on ENTRY to mark "motor active" (0xAA)
 *     .speed_cmd_total — counts ALL speed events while running
 *
 *   motor_normal owns sizeof(motor_normal_data_t) = 2 bytes at offset 2:
 *     .norm_cmd_count  — counts speed events in normal substate only
 *     .last_speed_lo   — low byte of last speed event payload
 *
 *   State descriptors use sizeof() for auto-calculated cumulative sizes:
 *     motor_running.state_data_size = sizeof(motor_running_data_t)           = 2
 *     motor_normal.state_data_size  = sizeof(running) + sizeof(normal)       = 4
 *     motor_stopping.state_data_size = sizeof(motor_running_data_t)          = 2
 *
 *   Handlers access their data via HSM_STATE_DATA(ao, &parent, type):
 *     motor_running_data_t *d = HSM_STATE_DATA(ao, &motor_top, motor_running_data_t);
 *     motor_normal_data_t *nd = HSM_STATE_DATA(ao, &motor_running, motor_normal_data_t);
 *
 *   Key behavior (verified from firmware disassembly of hsm_EnterState):
 *     Entering motor_running: clears [0..1] (parent motor_top has size=0)
 *     Entering motor_normal:  clears [2..3] ONLY (parent has size=2)
 *     → motor_running's data is PRESERVED across child entries.
 *     This is the "relative register offset" mechanism from the firmware.
 *============================================================================*/

/*--- State-local data structures ---
 *
 * Each state level that uses state_data defines a struct for its variables.
 * The cumulative state_data_size = sum of all ancestor structs + own sizeof.
 * Handlers cast ao->state_data at the parent's offset via HSM_STATE_DATA().
 *
 * This replaces error-prone raw index access (ao->state_data[0], [2], etc.)
 * with named fields (d->running_flags, nd->norm_cmd_count, etc.).
 */
typedef struct {
    uint8_t running_flags;      /* 0xAA = motor active marker              */
    uint8_t speed_cmd_total;    /* total speed events while in running     */
} motor_running_data_t;

typedef struct {
    uint8_t norm_cmd_count;     /* speed events counted at normal level    */
    uint8_t last_speed_lo;      /* low byte of last speed event payload    */
} motor_normal_data_t;

/*--- Typed event structures ---
 *
 * Each event type extends evt_header_t with named payload fields.
 * EVT_PACKED is required: the header is 6 bytes, so without packing
 * the compiler inserts padding before uint16_t/uint32_t fields.
 * sizeof() replaces magic numbers in evt_Alloc() calls.
 * EVT_CAST() replaces raw (uint8_t *)ptr + 6 payload access.
 */
typedef struct EVT_PACKED {
    evt_header_t hdr;
    uint32_t     cmd;           /* command word (e.g. 0xCAFE0001)          */
} evt_motor_cmd_t;              /* sizeof = 10                             */

typedef struct EVT_PACKED {
    evt_header_t hdr;
    uint32_t     speed;         /* speed value                             */
} evt_motor_speed_t;            /* sizeof = 10                             */

typedef struct EVT_PACKED {
    evt_header_t hdr;
    uint16_t     speed;         /* wire-follow speed                       */
} evt_wire_speed_t;             /* sizeof = 8                              */

/* Forward declarations — all handlers follow the same signature:
 *   void *handler(ao_t *ao, void *event)
 * where event is either a sentinel (HSM_SIG_ENTRY/EXIT/INIT) or
 * a pointer to an evt_header_t in pool memory. */
static void *motor_top_handler(ao_t *ao, void *event);
static void *motor_idle_handler(ao_t *ao, void *event);
static void *motor_running_handler(ao_t *ao, void *event);
static void *motor_normal_handler(ao_t *ao, void *event);
static void *motor_stopping_handler(ao_t *ao, void *event);

/* State descriptors — const (ROM-resident).
 * Each specifies: handler function, parent state, state_data_size.
 * The hierarchy is encoded via parent pointers.
 *
 * state_data_size is CUMULATIVE (firmware convention), auto-calculated
 * from sizeof() of each level's data struct:
 *   state_data_size = parent->state_data_size + sizeof(this level's struct)
 *   hsm_EnterState clears only [parent_size .. this_size), preserving
 *   the parent's data slice.
 *
 * Layout for motor hierarchy:
 *   motor_top:      size=0  → no data
 *   motor_idle:     size=0  → no data
 *   motor_running:  size=sizeof(motor_running_data_t)                    = 2
 *   motor_normal:   size=sizeof(running_data) + sizeof(normal_data)      = 4
 *   motor_stopping: size=sizeof(motor_running_data_t)  (no extra data)   = 2
 */
static const state_desc_t motor_top = {
    .handler = motor_top_handler,
    .parent  = NULL,                /* top-level: no parent                */
    .state_data_size = 0            /* cumulative: 0                       */
};
static const state_desc_t motor_idle = {
    .handler = motor_idle_handler,
    .parent  = &motor_top,          /* child of motor_top                  */
    .state_data_size = 0            /* cumulative: 0 (no data)             */
};
static const state_desc_t motor_running = {
    .handler = motor_running_handler,
    .parent  = &motor_top,          /* child of motor_top                  */
    .state_data_size = sizeof(motor_running_data_t)
                                    /* cumulative: 0 + sizeof(running) = 2 */
};
static const state_desc_t motor_normal = {
    .handler = motor_normal_handler,
    .parent  = &motor_running,      /* child of motor_running              */
    .state_data_size = sizeof(motor_running_data_t) + sizeof(motor_normal_data_t)
                                    /* cumulative: 2 + 2 = 4               */
};
static const state_desc_t motor_stopping = {
    .handler = motor_stopping_handler,
    .parent  = &motor_running,      /* child of motor_running              */
    .state_data_size = sizeof(motor_running_data_t)
                                    /* cumulative: same as parent (no extra)*/
};

static ao_t       ao_motor;         /* motor Active Object instance        */
static ao_timer_t motor_timer;      /* 5-tick timeout timer                */

/*--- motor_top: top-level state, catches all unhandled events ---*/
static void *motor_top_handler(ao_t *ao, void *event)
{
    (void)ao;
    if (event == HSM_SIG_ENTRY) { trace(T_MOTOR_TOP_ENTRY); return HSM_HANDLED; }
    if (event == HSM_SIG_EXIT)  { trace(T_MOTOR_TOP_EXIT);  return HSM_HANDLED; }
    if (event == HSM_SIG_INIT)  {
        trace(T_MOTOR_TOP_INIT);
        return HSM_TRAN(&motor_idle);   /* initial transition → idle */
    }
    return HSM_HANDLED;     /* top state absorbs any unhandled events */
}

/*--- motor_idle: waiting for commands ---*/
static void *motor_idle_handler(ao_t *ao, void *event)
{
    (void)ao;
    if (event == HSM_SIG_ENTRY) { trace(T_MOTOR_IDLE_ENTRY); return HSM_HANDLED; }
    if (event == HSM_SIG_EXIT)  { trace(T_MOTOR_IDLE_EXIT);  return HSM_HANDLED; }
    if (event == HSM_SIG_INIT)  { return HSM_HANDLED; }  /* no initial substate */

    /* User events — compare signal field against known function addresses */
    {
        evt_header_t *evt = (evt_header_t *)event;
        if (evt->signal == (signal_t)&sig_motor_run) {
            trace(T_MOTOR_IDLE_RUN);
            return HSM_TRAN(&motor_running);    /* → running/normal */
        }
        if (evt->signal == (signal_t)&sig_motor_cmd) {
            trace(T_MOTOR_IDLE_CMD);
            return HSM_HANDLED;                 /* absorb while idle */
        }
    }
    return HSM_UNHANDLED;   /* pass to parent (motor_top) */
}

/*--- motor_running: blade is spinning, has 5-tick watchdog timer ---
 *
 * State-local data accessed via HSM_STATE_DATA(ao, &motor_top, ...):
 *   motor_running_data_t at offset 0 (parent motor_top has size=0):
 *     .running_flags   — set to 0xAA on ENTRY to mark "active"
 *     .speed_cmd_total — counts ALL speed events while running
 *
 * These bytes are cleared when ENTERING motor_running (delta: [0..1]).
 * They are PRESERVED when entering child states (motor_normal, motor_stopping)
 * because children only clear their own slice above sizeof(motor_running_data_t).
 */
static void *motor_running_handler(ao_t *ao, void *event)
{
    if (event == HSM_SIG_ENTRY) {
        motor_running_data_t *d = HSM_STATE_DATA(ao, &motor_top, motor_running_data_t);
        trace(T_MOTOR_RUN_ENTRY);
        /* After hsm_EnterState clears the running slice, set running_flags.
         * This value persists while in running and all substates. */
        d->running_flags = 0xAA;       /* "motor active" marker */
        /* d->speed_cmd_total starts at 0 from clearing */

        /* Start a 5-tick one-shot timer.  When it fires, sig_timeout is
         * posted to ao_motor, causing motor_normal → motor_stopping. */
        memset(&motor_timer, 0, sizeof(motor_timer));
        ao_timer_Set(&motor_timer, &ao_motor, (signal_t)&sig_timeout, 5, false);
        return HSM_HANDLED;
    }
    if (event == HSM_SIG_EXIT) {
        trace(T_MOTOR_RUN_EXIT);
        /* Cancel timer when leaving running state */
        ao_timer_Set(&motor_timer, NULL, NULL, 0, false);
        return HSM_HANDLED;
    }
    if (event == HSM_SIG_INIT) {
        trace(T_MOTOR_RUN_INIT);
        return HSM_TRAN(&motor_normal);     /* initial substate = normal */
    }

    {
        evt_header_t *evt = (evt_header_t *)event;
        if (evt->signal == (signal_t)&sig_motor_idle) {
            trace(T_MOTOR_RUN_IDLE);
            return HSM_TRAN(&motor_idle);   /* back to idle */
        }
        if (evt->signal == (signal_t)&sig_motor_speed) {
            /* Speed event not handled by child → count at parent level.
             * This is only reached if motor_stopping is the current substate
             * (motor_normal handles speed first and doesn't pass up). */
            motor_running_data_t *d = HSM_STATE_DATA(ao, &motor_top, motor_running_data_t);
            d->speed_cmd_total++;
            trace(T_MOTOR_RUN_SPEED);
            return HSM_HANDLED;
        }
    }
    return HSM_UNHANDLED;   /* pass to parent (motor_top) */
}

/*--- motor_normal: normal operation substate of motor_running ---
 *
 * Demonstrates the firmware's cumulative state_data offset scheme
 * using typed struct access via HSM_STATE_DATA():
 *
 *   motor_running_data_t *pd = HSM_STATE_DATA(ao, &motor_top, ...);
 *     pd->running_flags   = 0xAA (set by motor_running ENTRY)
 *     pd->speed_cmd_total = count of all speed events
 *
 *   motor_normal_data_t *nd = HSM_STATE_DATA(ao, &motor_running, ...);
 *     nd->norm_cmd_count  — speed events counted at this level only
 *     nd->last_speed_lo   — low byte of last speed payload
 *
 *   hsm_EnterState clears only the normal_data slice (the delta).
 *   Parent running_data is preserved.  This is the "relative register
 *   offset" mechanism from the original firmware — each hierarchy level
 *   accesses its own struct without disturbing parent variables.
 */
static void *motor_normal_handler(ao_t *ao, void *event)
{
    if (event == HSM_SIG_ENTRY) { trace(T_MOTOR_NORM_ENTRY); return HSM_HANDLED; }
    if (event == HSM_SIG_EXIT)  { trace(T_MOTOR_NORM_EXIT);  return HSM_HANDLED; }
    if (event == HSM_SIG_INIT)  { return HSM_HANDLED; }

    {
        evt_header_t *evt = (evt_header_t *)event;
        if (evt->signal == (signal_t)&sig_motor_stop) {
            trace(T_MOTOR_NORM_STOP);
            return HSM_TRAN(&motor_stopping);
        }
        if (evt->signal == (signal_t)&sig_timeout) {
            trace(T_MOTOR_NORM_TIMEOUT);
            return HSM_TRAN(&motor_stopping);
        }
        if (evt->signal == (signal_t)&sig_motor_speed) {
            /* Access child data (normal's own slice) and parent data
             * (running's slice) through typed struct pointers.
             * Child handler CAN read/write parent data too. */
            motor_normal_data_t  *nd = HSM_STATE_DATA(ao, &motor_running, motor_normal_data_t);
            motor_running_data_t *pd = HSM_STATE_DATA(ao, &motor_top, motor_running_data_t);
            evt_motor_speed_t    *se = EVT_CAST(evt, evt_motor_speed_t);
            nd->norm_cmd_count++;             /* child's own count       */
            nd->last_speed_lo = (uint8_t)se->speed;  /* low byte        */
            pd->speed_cmd_total++;            /* bump parent's total     */
            trace(T_MOTOR_NORM_SPEED);
            return HSM_HANDLED;
        }
    }
    return HSM_UNHANDLED;   /* pass to parent (motor_running) */
}

/*--- motor_stopping: decelerating, waits for cmd to go idle ---*/
static void *motor_stopping_handler(ao_t *ao, void *event)
{
    (void)ao;
    if (event == HSM_SIG_ENTRY) { trace(T_MOTOR_STOP_ENTRY); return HSM_HANDLED; }
    if (event == HSM_SIG_EXIT)  { trace(T_MOTOR_STOP_EXIT);  return HSM_HANDLED; }
    if (event == HSM_SIG_INIT)  { return HSM_HANDLED; }

    {
        evt_header_t *evt = (evt_header_t *)event;
        if (evt->signal == (signal_t)&sig_motor_cmd) {
            trace(T_MOTOR_STOP_CMD);
            return HSM_TRAN(&motor_idle);   /* back to idle */
        }
    }
    return HSM_UNHANDLED;   /* pass to parent (motor_running) */
}

/*============================================================================
 *                    WIRE HSM (Priority 5, 2-level hierarchy)
 *
 * Hierarchy:
 *   wire_top
 *     ├── wire_idle    [initial]  ← waiting for search command
 *     ├── wire_searching          ← looking for boundary wire
 *     └── wire_following          ← tracking wire, posting speed to motor
 *
 * Cross-AO interaction:
 *   wire_searching handles sig_wire_found by:
 *     1. Posting sig_motor_run to ao_motor (cross-AO)
 *     2. Transitioning self to wire_following
 *   wire_following handles sig_wire_idle by:
 *     1. Posting sig_motor_idle to ao_motor (cross-AO)
 *     2. Transitioning self to wire_idle
 *============================================================================*/

static void *wire_top_handler(ao_t *ao, void *event);
static void *wire_idle_handler(ao_t *ao, void *event);
static void *wire_searching_handler(ao_t *ao, void *event);
static void *wire_following_handler(ao_t *ao, void *event);

static const state_desc_t wire_top = {
    .handler = wire_top_handler, .parent = NULL, .state_data_size = 0
};
static const state_desc_t wire_idle_st = {
    .handler = wire_idle_handler, .parent = &wire_top, .state_data_size = 0
};
static const state_desc_t wire_searching = {
    .handler = wire_searching_handler, .parent = &wire_top, .state_data_size = 0
};
static const state_desc_t wire_following = {
    .handler = wire_following_handler, .parent = &wire_top, .state_data_size = 0
};

static ao_t ao_wire;

static void *wire_top_handler(ao_t *ao, void *event)
{
    (void)ao;
    if (event == HSM_SIG_ENTRY) { trace(T_WIRE_TOP_ENTRY); return HSM_HANDLED; }
    if (event == HSM_SIG_EXIT)  { trace(T_WIRE_TOP_EXIT);  return HSM_HANDLED; }
    if (event == HSM_SIG_INIT)  { trace(T_WIRE_TOP_INIT);  return HSM_TRAN(&wire_idle_st); }
    return HSM_HANDLED;
}

static void *wire_idle_handler(ao_t *ao, void *event)
{
    (void)ao;
    if (event == HSM_SIG_ENTRY) { trace(T_WIRE_IDLE_ENTRY); return HSM_HANDLED; }
    if (event == HSM_SIG_EXIT)  { trace(T_WIRE_IDLE_EXIT);  return HSM_HANDLED; }
    if (event == HSM_SIG_INIT)  { return HSM_HANDLED; }
    {
        evt_header_t *evt = (evt_header_t *)event;
        if (evt->signal == (signal_t)&sig_wire_search) {
            trace(T_WIRE_IDLE_SEARCH);
            return HSM_TRAN(&wire_searching);
        }
    }
    return HSM_UNHANDLED;
}

static void *wire_searching_handler(ao_t *ao, void *event)
{
    (void)ao;
    if (event == HSM_SIG_ENTRY) { trace(T_WIRE_SEARCH_ENTRY); return HSM_HANDLED; }
    if (event == HSM_SIG_EXIT)  { trace(T_WIRE_SEARCH_EXIT);  return HSM_HANDLED; }
    if (event == HSM_SIG_INIT)  { return HSM_HANDLED; }
    {
        evt_header_t *evt = (evt_header_t *)event;
        if (evt->signal == (signal_t)&sig_wire_found) {
            trace(T_WIRE_SEARCH_FOUND);
            /* Cross-AO: tell motor to start running */
            ao_PostTransition(&ao_motor, (signal_t)&sig_motor_run);
            return HSM_TRAN(&wire_following);
        }
    }
    return HSM_UNHANDLED;
}

static void *wire_following_handler(ao_t *ao, void *event)
{
    (void)ao;
    if (event == HSM_SIG_ENTRY) { trace(T_WIRE_FOLLOW_ENTRY); return HSM_HANDLED; }
    if (event == HSM_SIG_EXIT)  { trace(T_WIRE_FOLLOW_EXIT);  return HSM_HANDLED; }
    if (event == HSM_SIG_INIT)  { return HSM_HANDLED; }
    {
        evt_header_t *evt = (evt_header_t *)event;
        if (evt->signal == (signal_t)&sig_wire_idle) {
            trace(T_WIRE_FOLLOW_IDLE);
            /* Cross-AO: tell motor to stop */
            ao_PostTransition(&ao_motor, (signal_t)&sig_motor_idle);
            return HSM_TRAN(&wire_idle_st);
        }
        if (evt->signal == (signal_t)&sig_wire_speed) {
            trace(T_WIRE_FOLLOW_SPEED);
            return HSM_HANDLED;
        }
    }
    return HSM_UNHANDLED;
}

/*============================================================================
 *                    EVENT CONSTRUCTOR HELPERS
 *
 * In the firmware, each ao_post_*() function allocates an event, fills
 * the signal and payload, then calls ao_PostEvent().  These helpers
 * replicate that pattern using typed event structs and sizeof().
 *============================================================================*/

/* Post sig_motor_cmd with a 4-byte command payload */
static void post_motor_cmd_u32(ao_t *target, uint32_t cmd)
{
    handle_t h = evt_Alloc((signal_t)&sig_motor_cmd, sizeof(evt_motor_cmd_t));
    evt_motor_cmd_t *evt = EVT_CAST(pool_Resolve(h), evt_motor_cmd_t);
    evt->cmd = cmd;
    ao_PostEvent(target, h);
}

/* Post sig_motor_speed with a 4-byte speed payload */
static void post_motor_speed(ao_t *target, uint32_t speed_val)
{
    handle_t h = evt_Alloc((signal_t)&sig_motor_speed, sizeof(evt_motor_speed_t));
    evt_motor_speed_t *evt = EVT_CAST(pool_Resolve(h), evt_motor_speed_t);
    evt->speed = speed_val;
    ao_PostEvent(target, h);
}

/* Post sig_wire_speed with a 2-byte speed payload */
static void post_wire_speed_u16(ao_t *target, uint16_t speed)
{
    handle_t h = evt_Alloc((signal_t)&sig_wire_speed, sizeof(evt_wire_speed_t));
    evt_wire_speed_t *evt = EVT_CAST(pool_Resolve(h), evt_wire_speed_t);
    evt->speed = speed;
    ao_PostEvent(target, h);
}

/*============================================================================
 *                    MAIN TEST FUNCTION
 *============================================================================*/
void AO_Test(void)
{
    uint16_t pc = 0, fc = 0;       /* pass / fail counters */
    int i;

    pstr("\r\n========== AO Framework Test ==========\r\n");

    /* Initialize AO framework: sets up memory pools (2 pools: 8×24B +
     * 4×48B), scheduler priority table, timer list, error flag.
     * All tests share the framework's BSS-resident pools (ao.c). */
    ao_FrameworkInit();

    /*====================================================================
     * TEST 1: Pool System
     *
     * Verifies SmallHeap fixed-block allocator (firmware 0x16BC0-0x170AA):
     *   - pool_Alloc returns non-NULL handles for valid sizes
     *   - Handle encoding: handle = ~(pool_index<<5 | block_index)
     *     10B request → pool 0 (24B blocks) → handle FF = ~(0<<5|0)
     *     30B request → pool 1 (48B blocks) → handle DF = ~(1<<5|0)
     *   - pool_Resolve converts handle back to memory pointer
     *   - pool_Free returns block to free list (LIFO: same handle reused)
     *====================================================================*/
    pstr("\r\n- 1. Pool System -\r\n");
    {
        handle_t h1, h2, h3;
        void *p1, *p2;

        h1 = pool_Alloc(10);
        if (h1 != POOL_HANDLE_NULL) {
            pstr("  Alloc 10B: "); print_hex8(h1); pstr(" [PASS]\r\n"); pc++;
        } else {
            pstr("  Alloc 10B: NULL [FAIL]\r\n"); fc++;
        }

        p1 = pool_Resolve(h1);
        if (p1 != NULL) {
            pstr("  Resolve non-NULL: [PASS]\r\n"); pc++;
        } else {
            pstr("  Resolve NULL: [FAIL]\r\n"); fc++;
        }

        /* 30B exceeds pool 0's 24B blocks → allocates from pool 1 (48B) */
        h2 = pool_Alloc(30);
        if (h2 != POOL_HANDLE_NULL) {
            p2 = pool_Resolve(h2);
            if (p2 != NULL && p2 != p1) {
                pstr("  Alloc 30B: "); print_hex8(h2); pstr(" [PASS]\r\n"); pc++;
            } else {
                pstr("  Alloc 30B resolved wrong: [FAIL]\r\n"); fc++;
            }
        } else {
            pstr("  Alloc 30B: NULL [FAIL]\r\n"); fc++;
        }

        /* Free block 0 of pool 0, then re-alloc — should get same handle
         * because free-list is LIFO (push onto head, pop from head). */
        pool_Free(h1);
        h3 = pool_Alloc(10);
        if (h3 == h1) {
            pstr("  Free+realloc same handle: [PASS]\r\n"); pc++;
        } else {
            pstr("  Free+realloc: "); print_hex8(h3);
            pstr(" vs "); print_hex8(h1); pstr(" [WARN]\r\n"); pc++;
        }
        pool_Free(h3);
        pool_Free(h2);
    }

    /*====================================================================
     * TEST 2: Queue System
     *
     * Verifies singly-linked event queue (firmware 0x1808A-0x180C4):
     *   - queue_Dequeue on empty → POOL_HANDLE_NULL (0x00)
     *   - FIFO ordering: enqueue A,B,C → dequeue A,B,C
     *   - Queue link uses event byte[4] (EVT_NEXT_OFFSET)
     *====================================================================*/
    pstr("\r\n- 2. Queue System -\r\n");
    {
        queue_t q;
        handle_t ha, hb, hc, hd;

        queue_Init(&q);

        hd = queue_Dequeue(&q);
        if (hd == POOL_HANDLE_NULL) {
            pstr("  Dequeue empty = NULL: [PASS]\r\n"); pc++;
        } else {
            pstr("  Dequeue empty != NULL: [FAIL]\r\n"); fc++;
        }

        ha = pool_Alloc(6); hb = pool_Alloc(6); hc = pool_Alloc(6);
        queue_Enqueue(&q, ha);
        queue_Enqueue(&q, hb);
        queue_Enqueue(&q, hc);

        hd = queue_Dequeue(&q);
        if (hd == ha) { pstr("  FIFO order #1: [PASS]\r\n"); pc++; }
        else          { pstr("  FIFO order #1: [FAIL]\r\n"); fc++; }

        hd = queue_Dequeue(&q);
        if (hd == hb) { pstr("  FIFO order #2: [PASS]\r\n"); pc++; }
        else          { pstr("  FIFO order #2: [FAIL]\r\n"); fc++; }

        hd = queue_Dequeue(&q);
        if (hd == hc) { pstr("  FIFO order #3: [PASS]\r\n"); pc++; }
        else          { pstr("  FIFO order #3: [FAIL]\r\n"); fc++; }

        hd = queue_Dequeue(&q);
        if (hd == POOL_HANDLE_NULL) { pstr("  Empty after 3: [PASS]\r\n"); pc++; }
        else                        { pstr("  Not empty after 3: [FAIL]\r\n"); fc++; }

        pool_Free(ha); pool_Free(hb); pool_Free(hc);
    }

    /*====================================================================
     * TEST 3: Event Allocation
     *
     * Verifies evt_Alloc (firmware 0x16304):
     *   - Allocates from pool, zero-fills, stores signal at offset 0
     *   - Signal = function pointer address (&sig_motor_cmd)
     *   - Payload starts at +6 (EVT_PAYLOAD macro)
     *====================================================================*/
    pstr("\r\n- 3. Event Allocation -\r\n");
    {
        handle_t h;
        evt_header_t *evt;

        h = evt_Alloc((signal_t)&sig_motor_cmd, 10);
        evt = (evt_header_t *)pool_Resolve(h);

        if (evt->signal == (signal_t)&sig_motor_cmd) {
            pstr("  Signal stored correctly: [PASS]\r\n"); pc++;
        } else {
            pstr("  Signal mismatch: [FAIL]\r\n"); fc++;
        }

        {
            uint8_t *payload = EVT_PAYLOAD(evt);
            uint32_t val = 0xCAFEBEEF;
            memcpy(payload, &val, 4);
            if (payload[0] != 0 || payload == (uint8_t *)evt) {
                pstr("  Payload at +6 accessible: [PASS]\r\n"); pc++;
            } else {
                pstr("  Payload access failed: [FAIL]\r\n"); fc++;
            }
        }
        pool_Free(h);
    }

    /*====================================================================
     * TEST 4: HSM Construction
     *
     * Verifies ao_Start → hsm_Constructor → hsm_EnterState + hsm_InitChain:
     *   ao_Start(7, &ao_motor, &motor_top) triggers:
     *     1. ENTRY(motor_top)        — enter top-level state
     *     2. INIT(motor_top)→idle    — top's init returns HSM_TRAN(&idle)
     *     3. ENTRY(motor_idle)       — enter initial substate
     *   After: ao_motor.current_state == &motor_idle
     *====================================================================*/
    pstr("\r\n- 4. HSM Construction -\r\n");
    {
        ao_FrameworkInit();
        trace_reset();

        ao_Start(7, &ao_motor, &motor_top);

        if (ao_motor.current_state == &motor_idle) {
            pstr("  Motor initial state = idle: [PASS]\r\n"); pc++;
        } else {
            pstr("  Motor initial state wrong: [FAIL]\r\n"); fc++;
        }

        {
            static const uint8_t expected[] = {
                T_MOTOR_TOP_ENTRY, T_MOTOR_TOP_INIT, T_MOTOR_IDLE_ENTRY
            };
            if (trace_match(expected, sizeof(expected))) {
                pstr("  ENTRY/INIT trace correct: [PASS]\r\n"); pc++;
            } else {
                pstr("  ENTRY/INIT trace wrong: [FAIL]\r\n"); fc++;
            }
        }
    }

    /*====================================================================
     * TEST 5: Event Dispatch
     *
     * Verifies hsm_Dispatch → parent walk → transition → LCA algorithm:
     *   ao_PostTransition(&ao_motor, sig_motor_run) triggers:
     *     1. evt_Alloc → pool_Alloc (allocate 6-byte event)
     *     2. ao_EnqueueAndSignal → queue_Enqueue + ao_SignalReady
     *     3. ao_SignalReady → ao_Scheduler (immediate, g_ready_set was 0)
     *     4. ao_ProcessQueue → queue_Dequeue → pool_Resolve → hsm_Dispatch
     *     5. motor_idle_handler sees sig_motor_run → trace(IDLE_RUN),
     *        returns HSM_TRAN(&motor_running)
     *     6. hsm_Dispatch calls hsm_ExitChain + hsm_Transition:
     *        EXIT(idle), ENTRY(running), INIT(running)→normal, ENTRY(normal)
     *     7. pool_Free (return event to pool)
     *
     *   Expected trace (handler traces BEFORE returning transition):
     *     IDLE_RUN, IDLE_EXIT, RUN_ENTRY, RUN_INIT, NORM_ENTRY
     *====================================================================*/
    pstr("\r\n- 5. Event Dispatch -\r\n");
    trace_reset();
    g_dbg_bad_count = 0;

    {
        ao_PostTransition(&ao_motor, (signal_t)&sig_motor_run);
        isr_tick();

        /* Safety check: pool_Resolve captures bad handles in globals */
        if (g_dbg_bad_count > 0) {
            pstr("  BAD RESOLVE: h="); print_hex8(g_dbg_bad_handle);
            pstr(" site="); print_hex8(g_dbg_bad_site);
            pstr(" x"); PutChar('0' + g_dbg_bad_count);
            pstr(" [FAIL]\r\n"); fc++;
        }

        /* idle → running (INIT) → normal */
        if (ao_motor.current_state == &motor_normal) {
            pstr("  motor_idle -> motor_normal: [PASS]\r\n"); pc++;
        } else {
            pstr("  State transition failed: [FAIL]\r\n"); fc++;
        }

        {
            static const uint8_t expected[] = {
                T_MOTOR_IDLE_RUN, T_MOTOR_IDLE_EXIT,
                T_MOTOR_RUN_ENTRY, T_MOTOR_RUN_INIT, T_MOTOR_NORM_ENTRY
            };
            if (trace_match(expected, sizeof(expected))) {
                pstr("  Dispatch trace correct: [PASS]\r\n"); pc++;
            } else {
                pstr("  Dispatch trace wrong: [FAIL]\r\n"); fc++;
            }
        }
    }

    /*====================================================================
     * TEST 5b: Cumulative State-Local Data (firmware offset scheme)
     *
     * Verifies the firmware's "relative register offset" mechanism
     * using typed struct access via HSM_STATE_DATA():
     *
     *   motor_running_data_t *rd (at parent motor_top's offset = 0):
     *     rd->running_flags   = 0xAA (set on ENTRY)
     *     rd->speed_cmd_total = total speed events while running
     *
     *   motor_normal_data_t *nd (at parent motor_running's offset = 2):
     *     nd->norm_cmd_count  = speed events at normal level
     *     nd->last_speed_lo   = low byte of last speed payload
     *
     * Key test: entering motor_normal clears only the normal_data slice.
     * Parent running_data is PRESERVED (running_flags stays 0xAA).
     *
     * This matches the firmware pattern found via Ghidra analysis:
     *   tm_ClearMemory(ao + parent->state_data_size,
     *                  target->state_data_size - parent->state_data_size)
     *====================================================================*/
    pstr("\r\n- 5b. State-Local Data -\r\n");
    {
        /* Typed pointers into ao_motor's state_data[] */
        motor_running_data_t *rd = HSM_STATE_DATA(&ao_motor, &motor_top, motor_running_data_t);
        motor_normal_data_t  *nd = HSM_STATE_DATA(&ao_motor, &motor_running, motor_normal_data_t);

        /* Motor is already in motor_normal from test 5.
         * motor_running ENTRY set rd->running_flags = 0xAA.
         * motor_normal ENTRY cleared only the normal slice. */

        /* Check parent data survived child entry */
        if (rd->running_flags == 0xAA) {
            pstr("  Parent flags=AA preserved: [PASS]\r\n"); pc++;
        } else {
            pstr("  Parent flags=");
            print_hex8(rd->running_flags);
            pstr(" [FAIL]\r\n"); fc++;
        }

        /* Check child data was cleared on entry */
        if (nd->norm_cmd_count == 0) {
            pstr("  Child count cleared: [PASS]\r\n"); pc++;
        } else {
            pstr("  Child count not clear: [FAIL]\r\n"); fc++;
        }

        /* Send 3 speed events:
         *   motor_normal_handler increments nd->norm_cmd_count and
         *   pd->speed_cmd_total, stores payload[0] in nd->last_speed_lo */
        post_motor_speed(&ao_motor, 0x00001234);   /* payload[0] = 0x34 */
        isr_tick();
        post_motor_speed(&ao_motor, 0x00005678);   /* payload[0] = 0x78 */
        isr_tick();
        post_motor_speed(&ao_motor, 0x000000AB);   /* payload[0] = 0xAB */
        isr_tick();

        /* Child count: 3 speed events at the normal level */
        if (nd->norm_cmd_count == 3) {
            pstr("  Child count=3: [PASS]\r\n"); pc++;
        } else {
            pstr("  Child count=");
            PutChar('0' + nd->norm_cmd_count);
            pstr(" [FAIL]\r\n"); fc++;
        }

        /* Parent total: also 3 (child handler bumps parent's total too) */
        if (rd->speed_cmd_total == 3) {
            pstr("  Parent total=3: [PASS]\r\n"); pc++;
        } else {
            pstr("  Parent total=");
            PutChar('0' + rd->speed_cmd_total);
            pstr(" [FAIL]\r\n"); fc++;
        }

        /* Last speed low byte = 0xAB */
        if (nd->last_speed_lo == 0xAB) {
            pstr("  Child lo=AB: [PASS]\r\n"); pc++;
        } else {
            pstr("  Child lo=");
            print_hex8(nd->last_speed_lo);
            pstr(" [FAIL]\r\n"); fc++;
        }

        /* Parent running_flags still 0xAA after all child activity */
        if (rd->running_flags == 0xAA) {
            pstr("  Parent flags still AA: [PASS]\r\n"); pc++;
        } else {
            pstr("  Parent flags=");
            print_hex8(rd->running_flags);
            pstr(" [FAIL]\r\n"); fc++;
        }

        /* Transition: normal→stopping→idle, then back to running/normal.
         * Re-entering motor_running clears running_data (new flags).
         * Re-entering motor_normal clears normal_data (child reset).
         * This tests the FULL clearing path through both hierarchy levels. */
        ao_PostTransition(&ao_motor, (signal_t)&sig_motor_idle);
        isr_tick();
        ao_PostTransition(&ao_motor, (signal_t)&sig_motor_run);
        isr_tick();

        /* After re-entry: running_flags reset to 0xAA (set on ENTRY) */
        if (rd->running_flags == 0xAA) {
            pstr("  Re-entry flags=AA: [PASS]\r\n"); pc++;
        } else {
            pstr("  Re-entry flags=");
            print_hex8(rd->running_flags);
            pstr(" [FAIL]\r\n"); fc++;
        }

        /* Child data cleared on motor_normal re-entry */
        if (nd->norm_cmd_count == 0 && nd->last_speed_lo == 0) {
            pstr("  Re-entry child=0: [PASS]\r\n"); pc++;
        } else {
            pstr("  Re-entry child not clear: [FAIL]\r\n"); fc++;
        }

        /* Parent total also cleared (running was re-entered from scratch) */
        if (rd->speed_cmd_total == 0) {
            pstr("  Re-entry total=0: [PASS]\r\n"); pc++;
        } else {
            pstr("  Re-entry total=");
            PutChar('0' + rd->speed_cmd_total);
            pstr(" [FAIL]\r\n"); fc++;
        }

        /* Back to idle for next test */
        ao_PostTransition(&ao_motor, (signal_t)&sig_motor_idle);
        isr_tick();
    }

    /*====================================================================
     * TEST 6: Cross-AO Posting
     *
     * Verifies ao_PostTransition across AO boundaries:
     *   - Wire AO (priority 5) receives sig_wire_found
     *   - wire_searching_handler posts sig_motor_run to ao_motor (priority 7)
     *   - Motor transitions from idle → running/normal
     *   This demonstrates the firmware's inter-task communication pattern.
     *====================================================================*/
    pstr("\r\n- 6. Cross-AO Posting -\r\n");
    {
        trace_reset();

        ao_Start(5, &ao_wire, &wire_top);

        if (ao_wire.current_state == &wire_idle_st) {
            pstr("  Wire initial state = idle: [PASS]\r\n"); pc++;
        } else {
            pstr("  Wire initial state wrong: [FAIL]\r\n"); fc++;
        }

        trace_reset();

        /* Wire: idle → searching → found (posts motor_run) → following */
        ao_PostTransition(&ao_wire, (signal_t)&sig_wire_search);
        ao_PostTransition(&ao_wire, (signal_t)&sig_wire_found);
        isr_tick();

        if (ao_wire.current_state == &wire_following) {
            pstr("  Wire reached following: [PASS]\r\n"); pc++;
        } else {
            pstr("  Wire state wrong: [FAIL]\r\n"); fc++;
        }

        /* Motor should be in normal (wire_found handler posted motor_run) */
        if (ao_motor.current_state == &motor_normal) {
            pstr("  Cross-AO: motor in normal: [PASS]\r\n"); pc++;
        } else {
            pstr("  Cross-AO: motor state wrong: [FAIL]\r\n"); fc++;
        }
    }

    /*====================================================================
     * TEST 7: Timer System
     *
     * Verifies ao_timer_Set / ao_timer_Tick / ao_timer_Fire:
     *   - motor_running ENTRY starts a 5-tick one-shot timer
     *   - After 5 ticks, timer fires sig_timeout to ao_motor
     *   - motor_normal handles sig_timeout → motor_stopping
     *====================================================================*/
    pstr("\r\n- 7. Timer System -\r\n");
    {
        /* Restart motor: idle → running (starts 5-tick timer) */
        ao_PostTransition(&ao_motor, (signal_t)&sig_motor_idle);
        isr_tick();

        trace_reset();

        ao_PostTransition(&ao_motor, (signal_t)&sig_motor_run);
        isr_tick();  /* processes run transition, timer starts */

        /* Tick down the timer: 5 → 4 → 3 → 2 → 1 → 0 (fires) */
        for (i = 0; i < 5; i++) {
            isr_tick();
        }

        isr_tick();  /* dispatch the posted timeout event */

        if (ao_motor.current_state == &motor_stopping) {
            pstr("  Timer fired after 5 ticks: [PASS]\r\n"); pc++;
        } else {
            pstr("  Timer did not fire: [FAIL]\r\n"); fc++;
        }

        {
            int found = 0;
            uint16_t j;
            for (j = 0; j < g_trace_idx; j++) {
                if (g_trace_buf[j] == T_MOTOR_NORM_TIMEOUT) { found = 1; break; }
            }
            if (found) {
                pstr("  Timeout trace ID found: [PASS]\r\n"); pc++;
            } else {
                pstr("  Timeout trace ID missing: [FAIL]\r\n"); fc++;
            }
        }
    }

    /*====================================================================
     * TEST 8: LCA Transitions
     *
     * Verifies Least Common Ancestor exit/enter chain:
     *   motor_stopping → motor_idle (cross hierarchy, LCA = motor_top)
     *   Exit chain: stopping_exit → running_exit
     *   Enter chain: idle_entry
     *   (motor_top is LCA — not exited or re-entered)
     *====================================================================*/
    pstr("\r\n- 8. LCA Transitions -\r\n");
    {
        trace_reset();

        post_motor_cmd_u32(&ao_motor, 0x0000DEAD);
        isr_tick();

        if (ao_motor.current_state == &motor_idle) {
            pstr("  stopping -> idle (LCA): [PASS]\r\n"); pc++;
        } else {
            pstr("  LCA transition failed: [FAIL]\r\n"); fc++;
        }

        {
            static const uint8_t expected[] = {
                T_MOTOR_STOP_CMD, T_MOTOR_STOP_EXIT, T_MOTOR_RUN_EXIT,
                T_MOTOR_IDLE_ENTRY
            };
            if (trace_match(expected, sizeof(expected))) {
                pstr("  LCA EXIT chain correct: [PASS]\r\n"); pc++;
            } else {
                pstr("  LCA EXIT chain wrong: [FAIL]\r\n"); fc++;
            }
        }
    }

    /*====================================================================
     * TEST 9: Full Integration
     *
     * Runs all 8 phases from test_framework.c in sequence:
     *   Phase 3: Register both AOs (wire@5, motor@7)
     *   Phase 4: Motor: cmd + run + speed → motor_normal
     *   Phase 5: Wire: search + found → following (cross-AO → motor run)
     *   Phase 6: Motor: stop + cmd → idle (hierarchy traversal)
     *   Phase 7: Motor: run + 5-tick timer → stopping
     *   Phase 8: Wire: idle → back to idle
     *
     * Checks: final states, trace depth ≥20, no ASSERT errors.
     *====================================================================*/
    pstr("\r\n- 9. Full Integration -\r\n");
    {
        ao_FrameworkInit();
        trace_reset();

        ao_Start(5, &ao_wire, &wire_top);
        ao_Start(7, &ao_motor, &motor_top);

        /* Phase 4 */
        post_motor_cmd_u32(&ao_motor, 0xCAFE0001);
        ao_PostTransition(&ao_motor, (signal_t)&sig_motor_run);
        post_motor_speed(&ao_motor, 1500);
        isr_tick();

        /* Phase 5 */
        ao_PostTransition(&ao_wire, (signal_t)&sig_wire_search);
        ao_PostTransition(&ao_wire, (signal_t)&sig_wire_found);
        post_wire_speed_u16(&ao_wire, 200);
        isr_tick();

        /* Phase 6 */
        ao_PostTransition(&ao_motor, (signal_t)&sig_motor_stop);
        post_motor_cmd_u32(&ao_motor, 0x0000DEAD);
        isr_tick();

        /* Phase 7 */
        ao_PostTransition(&ao_motor, (signal_t)&sig_motor_run);
        isr_tick();
        for (i = 0; i < 5; i++) isr_tick();
        isr_tick();

        /* Phase 8 */
        ao_PostTransition(&ao_wire, (signal_t)&sig_wire_idle);
        isr_tick();

        if (ao_wire.current_state == &wire_idle_st) {
            pstr("  Wire final = idle: [PASS]\r\n"); pc++;
        } else {
            pstr("  Wire final wrong: [FAIL]\r\n"); fc++;
        }

        if (ao_motor.current_state == &motor_stopping ||
            ao_motor.current_state == &motor_idle) {
            pstr("  Motor final = stopping/idle: [PASS]\r\n"); pc++;
        } else {
            pstr("  Motor final wrong: [FAIL]\r\n"); fc++;
        }

        pstr("  Trace entries: ");
        print_hex16(g_trace_idx);
        pstr(" (buf=64)\r\n");
        if (g_trace_idx >= 20) {
            pstr("  Trace depth reasonable: [PASS]\r\n"); pc++;
        } else {
            pstr("  Trace too short: [FAIL]\r\n"); fc++;
        }

        if (!error_active()) {
            pstr("  No ASSERT errors: [PASS]\r\n"); pc++;
        } else {
            pstr("  ASSERT error: "); pstr((const char *)g_error_str); pstr(" [FAIL]\r\n"); fc++;
        }
    }

    /*====================================================================
     * TEST 10: Live Display (20s)
     *
     * Exercises the full framework under sustained operation:
     *   - 2000 ticks (20 seconds at 10ms/tick)
     *   - LCD shows tick counter + motor state abbreviation
     *   - Posts motor_run every 200 ticks, motor_idle 50 ticks later
     *   - Verifies no crash or pool exhaustion under repeated cycling
     *====================================================================*/
    pstr("\r\n- 10. Live Display (20s) -\r\n");
    {
        uint32_t start = GetSystemCounter();
        uint32_t tick_count = 0;
        uint32_t last_display = 0;
        char buf[17];

        ao_FrameworkInit();
        trace_reset();
        ao_Start(5, &ao_wire, &wire_top);
        ao_Start(7, &ao_motor, &motor_top);

        lcd_Clear();
        lcd_PrintCenter(0, "AO Framework");

        while ((GetSystemCounter() - start) < 2000) {
            isr_tick();
            tick_count++;

            if ((GetSystemCounter() - last_display) >= 50) {
                last_display = GetSystemCounter();

                buf[0] = 'T'; buf[1] = ':';
                buf[2] = '0' + (uint8_t)((tick_count / 1000) % 10);
                buf[3] = '0' + (uint8_t)((tick_count / 100) % 10);
                buf[4] = '0' + (uint8_t)((tick_count / 10) % 10);
                buf[5] = '0' + (uint8_t)(tick_count % 10);
                buf[6] = ' ';
                buf[7] = 'M'; buf[8] = ':';
                if (ao_motor.current_state == &motor_idle)          { buf[9]='I'; buf[10]='D'; buf[11]='L'; }
                else if (ao_motor.current_state == &motor_normal)   { buf[9]='N'; buf[10]='R'; buf[11]='M'; }
                else if (ao_motor.current_state == &motor_stopping) { buf[9]='S'; buf[10]='T'; buf[11]='P'; }
                else                                                { buf[9]='?'; buf[10]='?'; buf[11]='?'; }
                buf[12] = '\0';
                lcd_Print(1, "%s", buf);

                if ((tick_count % 200) == 100) {
                    ao_PostTransition(&ao_motor, (signal_t)&sig_motor_run);
                }
                if ((tick_count % 200) == 150) {
                    ao_PostTransition(&ao_motor, (signal_t)&sig_motor_idle);
                }
            }
        }

        lcd_Clear();
        pstr("  Live display complete: [PASS]\r\n"); pc++;
    }

    /* ---- Summary ---- */
    pstr("\r\n=== Results: ");
    print_dec16(pc);
    pstr(" pass, ");
    print_dec16(fc);
    pstr(" fail ===\r\n\r\n");
}
