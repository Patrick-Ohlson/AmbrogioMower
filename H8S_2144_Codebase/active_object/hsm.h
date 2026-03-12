/*============================================================================
 * hsm.h — Hierarchical State Machine engine, Active Object, Timer, Events
 *
 * Reconstructed from Husqvarna L200 firmware (Renesas H8S/2144)
 * Original source: lib/active/framework/hsm.c
 *
 * Key design decisions from the firmware:
 *   - Signal = function pointer (event constructor address)
 *   - Sentinel signals ENTRY/INIT/EXIT are static data, distinguished by address
 *   - State handler returns: NULL=handled, HSM_UNHANDLED=pass to parent,
 *     other=transition target
 *   - Each state descriptor has a parent pointer for hierarchy walking
 *============================================================================*/

#ifndef HSM_H
#define HSM_H

#include "smallheap.h"
#include "queue.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * Forward declarations
 *============================================================================*/
typedef struct ao_t ao_t;
typedef struct state_desc_t state_desc_t;
typedef struct ao_timer_t ao_timer_t;

/*============================================================================
 * Signal system — function pointer as signal
 *
 * In the firmware, event->signal stores the address of the ao_post_*
 * function that created the event. State handlers compare against known
 * function addresses. We use a void* (signal_t) to replicate this.
 *============================================================================*/
typedef void *signal_t;

/*============================================================================
 * Sentinel signals — ENTRY / INIT / EXIT
 *
 * Firmware: three adjacent 4-byte slots at 0x1E512/16/1A, all 0xDEADBEEF,
 * distinguished by address. We use three static uint32_t variables.
 *============================================================================*/
extern uint32_t HSM_SIG_ENTRY_DATA;     /* address used as ENTRY signal */
extern uint32_t HSM_SIG_INIT_DATA;      /* address used as INIT signal  */
extern uint32_t HSM_SIG_EXIT_DATA;      /* address used as EXIT signal  */

#define HSM_SIG_ENTRY   ((void *)&HSM_SIG_ENTRY_DATA)
#define HSM_SIG_INIT    ((void *)&HSM_SIG_INIT_DATA)
#define HSM_SIG_EXIT    ((void *)&HSM_SIG_EXIT_DATA)

/*============================================================================
 * State handler return values
 *
 * Firmware uses:
 *   NULL  = event handled (stop dispatch)
 *   ""    = event unhandled (pass to parent state)
 *   other = pointer to target state (transition request)
 *
 * We use a dedicated sentinel for "unhandled" instead of "" for portability.
 *============================================================================*/
extern uint8_t HSM_UNHANDLED_SENTINEL;
#define HSM_HANDLED     NULL
#define HSM_UNHANDLED   ((void *)&HSM_UNHANDLED_SENTINEL)

/* Convenience macro: request transition to a state descriptor */
#define HSM_TRAN(target_state)  ((void *)(target_state))

/*============================================================================
 * Typed access to state-local data within ao->state_data[]
 *
 * In the firmware, each hierarchy level owns a slice of state_data:
 *   this state's data starts at parent->state_data_size offset.
 *   (top-level states with no parent start at 0.)
 *
 * Instead of error-prone raw array indices like ao->state_data[2],
 * define a struct for each level's data and use this macro:
 *
 *   typedef struct { uint8_t flags; uint8_t count; } running_data_t;
 *
 *   running_data_t *d = HSM_STATE_DATA(ao, &motor_top, running_data_t);
 *   d->flags = 0xAA;
 *   d->count++;
 *
 * And auto-calculate state_data_size using sizeof:
 *   .state_data_size = sizeof(running_data_t)           // top-level
 *   .state_data_size = sizeof(running_data_t) + sizeof(normal_data_t) // child
 *============================================================================*/
#define HSM_STATE_DATA(ao, parent_desc, type) \
    ((type *)&(ao)->state_data[(parent_desc)->state_data_size])

/*============================================================================
 * Event layout in memory
 *
 * Offset  Size  Field
 * ------  ----  -----
 * +0x00   4     signal      Function pointer (signal_t)
 * +0x04   1     next        Queue link (handle_t), managed by queue
 * +0x05   1     flags       Reserved
 * +0x06   N     payload     Event-specific data
 *
 * evt_header_t is 6 bytes (not naturally aligned).  Typed event structs
 * that extend it MUST be packed so payload fields start at exactly +6.
 *============================================================================*/
typedef struct {
    signal_t signal;            /* +0x00: event constructor function address */
    handle_t next;              /* +0x04: queue chain link                  */
    uint8_t  flags;             /* +0x05: reserved                         */
    /* payload follows at +0x06 */
} evt_header_t;

/* Minimum event size: 6 bytes (state transition, no payload) */
#define EVT_MIN_SIZE    6

/* Access payload from event header pointer (raw byte access) */
#define EVT_PAYLOAD(hdr)    ((uint8_t *)(hdr) + 6)

/*============================================================================
 * Typed event structs — extend evt_header_t with named payload fields
 *
 * Firmware events carry typed payloads after the 6-byte header.  Instead
 * of raw byte access via EVT_PAYLOAD(), define a packed struct per event
 * type and use sizeof() for allocation:
 *
 *   typedef struct EVT_PACKED {
 *       evt_header_t hdr;
 *       uint32_t     speed;
 *   } evt_motor_speed_t;
 *
 * Constructor:
 *   handle_t h = evt_Alloc(signal, sizeof(evt_motor_speed_t));  // = 10
 *   evt_motor_speed_t *e = EVT_CAST(pool_Resolve(h), evt_motor_speed_t);
 *   e->speed = 1500;
 *
 * Handler:
 *   evt_motor_speed_t *se = EVT_CAST(event, evt_motor_speed_t);
 *   use(se->speed);
 *
 * EVT_PACKED is required because evt_header_t is 6 bytes — without it,
 * the compiler may insert padding before uint16_t/uint32_t fields.
 *============================================================================*/
#define EVT_PACKED  __attribute__((packed))
#define EVT_CAST(ptr, type)  ((type *)(ptr))

/*============================================================================
 * State descriptor — defines a state in the HSM hierarchy (12 bytes)
 *
 * Firmware: state descriptors are in ROM/RAM. Each contains:
 *   +0x00  handler            (4B)  State handler function pointer
 *   +0x04  parent             (4B)  Parent state (NULL = top-level)
 *   +0x08  state_data_size    (4B)  CUMULATIVE byte offset into ao.state_data
 *
 * IMPORTANT — Cumulative offset scheme (verified from firmware):
 *   state_data_size is NOT the size of this state's own local data.
 *   It is the cumulative total: all ancestor data + this state's data.
 *
 *   Example hierarchy (from firmware MowerMain AO):
 *     ROOT:        state_data_size = 0x0E  (14 bytes, root's own data)
 *       child:     state_data_size = 0x0E  (same — no additional data)
 *         gchild:  state_data_size = 0x26  (38 — adds 24 bytes at offset 14)
 *           ggchild: state_data_size = 0x42 (66 — adds 28 bytes at offset 38)
 *
 *   On hsm_EnterState, only the DELTA between parent's size and this
 *   state's size is cleared. Parent data is preserved across child entry.
 *
 *   State handlers access their local data via computed offsets:
 *     parent vars:  ao->state_data[0 .. parent->state_data_size - 1]
 *     child vars:   ao->state_data[parent->state_data_size .. this->state_data_size - 1]
 *============================================================================*/

/**
 * State handler function signature
 * @ao:    Pointer to the Active Object
 * @event: Pointer to event (evt_header_t*), or sentinel signal
 *
 * Returns:
 *   HSM_HANDLED    — event was consumed, stop dispatch
 *   HSM_UNHANDLED  — pass event to parent state
 *   HSM_TRAN(desc) — request transition to target state
 */
typedef void *(*state_handler_fn)(ao_t *ao, void *event);

struct state_desc_t {
    state_handler_fn handler;       /* State handler function              */
    const state_desc_t *parent;     /* Parent state (NULL = top-level)     */
    uint32_t state_data_size;       /* Cumulative bytes used through this
                                     * state level.  hsm_EnterState clears
                                     * only [parent->size .. this->size).  */
};

/*============================================================================
 * Active Object structure (matches firmware layout concept)
 *
 * Firmware layout:
 *   +0x00  queue1 (2B)       Primary event queue
 *   +0x02  queue2 (2B)       Internal/secondary queue
 *   +0x04  priority (2B)     Priority level
 *   +0x06  dispatch_fn (4B)  HSM dispatch function pointer
 *   +0x0A  current_handle    Handle of event being processed
 *   +0x0C  event_dispatch    Default dispatch callback
 *   +0x10  next_ao (4B)      Linked list: next AO at same priority
 *============================================================================*/
struct ao_t {
    queue_t             queue1;         /* Primary event queue               */
    queue_t             queue2;         /* Internal/secondary queue          */
    uint16_t            priority;       /* Priority level (0 = highest)      */
    const state_desc_t *current_state;  /* Current HSM state descriptor      */
    handle_t            current_handle; /* Handle of event being dispatched  */
    uint8_t             pad;            /* Alignment padding                 */
    ao_t               *next_ao;        /* Next AO at same priority (chain)  */

    /* HSM state-local data — shared across all hierarchy levels.
     * Firmware AOs use up to 0x152 (338) bytes; 16 is enough for tests.
     * Each hierarchy level owns a slice: [parent_size .. this_size).
     * hsm_EnterState only clears the NEW slice, preserving parent data.
     * Handlers access their vars at known offsets within their slice. */
    uint8_t             state_data[16]; /* State-local scratch area          */
};

/*============================================================================
 * Timer structure (doubly-linked list, 24 bytes)
 *
 * Firmware layout:
 *   +0x00  prev (4B)
 *   +0x04  next (4B)
 *   +0x08  callback (4B)    — state handler / callback function
 *   +0x0C  ao_ptr (4B)      — AO to post event to
 *   +0x10  timeout (4B)     — ticks remaining
 *   +0x14  reload (4B)      — auto-reload (0 = one-shot)
 *============================================================================*/
struct ao_timer_t {
    ao_timer_t *prev;           /* Previous timer in list (NULL = head)  */
    ao_timer_t *next;           /* Next timer in list (NULL = tail)      */
    signal_t   callback;        /* Signal/function to post on expiry     */
    ao_t      *ao_ptr;          /* Target AO for the timer event         */
    uint32_t   timeout;         /* Ticks remaining until expiry          */
    uint32_t   reload;          /* Auto-reload value (0 = one-shot)      */
};

/*============================================================================
 * HSM Engine API
 *============================================================================*/

/**
 * hsm_Constructor - Create and enter an HSM
 * @ao:            Active Object containing the HSM
 * @initial_state: Initial state descriptor to enter
 *
 * Sets current state, sends ENTRY to initial state, then runs the
 * INIT chain to drill into nested initial substates.
 */
void hsm_Constructor(ao_t *ao, const state_desc_t *initial_state);

/**
 * hsm_Dispatch - Dispatch an event through the HSM
 * @ao:    Active Object
 * @event: Event pointer (evt_header_t* or sentinel signal)
 *
 * Calls current state handler. If unhandled, walks to parent states.
 * If a transition is requested, performs exit chain, LCA transition,
 * enter chain, and init chain.
 */
void hsm_Dispatch(ao_t *ao, void *event);

/**
 * hsm_EnterState - Enter a state (recursive parent-first entry)
 * @ao:     Active Object
 * @target: State to enter (enters parents first)
 *
 * Clears state-local data, then sends ENTRY signal to the state handler.
 */
void hsm_EnterState(ao_t *ao, const state_desc_t *target);

/**
 * hsm_InitChain - Process initial-transition chain
 * @ao: Active Object
 *
 * Repeatedly sends INIT to current state. If handler returns a transition
 * target, enters that state and continues. Stops on HSM_HANDLED or
 * HSM_UNHANDLED.
 */
void hsm_InitChain(ao_t *ao);

/**
 * hsm_Transition - Full LCA state transition
 * @ao:     Active Object
 * @target: Target state descriptor to transition to
 *
 * Computes Least Common Ancestor between current and target states,
 * exits from current up to LCA, enters from LCA down to target,
 * then runs init chain.
 */
void hsm_Transition(ao_t *ao, const state_desc_t *target);

/**
 * hsm_ExitChain - Exit states up to (but not including) a target ancestor
 * @ao:     Active Object
 * @target: Ancestor state to stop at
 */
void hsm_ExitChain(ao_t *ao, const state_desc_t *target);

/**
 * hsm_CalcDepth - Calculate nesting depth of a state
 * @state: State descriptor
 *
 * Returns: Number of parent levels above this state (0 = top-level).
 */
int16_t hsm_CalcDepth(const state_desc_t *state);

/*============================================================================
 * Active Object API
 *============================================================================*/

/**
 * ao_Constructor - Initialize an Active Object
 * @ao:       AO instance to initialize
 * @priority: Priority level (0 = highest)
 */
void ao_Constructor(ao_t *ao, uint16_t priority);

/**
 * ao_ProcessQueue - Dequeue and dispatch all pending events
 * @ao: Active Object
 *
 * Processes primary queue, then secondary queue, until both empty.
 * Frees each event after dispatch.
 */
void ao_ProcessQueue(ao_t *ao);

/**
 * ao_PostEvent - Route event to an AO queue
 * @ao:     Target AO (NULL = post to self / current context's queue2)
 * @handle: Event handle
 *
 * If ao != NULL: enqueue to ao's queue1 and signal scheduler.
 * If ao == NULL: enqueue to current context's queue2 (self-post).
 */
void ao_PostEvent(ao_t *ao, handle_t handle);

/**
 * ao_EnqueueAndSignal - Enqueue event and set scheduler ready bit
 * @ao:     Target AO
 * @handle: Event handle
 */
void ao_EnqueueAndSignal(ao_t *ao, handle_t handle);

/*============================================================================
 * Event Allocation API
 *============================================================================*/

/**
 * evt_Alloc - Allocate and initialize an event
 * @signal: Signal value (typically function pointer of the ao_post_* fn)
 * @size:   Total event size in bytes (>= EVT_MIN_SIZE)
 *
 * Allocates from pool, zero-fills, stores signal at offset 0.
 * Returns: Pool handle, or POOL_HANDLE_NULL on failure.
 */
handle_t evt_Alloc(signal_t signal, uint16_t size);

/**
 * ao_PostTransition - Post a state transition event
 * @ao:       Target AO (NULL = self)
 * @state_fn: Signal value identifying the target state
 *
 * Allocates a minimal 6-byte event with the given signal and posts it.
 * This is how all ao_post_state_* functions work internally.
 */
void ao_PostTransition(ao_t *ao, signal_t state_fn);

/*============================================================================
 * Timer API
 *============================================================================*/

/**
 * ao_timer_ResetAll - Clear the active timer list
 *
 * Resets internal timer list head to NULL. Call during framework init
 * to ensure no stale timer pointers persist across re-initialization.
 */
void ao_timer_ResetAll(void);

/**
 * ao_timer_Set - Add, modify, or remove an AO timer
 * @tmr:      Timer structure (caller-owned)
 * @ao:       Target AO to receive timer event
 * @callback: Signal to post when timer fires
 * @timeout:  Ticks until expiry (0 = remove timer)
 * @repeat:   If true, auto-reload after firing
 *
 * Note: Named ao_timer_Set to avoid conflict with timer/timer.h timer_Set.
 * Firmware equivalent: timer_Set (0x16AC0).
 */
void ao_timer_Set(ao_timer_t *tmr, ao_t *ao, signal_t callback,
                  uint32_t timeout, bool repeat);

/**
 * ao_timer_Fire - Called when a timer expires
 * @tmr: Timer that expired
 *
 * Posts the timer's signal to its target AO, reloads or removes.
 * Firmware equivalent: timer_Fire (0x16A5E).
 */
void ao_timer_Fire(ao_timer_t *tmr);

/**
 * ao_timer_Tick - Advance all active AO timers by one tick
 *
 * Should be called from a periodic interrupt or tick source.
 * Decrements timeout for all active timers. Fires any that reach zero.
 */
void ao_timer_Tick(void);

/*============================================================================
 * Scheduler access (defined in schedulerrt.c, needed by ao_EnqueueAndSignal)
 *============================================================================*/
extern void ao_SignalReady(uint8_t priority);

/*============================================================================
 * Current context (needed for self-posting)
 *============================================================================*/
extern ao_t *ao_current_context;

#endif /* HSM_H */
