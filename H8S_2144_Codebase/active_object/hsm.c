/*============================================================================
 * hsm.c — HSM engine, Active Object, Event allocation, Timer
 *
 * Reconstructed from Husqvarna L200 firmware (Renesas H8S/2144)
 * Original source: lib/active/framework/hsm.c
 *
 * Firmware addresses:
 *   hsm_Constructor  = 0x16628  (DoHSM)
 *   hsm_EnterState   = 0x1642E  (InitHSM)
 *   hsm_InitChain    = 0x164C4  (sub_164C4)
 *   hsm_Transition   = 0x16500  (sub_16500)
 *   hsm_ExitChain    = 0x163E2  (sub_163E2)
 *   hsm_CalcDepth    = 0x1640C  (tm_sub_1640C)
 *   hsm_Dispatch     = 0x16652  (sub_16652)
 *   ao_Constructor   = 0x16740  (sub_16740)
 *   ao_ProcessQueue  = 0x167EE  (ao_dispatch_single_event)
 *   ao_PostEvent     = 0x1683E  (ao_dispatch_event)
 *   ao_EnqueueAndSig = 0x16772  (ao_post_to_ao)
 *   ao_PostTransition= 0x16858  (ao_post_state_transition)
 *   evt_Alloc        = 0x16304  (ao_alloc_event)
 *   ao_timer_Set     = 0x16AC0  (ChangeState)  [renamed to avoid timer.h conflict]
 *   ao_timer_Fire    = 0x16A5E  (sub_16A5E)   [renamed to avoid timer.h conflict]
 *============================================================================*/

#include "hsm.h"
#include "error.h"
#include <string.h>

extern volatile uint8_t g_dbg_site;

/*============================================================================
 * Sentinel signal data — three adjacent uint32_t, all 0xDEADBEEF
 * Firmware: ROM addresses 0x1E512, 0x1E516, 0x1E51A
 * Distinguished by address, not by value.
 *============================================================================*/
uint32_t HSM_SIG_ENTRY_DATA = 0xDEADBEEF;
uint32_t HSM_SIG_INIT_DATA  = 0xDEADBEEF;
uint32_t HSM_SIG_EXIT_DATA  = 0xDEADBEEF;

/* Unhandled sentinel (firmware: "" / empty string) */
uint8_t HSM_UNHANDLED_SENTINEL = 0;

/*============================================================================
 * Current AO context (firmware: ao_current_context at RAM)
 *============================================================================*/
ao_t *ao_current_context = NULL;

/*============================================================================
 * Timer list head (firmware: g_timer_list_head at 0xFFEA72)
 *============================================================================*/
static ao_timer_t *g_timer_list_head = NULL;

/*============================================================================
 *                         HSM ENGINE
 *============================================================================*/

/*--------------------------------------------------------------------------
 * hsm_CalcDepth — Calculate nesting depth of a state
 *
 * Firmware equivalent (0x1640C):
 *   Walk parent chain, count levels.
 *   Returns 0 for top-level state.
 *--------------------------------------------------------------------------*/
int16_t hsm_CalcDepth(const state_desc_t *state)
{
    int16_t depth = 0;

    while (state->parent != NULL) {
        depth++;
        state = state->parent;
    }
    return depth;
}

/*--------------------------------------------------------------------------
 * hsm_EnterState — Enter a state (recursive parent-first entry)
 *
 * Firmware equivalent (0x1642E / InitHSM):
 *   If target != current: recurse to enter parent first.
 *   Clear only the NEW state-local data (delta between parent and child).
 *   Send ENTRY signal to state handler.
 *
 * Cumulative offset scheme (verified from firmware disassembly):
 *   state_data_size is the cumulative byte count through this level.
 *   Only bytes [parent->state_data_size .. target->state_data_size) are
 *   cleared on entry.  This preserves parent-level variables when
 *   entering a child state — the firmware's "relative register offset"
 *   mechanism for passing variables between hierarchy levels.
 *
 *   Example: motor_running.state_data_size = 2 (owns bytes [0..1])
 *            motor_normal.state_data_size  = 4 (owns bytes [2..3])
 *   Entering motor_normal clears only bytes [2..3]; bytes [0..1]
 *   (motor_running's data) are preserved.
 *--------------------------------------------------------------------------*/
void hsm_EnterState(ao_t *ao, const state_desc_t *target)
{
    if (target == NULL)
        return;

    /* If target has a parent that isn't already our current state,
       enter parent first (recursive) */
    if (target->parent != NULL && target->parent != ao->current_state) {
        hsm_EnterState(ao, target->parent);
    }

    /* Clear only the NEW state-local data slice for this level.
     * Firmware: tm_ClearMemory(ao + parent->state_data_size,
     *                          target->state_data_size - parent->state_data_size)
     * Parent data at lower offsets is preserved. */
    if (target->state_data_size > 0) {
        uint32_t clear_start = 0;
        uint32_t clear_len;

        if (target->parent != NULL) {
            clear_start = target->parent->state_data_size;
        }
        clear_len = target->state_data_size - clear_start;

        if (clear_len > 0 && clear_start < sizeof(ao->state_data)) {
            /* Clamp to state_data[] bounds */
            if (clear_start + clear_len > sizeof(ao->state_data))
                clear_len = sizeof(ao->state_data) - clear_start;
            memset(&ao->state_data[clear_start], 0, clear_len);
        }
    }

    /* Send ENTRY signal */
    ao->current_state = target;
    target->handler(ao, HSM_SIG_ENTRY);
}

/*--------------------------------------------------------------------------
 * hsm_InitChain — Process initial-transition chain (drill into substates)
 *
 * Firmware equivalent (0x164C4 / sub_164C4):
 *   Repeatedly send INIT to current state.
 *   If handler returns a transition target, enter it and continue.
 *   Stop on HANDLED or UNHANDLED.
 *--------------------------------------------------------------------------*/
void hsm_InitChain(ao_t *ao)
{
    void *result;

    while (1) {
        result = ao->current_state->handler(ao, HSM_SIG_INIT);

        if (result == HSM_HANDLED || result == HSM_UNHANDLED)
            break;

        /* result is a state_desc_t* — transition target */
        {
            const state_desc_t *target = (const state_desc_t *)result;
            hsm_EnterState(ao, target);
            ao->current_state = target;
        }
    }
}

/*--------------------------------------------------------------------------
 * hsm_ExitChain — Exit states up to (but not including) target ancestor
 *
 * Firmware equivalent (0x163E2 / sub_163E2):
 *   Walk from current state up to ancestor, sending EXIT to each.
 *--------------------------------------------------------------------------*/
void hsm_ExitChain(ao_t *ao, const state_desc_t *ancestor)
{
    const state_desc_t *s = ao->current_state;

    while (s != ancestor && s != NULL) {
        s->handler(ao, HSM_SIG_EXIT);
        s = s->parent;
    }
}

/*--------------------------------------------------------------------------
 * hsm_Transition — Full LCA state transition
 *
 * Firmware equivalent (0x16500 / sub_16500):
 *   1. If transitioning to self: just send EXIT, then re-enter
 *   2. Otherwise: compute depth of source and target,
 *      exit source side up to LCA, enter target side down from LCA,
 *      then run init chain.
 *
 * This implements the Least Common Ancestor algorithm from the firmware.
 *--------------------------------------------------------------------------*/
void hsm_Transition(ao_t *ao, const state_desc_t *target)
{
    const state_desc_t *source = ao->current_state;
    const state_desc_t *s, *t;
    int16_t s_depth, t_depth;

    if (source == target) {
        /* Self-transition: exit then re-enter same state.
         * Set current_state to parent so hsm_EnterState only
         * enters the target state itself, not all ancestors. */
        source->handler(ao, HSM_SIG_EXIT);
        ao->current_state = source->parent;  /* LCA is parent */
        hsm_EnterState(ao, target);
        hsm_InitChain(ao);
        return;
    }

    /* Calculate depths */
    s_depth = hsm_CalcDepth(source);
    t_depth = hsm_CalcDepth(target);

    s = source;
    t = target;

    /* Bring deeper side up to same level */
    while (s_depth > t_depth) {
        s->handler(ao, HSM_SIG_EXIT);
        s = s->parent;
        s_depth--;
    }
    while (t_depth > s_depth) {
        t = t->parent;
        t_depth--;
    }

    /* Walk both up until they meet at LCA */
    while (s != t) {
        ASSERT(s != NULL && t != NULL, "LCA:null parent");
        s->handler(ao, HSM_SIG_EXIT);
        s = s->parent;
        t = t->parent;
    }
    /* s == t == LCA now */

    /* Enter from LCA down to target, then run init chain.
     * Set current_state to LCA so hsm_EnterState recurses down
     * from LCA to target only — NOT from root. */
    ao->current_state = s;
    hsm_EnterState(ao, target);
    hsm_InitChain(ao);
}

/*--------------------------------------------------------------------------
 * hsm_Constructor — Create and enter an HSM
 *
 * Firmware equivalent (0x16628 / DoHSM):
 *   *state_ptr = initial_handler;
 *   hsm_EnterState(state_ptr, 0);
 *   hsm_InitChain(state_ptr);
 *--------------------------------------------------------------------------*/
void hsm_Constructor(ao_t *ao, const state_desc_t *initial_state)
{
    ao->current_state = initial_state;
    hsm_EnterState(ao, initial_state);
    hsm_InitChain(ao);
}

/*--------------------------------------------------------------------------
 * hsm_Dispatch — Dispatch event through HSM with parent walk
 *
 * Firmware equivalent (0x16652 / sub_16652):
 *   Save current state as source.
 *   Call handler; if UNHANDLED, walk to parent.
 *   If HANDLED (NULL), done.
 *   If transition target returned, do exit chain + LCA transition.
 *--------------------------------------------------------------------------*/
void hsm_Dispatch(ao_t *ao, void *event)
{
    const state_desc_t *source = ao->current_state;
    const state_desc_t *walk   = ao->current_state;
    void *result;

    while (walk != NULL) {
        result = walk->handler(ao, event);

        if (result == HSM_HANDLED) {
            /* Event consumed */
            return;
        }

        if (result != HSM_UNHANDLED) {
            /* Transition requested — result is target state_desc_t* */
            const state_desc_t *target = (const state_desc_t *)result;

            /* Exit from current state up to the handling state */
            hsm_ExitChain(ao, walk);

            /* Full LCA transition from walk's context to target */
            ao->current_state = walk;
            hsm_Transition(ao, target);
            return;
        }

        /* Unhandled: walk up to parent state */
        walk = walk->parent;
    }
    /* If we reach here, event was unhandled by all states (silently dropped) */
}

/*============================================================================
 *                       ACTIVE OBJECT
 *============================================================================*/

/*--------------------------------------------------------------------------
 * ao_Constructor — Initialize an Active Object
 *
 * Firmware equivalent (0x16740 / sub_16740):
 *   Init both queues, store priority.
 *--------------------------------------------------------------------------*/
void ao_Constructor(ao_t *ao, uint16_t priority)
{
    queue_Init(&ao->queue1);
    queue_Init(&ao->queue2);
    ao->priority       = priority;
    ao->current_state  = NULL;
    ao->current_handle = POOL_HANDLE_NULL;
    ao->next_ao        = NULL;
    memset(ao->state_data, 0, sizeof(ao->state_data));
}

/*--------------------------------------------------------------------------
 * ao_ProcessQueue — Dequeue and dispatch all pending events
 *
 * Firmware equivalent (0x167EE / ao_dispatch_single_event):
 *   1. Dequeue from queue1, dispatch, free
 *   2. Then process queue2 (internally-generated events)
 *   3. Repeat until both empty
 *--------------------------------------------------------------------------*/
void ao_ProcessQueue(ao_t *ao)
{
    handle_t h;

    /* Process primary queue */
    while (1) {
        h = queue_Dequeue(&ao->queue1);
        ao->current_handle = h;

        if (h == POOL_HANDLE_NULL)
            break;

        /* Dispatch through HSM */
        {
            g_dbg_site = 0x30;
            evt_header_t *evt = (evt_header_t *)pool_Resolve(h);
            if (evt == NULL) break;  /* safety: bad handle */
            hsm_Dispatch(ao, evt);
        }
        pool_Free(h);

        /* Process any events posted to queue2 during dispatch */
        while (1) {
            handle_t h2 = queue_Dequeue(&ao->queue2);
            if (h2 == POOL_HANDLE_NULL)
                break;

            ao->current_handle = h2;
            {
                g_dbg_site = 0x31;
                evt_header_t *evt = (evt_header_t *)pool_Resolve(h2);
                if (evt == NULL) break;  /* safety */
                hsm_Dispatch(ao, evt);
            }
            pool_Free(h2);
        }
    }
}

/*--------------------------------------------------------------------------
 * ao_PostEvent — Route event to AO queue
 *
 * Firmware equivalent (0x1683E / ao_dispatch_event):
 *   If ao != NULL: ao_EnqueueAndSignal(ao, handle)
 *   If ao == NULL: queue_Enqueue(current_context->queue2, handle)
 *--------------------------------------------------------------------------*/
void ao_PostEvent(ao_t *ao, handle_t handle)
{
    if (ao != NULL) {
        ao_EnqueueAndSignal(ao, handle);
    } else {
        /* Self-post to current context's internal queue */
        ASSERT(ao_current_context != NULL, "PostEvt:no ctx");
        queue_Enqueue(&ao_current_context->queue2, handle);
    }
}

/*--------------------------------------------------------------------------
 * ao_EnqueueAndSignal — Enqueue + set scheduler ready bit
 *
 * Firmware equivalent (0x16772 / ao_post_to_ao):
 *   queue_Enqueue(&ao->queue1, handle);
 *   ao_SignalReady(ao->priority);
 *--------------------------------------------------------------------------*/
void ao_EnqueueAndSignal(ao_t *ao, handle_t handle)
{
    queue_Enqueue(&ao->queue1, handle);
    ao_SignalReady(ao->priority);
}

/*============================================================================
 *                       EVENT ALLOCATION
 *============================================================================*/

/*--------------------------------------------------------------------------
 * evt_Alloc — Allocate and initialize an event
 *
 * Firmware equivalent (0x16304 / ao_alloc_event):
 *   handle = pool_Alloc(size);
 *   ptr = pool_Resolve(handle);
 *   memset(ptr, 0, size);
 *   ptr->signal = signal_fn;
 *   return handle;
 *--------------------------------------------------------------------------*/
handle_t evt_Alloc(signal_t signal, uint16_t size)
{
    handle_t h;
    evt_header_t *evt;

    ASSERT(size >= EVT_MIN_SIZE, "Alloc:sz<6");

    h = pool_Alloc(size);
    ASSERT(h != POOL_HANDLE_NULL, "Alloc:pool full");

    g_dbg_site = 0x40;  /* evt_Alloc: resolve after alloc */
    evt = (evt_header_t *)pool_Resolve(h);
    memset(evt, 0, size);
    evt->signal = signal;

    return h;
}

/*--------------------------------------------------------------------------
 * ao_PostTransition — Post a minimal state transition event (6 bytes)
 *
 * Firmware equivalent (0x16858 / ao_post_state_transition):
 *   handle = evt_Alloc(state_fn, 6);
 *   if (ao == NULL) queue_Enqueue(current->queue2, handle)
 *   else ao_EnqueueAndSignal(ao, handle)
 *--------------------------------------------------------------------------*/
void ao_PostTransition(ao_t *ao, signal_t state_fn)
{
    handle_t h = evt_Alloc(state_fn, EVT_MIN_SIZE);

    if (ao == NULL) {
        ASSERT(ao_current_context != NULL, "PostTran:no ctx");
        queue_Enqueue(&ao_current_context->queue2, h);
    } else {
        ao_EnqueueAndSignal(ao, h);
    }
}

/*============================================================================
 *                       TIMER SYSTEM
 *============================================================================*/

/*--- Internal: Remove timer from doubly-linked list ---*/
static void timer_Unlink(ao_timer_t *tmr)
{
    if (tmr->prev != NULL) {
        tmr->prev->next = tmr->next;
    } else {
        g_timer_list_head = tmr->next;  /* was head of list */
    }

    if (tmr->next != NULL) {
        tmr->next->prev = tmr->prev;
    }

    tmr->prev = NULL;
    tmr->next = NULL;
}

/*--- Internal: Insert timer at head of list ---*/
static void timer_InsertHead(ao_timer_t *tmr)
{
    tmr->prev = NULL;
    tmr->next = g_timer_list_head;

    if (g_timer_list_head != NULL) {
        g_timer_list_head->prev = tmr;
    }

    g_timer_list_head = tmr;
}

/*--- Internal: Check if timer is in the active list ---*/
static bool timer_IsActive(ao_timer_t *tmr)
{
    return (tmr->timeout != 0) || (tmr == g_timer_list_head);
}

/*--------------------------------------------------------------------------
 * ao_timer_ResetAll — Clear the timer list (for framework re-init)
 *
 * Resets g_timer_list_head to NULL. Must be called before re-using
 * the timer system (e.g., when tests re-init the framework).
 *--------------------------------------------------------------------------*/
void ao_timer_ResetAll(void)
{
    g_timer_list_head = NULL;
}

/*--------------------------------------------------------------------------
 * ao_timer_Set — Add, modify, or remove a timer
 *
 * Firmware equivalent (0x16AC0 / ChangeState):
 *   If timeout == 0: remove from list if present.
 *   If timeout != 0 and not in list: insert at head.
 *   Store callback, timeout, reload.
 *
 * Named ao_timer_Set to avoid conflict with timer/timer.h timer_Set.
 *--------------------------------------------------------------------------*/
void ao_timer_Set(ao_timer_t *tmr, ao_t *ao, signal_t callback,
                  uint32_t timeout, bool repeat)
{
    bool was_active = timer_IsActive(tmr);

    if (timeout == 0) {
        /* Remove timer */
        if (was_active) {
            timer_Unlink(tmr);
        }
        tmr->timeout = 0;
        tmr->reload  = 0;
        return;
    }

    if (!was_active) {
        timer_InsertHead(tmr);
    }

    tmr->callback = callback;
    tmr->ao_ptr   = ao;
    tmr->timeout  = timeout;
    tmr->reload   = repeat ? timeout : 0;
}

/*--------------------------------------------------------------------------
 * ao_timer_Fire — Timer expired handler
 *
 * Firmware equivalent (0x16A5E / sub_16A5E):
 *   Post the timer's signal to its AO.
 *   Reload or remove.
 *--------------------------------------------------------------------------*/
void ao_timer_Fire(ao_timer_t *tmr)
{
    /* Post a transition event using the timer's callback as signal */
    ao_PostTransition(tmr->ao_ptr, tmr->callback);

    /* Reload or remove */
    tmr->timeout = tmr->reload;
    if (tmr->reload == 0) {
        timer_Unlink(tmr);
    }
}

/*--------------------------------------------------------------------------
 * ao_timer_Tick — Advance all active timers by one tick
 *
 * Not directly in the firmware decompilation (called from ISR context),
 * but this is the logical tick driver for the timer list.
 *--------------------------------------------------------------------------*/
void ao_timer_Tick(void)
{
    ao_timer_t *tmr = g_timer_list_head;
    ao_timer_t *next;

    while (tmr != NULL) {
        next = tmr->next;   /* save next: timer_Fire may unlink */

        if (tmr->timeout > 0) {
            tmr->timeout--;
            if (tmr->timeout == 0) {
                ao_timer_Fire(tmr);
            }
        }

        tmr = next;
    }
}
