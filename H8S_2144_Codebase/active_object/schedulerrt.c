/*============================================================================
 * schedulerrt.c — Priority-based cooperative AO scheduler
 *
 * Reconstructed from Husqvarna L200 firmware (Renesas H8S/2144)
 * Original source: lib/active/framework/schedulerrt.c
 *
 * Firmware addresses:
 *   ao_Start       = 0x16984  (ScheduleTask)
 *   ao_Scheduler   = 0x1688A  (ao_event_loop_run)
 *   ao_SignalReady  = 0x16A18  (ao_signal_ready)
 *
 * Firmware globals:
 *   g_ao_priority_table = 0xFFEA6A  (off_FFEA6A)
 *   ao_ready_set        = RAM       (bitmask of priorities with events)
 *   ao_current_context  = RAM       (currently executing AO)
 *============================================================================*/

#include "schedulerrt.h"
#include "error.h"
#include <string.h>

/*--- Module globals ---*/

/* Priority table: array of AO linked-list heads, indexed by priority.
 * Firmware: off_FFEA6A / g_ao_priority_table at 0xFFEA6A */
static ao_t *g_priority_table[SCHED_MAX_PRIORITIES];

/* Bitmask of priorities with pending events (bit N = priority N).
 * Firmware: ao_ready_set */
static uint16_t g_ready_set = 0;

/* ao_current_context is declared extern in hsm.h, defined in hsm.c */

/*============================================================================
 * sched_Init — Initialize the scheduler
 *============================================================================*/
void sched_Init(void)
{
    memset(g_priority_table, 0, sizeof(g_priority_table));
    g_ready_set = 0;
    ao_current_context = NULL;
}

/*============================================================================
 * ao_Start — Register AO and construct its HSM
 *
 * Firmware equivalent (0x16984 / ScheduleTask):
 *   assert(thread < NTHREADS);
 *   ao_Constructor(ao, priority, ...);
 *   ao->next_ao = g_priority_table[priority];
 *   g_priority_table[priority] = ao;
 *   ao_PostInitEvent(ao, init_event);   // post initial event
 *
 * We simplify the init event: the HSM constructor already enters
 * the initial state. The initial event would be processed after
 * the scheduler starts.
 *============================================================================*/
void ao_Start(uint16_t priority, ao_t *ao, const state_desc_t *initial_state)
{
    ASSERT(priority < SCHED_MAX_PRIORITIES, "Start:bad prio");

    /* Initialize the AO */
    ao_Constructor(ao, priority);

    /* Insert at head of linked list for this priority level */
    ao->next_ao = g_priority_table[priority];
    g_priority_table[priority] = ao;

    /* Construct the HSM: enter initial state, run init chain */
    hsm_Constructor(ao, initial_state);
}

/*============================================================================
 * ao_Scheduler — Priority-based cooperative scheduler
 *
 * Firmware equivalent (0x1688A / ao_event_loop_run):
 *
 *   1. Save current context, build priority mask
 *   2. While (ready_set & mask) != 0:
 *      a. Find lowest-numbered set bit (highest priority)
 *      b. Clear that bit
 *      c. Walk AO linked list at that priority
 *      d. For each AO: ao_ProcessQueue(ao)
 *   3. Restore previous context
 *
 * The firmware builds a mask from the current context's priority to avoid
 * processing lower-priority AOs when called recursively. For simplicity
 * in this reconstruction, we process all priorities (mask = 0xFFFF for
 * top-level calls).
 *============================================================================*/
void ao_Scheduler(void)
{
    ao_t *saved_context = ao_current_context;
    uint16_t mask;
    uint8_t pri;

    /* Build priority mask: only process priorities up to current level.
     * If no current context (top-level), process all.
     * If in a context, only process higher-priority (lower number) AOs.
     */
    if (ao_current_context == NULL) {
        mask = 0xFFFF;      /* process all priorities */
    } else {
        /* Firmware: builds mask of bits 0..priority-1 */
        mask = (uint16_t)((1U << ao_current_context->priority) - 1);
        if (mask == 0) {
            /* Priority 0 — nothing higher to process */
            ao_current_context = saved_context;
            return;
        }
    }

    while ((g_ready_set & mask) != 0) {
        /* Find lowest-numbered set bit (highest priority) */
        for (pri = 0; pri < SCHED_MAX_PRIORITIES; pri++) {
            uint16_t bit = (uint16_t)(1U << pri);

            if ((g_ready_set & bit) == 0)
                continue;

            /* Clear the ready bit for this priority */
            g_ready_set &= ~bit;

            /* Walk AO linked list at this priority */
            {
                ao_t *ao = g_priority_table[pri];
                while (ao != NULL) {
                    ao_current_context = ao;
                    ao_ProcessQueue(ao);
                    ao = ao->next_ao;
                }
            }
            break;  /* restart from highest priority */
        }
    }

    ao_current_context = saved_context;
}

/*============================================================================
 * ao_SignalReady — Set priority bit; enter scheduler if idle
 *
 * Firmware equivalent (0x16A18 / ao_signal_ready):
 *   If ready_set == 0: set bit, call ao_Scheduler() directly.
 *   If ready_set != 0: just set bit (scheduler loop will pick it up).
 *
 * This is the key mechanism for cooperative immediate dispatch: when the
 * system is idle and an event is posted, the scheduler runs right away.
 *============================================================================*/
void ao_SignalReady(uint8_t priority)
{
    uint16_t bit = (uint16_t)(1U << priority);

    if (g_ready_set == 0) {
        /* Scheduler is idle — set bit and run immediately */
        g_ready_set = bit;
        ao_Scheduler();
    } else {
        /* Scheduler is active — just set the bit */
        g_ready_set |= bit;
    }
}
