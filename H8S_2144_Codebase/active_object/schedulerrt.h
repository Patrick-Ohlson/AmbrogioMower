/*============================================================================
 * schedulerrt.h — Priority-based cooperative AO scheduler
 *
 * Reconstructed from Husqvarna L200 firmware (Renesas H8S/2144)
 * Original source: lib/active/framework/schedulerrt.c
 *
 * The scheduler uses a priority bitmask (ao_ready_set) and a table of
 * AO linked-lists (one per priority level). When ao_SignalReady is called
 * and the scheduler is idle, it runs immediately (cooperative dispatch).
 *============================================================================*/

#ifndef SCHEDULERRT_H
#define SCHEDULERRT_H

#include "hsm.h"

/*--- Constants ---*/
#define SCHED_MAX_PRIORITIES    8       /* Firmware uses 0-8; 8 saves 32B BSS */

/*--- API Functions ---*/

/**
 * sched_Init - Initialize the scheduler
 *
 * Clears the priority table, ready set, and current context.
 * Must be called before ao_Start().
 */
void sched_Init(void);

/**
 * ao_Start - Register an Active Object and send its initial event
 * @priority:      Priority level (0 = highest)
 * @ao:            AO instance (already has memory allocated)
 * @initial_state: Initial HSM state descriptor
 *
 * Initializes the AO, inserts into the priority table, constructs the HSM.
 */
void ao_Start(uint16_t priority, ao_t *ao, const state_desc_t *initial_state);

/**
 * ao_Scheduler - Run the priority-based cooperative scheduler
 *
 * Processes all ready AOs in priority order (lowest number first).
 * For each priority with ready bit set, walks the AO linked list
 * and calls ao_ProcessQueue() on each AO.
 *
 * Firmware equivalent (0x1688A / ao_event_loop_run).
 */
void ao_Scheduler(void);

/**
 * ao_SignalReady - Signal that an AO at a given priority has pending events
 * @priority: Priority level to signal
 *
 * If scheduler is idle (ready_set == 0): sets bit AND calls ao_Scheduler()
 * directly for immediate dispatch.
 * If scheduler is active: just sets the bit.
 *
 * Firmware equivalent (0x16A18 / ao_signal_ready).
 */
void ao_SignalReady(uint8_t priority);

#endif /* SCHEDULERRT_H */
