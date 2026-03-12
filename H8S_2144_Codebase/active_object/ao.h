/*============================================================================
 * ao.h — Active Object Framework umbrella header
 *
 * Single include for the complete AO framework:
 *   - SmallHeap fixed-block memory pools
 *   - Event queues (singly-linked, handle-based)
 *   - HSM engine (hierarchical state machine with LCA transitions)
 *   - Active Objects (event-driven state machines with priority scheduling)
 *   - Timer system (one-shot and repeating, doubly-linked list)
 *   - Priority-based cooperative scheduler
 *   - Error reporting (assert + halt)
 *
 * Reconstructed from Husqvarna L200 firmware (Renesas H8S/2144)
 * Original source paths from firmware assert strings:
 *   lib/active/framework/hsm.c
 *   lib/active/framework/schedulerrt.c
 *   lib/active/options/smallheap.c
 *   lib/active/xio/queue.c
 *
 * Usage:
 *   #include "active_object/ao.h"
 *   ao_FrameworkInit();          // pools + scheduler
 *   ao_Start(priority, &ao, &initial_state);
 *============================================================================*/

#ifndef AO_H
#define AO_H

#include "smallheap.h"
#include "queue.h"
#include "hsm.h"
#include "schedulerrt.h"
#include "error.h"

/*============================================================================
 * ao_FrameworkInit — One-call setup matching firmware OldMain pattern
 *
 * Initializes:
 *   1. Error system (clear error flag)
 *   2. Memory pools (Pool 0: 24B x 16, Pool 1: 48B x 8)
 *   3. Scheduler (clear priority table + ready set)
 *
 * After this call, use ao_Start() to register Active Objects.
 *============================================================================*/
void ao_FrameworkInit(void);

#endif /* AO_H */
