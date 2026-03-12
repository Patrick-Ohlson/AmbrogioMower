/*============================================================================
 * queue.h — Singly-linked event queue using pool handles
 *
 * Reconstructed from Husqvarna L200 firmware (Renesas H8S/2144)
 * Original source: lib/active/xio/queue.c
 *
 * Events are linked through a "next" field at offset +4 in the event block.
 * Queue stores head/tail as single-byte pool handles (not pointers).
 *============================================================================*/

#ifndef QUEUE_H
#define QUEUE_H

#include "smallheap.h"

/*--- Queue Structure (2 bytes, matches firmware AO layout) ---*/
typedef struct {
    handle_t head;          /* Handle of first event (0 = empty) */
    handle_t tail;          /* Handle of last event  (0 = empty) */
} queue_t;

/*--- Event header: next-pointer field offset ---*/
#define EVT_NEXT_OFFSET     4   /* Byte offset of "next" handle within event */

/*--- API Functions ---*/

/**
 * queue_Init - Initialize a queue to empty
 * @q: Pointer to queue structure
 *
 * Sets head=0, tail=0.
 */
void queue_Init(queue_t *q);

/**
 * queue_Enqueue - Append an event handle to the queue tail
 * @q:      Pointer to queue structure
 * @handle: Pool handle of the event to enqueue
 *
 * Resolves the handle to set the event's next field to 0.
 * If queue was non-empty, links from current tail.
 */
void queue_Enqueue(queue_t *q, handle_t handle);

/**
 * queue_Dequeue - Pop the first event from the queue head
 * @q: Pointer to queue structure
 *
 * Returns: Handle of the dequeued event, or POOL_HANDLE_NULL if empty.
 * Advances head to the next event in the chain.
 */
handle_t queue_Dequeue(queue_t *q);

#endif /* QUEUE_H */
