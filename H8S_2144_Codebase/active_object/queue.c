/*============================================================================
 * queue.c — Singly-linked event queue using pool handles
 *
 * Reconstructed from Husqvarna L200 firmware (Renesas H8S/2144)
 * Original source: lib/active/xio/queue.c
 *
 * Firmware addresses:
 *   queue_Init    = 0x1808A  (sub_1808A)
 *   queue_Enqueue = 0x18094  (ao_queue_enqueue)
 *   queue_Dequeue = 0x180C4  (tm_sub_180C4)
 *============================================================================*/

#include "queue.h"
#include <string.h>

extern volatile uint8_t g_dbg_site;

/*============================================================================
 * queue_Init — Initialize queue to empty
 *
 * Firmware equivalent (0x1808A):
 *   void sub_1808A(uint8_t *param_1) {
 *       *param_1 = 0;
 *       param_1[1] = 0;
 *   }
 *============================================================================*/
void queue_Init(queue_t *q)
{
    q->head = POOL_HANDLE_NULL;
    q->tail = POOL_HANDLE_NULL;
}

/*============================================================================
 * queue_Enqueue — Append event to queue tail
 *
 * Firmware equivalent (0x18094):
 *   Resolves handle, clears event->next (offset +4) to 0.
 *   If queue empty: head = handle.
 *   If queue non-empty: resolve tail, set tail->next = handle.
 *   tail = handle.
 *============================================================================*/
void queue_Enqueue(queue_t *q, handle_t handle)
{
    uint8_t *evt_ptr;

    /* Resolve new event, clear its next pointer */
    g_dbg_site = 0x10;  /* enqueue: resolve new */
    evt_ptr = (uint8_t *)pool_Resolve(handle);
    evt_ptr[EVT_NEXT_OFFSET] = POOL_HANDLE_NULL;

    if (q->tail == POOL_HANDLE_NULL) {
        /* Queue was empty: new event is both head and tail */
        q->head = handle;
    } else {
        /* Link from current tail */
        g_dbg_site = 0x11;  /* enqueue: resolve tail */
        uint8_t *tail_ptr = (uint8_t *)pool_Resolve(q->tail);
        tail_ptr[EVT_NEXT_OFFSET] = handle;
    }

    q->tail = handle;
}

/*============================================================================
 * queue_Dequeue — Pop first event from queue head
 *
 * Firmware equivalent (0x180C4):
 *   handle = head; if (handle != 0) { head = event->next; if (head==0) tail=0; }
 *   return handle;
 *============================================================================*/
handle_t queue_Dequeue(queue_t *q)
{
    handle_t h = q->head;

    if (h != POOL_HANDLE_NULL) {
        g_dbg_site = 0x20;  /* dequeue: resolve head */
        uint8_t *evt_ptr = (uint8_t *)pool_Resolve(h);
        handle_t next_h  = evt_ptr[EVT_NEXT_OFFSET];

        q->head = next_h;
        if (next_h == POOL_HANDLE_NULL) {
            q->tail = POOL_HANDLE_NULL;
        }
    }

    return h;
}
