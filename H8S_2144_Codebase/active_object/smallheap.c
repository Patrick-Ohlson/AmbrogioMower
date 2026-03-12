/*============================================================================
 * smallheap.c — Fixed-block memory pool allocator (SmallHeap)
 *
 * Reconstructed from Husqvarna L200 firmware (Renesas H8S/2144)
 * Original source: lib/active/options/smallheap.c
 *
 * Firmware addresses:
 *   pool_Init     = 0x16BC0  (SmallHeap_INIT)
 *   pool_Register = 0x16C30  (SmallHeapAlloc)
 *   pool_Alloc    = 0x16DC2  (ao_pool_alloc)
 *   pool_Resolve  = 0x170AA  (ao_pool_resolve)
 *   pool_Free     = 0x16F2C  (tm_sub_16F2C)
 *============================================================================*/

#include "smallheap.h"
#include "error.h"
#include <string.h>

/*--- Module globals (firmware: ao_pool_chunks, ao_pool_count) ---*/
static pool_desc_t *g_pool_chunks = NULL;
static uint8_t      g_pool_count  = 0;

/*--- Debug: lightweight error capture (no UART in hot path) ---*/
volatile uint8_t  g_dbg_bad_handle = 0;
volatile uint8_t  g_dbg_bad_site   = 0;
volatile uint8_t  g_dbg_bad_count  = 0;

/*--- Internal: Encode handle from pool_index and block_index ---*/
static handle_t handle_encode(uint8_t pool_index, uint8_t block_index)
{
    return (handle_t)~((pool_index << 5) | block_index);
}

/*--- Internal: Decode pool_index from handle ---*/
/* Note: ~h promotes uint8_t to int; must truncate back to 8 bits
 * before shifting, otherwise high bits corrupt the pool index. */
static uint8_t handle_pool_index(handle_t h)
{
    uint8_t inv = (uint8_t)~h;
    return inv >> 5;
}

/*--- Internal: Decode block_index from handle ---*/
static uint8_t handle_block_index(handle_t h)
{
    uint8_t inv = (uint8_t)~h;
    return inv & 0x1F;
}

/*============================================================================
 * pool_Init — Initialize pool system
 *
 * Firmware equivalent (0x16BC0):
 *   void SmallHeap_INIT(void *param_1, uint param_2) {
 *       if (8 < param_2) DebugPrint(..., "count <= 8", ...);
 *       ao_pool_count = (uint8_t)param_2;
 *       ao_pool_chunks = param_1;
 *       tm_ClearMemory(param_1, param_2);
 *   }
 *============================================================================*/
void pool_Init(pool_desc_t *desc_array, uint8_t count)
{
    ASSERT(count <= POOL_MAX_POOLS, "Init:count>8");

    g_pool_count  = count;
    g_pool_chunks = desc_array;
    memset(desc_array, 0, count * sizeof(pool_desc_t));
}

/*============================================================================
 * pool_Register — Register a fixed-block pool
 *
 * Firmware equivalent (0x16C30):
 *   Finds next empty slot, asserts ordering and block count limits,
 *   initializes free-list chain: block[0]→1→2→...→N→0xFF,
 *   records base_ptr, block_size, num_blocks.
 *
 * Pools must be registered in ascending block_size order. Max 32 blocks
 * per pool (31 for pools 0-6, 32 for pool 7 — we allow 32 uniformly).
 *============================================================================*/
void pool_Register(uint8_t *memory, uint16_t block_size, uint8_t num_blocks)
{
    uint8_t slot;
    pool_desc_t *pd;
    uint8_t i;
    uint8_t *p;

    /* Find next empty slot */
    for (slot = 0; slot < g_pool_count; slot++) {
        if (g_pool_chunks[slot].base_ptr == NULL)
            break;
    }
    ASSERT(slot < g_pool_count, "Reg:no slot");
    ASSERT(num_blocks > 0 && num_blocks <= POOL_MAX_BLOCKS, "Reg:blk count");

    /* Firmware asserts ascending block_size order */
    if (slot > 0) {
        ASSERT(block_size > g_pool_chunks[slot - 1].block_size, "Reg:order");
    }

    /* Initialize free-list chain: each block's byte 0 = index of next */
    p = memory;
    for (i = 0; i < num_blocks - 1; i++) {
        *p = i + 1;            /* point to next block index */
        p += block_size;
    }
    *p = POOL_FREE_END;        /* last block: 0xFF = end of free-list */

    /* Fill descriptor */
    pd = &g_pool_chunks[slot];
    pd->base_ptr      = memory;
    pd->block_size    = block_size;
    pd->min_used_size = block_size;     /* initially = block_size */
    pd->num_blocks    = num_blocks;
    pd->free_head     = 0;              /* first free = block 0 */
    pd->alloc_count   = 0;
    pd->alloc_peak    = 0;
}

/*============================================================================
 * pool_Alloc — Allocate from smallest suitable pool
 *
 * Firmware equivalent (0x16DC2):
 *   Scans pools for block_size >= size, pops free_head,
 *   returns inverted handle or 0 on failure.
 *============================================================================*/
handle_t pool_Alloc(uint16_t size)
{
    uint8_t i;

    for (i = 0; i < g_pool_count; i++) {
        pool_desc_t *pd = &g_pool_chunks[i];

        if (pd->base_ptr == NULL)
            continue;                   /* empty slot */

        if (pd->block_size < size)
            continue;                   /* too small */

        if (pd->free_head == POOL_FREE_END) {
            continue;                   /* pool exhausted, try next */
        }

        /* Pop free-list head */
        {
            uint8_t  block_idx = pd->free_head;
            uint8_t *block_ptr = pd->base_ptr + (uint16_t)block_idx * pd->block_size;

            pd->free_head = *block_ptr; /* advance to next free */
            pd->alloc_count++;

            if (pd->alloc_count > pd->alloc_peak)
                pd->alloc_peak = pd->alloc_count;

            /* Track smallest allocation for waste analysis */
            if (size < pd->min_used_size)
                pd->min_used_size = size;

            return handle_encode(i, block_idx);
        }
    }

    return POOL_HANDLE_NULL;            /* allocation failed */
}

/*============================================================================
 * pool_Resolve — Decode handle to memory pointer
 *
 * Firmware equivalent (0x170AA):
 *   pool_index  = (~handle) >> 5
 *   block_index = (~handle) & 0x1F
 *   return base_ptr + block_index * block_size
 *============================================================================*/
void *pool_Resolve(handle_t handle)
{
    uint8_t pi = handle_pool_index(handle);
    uint8_t bi = handle_block_index(handle);
    pool_desc_t *pd;

    if (pi >= g_pool_count) {
        /* Capture bad handle info in globals (no UART — avoid stack depth) */
        extern volatile uint8_t g_dbg_site;
        g_dbg_bad_handle = handle;
        g_dbg_bad_site   = g_dbg_site;
        g_dbg_bad_count++;
        return NULL;
    }
    pd = &g_pool_chunks[pi];

    return (void *)(pd->base_ptr + (uint16_t)bi * pd->block_size);
}

/*============================================================================
 * pool_Free — Return block to free-list
 *
 * Firmware equivalent (0x16F2C):
 *   Asserts poolindex < npools_, pushes block onto free-list head,
 *   decrements alloc_count.
 *============================================================================*/
void pool_Free(handle_t handle)
{
    uint8_t pi = handle_pool_index(handle);
    uint8_t bi = handle_block_index(handle);
    pool_desc_t *pd;
    uint8_t *block_ptr;

    ASSERT(pi < g_pool_count, "Free:bad hdl");
    if (pi >= g_pool_count) {
        return;  /* Don't write to out-of-bounds memory */
    }

    pd = &g_pool_chunks[pi];
    block_ptr = pd->base_ptr + (uint16_t)bi * pd->block_size;

    /* Push onto free-list head */
    *block_ptr = pd->free_head;
    pd->free_head = bi;
    pd->alloc_count--;
}
