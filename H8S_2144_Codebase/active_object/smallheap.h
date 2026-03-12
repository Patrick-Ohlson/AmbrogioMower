/*============================================================================
 * smallheap.h — Fixed-block memory pool allocator (SmallHeap)
 *
 * Reconstructed from Husqvarna L200 firmware (Renesas H8S/2144)
 * Original source: lib/active/options/smallheap.c
 *
 * Pool system using single-byte handles instead of pointers.
 * Handle encoding: handle = ~(pool_index << 5 | block_index)
 *   - pool_index  = (~handle) >> 5        (0-7)
 *   - block_index = (~handle) & 0x1F      (0-31)
 *   - handle 0x00 = NULL (no event)
 *
 * Supports up to 8 pools, each with up to 32 blocks.
 *============================================================================*/

#ifndef SMALLHEAP_H
#define SMALLHEAP_H

#include <stdint.h>
#include <stddef.h>

/*--- Constants ---*/
#define POOL_MAX_POOLS      8       /* Maximum number of pools                */
#define POOL_MAX_BLOCKS     32      /* Maximum blocks per pool (5 bits)       */
#define POOL_HANDLE_NULL    0x00    /* Null handle                            */
#define POOL_FREE_END       0xFF    /* Free-list terminator                   */

/*--- Types ---*/
typedef uint8_t handle_t;           /* Opaque event handle (inverted encoding)*/

/*--- Pool Descriptor (16 bytes, matches firmware layout) ---*/
typedef struct {
    uint8_t *base_ptr;              /* +0x00: Pointer to memory block array   */
    uint16_t block_size;            /* +0x04: Size of each block in bytes     */
    uint16_t min_used_size;         /* +0x08: Smallest allocation (waste track)*/
    uint8_t  num_blocks;            /* +0x0C: Total blocks in pool            */
    uint8_t  free_head;             /* +0x0D: Index of first free block       */
    uint8_t  alloc_count;           /* +0x0E: Current allocated blocks        */
    uint8_t  alloc_peak;            /* +0x0F: High-water mark of alloc_count  */
} pool_desc_t;

/*--- API Functions ---*/

/**
 * pool_Init - Initialize the pool system
 * @desc_array: Array of pool_desc_t descriptors (caller-provided storage)
 * @count:      Number of pool slots (max 8)
 *
 * Stores the descriptor array pointer and count, zero-fills all descriptors.
 * Must be called before pool_Register() or any allocation.
 */
void pool_Init(pool_desc_t *desc_array, uint8_t count);

/**
 * pool_Register - Register a fixed-block pool
 * @memory:     Pointer to pre-allocated memory block array
 * @block_size: Size of each block in bytes
 * @num_blocks: Number of blocks in the array
 *
 * Finds the next empty slot in the descriptor array, initializes the
 * free-list chain (each block's first byte points to the next block index),
 * and records block_size and num_blocks.
 *
 * Pools MUST be registered in ascending order of block_size.
 */
void pool_Register(uint8_t *memory, uint16_t block_size, uint8_t num_blocks);

/**
 * pool_Alloc - Allocate a block from the smallest suitable pool
 * @size: Minimum required block size in bytes
 *
 * Scans pools for first with block_size >= size, pops the free-list head.
 * Returns: Inverted handle, or POOL_HANDLE_NULL (0) on failure.
 */
handle_t pool_Alloc(uint16_t size);

/**
 * pool_Resolve - Decode a handle to a memory pointer
 * @handle: Handle previously returned by pool_Alloc()
 *
 * Computes: base_ptr + block_index * block_size
 * Returns: Pointer to the allocated block.
 */
void *pool_Resolve(handle_t handle);

/**
 * pool_Free - Return a block to its pool's free-list
 * @handle: Handle previously returned by pool_Alloc()
 *
 * Pushes the block back onto the free-list head and decrements alloc_count.
 */
void pool_Free(handle_t handle);

/*--- Debug: lightweight error capture from pool_Resolve ---*/
extern volatile uint8_t g_dbg_bad_handle;  /* last bad handle */
extern volatile uint8_t g_dbg_bad_site;    /* g_dbg_site at time of error */
extern volatile uint8_t g_dbg_bad_count;   /* number of bad resolves */

#endif /* SMALLHEAP_H */
