/*============================================================================
 * ao.c — Active Object Framework initialization
 *
 * Provides ao_FrameworkInit() which replicates the OldMain initialization
 * sequence from the L200 firmware:
 *   1. Clear error flag
 *   2. Initialize pool system with 2 pools
 *   3. Register Pool 0: 24-byte blocks (8 blocks)  — small events (6-22B)
 *   4. Register Pool 1: 48-byte blocks (4 blocks)  — large events / config
 *   5. Initialize scheduler (priority table + ready set)
 *
 * Firmware reference: OldMain Phase 2 (pool setup) + Phase 3 (sched init)
 *============================================================================*/

#include "ao.h"
#include <string.h>

/* Debug call-site tag: set before pool_Resolve to track caller */
volatile uint8_t g_dbg_site = 0;

/*--- Pool configuration ---*/
/* Firmware OldMain uses 16/8 blocks, but TestFirmware has tight RAM (4KB).
 * Tests need at most ~6 concurrent events; these counts are sufficient
 * and save 384 bytes of BSS vs firmware sizes. */
#define POOL_COUNT          2
#define POOL0_BLOCK_SIZE    24
#define POOL0_BLOCK_COUNT   8
#define POOL1_BLOCK_SIZE    48
#define POOL1_BLOCK_COUNT   4

/*--- Pool storage (statically allocated) ---*/
static pool_desc_t  g_pool_descs[POOL_COUNT];
static uint8_t      g_pool0_mem[POOL0_BLOCK_SIZE * POOL0_BLOCK_COUNT];
static uint8_t      g_pool1_mem[POOL1_BLOCK_SIZE * POOL1_BLOCK_COUNT];

/*============================================================================
 * ao_FrameworkInit — Initialize pools + scheduler
 *============================================================================*/
void ao_FrameworkInit(void)
{
    /* Clear error flag — matches firmware RealBoot */
    error_clear();

    /* Initialize pool system */
    pool_Init(g_pool_descs, POOL_COUNT);
    pool_Register(g_pool0_mem, POOL0_BLOCK_SIZE, POOL0_BLOCK_COUNT);
    pool_Register(g_pool1_mem, POOL1_BLOCK_SIZE, POOL1_BLOCK_COUNT);

    /* Initialize scheduler */
    sched_Init();

    /* Reset timer list (clear stale pointers from previous run) */
    ao_timer_ResetAll();

    /* Reset current context */
    ao_current_context = NULL;
}
