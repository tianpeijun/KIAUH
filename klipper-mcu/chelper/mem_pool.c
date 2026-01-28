/**
 * @file    mem_pool.c
 * @brief   Static memory pool implementation for MCU use
 * 
 * Provides generic memory pool allocation functions to replace
 * malloc/free in embedded environments. Uses fixed-size block pools
 * for predictable memory usage.
 * 
 * Memory Layout:
 * - Small pool:  16 blocks x 64 bytes  = 1KB
 * - Medium pool: 16 blocks x 256 bytes = 4KB
 * - Large pool:  8 blocks x 512 bytes  = 4KB
 * - Total: ~9KB
 */

#include "mem_pool.h"
#include <string.h>

/* ========== Static Memory Pools ========== */

/**
 * @brief Small block pool (64 bytes each)
 */
static uint8_t s_small_pool[MEM_POOL_SMALL_COUNT][MEM_POOL_BLOCK_SMALL];
static uint8_t s_small_used[MEM_POOL_SMALL_COUNT];

/**
 * @brief Medium block pool (256 bytes each)
 */
static uint8_t s_medium_pool[MEM_POOL_MEDIUM_COUNT][MEM_POOL_BLOCK_MEDIUM];
static uint8_t s_medium_used[MEM_POOL_MEDIUM_COUNT];

/**
 * @brief Large block pool (512 bytes each)
 */
static uint8_t s_large_pool[MEM_POOL_LARGE_COUNT][MEM_POOL_BLOCK_LARGE];
static uint8_t s_large_used[MEM_POOL_LARGE_COUNT];

/**
 * @brief Pool statistics
 */
static mem_pool_stats_t s_stats;

/* ========== Internal Helper Functions ========== */

/**
 * @brief Disable interrupts and return previous state
 * @return Previous interrupt state
 */
static inline uint32_t
irq_disable(void)
{
#if defined(MCU_BUILD) && (defined(__arm__) || defined(__thumb__))
    uint32_t primask;
    __asm volatile ("mrs %0, primask" : "=r" (primask));
    __asm volatile ("cpsid i" ::: "memory");
    return primask;
#else
    return 0;
#endif
}

/**
 * @brief Restore interrupt state
 * @param state Previous interrupt state from irq_disable()
 */
static inline void
irq_restore(uint32_t state)
{
#if defined(MCU_BUILD) && (defined(__arm__) || defined(__thumb__))
    __asm volatile ("msr primask, %0" :: "r" (state) : "memory");
#else
    (void)state;
#endif
}

/**
 * @brief Update peak usage statistics
 */
static void
update_peak_stats(void)
{
    if (s_stats.small_used > s_stats.small_peak) {
        s_stats.small_peak = s_stats.small_used;
    }
    if (s_stats.medium_used > s_stats.medium_peak) {
        s_stats.medium_peak = s_stats.medium_used;
    }
    if (s_stats.large_used > s_stats.large_peak) {
        s_stats.large_peak = s_stats.large_used;
    }
}

/* ========== Public Functions ========== */

void
mem_pool_init(void)
{
    /* Clear all usage flags */
    memset(s_small_used, 0, sizeof(s_small_used));
    memset(s_medium_used, 0, sizeof(s_medium_used));
    memset(s_large_used, 0, sizeof(s_large_used));
    
    /* Clear statistics */
    memset(&s_stats, 0, sizeof(s_stats));
}

void *
mem_pool_alloc(size_t size)
{
    void *ptr = NULL;
    
    if (size == 0) {
        return NULL;
    }
    
    s_stats.total_allocs++;
    
    /* Try small pool first */
    if (size <= MEM_POOL_BLOCK_SMALL) {
        for (int i = 0; i < MEM_POOL_SMALL_COUNT; i++) {
            if (!s_small_used[i]) {
                s_small_used[i] = 1;
                s_stats.small_used++;
                update_peak_stats();
                ptr = s_small_pool[i];
                return ptr;
            }
        }
    }
    
    /* Try medium pool */
    if (size <= MEM_POOL_BLOCK_MEDIUM) {
        for (int i = 0; i < MEM_POOL_MEDIUM_COUNT; i++) {
            if (!s_medium_used[i]) {
                s_medium_used[i] = 1;
                s_stats.medium_used++;
                update_peak_stats();
                ptr = s_medium_pool[i];
                return ptr;
            }
        }
    }
    
    /* Try large pool */
    if (size <= MEM_POOL_BLOCK_LARGE) {
        for (int i = 0; i < MEM_POOL_LARGE_COUNT; i++) {
            if (!s_large_used[i]) {
                s_large_used[i] = 1;
                s_stats.large_used++;
                update_peak_stats();
                ptr = s_large_pool[i];
                return ptr;
            }
        }
    }
    
    /* Allocation failed */
    s_stats.failed_allocs++;
    return NULL;
}

void
mem_pool_free(void *ptr)
{
    if (ptr == NULL) {
        return;
    }
    
    s_stats.total_frees++;
    
    /* Check if pointer is in small pool */
    for (int i = 0; i < MEM_POOL_SMALL_COUNT; i++) {
        if (ptr == s_small_pool[i]) {
            s_small_used[i] = 0;
            if (s_stats.small_used > 0) {
                s_stats.small_used--;
            }
            return;
        }
    }
    
    /* Check if pointer is in medium pool */
    for (int i = 0; i < MEM_POOL_MEDIUM_COUNT; i++) {
        if (ptr == s_medium_pool[i]) {
            s_medium_used[i] = 0;
            if (s_stats.medium_used > 0) {
                s_stats.medium_used--;
            }
            return;
        }
    }
    
    /* Check if pointer is in large pool */
    for (int i = 0; i < MEM_POOL_LARGE_COUNT; i++) {
        if (ptr == s_large_pool[i]) {
            s_large_used[i] = 0;
            if (s_stats.large_used > 0) {
                s_stats.large_used--;
            }
            return;
        }
    }
    
    /* Pointer not from pool - ignore (could log warning in debug) */
}

void *
mem_pool_alloc_safe(size_t size)
{
    uint32_t state = irq_disable();
    void *ptr = mem_pool_alloc(size);
    irq_restore(state);
    return ptr;
}

void
mem_pool_free_safe(void *ptr)
{
    uint32_t state = irq_disable();
    mem_pool_free(ptr);
    irq_restore(state);
}

void
mem_pool_get_stats(mem_pool_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    memcpy(stats, &s_stats, sizeof(mem_pool_stats_t));
}

void
mem_pool_reset_stats(void)
{
    /* Keep current usage counts, reset others */
    uint32_t small_used = s_stats.small_used;
    uint32_t medium_used = s_stats.medium_used;
    uint32_t large_used = s_stats.large_used;
    
    memset(&s_stats, 0, sizeof(s_stats));
    
    s_stats.small_used = small_used;
    s_stats.medium_used = medium_used;
    s_stats.large_used = large_used;
    s_stats.small_peak = small_used;
    s_stats.medium_peak = medium_used;
    s_stats.large_peak = large_used;
}

int
mem_pool_is_from_pool(const void *ptr)
{
    if (ptr == NULL) {
        return 0;
    }
    
    /* Check small pool range */
    const uint8_t *p = (const uint8_t *)ptr;
    const uint8_t *small_start = &s_small_pool[0][0];
    const uint8_t *small_end = &s_small_pool[MEM_POOL_SMALL_COUNT - 1][MEM_POOL_BLOCK_SMALL - 1];
    if (p >= small_start && p <= small_end) {
        return 1;
    }
    
    /* Check medium pool range */
    const uint8_t *medium_start = &s_medium_pool[0][0];
    const uint8_t *medium_end = &s_medium_pool[MEM_POOL_MEDIUM_COUNT - 1][MEM_POOL_BLOCK_MEDIUM - 1];
    if (p >= medium_start && p <= medium_end) {
        return 1;
    }
    
    /* Check large pool range */
    const uint8_t *large_start = &s_large_pool[0][0];
    const uint8_t *large_end = &s_large_pool[MEM_POOL_LARGE_COUNT - 1][MEM_POOL_BLOCK_LARGE - 1];
    if (p >= large_start && p <= large_end) {
        return 1;
    }
    
    return 0;
}

size_t
mem_pool_block_size(const void *ptr)
{
    if (ptr == NULL) {
        return 0;
    }
    
    /* Check small pool */
    for (int i = 0; i < MEM_POOL_SMALL_COUNT; i++) {
        if (ptr == s_small_pool[i]) {
            return MEM_POOL_BLOCK_SMALL;
        }
    }
    
    /* Check medium pool */
    for (int i = 0; i < MEM_POOL_MEDIUM_COUNT; i++) {
        if (ptr == s_medium_pool[i]) {
            return MEM_POOL_BLOCK_MEDIUM;
        }
    }
    
    /* Check large pool */
    for (int i = 0; i < MEM_POOL_LARGE_COUNT; i++) {
        if (ptr == s_large_pool[i]) {
            return MEM_POOL_BLOCK_LARGE;
        }
    }
    
    return 0;
}

uint32_t
mem_pool_available(size_t size)
{
    uint32_t count = 0;
    
    if (size == 0) {
        return 0;
    }
    
    /* Count available small blocks */
    if (size <= MEM_POOL_BLOCK_SMALL) {
        for (int i = 0; i < MEM_POOL_SMALL_COUNT; i++) {
            if (!s_small_used[i]) {
                count++;
            }
        }
    }
    
    /* Count available medium blocks */
    if (size <= MEM_POOL_BLOCK_MEDIUM) {
        for (int i = 0; i < MEM_POOL_MEDIUM_COUNT; i++) {
            if (!s_medium_used[i]) {
                count++;
            }
        }
    }
    
    /* Count available large blocks */
    if (size <= MEM_POOL_BLOCK_LARGE) {
        for (int i = 0; i < MEM_POOL_LARGE_COUNT; i++) {
            if (!s_large_used[i]) {
                count++;
            }
        }
    }
    
    return count;
}
