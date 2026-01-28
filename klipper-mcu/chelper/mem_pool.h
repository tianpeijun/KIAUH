/**
 * @file    mem_pool.h
 * @brief   Static memory pool interface for MCU use
 * 
 * Provides generic memory pool allocation functions to replace
 * malloc/free in embedded environments. This module is part of
 * the chelper adaptation layer for MCU builds.
 * 
 * Key features:
 * - No dynamic memory allocation (no malloc/free)
 * - Fixed-size block pools for predictable memory usage
 * - Thread-safe with interrupt disable (optional)
 * - Pool statistics for debugging
 * 
 * Usage:
 * 1. Call mem_pool_init() once at startup
 * 2. Use mem_pool_alloc() / mem_pool_free() for allocations
 * 3. Use mem_pool_get_stats() for debugging
 */

#ifndef CHELPER_MEM_POOL_H
#define CHELPER_MEM_POOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== Memory Pool Configuration ========== */

/**
 * @brief Memory pool block sizes
 * 
 * Define different block sizes for various allocation needs.
 * Blocks are allocated from the smallest pool that fits the request.
 */
#define MEM_POOL_BLOCK_SMALL    64      /**< Small blocks (64 bytes) */
#define MEM_POOL_BLOCK_MEDIUM   256     /**< Medium blocks (256 bytes) */
#define MEM_POOL_BLOCK_LARGE    512     /**< Large blocks (512 bytes) */

/**
 * @brief Number of blocks in each pool
 */
#define MEM_POOL_SMALL_COUNT    16      /**< 16 x 64 = 1KB */
#define MEM_POOL_MEDIUM_COUNT   16      /**< 16 x 256 = 4KB */
#define MEM_POOL_LARGE_COUNT    8       /**< 8 x 512 = 4KB */

/**
 * @brief Total memory pool size (approximately 9KB)
 */
#define MEM_POOL_TOTAL_SIZE     (MEM_POOL_BLOCK_SMALL * MEM_POOL_SMALL_COUNT + \
                                 MEM_POOL_BLOCK_MEDIUM * MEM_POOL_MEDIUM_COUNT + \
                                 MEM_POOL_BLOCK_LARGE * MEM_POOL_LARGE_COUNT)

/* ========== Memory Pool Statistics ========== */

/**
 * @brief Memory pool statistics structure
 */
typedef struct {
    uint32_t total_allocs;      /**< Total allocation requests */
    uint32_t total_frees;       /**< Total free requests */
    uint32_t failed_allocs;     /**< Failed allocation attempts */
    uint32_t small_used;        /**< Small blocks currently in use */
    uint32_t medium_used;       /**< Medium blocks currently in use */
    uint32_t large_used;        /**< Large blocks currently in use */
    uint32_t small_peak;        /**< Peak small block usage */
    uint32_t medium_peak;       /**< Peak medium block usage */
    uint32_t large_peak;        /**< Peak large block usage */
} mem_pool_stats_t;

/* ========== Function Prototypes ========== */

/**
 * @brief Initialize all memory pools
 * 
 * Must be called once at startup before any allocations.
 * Clears all pools and resets statistics.
 */
void mem_pool_init(void);

/**
 * @brief Allocate memory from the pool
 * 
 * @param size  Requested allocation size in bytes
 * @return Pointer to allocated memory, or NULL if pool exhausted
 * 
 * The allocation is satisfied from the smallest pool that can
 * accommodate the requested size. If no suitable block is available,
 * returns NULL.
 * 
 * @note This function is NOT thread-safe by default. Use
 *       mem_pool_alloc_safe() for interrupt-safe allocation.
 */
void *mem_pool_alloc(size_t size);

/**
 * @brief Free memory back to the pool
 * 
 * @param ptr   Pointer to memory previously allocated with mem_pool_alloc()
 * 
 * If ptr is NULL, this function does nothing.
 * If ptr was not allocated from the pool, behavior is undefined.
 * 
 * @note This function is NOT thread-safe by default. Use
 *       mem_pool_free_safe() for interrupt-safe deallocation.
 */
void mem_pool_free(void *ptr);

/**
 * @brief Allocate memory from the pool (interrupt-safe)
 * 
 * @param size  Requested allocation size in bytes
 * @return Pointer to allocated memory, or NULL if pool exhausted
 * 
 * Same as mem_pool_alloc() but disables interrupts during allocation.
 */
void *mem_pool_alloc_safe(size_t size);

/**
 * @brief Free memory back to the pool (interrupt-safe)
 * 
 * @param ptr   Pointer to memory previously allocated
 * 
 * Same as mem_pool_free() but disables interrupts during deallocation.
 */
void mem_pool_free_safe(void *ptr);

/**
 * @brief Get memory pool statistics
 * 
 * @param stats Pointer to statistics structure to fill
 * 
 * Copies current pool statistics to the provided structure.
 */
void mem_pool_get_stats(mem_pool_stats_t *stats);

/**
 * @brief Reset memory pool statistics
 * 
 * Clears all statistics counters without affecting allocations.
 */
void mem_pool_reset_stats(void);

/**
 * @brief Check if a pointer belongs to the memory pool
 * 
 * @param ptr   Pointer to check
 * @return 1 if pointer is from pool, 0 otherwise
 */
int mem_pool_is_from_pool(const void *ptr);

/**
 * @brief Get the block size for a given pointer
 * 
 * @param ptr   Pointer to allocated memory
 * @return Block size in bytes, or 0 if not from pool
 */
size_t mem_pool_block_size(const void *ptr);

/**
 * @brief Get available blocks count for a given size
 * 
 * @param size  Requested allocation size
 * @return Number of available blocks that can satisfy the request
 */
uint32_t mem_pool_available(size_t size);

#ifdef __cplusplus
}
#endif

#endif /* CHELPER_MEM_POOL_H */
