/**
 * @file    misc.h
 * @brief   Miscellaneous board utilities
 * 
 * Common utility functions and definitions for the board.
 */

#ifndef BOARD_MISC_H
#define BOARD_MISC_H

#include <stdint.h>
#include <stddef.h>

/* ========== Compiler Utilities ========== */

/* Likely/unlikely branch hints */
#define likely(x)               __builtin_expect(!!(x), 1)
#define unlikely(x)             __builtin_expect(!!(x), 0)

/* Unused parameter/variable */
#define UNUSED(x)               ((void)(x))

/* Container of macro */
#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))

/* Min/max macros */
#define MIN(a, b)               (((a) < (b)) ? (a) : (b))
#define MAX(a, b)               (((a) > (b)) ? (a) : (b))
#define CLAMP(val, lo, hi)      MIN(MAX(val, lo), hi)

/* Absolute value */
#define ABS(x)                  (((x) < 0) ? -(x) : (x))

/* ========== Time Utilities ========== */

/* Time conversion macros */
#define NSECS_PER_USEC          1000
#define USECS_PER_MSEC          1000
#define MSECS_PER_SEC           1000
#define USECS_PER_SEC           1000000
#define NSECS_PER_SEC           1000000000

/* Convert between time units */
#define USEC_TO_NSEC(us)        ((us) * NSECS_PER_USEC)
#define MSEC_TO_USEC(ms)        ((ms) * USECS_PER_MSEC)
#define SEC_TO_USEC(s)          ((s) * USECS_PER_SEC)
#define SEC_TO_MSEC(s)          ((s) * MSECS_PER_SEC)

/* ========== Memory Utilities ========== */

/**
 * @brief   Zero memory region
 * @param   ptr     Pointer to memory
 * @param   size    Size in bytes
 */
static inline void mem_zero(void *ptr, size_t size)
{
    uint8_t *p = (uint8_t *)ptr;
    while (size--) {
        *p++ = 0;
    }
}

/**
 * @brief   Copy memory region
 * @param   dst     Destination pointer
 * @param   src     Source pointer
 * @param   size    Size in bytes
 */
static inline void mem_copy(void *dst, const void *src, size_t size)
{
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;
    while (size--) {
        *d++ = *s++;
    }
}

/**
 * @brief   Compare memory regions
 * @param   a       First pointer
 * @param   b       Second pointer
 * @param   size    Size in bytes
 * @return  0 if equal, non-zero otherwise
 */
static inline int mem_compare(const void *a, const void *b, size_t size)
{
    const uint8_t *pa = (const uint8_t *)a;
    const uint8_t *pb = (const uint8_t *)b;
    while (size--) {
        if (*pa != *pb) {
            return *pa - *pb;
        }
        pa++;
        pb++;
    }
    return 0;
}

/* ========== String Utilities ========== */

/**
 * @brief   Get string length
 * @param   str     String pointer
 * @return  Length (not including null terminator)
 */
static inline size_t str_len(const char *str)
{
    size_t len = 0;
    while (*str++) {
        len++;
    }
    return len;
}

/**
 * @brief   Copy string
 * @param   dst     Destination buffer
 * @param   src     Source string
 * @param   size    Maximum size (including null terminator)
 * @return  Destination pointer
 */
static inline char *str_copy(char *dst, const char *src, size_t size)
{
    char *d = dst;
    if (size > 0) {
        while (--size && *src) {
            *d++ = *src++;
        }
        *d = '\0';
    }
    return dst;
}

/* ========== Debug Utilities ========== */

/**
 * @brief   Output debug character (platform specific)
 * @param   c       Character to output
 */
void debug_putc(char c);

/**
 * @brief   Output debug string
 * @param   str     String to output
 */
void debug_puts(const char *str);

/**
 * @brief   Output debug hex value
 * @param   val     Value to output
 */
void debug_hex(uint32_t val);

/**
 * @brief   System panic - halt execution
 * @param   msg     Panic message
 */
void panic(const char *msg) __attribute__((noreturn));

/* ========== CRC Utilities ========== */

/**
 * @brief   Calculate CRC-16 (CCITT)
 * @param   data    Data buffer
 * @param   len     Data length
 * @return  CRC-16 value
 */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/**
 * @brief   Calculate CRC-32
 * @param   data    Data buffer
 * @param   len     Data length
 * @return  CRC-32 value
 */
uint32_t crc32(const uint8_t *data, size_t len);

/* ========== Atomic Operations ========== */

/**
 * @brief   Atomic load 32-bit value
 * @param   ptr     Pointer to value
 * @return  Value
 */
static inline uint32_t atomic_load(volatile uint32_t *ptr)
{
    return *ptr;
}

/**
 * @brief   Atomic store 32-bit value
 * @param   ptr     Pointer to value
 * @param   val     Value to store
 */
static inline void atomic_store(volatile uint32_t *ptr, uint32_t val)
{
    *ptr = val;
}

/**
 * @brief   Atomic add and return previous value
 * @param   ptr     Pointer to value
 * @param   val     Value to add
 * @return  Previous value
 */
uint32_t atomic_fetch_add(volatile uint32_t *ptr, uint32_t val);

/**
 * @brief   Atomic compare and swap
 * @param   ptr         Pointer to value
 * @param   expected    Expected value
 * @param   desired     Desired value
 * @return  1 if swapped, 0 otherwise
 */
int atomic_compare_swap(volatile uint32_t *ptr, uint32_t expected, uint32_t desired);

#endif /* BOARD_MISC_H */
