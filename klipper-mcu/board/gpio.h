/**
 * @file    gpio.h
 * @brief   Board-level GPIO interface abstraction
 * 
 * This file provides a board-level abstraction for GPIO operations.
 * It wraps the STM32-specific GPIO driver to provide a portable interface
 * that can be used by application code without direct hardware dependencies.
 * 
 * For STM32 targets, this file includes the STM32 GPIO driver and provides
 * additional board-level definitions and convenience macros.
 */

#ifndef BOARD_GPIO_H
#define BOARD_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Include the platform-specific GPIO implementation */
#include "src/stm32/internal.h" /* For GPIO() macro and GPIO_Pxy definitions */
#include "src/stm32/gpio.h"     /* For GPIO functions and types */

/* ========== GPIO Pin Encoding ========== */

/*
 * GPIO pin encoding uses a single byte: upper 4 bits for port, lower 4 bits for pin.
 * 
 * The following are provided by stm32/internal.h:
 * - GPIO(port, pin) - Create GPIO pin identifier
 * - GPIO_PORT(gpio) - Extract port number
 * - GPIO_PIN(gpio)  - Extract pin number
 * - GPIO_Pxy        - Pre-defined pin identifiers (e.g., GPIO_PA0, GPIO_PB5)
 * - GPIO_PORT_A..I  - Port number constants
 */

/* ========== GPIO Pull Configuration ========== */

/*
 * Pull-up/pull-down configuration type.
 * Maps to the STM32 gpio_pupd_t enum for compatibility.
 */
typedef gpio_pupd_t gpio_pull_t;

/* Pull configuration values */
#define GPIO_PULL_NONE                  GPIO_PUPD_NONE
#define GPIO_PULL_UP                    GPIO_PUPD_UP
#define GPIO_PULL_DOWN                  GPIO_PUPD_DOWN

/* ========== Invalid GPIO Marker ========== */

/* Use this value to indicate an unconfigured or invalid GPIO pin */
#define GPIO_INVALID                    0xFF

/**
 * @brief   Check if GPIO pin is valid
 * @param   gpio    GPIO pin identifier
 * @return  1 if valid, 0 if invalid
 */
static inline int gpio_is_valid(uint8_t gpio)
{
    return gpio != GPIO_INVALID;
}

/* ========== Convenience Inline Functions ========== */

/**
 * @brief   Set GPIO output high
 * @param   gpio    GPIO pin identifier
 * 
 * Convenience wrapper for gpio_out_write(gpio, 1).
 */
static inline void gpio_set(uint8_t gpio)
{
    gpio_out_write(gpio, 1);
}

/**
 * @brief   Set GPIO output low
 * @param   gpio    GPIO pin identifier
 * 
 * Convenience wrapper for gpio_out_write(gpio, 0).
 */
static inline void gpio_clear(uint8_t gpio)
{
    gpio_out_write(gpio, 0);
}

/**
 * @brief   Read GPIO pin state
 * @param   gpio    GPIO pin identifier
 * @return  Pin value (0 or 1)
 * 
 * Convenience wrapper for gpio_in_read().
 */
static inline uint8_t gpio_read(uint8_t gpio)
{
    return gpio_in_read(gpio);
}

/**
 * @brief   Write GPIO pin state
 * @param   gpio    GPIO pin identifier
 * @param   val     Output value (0 or 1)
 * 
 * Convenience wrapper for gpio_out_write().
 */
static inline void gpio_write(uint8_t gpio, uint8_t val)
{
    gpio_out_write(gpio, val);
}

#ifdef __cplusplus
}
#endif

#endif /* BOARD_GPIO_H */
