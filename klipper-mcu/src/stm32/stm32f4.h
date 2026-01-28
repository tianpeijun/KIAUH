/**
 * @file    stm32f4.h
 * @brief   STM32F407 chip initialization interface
 * 
 * System clock configuration and chip initialization for STM32F407.
 * Follows Klipper coding style (C99, snake_case).
 */

#ifndef STM32_STM32F4_H
#define STM32_STM32F4_H

#include <stdint.h>

/* ========== Clock Configuration ========== */

/* System clock frequency (168 MHz) */
#define SYSCLK_FREQ             168000000

/* AHB clock frequency (168 MHz) */
#define HCLK_FREQ               168000000

/* APB1 clock frequency (42 MHz) */
#define PCLK1_FREQ              42000000

/* APB2 clock frequency (84 MHz) */
#define PCLK2_FREQ              84000000

/* APB1 timer clock frequency (84 MHz) */
#define TIM_PCLK1_FREQ          84000000

/* APB2 timer clock frequency (168 MHz) */
#define TIM_PCLK2_FREQ          168000000

/* ========== System Functions ========== */

/**
 * @brief   System initialization
 * 
 * Configures system clock to 168 MHz from 8 MHz HSE.
 * Initializes SysTick for 1ms interrupt.
 * Initializes GPIO subsystem.
 */
void system_init(void);

/**
 * @brief   Get peripheral clock frequency
 * @param   periph_base     Peripheral base address
 * @return  Clock frequency in Hz
 */
uint32_t get_pclock_frequency(uint32_t periph_base);

/**
 * @brief   Enable peripheral clock
 * @param   periph_base     Peripheral base address
 */
void enable_pclock(uint32_t periph_base);

/**
 * @brief   Check if peripheral clock is enabled
 * @param   periph_base     Peripheral base address
 * @return  1 if enabled, 0 otherwise
 */
int is_enabled_pclock(uint32_t periph_base);

/**
 * @brief   Get timer clock frequency
 * @return  Timer clock frequency in Hz
 */
uint32_t timer_get_clock(void);

/**
 * @brief   Microsecond delay
 * @param   us      Delay in microseconds
 */
void udelay(uint32_t us);

/**
 * @brief   Get system tick count (milliseconds)
 * @return  Tick count since system start
 */
uint32_t systick_get(void);

/**
 * @brief   Get high-resolution timer value (microseconds)
 * @return  Timer value in microseconds
 */
uint32_t timer_read_time(void);

/**
 * @brief   Check if timer value has passed
 * @param   time1   First timer value
 * @param   time2   Second timer value
 * @return  1 if time1 is before time2, 0 otherwise
 */
int timer_is_before(uint32_t time1, uint32_t time2);

/* ========== Reset and Clock Control ========== */

/**
 * @brief   Software system reset
 */
void system_reset(void);

/**
 * @brief   Enter low power sleep mode
 */
void system_sleep(void);

/**
 * @brief   Enter deep sleep (stop) mode
 */
void system_deep_sleep(void);

#endif /* STM32_STM32F4_H */

