/**
 * @file    internal.h
 * @brief   STM32 HAL internal definitions
 * 
 * Internal definitions and macros for STM32F407 HAL layer.
 * This file provides common definitions used across HAL modules.
 */

#ifndef STM32_INTERNAL_H
#define STM32_INTERNAL_H

#include <stdint.h>
#include "autoconf.h"           /* Compile-time configuration */
#include "board/irq.h"

/* ========== Clock Configuration ========== */

/* APB1/APB2 peripheral clock frequencies */
#define APB1_FREQ               42000000    /* 42 MHz */
#define APB2_FREQ               84000000    /* 84 MHz */

/* HSE (High Speed External) oscillator frequency */
#define HSE_FREQ                8000000     /* 8 MHz crystal */

/* ========== GPIO Definitions ========== */

/* GPIO port base addresses */
#define GPIO_PORT_A             0
#define GPIO_PORT_B             1
#define GPIO_PORT_C             2
#define GPIO_PORT_D             3
#define GPIO_PORT_E             4
#define GPIO_PORT_F             5
#define GPIO_PORT_G             6
#define GPIO_PORT_H             7
#define GPIO_PORT_I             8

/* GPIO pin encoding: port (4 bits) | pin (4 bits) */
#define GPIO(PORT, PIN)         (((PORT) << 4) | (PIN))
#define GPIO_PORT(gpio)         (((gpio) >> 4) & 0x0F)
#define GPIO_PIN(gpio)          ((gpio) & 0x0F)

/* Common GPIO pin definitions */
#define GPIO_PA0                GPIO(GPIO_PORT_A, 0)
#define GPIO_PA1                GPIO(GPIO_PORT_A, 1)
#define GPIO_PA2                GPIO(GPIO_PORT_A, 2)
#define GPIO_PA3                GPIO(GPIO_PORT_A, 3)
#define GPIO_PA4                GPIO(GPIO_PORT_A, 4)
#define GPIO_PA5                GPIO(GPIO_PORT_A, 5)
#define GPIO_PA6                GPIO(GPIO_PORT_A, 6)
#define GPIO_PA7                GPIO(GPIO_PORT_A, 7)
#define GPIO_PA8                GPIO(GPIO_PORT_A, 8)
#define GPIO_PA9                GPIO(GPIO_PORT_A, 9)
#define GPIO_PA10               GPIO(GPIO_PORT_A, 10)
#define GPIO_PA11               GPIO(GPIO_PORT_A, 11)
#define GPIO_PA12               GPIO(GPIO_PORT_A, 12)
#define GPIO_PA13               GPIO(GPIO_PORT_A, 13)
#define GPIO_PA14               GPIO(GPIO_PORT_A, 14)
#define GPIO_PA15               GPIO(GPIO_PORT_A, 15)

#define GPIO_PB0                GPIO(GPIO_PORT_B, 0)
#define GPIO_PB1                GPIO(GPIO_PORT_B, 1)
#define GPIO_PB2                GPIO(GPIO_PORT_B, 2)
#define GPIO_PB3                GPIO(GPIO_PORT_B, 3)
#define GPIO_PB4                GPIO(GPIO_PORT_B, 4)
#define GPIO_PB5                GPIO(GPIO_PORT_B, 5)
#define GPIO_PB6                GPIO(GPIO_PORT_B, 6)
#define GPIO_PB7                GPIO(GPIO_PORT_B, 7)
#define GPIO_PB8                GPIO(GPIO_PORT_B, 8)
#define GPIO_PB9                GPIO(GPIO_PORT_B, 9)
#define GPIO_PB10               GPIO(GPIO_PORT_B, 10)
#define GPIO_PB11               GPIO(GPIO_PORT_B, 11)
#define GPIO_PB12               GPIO(GPIO_PORT_B, 12)
#define GPIO_PB13               GPIO(GPIO_PORT_B, 13)
#define GPIO_PB14               GPIO(GPIO_PORT_B, 14)
#define GPIO_PB15               GPIO(GPIO_PORT_B, 15)

#define GPIO_PC0                GPIO(GPIO_PORT_C, 0)
#define GPIO_PC1                GPIO(GPIO_PORT_C, 1)
#define GPIO_PC2                GPIO(GPIO_PORT_C, 2)
#define GPIO_PC3                GPIO(GPIO_PORT_C, 3)
#define GPIO_PC4                GPIO(GPIO_PORT_C, 4)
#define GPIO_PC5                GPIO(GPIO_PORT_C, 5)
#define GPIO_PC6                GPIO(GPIO_PORT_C, 6)
#define GPIO_PC7                GPIO(GPIO_PORT_C, 7)
#define GPIO_PC8                GPIO(GPIO_PORT_C, 8)
#define GPIO_PC9                GPIO(GPIO_PORT_C, 9)
#define GPIO_PC10               GPIO(GPIO_PORT_C, 10)
#define GPIO_PC11               GPIO(GPIO_PORT_C, 11)
#define GPIO_PC12               GPIO(GPIO_PORT_C, 12)
#define GPIO_PC13               GPIO(GPIO_PORT_C, 13)
#define GPIO_PC14               GPIO(GPIO_PORT_C, 14)
#define GPIO_PC15               GPIO(GPIO_PORT_C, 15)

#define GPIO_PD0                GPIO(GPIO_PORT_D, 0)
#define GPIO_PD1                GPIO(GPIO_PORT_D, 1)
#define GPIO_PD2                GPIO(GPIO_PORT_D, 2)

/* ========== Timer Definitions ========== */

/* Timer base addresses (for reference) */
#define TIM1_BASE               0x40010000
#define TIM2_BASE               0x40000000
#define TIM3_BASE               0x40000400
#define TIM4_BASE               0x40000800
#define TIM5_BASE               0x40000C00

/* ========== ADC Definitions ========== */

#define ADC_CHANNEL_COUNT       16
#define ADC_MAX_VALUE           4095        /* 12-bit ADC */

/* ========== USART Definitions ========== */

#define USART1_BASE             0x40011000
#define USART2_BASE             0x40004400
#define USART3_BASE             0x40004800

/* ========== Utility Macros ========== */

/* Bit manipulation */
#define BIT(n)                  (1UL << (n))
#define ARRAY_SIZE(arr)         (sizeof(arr) / sizeof((arr)[0]))

/* Memory barrier */
#define barrier()               __asm__ __volatile__("" ::: "memory")

/* Compiler attributes */
#ifndef __always_inline
#define __always_inline         inline __attribute__((always_inline))
#endif
#define __section(s)            __attribute__((section(s)))
#define __weak                  __attribute__((weak))
#define __packed                __attribute__((packed))
#define __aligned(n)            __attribute__((aligned(n)))

/* Read/write volatile memory */
#define readl(addr)             (*(volatile uint32_t *)(addr))
#define writel(val, addr)       (*(volatile uint32_t *)(addr) = (val))
#define readb(addr)             (*(volatile uint8_t *)(addr))
#define writeb(val, addr)       (*(volatile uint8_t *)(addr) = (val))

/* ========== System Functions ========== */

/* System initialization */
void system_init(void);

/* Get system clock frequency */
uint32_t get_pclock_frequency(uint32_t periph_base);

/* Enable peripheral clock */
void enable_pclock(uint32_t periph_base);

/* Check if peripheral clock is enabled */
int is_enabled_pclock(uint32_t periph_base);

/* Get timer clock frequency */
uint32_t timer_get_clock(void);

/* Microsecond delay */
void udelay(uint32_t us);

#endif /* STM32_INTERNAL_H */
