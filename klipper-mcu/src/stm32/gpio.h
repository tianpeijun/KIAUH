/**
 * @file    gpio.h
 * @brief   STM32F407 GPIO driver interface
 * 
 * Provides GPIO configuration and control functions for STM32F407.
 * Follows Klipper coding style (C99, snake_case).
 */

#ifndef STM32_GPIO_H
#define STM32_GPIO_H

#include <stdint.h>

/* ========== GPIO Mode Definitions ========== */

/* GPIO mode */
typedef enum {
    GPIO_MODE_INPUT     = 0,    /* Input mode */
    GPIO_MODE_OUTPUT    = 1,    /* Output mode */
    GPIO_MODE_AF        = 2,    /* Alternate function mode */
    GPIO_MODE_ANALOG    = 3     /* Analog mode */
} gpio_mode_t;

/* GPIO output type */
typedef enum {
    GPIO_OTYPE_PP       = 0,    /* Push-pull */
    GPIO_OTYPE_OD       = 1     /* Open-drain */
} gpio_otype_t;

/* GPIO output speed */
typedef enum {
    GPIO_SPEED_LOW      = 0,    /* Low speed (2 MHz) */
    GPIO_SPEED_MEDIUM   = 1,    /* Medium speed (25 MHz) */
    GPIO_SPEED_FAST     = 2,    /* Fast speed (50 MHz) */
    GPIO_SPEED_HIGH     = 3     /* High speed (100 MHz) */
} gpio_speed_t;

/* GPIO pull-up/pull-down */
typedef enum {
    GPIO_PUPD_NONE      = 0,    /* No pull-up/pull-down */
    GPIO_PUPD_UP        = 1,    /* Pull-up */
    GPIO_PUPD_DOWN      = 2     /* Pull-down */
} gpio_pupd_t;

/* GPIO configuration structure */
typedef struct {
    gpio_mode_t mode;           /* Pin mode */
    gpio_otype_t otype;         /* Output type */
    gpio_speed_t speed;         /* Output speed */
    gpio_pupd_t pupd;           /* Pull-up/pull-down */
    uint8_t af;                 /* Alternate function (0-15) */
} gpio_config_t;

/* ========== GPIO Functions ========== */

/**
 * @brief   Initialize GPIO subsystem
 * 
 * Enables clocks for all GPIO ports.
 */
void gpio_init(void);

/**
 * @brief   Configure GPIO pin
 * @param   gpio    GPIO pin (use GPIO() macro)
 * @param   config  Configuration structure
 */
void gpio_configure(uint8_t gpio, const gpio_config_t *config);

/**
 * @brief   Configure GPIO pin as input
 * @param   gpio    GPIO pin
 * @param   pupd    Pull-up/pull-down configuration
 */
void gpio_in_setup(uint8_t gpio, gpio_pupd_t pupd);

/**
 * @brief   Configure GPIO pin as output
 * @param   gpio    GPIO pin
 * @param   val     Initial output value (0 or 1)
 */
void gpio_out_setup(uint8_t gpio, uint8_t val);

/**
 * @brief   Configure GPIO pin as open-drain output
 * @param   gpio    GPIO pin
 * @param   val     Initial output value (0 or 1)
 */
void gpio_out_od_setup(uint8_t gpio, uint8_t val);

/**
 * @brief   Configure GPIO pin for alternate function
 * @param   gpio    GPIO pin
 * @param   af      Alternate function number (0-15)
 */
void gpio_af_setup(uint8_t gpio, uint8_t af);

/**
 * @brief   Configure GPIO pin for analog mode (ADC)
 * @param   gpio    GPIO pin
 */
void gpio_analog_setup(uint8_t gpio);

/**
 * @brief   Read GPIO input value
 * @param   gpio    GPIO pin
 * @return  Pin value (0 or 1)
 */
uint8_t gpio_in_read(uint8_t gpio);

/**
 * @brief   Write GPIO output value
 * @param   gpio    GPIO pin
 * @param   val     Output value (0 or 1)
 */
void gpio_out_write(uint8_t gpio, uint8_t val);

/**
 * @brief   Toggle GPIO output
 * @param   gpio    GPIO pin
 */
void gpio_out_toggle(uint8_t gpio);

/**
 * @brief   Set GPIO output high
 * @param   gpio    GPIO pin
 */
void gpio_out_set(uint8_t gpio);

/**
 * @brief   Set GPIO output low
 * @param   gpio    GPIO pin
 */
void gpio_out_clear(uint8_t gpio);

/**
 * @brief   Read current GPIO output value
 * @param   gpio    GPIO pin
 * @return  Current output value (0 or 1)
 */
uint8_t gpio_out_read(uint8_t gpio);

/* ========== GPIO Peripheral Structure ========== */

/* GPIO register structure (for direct register access) */
typedef struct {
    volatile uint32_t MODER;    /* Mode register */
    volatile uint32_t OTYPER;   /* Output type register */
    volatile uint32_t OSPEEDR;  /* Output speed register */
    volatile uint32_t PUPDR;    /* Pull-up/pull-down register */
    volatile uint32_t IDR;      /* Input data register */
    volatile uint32_t ODR;      /* Output data register */
    volatile uint32_t BSRR;     /* Bit set/reset register */
    volatile uint32_t LCKR;     /* Lock register */
    volatile uint32_t AFRL;     /* Alternate function low register */
    volatile uint32_t AFRH;     /* Alternate function high register */
} gpio_regs_t;

/* GPIO port base addresses */
#define GPIOA_BASE              0x40020000
#define GPIOB_BASE              0x40020400
#define GPIOC_BASE              0x40020800
#define GPIOD_BASE              0x40020C00
#define GPIOE_BASE              0x40021000
#define GPIOF_BASE              0x40021400
#define GPIOG_BASE              0x40021800
#define GPIOH_BASE              0x40021C00
#define GPIOI_BASE              0x40022000

/* GPIO port pointers */
#define GPIOA                   ((gpio_regs_t *)GPIOA_BASE)
#define GPIOB                   ((gpio_regs_t *)GPIOB_BASE)
#define GPIOC                   ((gpio_regs_t *)GPIOC_BASE)
#define GPIOD                   ((gpio_regs_t *)GPIOD_BASE)
#define GPIOE                   ((gpio_regs_t *)GPIOE_BASE)
#define GPIOF                   ((gpio_regs_t *)GPIOF_BASE)
#define GPIOG                   ((gpio_regs_t *)GPIOG_BASE)
#define GPIOH                   ((gpio_regs_t *)GPIOH_BASE)
#define GPIOI                   ((gpio_regs_t *)GPIOI_BASE)

/**
 * @brief   Get GPIO port registers
 * @param   gpio    GPIO pin
 * @return  Pointer to GPIO registers
 */
gpio_regs_t *gpio_get_port(uint8_t gpio);

/* ========== Alternate Function Mappings ========== */

/* USART alternate functions */
#define GPIO_AF_USART1          7
#define GPIO_AF_USART2          7
#define GPIO_AF_USART3          7
#define GPIO_AF_UART4           8
#define GPIO_AF_UART5           8
#define GPIO_AF_USART6          8

/* Timer alternate functions */
#define GPIO_AF_TIM1            1
#define GPIO_AF_TIM2            1
#define GPIO_AF_TIM3            2
#define GPIO_AF_TIM4            2
#define GPIO_AF_TIM5            2
#define GPIO_AF_TIM8            3
#define GPIO_AF_TIM9            3
#define GPIO_AF_TIM10           3
#define GPIO_AF_TIM11           3
#define GPIO_AF_TIM12           9
#define GPIO_AF_TIM13           9
#define GPIO_AF_TIM14           9

/* SPI alternate functions */
#define GPIO_AF_SPI1            5
#define GPIO_AF_SPI2            5
#define GPIO_AF_SPI3            6

/* I2C alternate functions */
#define GPIO_AF_I2C1            4
#define GPIO_AF_I2C2            4
#define GPIO_AF_I2C3            4

#endif /* STM32_GPIO_H */
