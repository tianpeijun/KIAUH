/**
 * @file    gpio.c
 * @brief   STM32F407 GPIO driver implementation
 * 
 * GPIO configuration and control for STM32F407.
 * Follows Klipper coding style (C99, snake_case).
 */

#include "gpio.h"
#include "internal.h"
#include "board/irq.h"

/* ========== Private Definitions ========== */

/* RCC AHB1ENR register for GPIO clock enable */
#define RCC_BASE                0x40023800
#define RCC_AHB1ENR             (*(volatile uint32_t *)(RCC_BASE + 0x30))

/* GPIO port base addresses array */
static const uint32_t gpio_bases[] = {
    GPIOA_BASE,
    GPIOB_BASE,
    GPIOC_BASE,
    GPIOD_BASE,
    GPIOE_BASE,
    GPIOF_BASE,
    GPIOG_BASE,
    GPIOH_BASE,
    GPIOI_BASE
};

/* ========== Public Functions ========== */

/**
 * @brief   Get GPIO port registers
 */
gpio_regs_t *
gpio_get_port(uint8_t gpio)
{
    uint8_t port = GPIO_PORT(gpio);
    if (port >= ARRAY_SIZE(gpio_bases)) {
        return (gpio_regs_t *)0;
    }
    return (gpio_regs_t *)(uintptr_t)gpio_bases[port];
}

/**
 * @brief   Initialize GPIO subsystem
 */
void
gpio_init(void)
{
    /* Enable clocks for GPIO ports A-I */
    RCC_AHB1ENR |= (1 << 0)     /* GPIOA */
                 | (1 << 1)     /* GPIOB */
                 | (1 << 2)     /* GPIOC */
                 | (1 << 3)     /* GPIOD */
                 | (1 << 4)     /* GPIOE */
                 | (1 << 5)     /* GPIOF */
                 | (1 << 6)     /* GPIOG */
                 | (1 << 7)     /* GPIOH */
                 | (1 << 8);    /* GPIOI */
}

/**
 * @brief   Configure GPIO pin
 */
void
gpio_configure(uint8_t gpio, const gpio_config_t *config)
{
    gpio_regs_t *port = gpio_get_port(gpio);
    if (!port) {
        return;
    }
    
    uint8_t pin = GPIO_PIN(gpio);
    uint32_t irqflag = irq_disable();
    
    /* Configure mode (2 bits per pin) */
    uint32_t moder = port->MODER;
    moder &= ~(0x03 << (pin * 2));
    moder |= ((uint32_t)config->mode << (pin * 2));
    port->MODER = moder;
    
    /* Configure output type (1 bit per pin) */
    uint32_t otyper = port->OTYPER;
    otyper &= ~(1 << pin);
    otyper |= ((uint32_t)config->otype << pin);
    port->OTYPER = otyper;
    
    /* Configure output speed (2 bits per pin) */
    uint32_t ospeedr = port->OSPEEDR;
    ospeedr &= ~(0x03 << (pin * 2));
    ospeedr |= ((uint32_t)config->speed << (pin * 2));
    port->OSPEEDR = ospeedr;
    
    /* Configure pull-up/pull-down (2 bits per pin) */
    uint32_t pupdr = port->PUPDR;
    pupdr &= ~(0x03 << (pin * 2));
    pupdr |= ((uint32_t)config->pupd << (pin * 2));
    port->PUPDR = pupdr;
    
    /* Configure alternate function (4 bits per pin) */
    if (config->mode == GPIO_MODE_AF) {
        if (pin < 8) {
            uint32_t afrl = port->AFRL;
            afrl &= ~(0x0F << (pin * 4));
            afrl |= ((uint32_t)config->af << (pin * 4));
            port->AFRL = afrl;
        } else {
            uint32_t afrh = port->AFRH;
            afrh &= ~(0x0F << ((pin - 8) * 4));
            afrh |= ((uint32_t)config->af << ((pin - 8) * 4));
            port->AFRH = afrh;
        }
    }
    
    irq_restore(irqflag);
}

/**
 * @brief   Configure GPIO pin as input
 */
void
gpio_in_setup(uint8_t gpio, gpio_pupd_t pupd)
{
    gpio_config_t config = {
        .mode = GPIO_MODE_INPUT,
        .otype = GPIO_OTYPE_PP,
        .speed = GPIO_SPEED_LOW,
        .pupd = pupd,
        .af = 0
    };
    gpio_configure(gpio, &config);
}

/**
 * @brief   Configure GPIO pin as output
 */
void
gpio_out_setup(uint8_t gpio, uint8_t val)
{
    /* Set initial value before configuring as output */
    gpio_out_write(gpio, val);
    
    gpio_config_t config = {
        .mode = GPIO_MODE_OUTPUT,
        .otype = GPIO_OTYPE_PP,
        .speed = GPIO_SPEED_HIGH,
        .pupd = GPIO_PUPD_NONE,
        .af = 0
    };
    gpio_configure(gpio, &config);
}

/**
 * @brief   Configure GPIO pin as open-drain output
 */
void
gpio_out_od_setup(uint8_t gpio, uint8_t val)
{
    /* Set initial value before configuring as output */
    gpio_out_write(gpio, val);
    
    gpio_config_t config = {
        .mode = GPIO_MODE_OUTPUT,
        .otype = GPIO_OTYPE_OD,
        .speed = GPIO_SPEED_HIGH,
        .pupd = GPIO_PUPD_UP,
        .af = 0
    };
    gpio_configure(gpio, &config);
}

/**
 * @brief   Configure GPIO pin for alternate function
 */
void
gpio_af_setup(uint8_t gpio, uint8_t af)
{
    gpio_config_t config = {
        .mode = GPIO_MODE_AF,
        .otype = GPIO_OTYPE_PP,
        .speed = GPIO_SPEED_HIGH,
        .pupd = GPIO_PUPD_NONE,
        .af = af
    };
    gpio_configure(gpio, &config);
}

/**
 * @brief   Configure GPIO pin for analog mode (ADC)
 */
void
gpio_analog_setup(uint8_t gpio)
{
    gpio_config_t config = {
        .mode = GPIO_MODE_ANALOG,
        .otype = GPIO_OTYPE_PP,
        .speed = GPIO_SPEED_LOW,
        .pupd = GPIO_PUPD_NONE,
        .af = 0
    };
    gpio_configure(gpio, &config);
}

/**
 * @brief   Read GPIO input value
 */
uint8_t
gpio_in_read(uint8_t gpio)
{
    gpio_regs_t *port = gpio_get_port(gpio);
    if (!port) {
        return 0;
    }
    
    uint8_t pin = GPIO_PIN(gpio);
    return (port->IDR >> pin) & 1;
}

/**
 * @brief   Write GPIO output value
 */
void
gpio_out_write(uint8_t gpio, uint8_t val)
{
    gpio_regs_t *port = gpio_get_port(gpio);
    if (!port) {
        return;
    }
    
    uint8_t pin = GPIO_PIN(gpio);
    if (val) {
        port->BSRR = (1 << pin);        /* Set bit */
    } else {
        port->BSRR = (1 << (pin + 16)); /* Reset bit */
    }
}

/**
 * @brief   Toggle GPIO output
 */
void
gpio_out_toggle(uint8_t gpio)
{
    gpio_regs_t *port = gpio_get_port(gpio);
    if (!port) {
        return;
    }
    
    uint8_t pin = GPIO_PIN(gpio);
    port->ODR ^= (1 << pin);
}

/**
 * @brief   Set GPIO output high
 */
void
gpio_out_set(uint8_t gpio)
{
    gpio_out_write(gpio, 1);
}

/**
 * @brief   Set GPIO output low
 */
void
gpio_out_clear(uint8_t gpio)
{
    gpio_out_write(gpio, 0);
}

/**
 * @brief   Read current GPIO output value
 */
uint8_t
gpio_out_read(uint8_t gpio)
{
    gpio_regs_t *port = gpio_get_port(gpio);
    if (!port) {
        return 0;
    }
    
    uint8_t pin = GPIO_PIN(gpio);
    return (port->ODR >> pin) & 1;
}


/* ========== PWM Functions (Software PWM via GPIO) ========== */

/**
 * @brief   Setup PWM on a GPIO pin (software PWM)
 * @param   pin         GPIO pin
 * @param   cycle_time  PWM cycle time in clock ticks
 * @param   value       Initial duty cycle value (0-255)
 * 
 * @note    This is a simplified software PWM implementation.
 *          For hardware PWM, use timer-based implementation.
 */
void
pwm_setup(uint8_t pin, uint32_t cycle_time, uint8_t value)
{
    (void)cycle_time;
    
    /* Configure pin as output */
    gpio_out_setup(pin, value ? 1 : 0);
}

/**
 * @brief   Write PWM duty cycle value
 * @param   pin     GPIO pin
 * @param   value   Duty cycle value (0-255, 0=off, 255=full on)
 * 
 * @note    Simplified implementation - just on/off based on threshold
 */
void
pwm_write(uint8_t pin, uint8_t value)
{
    /* Simple threshold: >127 = on, <=127 = off */
    gpio_out_write(pin, value > 127 ? 1 : 0);
}
