/**
 * @file    stm32f4.c
 * @brief   STM32F407 chip initialization
 * 
 * System clock configuration and chip initialization for STM32F407.
 * Configures PLL for 168 MHz system clock from 8 MHz HSE.
 * Follows Klipper coding style (C99, snake_case).
 */

#include "internal.h"
#include "gpio.h"
#include "board/irq.h"

/* ========== RCC Register Definitions ========== */

#define RCC_BASE                0x40023800

/* RCC registers */
#define RCC_CR                  (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_PLLCFGR             (*(volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_CFGR                (*(volatile uint32_t *)(RCC_BASE + 0x08))
#define RCC_CIR                 (*(volatile uint32_t *)(RCC_BASE + 0x0C))
#define RCC_AHB1RSTR            (*(volatile uint32_t *)(RCC_BASE + 0x10))
#define RCC_AHB2RSTR            (*(volatile uint32_t *)(RCC_BASE + 0x14))
#define RCC_AHB3RSTR            (*(volatile uint32_t *)(RCC_BASE + 0x18))
#define RCC_APB1RSTR            (*(volatile uint32_t *)(RCC_BASE + 0x20))
#define RCC_APB2RSTR            (*(volatile uint32_t *)(RCC_BASE + 0x24))
#define RCC_AHB1ENR             (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_AHB2ENR             (*(volatile uint32_t *)(RCC_BASE + 0x34))
#define RCC_AHB3ENR             (*(volatile uint32_t *)(RCC_BASE + 0x38))
#define RCC_APB1ENR             (*(volatile uint32_t *)(RCC_BASE + 0x40))
#define RCC_APB2ENR             (*(volatile uint32_t *)(RCC_BASE + 0x44))

/* RCC_CR bits */
#define RCC_CR_HSION            (1 << 0)
#define RCC_CR_HSIRDY           (1 << 1)
#define RCC_CR_HSEON            (1 << 16)
#define RCC_CR_HSERDY           (1 << 17)
#define RCC_CR_HSEBYP           (1 << 18)
#define RCC_CR_CSSON            (1 << 19)
#define RCC_CR_PLLON            (1 << 24)
#define RCC_CR_PLLRDY           (1 << 25)

/* RCC_CFGR bits */
#define RCC_CFGR_SW_HSI         (0 << 0)
#define RCC_CFGR_SW_HSE         (1 << 0)
#define RCC_CFGR_SW_PLL         (2 << 0)
#define RCC_CFGR_SW_MASK        (3 << 0)
#define RCC_CFGR_SWS_HSI        (0 << 2)
#define RCC_CFGR_SWS_HSE        (1 << 2)
#define RCC_CFGR_SWS_PLL        (2 << 2)
#define RCC_CFGR_SWS_MASK       (3 << 2)
#define RCC_CFGR_HPRE_DIV1      (0 << 4)
#define RCC_CFGR_PPRE1_DIV4     (5 << 10)   /* APB1 = HCLK/4 = 42 MHz */
#define RCC_CFGR_PPRE2_DIV2     (4 << 13)   /* APB2 = HCLK/2 = 84 MHz */

/* ========== Flash Register Definitions ========== */

#define FLASH_BASE              0x40023C00
#define FLASH_ACR               (*(volatile uint32_t *)(FLASH_BASE + 0x00))

/* FLASH_ACR bits */
#define FLASH_ACR_LATENCY_5WS   (5 << 0)    /* 5 wait states for 168 MHz */
#define FLASH_ACR_PRFTEN        (1 << 8)    /* Prefetch enable */
#define FLASH_ACR_ICEN          (1 << 9)    /* Instruction cache enable */
#define FLASH_ACR_DCEN          (1 << 10)   /* Data cache enable */

/* ========== PWR Register Definitions ========== */

#define PWR_BASE                0x40007000
#define PWR_CR                  (*(volatile uint32_t *)(PWR_BASE + 0x00))
#define PWR_CSR                 (*(volatile uint32_t *)(PWR_BASE + 0x04))

/* PWR_CR bits */
#define PWR_CR_VOS              (1 << 14)   /* Voltage scaling output selection */

/* ========== SysTick Definitions ========== */

/* SysTick register access macros (use addresses from board/irq.h) */
#define SYSTICK_CSR_REG         (*(volatile uint32_t *)0xE000E010)
#define SYSTICK_RVR_REG         (*(volatile uint32_t *)0xE000E014)
#define SYSTICK_CVR_REG         (*(volatile uint32_t *)0xE000E018)

#define SYSTICK_CSR_ENABLE      (1 << 0)
#define SYSTICK_CSR_TICKINT     (1 << 1)
#define SYSTICK_CSR_CLKSOURCE   (1 << 2)

/* ========== Private Variables ========== */

/* System tick counter (incremented every 1ms) */
static volatile uint32_t systick_count = 0;

/* ========== Private Functions ========== */

/**
 * @brief   Configure PLL for 168 MHz from 8 MHz HSE
 * 
 * PLL configuration:
 * - PLLM = 8 (VCO input = 8 MHz / 8 = 1 MHz)
 * - PLLN = 336 (VCO output = 1 MHz * 336 = 336 MHz)
 * - PLLP = 2 (System clock = 336 MHz / 2 = 168 MHz)
 * - PLLQ = 7 (USB clock = 336 MHz / 7 = 48 MHz)
 */
static void
clock_setup_pll(void)
{
    /* Enable HSE */
    RCC_CR |= RCC_CR_HSEON;
    
    /* Wait for HSE ready */
    while (!(RCC_CR & RCC_CR_HSERDY)) {
        /* Wait */
    }
    
    /* Enable power interface clock */
    RCC_APB1ENR |= (1 << 28);   /* PWREN */
    
    /* Set voltage scaling to Scale 1 (required for 168 MHz) */
    PWR_CR |= PWR_CR_VOS;
    
    /* Configure Flash latency */
    FLASH_ACR = FLASH_ACR_LATENCY_5WS 
              | FLASH_ACR_PRFTEN 
              | FLASH_ACR_ICEN 
              | FLASH_ACR_DCEN;
    
    /* Configure PLL */
    /* PLLM=8, PLLN=336, PLLP=2, PLLQ=7, PLLSRC=HSE */
    RCC_PLLCFGR = (8 << 0)      /* PLLM */
                | (336 << 6)    /* PLLN */
                | (0 << 16)     /* PLLP = 2 (00) */
                | (1 << 22)     /* PLLSRC = HSE */
                | (7 << 24);    /* PLLQ */
    
    /* Enable PLL */
    RCC_CR |= RCC_CR_PLLON;
    
    /* Wait for PLL ready */
    while (!(RCC_CR & RCC_CR_PLLRDY)) {
        /* Wait */
    }
    
    /* Configure bus clocks */
    /* HCLK = SYSCLK, APB1 = HCLK/4, APB2 = HCLK/2 */
    RCC_CFGR = RCC_CFGR_HPRE_DIV1 
             | RCC_CFGR_PPRE1_DIV4 
             | RCC_CFGR_PPRE2_DIV2;
    
    /* Switch to PLL as system clock */
    RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW_MASK) | RCC_CFGR_SW_PLL;
    
    /* Wait for PLL as system clock */
    while ((RCC_CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL) {
        /* Wait */
    }
}

/**
 * @brief   Configure SysTick for 1ms interrupt
 */
static void
systick_setup(void)
{
    /* Configure SysTick for 1ms interrupt */
    /* Reload value = (168 MHz / 1000) - 1 = 167999 */
    SYSTICK_RVR_REG = (CONFIG_CLOCK_FREQ / 1000) - 1;
    SYSTICK_CVR_REG = 0;
    SYSTICK_CSR_REG = SYSTICK_CSR_ENABLE 
                    | SYSTICK_CSR_TICKINT 
                    | SYSTICK_CSR_CLKSOURCE;
}

/* ========== Public Functions ========== */

/**
 * @brief   System initialization
 * 
 * Configures system clock to 168 MHz and initializes peripherals.
 */
void
system_init(void)
{
    /* Configure PLL for 168 MHz */
    clock_setup_pll();
    
    /* Configure SysTick */
    systick_setup();
    
    /* Initialize GPIO */
    gpio_init();
}

/**
 * @brief   Get peripheral clock frequency
 * @param   periph_base     Peripheral base address
 * @return  Clock frequency in Hz
 */
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    /* APB1 peripherals (0x40000000 - 0x4000FFFF) */
    if (periph_base >= 0x40000000 && periph_base < 0x40010000) {
        return APB1_FREQ;
    }
    
    /* APB2 peripherals (0x40010000 - 0x4001FFFF) */
    if (periph_base >= 0x40010000 && periph_base < 0x40020000) {
        return APB2_FREQ;
    }
    
    /* AHB peripherals */
    return CONFIG_CLOCK_FREQ;
}

/**
 * @brief   Enable peripheral clock
 * @param   periph_base     Peripheral base address
 */
void
enable_pclock(uint32_t periph_base)
{
    /* GPIO ports (AHB1) */
    if (periph_base >= GPIOA_BASE && periph_base <= GPIOI_BASE) {
        uint32_t port = (periph_base - GPIOA_BASE) / 0x400;
        RCC_AHB1ENR |= (1 << port);
        return;
    }
    
    /* USART1 (APB2) */
    if (periph_base == USART1_BASE) {
        RCC_APB2ENR |= (1 << 4);
        return;
    }
    
    /* USART2 (APB1) */
    if (periph_base == USART2_BASE) {
        RCC_APB1ENR |= (1 << 17);
        return;
    }
    
    /* USART3 (APB1) */
    if (periph_base == USART3_BASE) {
        RCC_APB1ENR |= (1 << 18);
        return;
    }
    
    /* TIM2 (APB1) */
    if (periph_base == TIM2_BASE) {
        RCC_APB1ENR |= (1 << 0);
        return;
    }
    
    /* TIM3 (APB1) */
    if (periph_base == TIM3_BASE) {
        RCC_APB1ENR |= (1 << 1);
        return;
    }
    
    /* TIM4 (APB1) */
    if (periph_base == TIM4_BASE) {
        RCC_APB1ENR |= (1 << 2);
        return;
    }
    
    /* ADC1 (APB2) */
    if (periph_base == 0x40012000) {
        RCC_APB2ENR |= (1 << 8);
        return;
    }
}

/**
 * @brief   Check if peripheral clock is enabled
 * @param   periph_base     Peripheral base address
 * @return  1 if enabled, 0 otherwise
 */
int
is_enabled_pclock(uint32_t periph_base)
{
    /* GPIO ports (AHB1) */
    if (periph_base >= GPIOA_BASE && periph_base <= GPIOI_BASE) {
        uint32_t port = (periph_base - GPIOA_BASE) / 0x400;
        return (RCC_AHB1ENR >> port) & 1;
    }
    
    /* USART1 (APB2) */
    if (periph_base == USART1_BASE) {
        return (RCC_APB2ENR >> 4) & 1;
    }
    
    /* USART2 (APB1) */
    if (periph_base == USART2_BASE) {
        return (RCC_APB1ENR >> 17) & 1;
    }
    
    return 0;
}

/**
 * @brief   Get timer clock frequency
 * @return  Timer clock frequency in Hz
 */
uint32_t
timer_get_clock(void)
{
    /* APB1 timer clock = APB1 * 2 = 84 MHz */
    return APB1_FREQ * 2;
}

/**
 * @brief   Microsecond delay
 * @param   us      Delay in microseconds
 */
void
udelay(uint32_t us)
{
    /* Simple busy-wait delay */
    /* At 168 MHz, ~168 cycles per microsecond */
    uint32_t cycles = us * (CONFIG_CLOCK_FREQ / 1000000);
    while (cycles--) {
        __asm__ __volatile__("nop");
    }
}

/**
 * @brief   Get system tick count (milliseconds)
 * @return  Tick count
 */
uint32_t
systick_get(void)
{
    return systick_count;
}

/**
 * @brief   SysTick interrupt handler
 */
void
SysTick_Handler(void)
{
    systick_count++;
}

/**
 * @brief   Get high-resolution timer value (microseconds)
 * @return  Timer value in microseconds
 */
uint32_t
timer_read_time(void)
{
    /* Use SysTick current value for sub-millisecond resolution */
    uint32_t ms = systick_count;
    uint32_t ticks = SYSTICK_RVR_REG - SYSTICK_CVR_REG;
    uint32_t us = (ticks * 1000) / (CONFIG_CLOCK_FREQ / 1000);
    return (ms * 1000) + us;
}

/**
 * @brief   Check if timer value has passed
 * @param   time    Timer value to check
 * @return  1 if passed, 0 otherwise
 */
int
timer_is_before(uint32_t time1, uint32_t time2)
{
    return (int32_t)(time1 - time2) < 0;
}
