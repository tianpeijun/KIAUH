/**
 * @file    irq.h
 * @brief   Interrupt management interface
 * 
 * Provides interrupt enable/disable functions and critical section
 * management for STM32F407.
 */

#ifndef BOARD_IRQ_H
#define BOARD_IRQ_H

#include <stdint.h>

/* ========== Interrupt Control ========== */

/**
 * @brief   Disable all interrupts
 * @return  Previous interrupt state (for restore)
 */
static inline uint32_t irq_disable(void)
{
    uint32_t primask;
    __asm__ __volatile__(
        "mrs %0, primask\n"
        "cpsid i\n"
        : "=r" (primask)
        :
        : "memory"
    );
    return primask;
}

/**
 * @brief   Enable all interrupts
 */
static inline void irq_enable(void)
{
    __asm__ __volatile__("cpsie i" ::: "memory");
}

/**
 * @brief   Restore interrupt state
 * @param   flag    Previous interrupt state from irq_disable()
 */
static inline void irq_restore(uint32_t flag)
{
    __asm__ __volatile__(
        "msr primask, %0\n"
        :
        : "r" (flag)
        : "memory"
    );
}

/**
 * @brief   Check if interrupts are enabled
 * @return  1 if enabled, 0 if disabled
 */
static inline int irq_enabled(void)
{
    uint32_t primask;
    __asm__ __volatile__("mrs %0, primask" : "=r" (primask));
    return (primask & 1) == 0;
}

/**
 * @brief   Wait for interrupt (low power)
 */
static inline void irq_wait(void)
{
    __asm__ __volatile__("wfi" ::: "memory");
}

/* ========== Critical Section ========== */

/**
 * @brief   Enter critical section (disable interrupts)
 * @return  Previous interrupt state
 */
static inline uint32_t critical_enter(void)
{
    return irq_disable();
}

/**
 * @brief   Exit critical section (restore interrupts)
 * @param   flag    Previous interrupt state
 */
static inline void critical_exit(uint32_t flag)
{
    irq_restore(flag);
}

/* ========== NVIC Functions ========== */

/* NVIC base address */
#define NVIC_BASE               0xE000E100
#define NVIC_ISER               (NVIC_BASE + 0x000)  /* Interrupt Set Enable */
#define NVIC_ICER               (NVIC_BASE + 0x080)  /* Interrupt Clear Enable */
#define NVIC_ISPR               (NVIC_BASE + 0x100)  /* Interrupt Set Pending */
#define NVIC_ICPR               (NVIC_BASE + 0x180)  /* Interrupt Clear Pending */
#define NVIC_IPR                (NVIC_BASE + 0x300)  /* Interrupt Priority */

/* SCB base address */
#define SCB_BASE                0xE000ED00
#define SCB_ICSR                (SCB_BASE + 0x04)    /* Interrupt Control State */
#define SCB_VTOR                (SCB_BASE + 0x08)    /* Vector Table Offset */
#define SCB_AIRCR               (SCB_BASE + 0x0C)    /* App Interrupt/Reset Control */
#define SCB_SCR                 (SCB_BASE + 0x10)    /* System Control Register */

/* SysTick base address */
#define SYSTICK_BASE            0xE000E010
#define SYSTICK_CSR             (SYSTICK_BASE + 0x00) /* Control and Status */
#define SYSTICK_RVR             (SYSTICK_BASE + 0x04) /* Reload Value */
#define SYSTICK_CVR             (SYSTICK_BASE + 0x08) /* Current Value */

/**
 * @brief   Enable NVIC interrupt
 * @param   irq     IRQ number
 */
void nvic_enable_irq(uint8_t irq);

/**
 * @brief   Disable NVIC interrupt
 * @param   irq     IRQ number
 */
void nvic_disable_irq(uint8_t irq);

/**
 * @brief   Set NVIC interrupt priority
 * @param   irq         IRQ number
 * @param   priority    Priority (0-255, lower = higher priority)
 */
void nvic_set_priority(uint8_t irq, uint8_t priority);

/**
 * @brief   Clear pending NVIC interrupt
 * @param   irq     IRQ number
 */
void nvic_clear_pending(uint8_t irq);

/* ========== STM32F407 IRQ Numbers ========== */

typedef enum {
    /* Cortex-M4 internal interrupts */
    IRQ_NMI             = -14,
    IRQ_HARDFAULT       = -13,
    IRQ_MEMMANAGE       = -12,
    IRQ_BUSFAULT        = -11,
    IRQ_USAGEFAULT      = -10,
    IRQ_SVCALL          = -5,
    IRQ_DEBUGMON        = -4,
    IRQ_PENDSV          = -2,
    IRQ_SYSTICK         = -1,
    
    /* STM32F407 peripheral interrupts */
    IRQ_WWDG            = 0,
    IRQ_PVD             = 1,
    IRQ_TAMP_STAMP      = 2,
    IRQ_RTC_WKUP        = 3,
    IRQ_FLASH           = 4,
    IRQ_RCC             = 5,
    IRQ_EXTI0           = 6,
    IRQ_EXTI1           = 7,
    IRQ_EXTI2           = 8,
    IRQ_EXTI3           = 9,
    IRQ_EXTI4           = 10,
    IRQ_DMA1_STREAM0    = 11,
    IRQ_DMA1_STREAM1    = 12,
    IRQ_DMA1_STREAM2    = 13,
    IRQ_DMA1_STREAM3    = 14,
    IRQ_DMA1_STREAM4    = 15,
    IRQ_DMA1_STREAM5    = 16,
    IRQ_DMA1_STREAM6    = 17,
    IRQ_ADC             = 18,
    IRQ_CAN1_TX         = 19,
    IRQ_CAN1_RX0        = 20,
    IRQ_CAN1_RX1        = 21,
    IRQ_CAN1_SCE        = 22,
    IRQ_EXTI9_5         = 23,
    IRQ_TIM1_BRK_TIM9   = 24,
    IRQ_TIM1_UP_TIM10   = 25,
    IRQ_TIM1_TRG_COM_TIM11 = 26,
    IRQ_TIM1_CC         = 27,
    IRQ_TIM2            = 28,
    IRQ_TIM3            = 29,
    IRQ_TIM4            = 30,
    IRQ_I2C1_EV         = 31,
    IRQ_I2C1_ER         = 32,
    IRQ_I2C2_EV         = 33,
    IRQ_I2C2_ER         = 34,
    IRQ_SPI1            = 35,
    IRQ_SPI2            = 36,
    IRQ_USART1          = 37,
    IRQ_USART2          = 38,
    IRQ_USART3          = 39,
    IRQ_EXTI15_10       = 40,
    IRQ_RTC_ALARM       = 41,
    IRQ_OTG_FS_WKUP     = 42,
    IRQ_TIM8_BRK_TIM12  = 43,
    IRQ_TIM8_UP_TIM13   = 44,
    IRQ_TIM8_TRG_COM_TIM14 = 45,
    IRQ_TIM8_CC         = 46,
    IRQ_DMA1_STREAM7    = 47,
    IRQ_FSMC            = 48,
    IRQ_SDIO            = 49,
    IRQ_TIM5            = 50,
    IRQ_SPI3            = 51,
    IRQ_UART4           = 52,
    IRQ_UART5           = 53,
    IRQ_TIM6_DAC        = 54,
    IRQ_TIM7            = 55,
    IRQ_DMA2_STREAM0    = 56,
    IRQ_DMA2_STREAM1    = 57,
    IRQ_DMA2_STREAM2    = 58,
    IRQ_DMA2_STREAM3    = 59,
    IRQ_DMA2_STREAM4    = 60,
    IRQ_ETH             = 61,
    IRQ_ETH_WKUP        = 62,
    IRQ_CAN2_TX         = 63,
    IRQ_CAN2_RX0        = 64,
    IRQ_CAN2_RX1        = 65,
    IRQ_CAN2_SCE        = 66,
    IRQ_OTG_FS          = 67,
    IRQ_DMA2_STREAM5    = 68,
    IRQ_DMA2_STREAM6    = 69,
    IRQ_DMA2_STREAM7    = 70,
    IRQ_USART6          = 71,
    IRQ_I2C3_EV         = 72,
    IRQ_I2C3_ER         = 73,
    IRQ_OTG_HS_EP1_OUT  = 74,
    IRQ_OTG_HS_EP1_IN   = 75,
    IRQ_OTG_HS_WKUP     = 76,
    IRQ_OTG_HS          = 77,
    IRQ_DCMI            = 78,
    IRQ_CRYP            = 79,
    IRQ_HASH_RNG        = 80,
    IRQ_FPU             = 81,
    
    IRQ_COUNT           = 82
} irq_number_t;

#endif /* BOARD_IRQ_H */
