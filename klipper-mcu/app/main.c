/**
 * @file    main.c
 * @brief   程序入口和主循环
 * 
 * Klipper MCU 移植项目 - 主程序入口
 * 实现硬件初始化和主循环框架
 * 
 * @note    验收标准: 5.1.1, 5.1.4
 */

#include <stdint.h>
#include <stddef.h>

#include "autoconf.h"
#include "config.h"
#include "src/sched.h"
#include "src/stm32/internal.h"
#include "src/stm32/serial.h"
#include "board/irq.h"
#include "board/misc.h"

/* ========== 外部函数声明 ========== */

/* 模块初始化函数 (后续任务实现) */
extern void gcode_init(void) __attribute__((weak));
extern void gcode_process(void) __attribute__((weak));
extern void toolhead_init(void) __attribute__((weak));
extern void heater_init(void) __attribute__((weak));
extern void fan_init(void) __attribute__((weak));

/* ========== 私有变量 ========== */

/* 系统启动标志 */
static volatile uint8_t s_system_ready = 0;

/* ========== NVIC 函数实现 ========== */

/**
 * @brief   使能 NVIC 中断
 * @param   irq     中断号
 */
void
nvic_enable_irq(uint8_t irq)
{
    volatile uint32_t* p_iser = (volatile uint32_t*)(NVIC_ISER);
    p_iser[irq / 32] = (1UL << (irq % 32));
}

/**
 * @brief   禁用 NVIC 中断
 * @param   irq     中断号
 */
void
nvic_disable_irq(uint8_t irq)
{
    volatile uint32_t* p_icer = (volatile uint32_t*)(NVIC_ICER);
    p_icer[irq / 32] = (1UL << (irq % 32));
}

/**
 * @brief   设置 NVIC 中断优先级
 * @param   irq         中断号
 * @param   priority    优先级 (0-255, 越小优先级越高)
 */
void
nvic_set_priority(uint8_t irq, uint8_t priority)
{
    volatile uint8_t* p_ipr = (volatile uint8_t*)(NVIC_IPR);
    p_ipr[irq] = priority;
}

/**
 * @brief   清除 NVIC 中断挂起标志
 * @param   irq     中断号
 */
void
nvic_clear_pending(uint8_t irq)
{
    volatile uint32_t* p_icpr = (volatile uint32_t*)(NVIC_ICPR);
    p_icpr[irq / 32] = (1UL << (irq % 32));
}

/**
 * @brief   保存中断状态并禁用中断
 * @retval  之前的中断状态
 * @note    供 sched.c 使用
 */
uint32_t
irq_save(void)
{
    return irq_disable();
}

/* ========== 中断向量表 ========== */

/* 栈顶指针 (由链接脚本定义) */
extern uint32_t _estack;

/* 复位处理函数 */
void Reset_Handler(void);

/* 默认中断处理函数 */
void Default_Handler(void);

/* 系统异常处理函数 */
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void);

/* 外设中断处理函数 */
void WWDG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PVD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TAMP_STAMP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_TX_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_RX0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_RX1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_SCE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_TIM9_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_TIM10_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM11_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void OTG_FS_WKUP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM8_BRK_TIM12_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM8_UP_TIM13_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM8_TRG_COM_TIM14_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM8_CC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FSMC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SDIO_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM6_DAC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ETH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ETH_WKUP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN2_TX_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN2_RX0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN2_RX1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN2_SCE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void OTG_FS_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void OTG_HS_EP1_OUT_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void OTG_HS_EP1_IN_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void OTG_HS_WKUP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void OTG_HS_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DCMI_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CRYP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HASH_RNG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FPU_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

/* 中断向量表 - 放置在 .isr_vector 段 */
__attribute__((section(".isr_vector")))
const uint32_t g_vector_table[] = {
    /* 栈顶指针 */
    (uint32_t)&_estack,
    
    /* Cortex-M4 系统异常 */
    (uint32_t)Reset_Handler,
    (uint32_t)NMI_Handler,
    (uint32_t)HardFault_Handler,
    (uint32_t)MemManage_Handler,
    (uint32_t)BusFault_Handler,
    (uint32_t)UsageFault_Handler,
    0, 0, 0, 0,                     /* 保留 */
    (uint32_t)SVC_Handler,
    (uint32_t)DebugMon_Handler,
    0,                              /* 保留 */
    (uint32_t)PendSV_Handler,
    (uint32_t)SysTick_Handler,
    
    /* STM32F407 外设中断 */
    (uint32_t)WWDG_IRQHandler,              /* 0 */
    (uint32_t)PVD_IRQHandler,               /* 1 */
    (uint32_t)TAMP_STAMP_IRQHandler,        /* 2 */
    (uint32_t)RTC_WKUP_IRQHandler,          /* 3 */
    (uint32_t)FLASH_IRQHandler,             /* 4 */
    (uint32_t)RCC_IRQHandler,               /* 5 */
    (uint32_t)EXTI0_IRQHandler,             /* 6 */
    (uint32_t)EXTI1_IRQHandler,             /* 7 */
    (uint32_t)EXTI2_IRQHandler,             /* 8 */
    (uint32_t)EXTI3_IRQHandler,             /* 9 */
    (uint32_t)EXTI4_IRQHandler,             /* 10 */
    (uint32_t)DMA1_Stream0_IRQHandler,      /* 11 */
    (uint32_t)DMA1_Stream1_IRQHandler,      /* 12 */
    (uint32_t)DMA1_Stream2_IRQHandler,      /* 13 */
    (uint32_t)DMA1_Stream3_IRQHandler,      /* 14 */
    (uint32_t)DMA1_Stream4_IRQHandler,      /* 15 */
    (uint32_t)DMA1_Stream5_IRQHandler,      /* 16 */
    (uint32_t)DMA1_Stream6_IRQHandler,      /* 17 */
    (uint32_t)ADC_IRQHandler,               /* 18 */
    (uint32_t)CAN1_TX_IRQHandler,           /* 19 */
    (uint32_t)CAN1_RX0_IRQHandler,          /* 20 */
    (uint32_t)CAN1_RX1_IRQHandler,          /* 21 */
    (uint32_t)CAN1_SCE_IRQHandler,          /* 22 */
    (uint32_t)EXTI9_5_IRQHandler,           /* 23 */
    (uint32_t)TIM1_BRK_TIM9_IRQHandler,     /* 24 */
    (uint32_t)TIM1_UP_TIM10_IRQHandler,     /* 25 */
    (uint32_t)TIM1_TRG_COM_TIM11_IRQHandler,/* 26 */
    (uint32_t)TIM1_CC_IRQHandler,           /* 27 */
    (uint32_t)TIM2_IRQHandler,              /* 28 */
    (uint32_t)TIM3_IRQHandler,              /* 29 */
    (uint32_t)TIM4_IRQHandler,              /* 30 */
    (uint32_t)I2C1_EV_IRQHandler,           /* 31 */
    (uint32_t)I2C1_ER_IRQHandler,           /* 32 */
    (uint32_t)I2C2_EV_IRQHandler,           /* 33 */
    (uint32_t)I2C2_ER_IRQHandler,           /* 34 */
    (uint32_t)SPI1_IRQHandler,              /* 35 */
    (uint32_t)SPI2_IRQHandler,              /* 36 */
    (uint32_t)USART1_IRQHandler,            /* 37 */
    (uint32_t)USART2_IRQHandler,            /* 38 */
    (uint32_t)USART3_IRQHandler,            /* 39 */
    (uint32_t)EXTI15_10_IRQHandler,         /* 40 */
    (uint32_t)RTC_Alarm_IRQHandler,         /* 41 */
    (uint32_t)OTG_FS_WKUP_IRQHandler,       /* 42 */
    (uint32_t)TIM8_BRK_TIM12_IRQHandler,    /* 43 */
    (uint32_t)TIM8_UP_TIM13_IRQHandler,     /* 44 */
    (uint32_t)TIM8_TRG_COM_TIM14_IRQHandler,/* 45 */
    (uint32_t)TIM8_CC_IRQHandler,           /* 46 */
    (uint32_t)DMA1_Stream7_IRQHandler,      /* 47 */
    (uint32_t)FSMC_IRQHandler,              /* 48 */
    (uint32_t)SDIO_IRQHandler,              /* 49 */
    (uint32_t)TIM5_IRQHandler,              /* 50 */
    (uint32_t)SPI3_IRQHandler,              /* 51 */
    (uint32_t)UART4_IRQHandler,             /* 52 */
    (uint32_t)UART5_IRQHandler,             /* 53 */
    (uint32_t)TIM6_DAC_IRQHandler,          /* 54 */
    (uint32_t)TIM7_IRQHandler,              /* 55 */
    (uint32_t)DMA2_Stream0_IRQHandler,      /* 56 */
    (uint32_t)DMA2_Stream1_IRQHandler,      /* 57 */
    (uint32_t)DMA2_Stream2_IRQHandler,      /* 58 */
    (uint32_t)DMA2_Stream3_IRQHandler,      /* 59 */
    (uint32_t)DMA2_Stream4_IRQHandler,      /* 60 */
    (uint32_t)ETH_IRQHandler,               /* 61 */
    (uint32_t)ETH_WKUP_IRQHandler,          /* 62 */
    (uint32_t)CAN2_TX_IRQHandler,           /* 63 */
    (uint32_t)CAN2_RX0_IRQHandler,          /* 64 */
    (uint32_t)CAN2_RX1_IRQHandler,          /* 65 */
    (uint32_t)CAN2_SCE_IRQHandler,          /* 66 */
    (uint32_t)OTG_FS_IRQHandler,            /* 67 */
    (uint32_t)DMA2_Stream5_IRQHandler,      /* 68 */
    (uint32_t)DMA2_Stream6_IRQHandler,      /* 69 */
    (uint32_t)DMA2_Stream7_IRQHandler,      /* 70 */
    (uint32_t)USART6_IRQHandler,            /* 71 */
    (uint32_t)I2C3_EV_IRQHandler,           /* 72 */
    (uint32_t)I2C3_ER_IRQHandler,           /* 73 */
    (uint32_t)OTG_HS_EP1_OUT_IRQHandler,    /* 74 */
    (uint32_t)OTG_HS_EP1_IN_IRQHandler,     /* 75 */
    (uint32_t)OTG_HS_WKUP_IRQHandler,       /* 76 */
    (uint32_t)OTG_HS_IRQHandler,            /* 77 */
    (uint32_t)DCMI_IRQHandler,              /* 78 */
    (uint32_t)CRYP_IRQHandler,              /* 79 */
    (uint32_t)HASH_RNG_IRQHandler,          /* 80 */
    (uint32_t)FPU_IRQHandler,               /* 81 */
};

/* ========== 私有函数 ========== */

/**
 * @brief   初始化 BSS 段 (清零)
 */
static void
bss_init(void)
{
    extern uint32_t _sbss;
    extern uint32_t _ebss;
    
    uint32_t* p_dst = &_sbss;
    while (p_dst < &_ebss) {
        *p_dst++ = 0;
    }
}

/**
 * @brief   初始化 DATA 段 (从 Flash 复制到 RAM)
 */
static void
data_init(void)
{
    extern uint32_t _sidata;
    extern uint32_t _sdata;
    extern uint32_t _edata;
    
    uint32_t* p_src = &_sidata;
    uint32_t* p_dst = &_sdata;
    
    while (p_dst < &_edata) {
        *p_dst++ = *p_src++;
    }
}

/**
 * @brief   启用 FPU (浮点单元)
 */
static void
fpu_enable(void)
{
    /* 启用 CP10 和 CP11 协处理器 */
    /* SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2)) */
    volatile uint32_t* p_cpacr = (volatile uint32_t*)0xE000ED88;
    *p_cpacr |= ((3UL << 20) | (3UL << 22));
    
    /* 数据同步屏障 */
    __asm__ __volatile__("dsb" ::: "memory");
    __asm__ __volatile__("isb" ::: "memory");
}

/* ========== 公有函数 ========== */

/**
 * @brief   硬件初始化
 * 
 * 初始化所有硬件外设:
 * - 系统时钟 (168 MHz)
 * - GPIO
 * - 串口 (115200 baud)
 * - ADC
 * 
 * @retval  0 成功，负数 失败
 */
int
board_init(void)
{
    /* 系统初始化 (时钟、GPIO) */
    system_init();
    
    /* 串口初始化 */
    serial_init();
    
    /* 输出启动信息 */
    serial_puts("\r\n");
    serial_puts("========================================\r\n");
    serial_puts("  Klipper MCU Firmware v" CONFIG_VERSION "\r\n");
    serial_puts("  Build: " CONFIG_BUILD_DATE " " CONFIG_BUILD_TIME "\r\n");
    serial_puts("========================================\r\n");
    serial_puts("Board initialized.\r\n");
    
    return 0;
}

/**
 * @brief   复位处理函数 (启动入口)
 * 
 * MCU 复位后首先执行此函数:
 * 1. 初始化 BSS 段
 * 2. 初始化 DATA 段
 * 3. 启用 FPU
 * 4. 调用 main()
 */
void
Reset_Handler(void)
{
    /* 初始化 BSS 段 (清零) */
    bss_init();
    
    /* 初始化 DATA 段 (从 Flash 复制) */
    data_init();
    
    /* 启用 FPU */
    fpu_enable();
    
    /* 调用 main 函数 */
    extern int main(void);
    main();
    
    /* main 不应返回，如果返回则进入死循环 */
    for (;;) {
        __asm__ __volatile__("wfi");
    }
}

/**
 * @brief   默认中断处理函数
 * 
 * 未实现的中断会跳转到此处
 */
void
Default_Handler(void)
{
    /* 进入死循环 */
    for (;;) {
        __asm__ __volatile__("nop");
    }
}

/**
 * @brief   硬件错误处理函数
 */
void
HardFault_Handler(void)
{
    /* 输出错误信息 (如果串口可用) */
    serial_puts("\r\n!!! HardFault !!!\r\n");
    
    /* 进入死循环 */
    for (;;) {
        __asm__ __volatile__("nop");
    }
}

/**
 * @brief   主函数
 * 
 * 程序入口点，初始化系统并进入主循环
 * 
 * @retval  0 (永不返回)
 */
int
main(void)
{
    /* 硬件初始化 */
    board_init();
    
    /* 调度器初始化 */
    sched_init();
    serial_puts("Scheduler initialized.\r\n");
    
    /* 模块初始化 (弱符号，后续任务实现) */
    if (toolhead_init) {
        toolhead_init();
        serial_puts("Toolhead initialized.\r\n");
    }
    
    if (heater_init) {
        heater_init();
        serial_puts("Heater initialized.\r\n");
    }
    
    if (fan_init) {
        fan_init();
        serial_puts("Fan initialized.\r\n");
    }
    
    if (gcode_init) {
        gcode_init();
        serial_puts("G-code parser initialized.\r\n");
    }
    
    /* 系统就绪 */
    s_system_ready = 1;
    serial_puts("\r\nSystem ready. Entering main loop...\r\n");
    serial_puts("ok\r\n");
    
    /* 主循环 */
    for (;;) {
        /* 调度器主循环 - 处理定时器回调 */
        sched_main();
        
        /* 处理 G-code 输入 (弱符号，后续任务实现) */
        if (gcode_process) {
            gcode_process();
        }
        
        /* 检查系统是否关闭 */
        if (sched_is_shutdown()) {
            serial_puts("\r\n!!! System shutdown !!!\r\n");
            break;
        }
    }
    
    /* 系统关闭后进入低功耗等待 */
    for (;;) {
        __asm__ __volatile__("wfi");
    }
    
    return 0;
}

/* ========== 调试输出函数 ========== */

/**
 * @brief   输出调试字符
 * @param   c   字符
 */
void
debug_putc(char c)
{
    serial_putc((uint8_t)c);
}

/**
 * @brief   输出调试字符串
 * @param   str 字符串
 */
void
debug_puts(const char* str)
{
    serial_puts(str);
}

/**
 * @brief   输出调试十六进制值
 * @param   val 值
 */
void
debug_hex(uint32_t val)
{
    static const char hex_chars[] = "0123456789ABCDEF";
    char buf[11];
    int i;
    
    buf[0] = '0';
    buf[1] = 'x';
    
    for (i = 0; i < 8; i++) {
        buf[9 - i] = hex_chars[val & 0x0F];
        val >>= 4;
    }
    buf[10] = '\0';
    
    serial_puts(buf);
}

/**
 * @brief   系统崩溃处理
 * @param   msg 崩溃信息
 */
void
panic(const char* msg)
{
    /* 禁用中断 */
    irq_disable();
    
    /* 输出崩溃信息 */
    serial_puts("\r\n!!! PANIC: ");
    serial_puts(msg);
    serial_puts(" !!!\r\n");
    
    /* 进入死循环 */
    for (;;) {
        __asm__ __volatile__("nop");
    }
}
