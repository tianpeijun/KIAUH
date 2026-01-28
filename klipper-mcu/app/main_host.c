/**
 * @file    main_host.c
 * @brief   主机编译版本的主程序入口
 * 
 * 用于在 Linux/macOS 上使用 GCC 编译验证代码
 * 不包含 STM32 特有的中断向量表和启动代码
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>

/* ========== 外部函数声明 ========== */

extern void gcode_init(void);
extern void gcode_process(void);
extern void toolhead_init(void);
extern void heater_init(void);
extern void heater_task(void);
extern void fan_init(void);

/* ========== 私有变量 ========== */

static volatile int s_running = 1;

/* ========== 信号处理 ========== */

static void
signal_handler(int sig)
{
    (void)sig;
    s_running = 0;
    printf("\nReceived signal, shutting down...\n");
}

/* ========== 桩函数 ========== */

/* 这些函数在 host_stubs.c 中定义 */
extern void sched_init(void);
extern void sched_main(void);
extern void serial_init(void);
extern void serial_putc(uint8_t c);
extern void serial_puts(const char* s);

void system_init(void) { }

uint32_t irq_disable(void) { return 0; }
void irq_restore(uint32_t flag) { (void)flag; }
int sched_is_shutdown(void) { return !s_running; }

/* ========== 主函数 ========== */

int
board_init(void)
{
    printf("\n");
    printf("========================================\n");
    printf("  Klipper MCU Firmware (Host Build)\n");
    printf("  For verification only\n");
    printf("========================================\n");
    printf("Board initialized (host simulation).\n");
    
    return 0;
}

int
main(int argc, char* argv[])
{
    int loop_count = 0;
    int max_loops = 10;  /* 默认运行 10 次循环后退出 */
    
    (void)argc;
    (void)argv;
    
    /* 设置信号处理 */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* 硬件初始化 */
    board_init();
    
    /* 调度器初始化 */
    sched_init();
    printf("Scheduler initialized.\n");
    
    /* 模块初始化 */
    toolhead_init();
    printf("Toolhead initialized.\n");
    
    heater_init();
    printf("Heater initialized.\n");
    
    fan_init();
    printf("Fan initialized.\n");
    
    gcode_init();
    printf("G-code parser initialized.\n");
    
    /* 系统就绪 */
    printf("\nSystem ready. Running %d loop iterations...\n", max_loops);
    printf("ok\n");
    
    /* 主循环 (有限次数) */
    while (s_running && loop_count < max_loops) {
        /* 调度器主循环 */
        sched_main();
        
        /* 处理 G-code 输入 */
        gcode_process();
        
        /* 温度控制任务 */
        heater_task();
        
        loop_count++;
    }
    
    printf("\nMain loop completed (%d iterations).\n", loop_count);
    printf("Host build verification successful!\n");
    
    return 0;
}
