/**
 * @file    sched.c
 * @brief   任务调度器实现
 * 
 * 复用自 Klipper src/sched.c
 * 提供定时器回调注册和任务调度功能
 */

#include "sched.h"
#include "board/irq.h"
#include <stddef.h>

/* ========== 私有变量 ========== */

/* 定时器链表头 */
static sched_timer_t* s_timer_list = NULL;

/* 系统关闭标志 */
static volatile int s_shutdown_flag = 0;

/* 关闭原因 */
static const char* s_shutdown_reason = NULL;

/* ========== 时间接口实现 ========== */

/**
 * @brief  获取当前系统时间（微秒级）
 * @note   需要 HAL 层实现具体的硬件定时器读取
 * @retval 当前时间戳
 */
sched_time_t sched_get_time(void)
{
    /* TODO: 从硬件定时器读取当前时间 */
    /* 需要 HAL 层提供 timer_read_time() 函数 */
    extern uint32_t timer_read_time(void);
    return timer_read_time();
}

/**
 * @brief  检查时间是否已到
 * @param  time 目标时间
 * @retval 1 已到，0 未到
 */
int sched_is_before(sched_time_t time)
{
    sched_time_t now = sched_get_time();
    return sched_time_diff(time, now) <= 0;
}

/**
 * @brief  计算时间差（考虑溢出）
 * @param  t1 时间 1
 * @param  t2 时间 2
 * @retval t1 - t2 的差值
 */
int32_t sched_time_diff(sched_time_t t1, sched_time_t t2)
{
    return (int32_t)(t1 - t2);
}

/* ========== 关键区保护实现 ========== */

/**
 * @brief  进入关键区（禁用中断）
 * @retval 之前的中断状态
 */
uint32_t sched_irq_save(void)
{
    /* TODO: 需要 HAL 层实现 */
    extern uint32_t irq_save(void);
    return irq_save();
}

/**
 * @brief  退出关键区（恢复中断）
 * @param  flag 之前的中断状态
 */
void sched_irq_restore(uint32_t flag)
{
    /* TODO: 需要 HAL 层实现 */
    extern void irq_restore(uint32_t flag);
    irq_restore(flag);
}

/* ========== 定时器管理实现 ========== */

/**
 * @brief  添加定时器到链表（按唤醒时间排序）
 * @param  timer 定时器结构体指针
 */
void sched_add_timer(sched_timer_t* timer)
{
    uint32_t flag;
    sched_timer_t** p_prev;
    
    if (timer == NULL || timer->func == NULL) {
        return;
    }
    
    flag = sched_irq_save();
    
    /* 按唤醒时间排序插入链表 */
    p_prev = &s_timer_list;
    while (*p_prev != NULL) {
        if (sched_time_diff(timer->waketime, (*p_prev)->waketime) < 0) {
            break;
        }
        p_prev = &((*p_prev)->next);
    }
    
    timer->next = *p_prev;
    *p_prev = timer;
    
    sched_irq_restore(flag);
}

/**
 * @brief  从链表移除定时器
 * @param  timer 定时器结构体指针
 */
void sched_del_timer(sched_timer_t* timer)
{
    uint32_t flag;
    sched_timer_t** p_prev;
    
    if (timer == NULL) {
        return;
    }
    
    flag = sched_irq_save();
    
    /* 查找并移除 */
    p_prev = &s_timer_list;
    while (*p_prev != NULL) {
        if (*p_prev == timer) {
            *p_prev = timer->next;
            timer->next = NULL;
            break;
        }
        p_prev = &((*p_prev)->next);
    }
    
    sched_irq_restore(flag);
}

/* ========== 调度器核心实现 ========== */

/**
 * @brief  初始化调度器
 * @retval 0 成功，负数 失败
 */
int sched_init(void)
{
    s_timer_list = NULL;
    s_shutdown_flag = 0;
    s_shutdown_reason = NULL;
    
    /* TODO: 初始化硬件定时器 */
    /* 需要 HAL 层提供 timer_init() 函数 */
    
    return 0;
}

/**
 * @brief  调度器主循环
 * @note   处理到期的定时器回调
 */
void sched_main(void)
{
    sched_timer_t* timer;
    sched_time_t waketime;
    sched_time_t next_waketime;
    uint32_t flag;
    
    /* 检查系统是否已关闭 */
    if (s_shutdown_flag) {
        return;
    }
    
    /* 处理到期的定时器 */
    for (;;) {
        flag = sched_irq_save();
        
        timer = s_timer_list;
        if (timer == NULL) {
            sched_irq_restore(flag);
            break;
        }
        
        /* 检查是否到期 */
        if (!sched_is_before(timer->waketime)) {
            sched_irq_restore(flag);
            break;
        }
        
        /* 从链表移除 */
        s_timer_list = timer->next;
        timer->next = NULL;
        waketime = timer->waketime;
        
        sched_irq_restore(flag);
        
        /* 执行回调 */
        next_waketime = timer->func(waketime);
        
        /* 如果返回非零，重新添加定时器 */
        if (next_waketime != 0) {
            timer->waketime = next_waketime;
            sched_add_timer(timer);
        }
    }
}

/* ========== 系统状态实现 ========== */

/**
 * @brief  检查系统是否已关闭
 * @retval 1 已关闭，0 运行中
 */
int sched_is_shutdown(void)
{
    return s_shutdown_flag;
}

/**
 * @brief  触发系统关闭
 * @param  reason 关闭原因字符串
 */
void sched_shutdown(const char* reason)
{
    uint32_t flag;
    
    flag = sched_irq_save();
    
    if (!s_shutdown_flag) {
        s_shutdown_flag = 1;
        s_shutdown_reason = reason;
        
        /* TODO: 停止所有外设 */
        /* 禁用步进电机、加热器等 */
    }
    
    sched_irq_restore(flag);
}
