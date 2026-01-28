/**
 * @file    sched.h
 * @brief   任务调度器接口
 * 
 * 复用自 Klipper src/sched.h
 * 提供定时器回调注册和任务调度功能
 */

#ifndef SCHED_H
#define SCHED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 时间类型定义 ========== */

/* 系统时钟计数类型 */
typedef uint32_t sched_time_t;

/* ========== 定时器回调 ========== */

/**
 * @brief  定时器回调函数类型
 * @param  waketime 唤醒时间
 * @retval 下次唤醒时间，返回 0 表示不再调度
 */
typedef sched_time_t (*sched_timer_fn_t)(sched_time_t waketime);

/* 定时器结构体 */
typedef struct sched_timer {
    struct sched_timer* next;   /* 链表下一个节点 */
    sched_time_t waketime;      /* 唤醒时间 */
    sched_timer_fn_t func;      /* 回调函数 */
} sched_timer_t;

/* ========== 任务回调 ========== */

/**
 * @brief  任务回调函数类型
 */
typedef void (*sched_task_fn_t)(void);

/* ========== 调度器接口 ========== */

/**
 * @brief  初始化调度器
 * @retval 0 成功，负数 失败
 */
int sched_init(void);

/**
 * @brief  调度器主循环
 * @note   在 main() 的无限循环中调用
 */
void sched_main(void);

/**
 * @brief  添加定时器
 * @param  timer 定时器结构体指针
 */
void sched_add_timer(sched_timer_t* timer);

/**
 * @brief  移除定时器
 * @param  timer 定时器结构体指针
 */
void sched_del_timer(sched_timer_t* timer);

/* ========== 时间接口 ========== */

/**
 * @brief  获取当前系统时间（微秒级）
 * @retval 当前时间戳
 */
sched_time_t sched_get_time(void);

/**
 * @brief  检查时间是否已到
 * @param  time 目标时间
 * @retval 1 已到，0 未到
 */
int sched_is_before(sched_time_t time);

/**
 * @brief  计算时间差
 * @param  t1 时间 1
 * @param  t2 时间 2
 * @retval t1 - t2 的差值（考虑溢出）
 */
int32_t sched_time_diff(sched_time_t t1, sched_time_t t2);

/* ========== 关键区保护 ========== */

/**
 * @brief  进入关键区（禁用中断）
 * @retval 之前的中断状态
 */
uint32_t sched_irq_save(void);

/**
 * @brief  退出关键区（恢复中断）
 * @param  flag 之前的中断状态
 */
void sched_irq_restore(uint32_t flag);

/* ========== 系统状态 ========== */

/**
 * @brief  检查系统是否已关闭
 * @retval 1 已关闭，0 运行中
 */
int sched_is_shutdown(void);

/**
 * @brief  触发系统关闭
 * @param  reason 关闭原因字符串
 */
void sched_shutdown(const char* reason);

#ifdef __cplusplus
}
#endif

#endif /* SCHED_H */
