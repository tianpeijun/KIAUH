/**
 * @file    endstop.c
 * @brief   限位开关检测实现
 * 
 * 复用自 Klipper src/endstop.c
 * 提供限位开关状态检测和归零停止功能
 */

#include "endstop.h"
#include "sched.h"
#include "stepper.h"
#include <stddef.h>

/* ========== 私有类型定义 ========== */

/* 限位开关状态 */
typedef struct {
    uint8_t pin;                /* GPIO 引脚 */
    uint8_t invert;             /* 电平反相 */
    uint8_t configured;         /* 是否已配置 */
    uint8_t triggered;          /* 触发状态 */
    uint8_t homing;             /* 归零模式 */
    stepper_id_t stepper_id;    /* 关联的步进电机 */
    endstop_callback_fn_t callback; /* 触发回调 */
    void* callback_arg;         /* 回调参数 */
} endstop_state_t;

/* ========== 私有变量 ========== */

/* 限位开关状态数组 */
static endstop_state_t s_endstops[ENDSTOP_COUNT];

/* 采样定时器 */
static sched_timer_t s_endstop_timer;

/* 采样间隔（时钟周期） */
#define ENDSTOP_SAMPLE_INTERVAL     1000    /* 约 1ms */

/* ========== GPIO 操作 ========== */

/* TODO: 需要 HAL 层实现这些函数 */
extern void gpio_in_setup(uint8_t pin, int8_t pull);
extern uint8_t gpio_in_read(uint8_t pin);

/* ========== 私有函数 ========== */

/**
 * @brief  读取限位开关状态
 * @param  endstop 限位开关状态指针
 * @retval 1 触发，0 未触发
 */
static int endstop_read_state(endstop_state_t* endstop)
{
    uint8_t value;
    
    if (endstop == NULL || !endstop->configured) {
        return 0;
    }
    
    value = gpio_in_read(endstop->pin);
    
    if (endstop->invert) {
        value = !value;
    }
    
    return value ? 1 : 0;
}

/**
 * @brief  限位开关采样定时器回调
 * @param  waketime 唤醒时间
 * @retval 下次唤醒时间
 */
static sched_time_t endstop_timer_callback(sched_time_t waketime)
{
    int i;
    int state;
    
    /* 遍历所有限位开关 */
    for (i = 0; i < ENDSTOP_COUNT; i++) {
        endstop_state_t* endstop = &s_endstops[i];
        
        if (!endstop->configured) {
            continue;
        }
        
        /* 读取当前状态 */
        state = endstop_read_state(endstop);
        
        /* 检测触发 */
        if (state && !endstop->triggered) {
            endstop->triggered = 1;
            
            /* 如果在归零模式，停止关联的步进电机 */
            if (endstop->homing) {
                stepper_stop(endstop->stepper_id);
            }
            
            /* 调用回调函数 */
            if (endstop->callback != NULL) {
                endstop->callback((endstop_id_t)i, endstop->callback_arg);
            }
        } else if (!state && endstop->triggered) {
            /* 限位开关释放 */
            endstop->triggered = 0;
        }
    }
    
    /* 返回下次采样时间 */
    return waketime + ENDSTOP_SAMPLE_INTERVAL;
}

/* ========== 公共接口实现 ========== */

/**
 * @brief  初始化限位开关模块
 * @retval 0 成功，负数 失败
 */
int endstop_init(void)
{
    int i;
    
    /* 清零所有状态 */
    for (i = 0; i < ENDSTOP_COUNT; i++) {
        s_endstops[i].pin = 0;
        s_endstops[i].invert = 0;
        s_endstops[i].configured = 0;
        s_endstops[i].triggered = 0;
        s_endstops[i].homing = 0;
        s_endstops[i].stepper_id = STEPPER_X;
        s_endstops[i].callback = NULL;
        s_endstops[i].callback_arg = NULL;
    }
    
    /* 初始化采样定时器 */
    s_endstop_timer.func = endstop_timer_callback;
    s_endstop_timer.waketime = sched_get_time() + ENDSTOP_SAMPLE_INTERVAL;
    s_endstop_timer.next = NULL;
    
    /* 启动采样定时器 */
    sched_add_timer(&s_endstop_timer);
    
    return 0;
}

/**
 * @brief  配置限位开关
 * @param  id 限位开关 ID
 * @param  config 配置参数
 * @retval 0 成功，负数 失败
 */
int endstop_config(endstop_id_t id, const endstop_config_t* config)
{
    endstop_state_t* endstop;
    
    if (id >= ENDSTOP_COUNT || config == NULL) {
        return -1;
    }
    
    endstop = &s_endstops[id];
    endstop->pin = config->pin;
    endstop->invert = config->invert;
    endstop->stepper_id = config->stepper_id;
    endstop->configured = 1;
    
    /* 配置 GPIO 为输入，带上拉 */
    gpio_in_setup(config->pin, 1);  /* 1 = 上拉 */
    
    return 0;
}

/**
 * @brief  读取限位开关状态
 * @param  id 限位开关 ID
 * @retval 1 触发，0 未触发，负数 错误
 */
int endstop_get_state(endstop_id_t id)
{
    if (id >= ENDSTOP_COUNT) {
        return -1;
    }
    
    if (!s_endstops[id].configured) {
        return -2;
    }
    
    return endstop_read_state(&s_endstops[id]);
}

/**
 * @brief  检查限位开关是否已触发
 * @param  id 限位开关 ID
 * @retval 1 已触发，0 未触发
 */
int endstop_is_triggered(endstop_id_t id)
{
    if (id >= ENDSTOP_COUNT) {
        return 0;
    }
    
    return s_endstops[id].triggered;
}

/**
 * @brief  启用归零模式
 * @param  id 限位开关 ID
 * @note   归零模式下，触发时自动停止关联的步进电机
 */
void endstop_home_start(endstop_id_t id)
{
    if (id >= ENDSTOP_COUNT) {
        return;
    }
    
    s_endstops[id].homing = 1;
    s_endstops[id].triggered = 0;
}

/**
 * @brief  结束归零模式
 * @param  id 限位开关 ID
 */
void endstop_home_end(endstop_id_t id)
{
    if (id >= ENDSTOP_COUNT) {
        return;
    }
    
    s_endstops[id].homing = 0;
}

/**
 * @brief  设置触发回调函数
 * @param  id 限位开关 ID
 * @param  callback 回调函数
 * @param  arg 回调参数
 */
void endstop_set_callback(endstop_id_t id, endstop_callback_fn_t callback, void* arg)
{
    if (id >= ENDSTOP_COUNT) {
        return;
    }
    
    s_endstops[id].callback = callback;
    s_endstops[id].callback_arg = arg;
}
