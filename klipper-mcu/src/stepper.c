/**
 * @file    stepper.c
 * @brief   步进电机驱动实现
 * 
 * 复用自 Klipper src/stepper.c
 * 提供步进脉冲生成和电机控制功能
 */

#include "stepper.h"
#include "sched.h"
#include <stddef.h>

/* ========== 私有类型定义 ========== */

/* 步进电机状态 */
typedef struct {
    stepper_config_t config;    /* 配置参数 */
    int32_t position;           /* 当前位置（步数） */
    int8_t dir;                 /* 当前方向 */
    uint8_t enabled;            /* 使能状态 */
    uint8_t configured;         /* 是否已配置 */
    
    /* 运动队列 */
    uint32_t interval;          /* 步进间隔 */
    uint32_t count;             /* 剩余步数 */
    sched_time_t next_step_time; /* 下次步进时间 */
} stepper_state_t;

/* ========== 私有变量 ========== */

/* 步进电机状态数组 */
static stepper_state_t s_steppers[STEPPER_COUNT];

/* 步进定时器 */
static sched_timer_t s_stepper_timer;

/* 最小步进间隔（防止过快） */
#define MIN_STEP_INTERVAL   100     /* 时钟周期 */

/* 步进脉冲宽度 */
#define STEP_PULSE_WIDTH    2       /* 微秒 */

/* ========== GPIO 操作宏 ========== */

/* TODO: 需要 HAL 层实现这些函数 */
extern void gpio_out_write(uint8_t pin, uint8_t value);
extern uint8_t gpio_in_read(uint8_t pin);

/* ========== 私有函数 ========== */

/**
 * @brief  输出步进脉冲
 * @param  stepper 步进电机状态指针
 */
static void stepper_do_step(stepper_state_t* stepper)
{
    uint8_t step_value;
    
    if (stepper == NULL || !stepper->configured || !stepper->enabled) {
        return;
    }
    
    /* 计算脉冲电平 */
    step_value = stepper->config.invert_step ? 0 : 1;
    
    /* 输出高电平 */
    gpio_out_write(stepper->config.step_pin, step_value);
    
    /* 简单延时（实际应使用定时器） */
    /* TODO: 使用精确延时 */
    volatile int i;
    for (i = 0; i < 10; i++) {
        /* 空循环延时 */
    }
    
    /* 输出低电平 */
    gpio_out_write(stepper->config.step_pin, !step_value);
    
    /* 更新位置 */
    if (stepper->dir == STEPPER_DIR_FORWARD) {
        stepper->position++;
    } else {
        stepper->position--;
    }
}

/* ========== 公共接口实现 ========== */

/**
 * @brief  初始化步进电机模块
 * @retval 0 成功，负数 失败
 */
int stepper_init(void)
{
    int i;
    
    /* 清零所有状态 */
    for (i = 0; i < STEPPER_COUNT; i++) {
        s_steppers[i].position = 0;
        s_steppers[i].dir = STEPPER_DIR_FORWARD;
        s_steppers[i].enabled = 0;
        s_steppers[i].configured = 0;
        s_steppers[i].interval = 0;
        s_steppers[i].count = 0;
        s_steppers[i].next_step_time = 0;
    }
    
    /* 初始化定时器 */
    s_stepper_timer.func = stepper_timer_callback;
    s_stepper_timer.waketime = 0;
    s_stepper_timer.next = NULL;
    
    return 0;
}

/**
 * @brief  配置单个步进电机
 * @param  id 电机 ID
 * @param  config 配置参数
 * @retval 0 成功，负数 失败
 */
int stepper_config(stepper_id_t id, const stepper_config_t* config)
{
    stepper_state_t* stepper;
    
    if (id >= STEPPER_COUNT || config == NULL) {
        return -1;
    }
    
    stepper = &s_steppers[id];
    stepper->config = *config;
    stepper->configured = 1;
    
    /* 配置 GPIO 引脚 */
    /* TODO: 需要 HAL 层实现 gpio_out_setup() */
    extern void gpio_out_setup(uint8_t pin, uint8_t value);
    
    /* 步进引脚初始化为低电平 */
    gpio_out_setup(config->step_pin, config->invert_step ? 1 : 0);
    
    /* 方向引脚初始化 */
    gpio_out_setup(config->dir_pin, config->invert_dir ? 1 : 0);
    
    /* 使能引脚初始化为禁用状态 */
    gpio_out_setup(config->enable_pin, config->invert_enable ? 0 : 1);
    
    return 0;
}

/**
 * @brief  使能步进电机
 * @param  id 电机 ID
 * @param  enable 1 使能，0 禁用
 */
void stepper_enable(stepper_id_t id, int enable)
{
    stepper_state_t* stepper;
    uint8_t pin_value;
    
    if (id >= STEPPER_COUNT) {
        return;
    }
    
    stepper = &s_steppers[id];
    if (!stepper->configured) {
        return;
    }
    
    stepper->enabled = enable ? 1 : 0;
    
    /* 计算使能引脚电平 */
    pin_value = enable ? 0 : 1;
    if (stepper->config.invert_enable) {
        pin_value = !pin_value;
    }
    
    gpio_out_write(stepper->config.enable_pin, pin_value);
}

/**
 * @brief  设置步进方向
 * @param  id 电机 ID
 * @param  dir 方向
 */
void stepper_set_dir(stepper_id_t id, stepper_dir_t dir)
{
    stepper_state_t* stepper;
    uint8_t pin_value;
    
    if (id >= STEPPER_COUNT) {
        return;
    }
    
    stepper = &s_steppers[id];
    if (!stepper->configured) {
        return;
    }
    
    stepper->dir = dir;
    
    /* 计算方向引脚电平 */
    pin_value = (dir == STEPPER_DIR_FORWARD) ? 0 : 1;
    if (stepper->config.invert_dir) {
        pin_value = !pin_value;
    }
    
    gpio_out_write(stepper->config.dir_pin, pin_value);
}

/**
 * @brief  产生一个步进脉冲
 * @param  id 电机 ID
 */
void stepper_step(stepper_id_t id)
{
    if (id >= STEPPER_COUNT) {
        return;
    }
    
    stepper_do_step(&s_steppers[id]);
}

/**
 * @brief  获取当前位置（步数）
 * @param  id 电机 ID
 * @retval 当前位置
 */
int32_t stepper_get_position(stepper_id_t id)
{
    if (id >= STEPPER_COUNT) {
        return 0;
    }
    
    return s_steppers[id].position;
}

/**
 * @brief  设置当前位置（步数）
 * @param  id 电机 ID
 * @param  position 位置值
 */
void stepper_set_position(stepper_id_t id, int32_t position)
{
    if (id >= STEPPER_COUNT) {
        return;
    }
    
    s_steppers[id].position = position;
}

/**
 * @brief  添加步进运动
 * @param  id 电机 ID
 * @param  move 运动参数
 * @retval 0 成功，负数 失败
 */
int stepper_add_move(stepper_id_t id, const stepper_move_t* move)
{
    stepper_state_t* stepper;
    
    if (id >= STEPPER_COUNT || move == NULL) {
        return -1;
    }
    
    stepper = &s_steppers[id];
    if (!stepper->configured) {
        return -2;
    }
    
    /* 设置运动参数 */
    stepper->interval = move->interval;
    stepper->count = move->count;
    
    /* 设置方向 */
    if (move->dir >= 0) {
        stepper_set_dir(id, STEPPER_DIR_FORWARD);
    } else {
        stepper_set_dir(id, STEPPER_DIR_BACKWARD);
    }
    
    /* 设置下次步进时间 */
    stepper->next_step_time = sched_get_time() + stepper->interval;
    
    /* 添加定时器 */
    s_stepper_timer.waketime = stepper->next_step_time;
    sched_add_timer(&s_stepper_timer);
    
    return 0;
}

/**
 * @brief  停止步进电机
 * @param  id 电机 ID
 */
void stepper_stop(stepper_id_t id)
{
    if (id >= STEPPER_COUNT) {
        return;
    }
    
    s_steppers[id].count = 0;
    s_steppers[id].interval = 0;
}

/**
 * @brief  停止所有步进电机
 */
void stepper_stop_all(void)
{
    int i;
    
    for (i = 0; i < STEPPER_COUNT; i++) {
        stepper_stop((stepper_id_t)i);
    }
    
    /* 移除定时器 */
    sched_del_timer(&s_stepper_timer);
}

/**
 * @brief  检查步进电机是否在运动
 * @param  id 电机 ID
 * @retval 1 运动中，0 停止
 */
int stepper_is_moving(stepper_id_t id)
{
    if (id >= STEPPER_COUNT) {
        return 0;
    }
    
    return s_steppers[id].count > 0 ? 1 : 0;
}

/**
 * @brief  步进电机定时器回调
 * @param  waketime 唤醒时间
 * @retval 下次唤醒时间，0 表示不再调度
 */
sched_time_t stepper_timer_callback(sched_time_t waketime)
{
    int i;
    sched_time_t next_time = 0;
    sched_time_t min_next_time = 0;
    int has_active = 0;
    
    (void)waketime;
    
    /* 遍历所有步进电机 */
    for (i = 0; i < STEPPER_COUNT; i++) {
        stepper_state_t* stepper = &s_steppers[i];
        
        if (stepper->count == 0) {
            continue;
        }
        
        /* 检查是否到达步进时间 */
        if (sched_is_before(stepper->next_step_time)) {
            /* 执行步进 */
            stepper_do_step(stepper);
            stepper->count--;
            
            if (stepper->count > 0) {
                stepper->next_step_time += stepper->interval;
            }
        }
        
        /* 计算下次唤醒时间 */
        if (stepper->count > 0) {
            next_time = stepper->next_step_time;
            if (!has_active || sched_time_diff(next_time, min_next_time) < 0) {
                min_next_time = next_time;
            }
            has_active = 1;
        }
    }
    
    return has_active ? min_next_time : 0;
}
