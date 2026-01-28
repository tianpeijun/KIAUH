/**
 * @file    stepper.h
 * @brief   步进电机驱动接口
 * 
 * 复用自 Klipper src/stepper.h
 * 提供步进脉冲生成和电机控制功能
 */

#ifndef STEPPER_H
#define STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "sched.h"

/* ========== 步进电机 ID ========== */

typedef enum {
    STEPPER_X = 0,
    STEPPER_Y,
    STEPPER_Z,
    STEPPER_E,
    STEPPER_COUNT
} stepper_id_t;

/* ========== 步进方向 ========== */

typedef enum {
    STEPPER_DIR_FORWARD = 0,    /* 正向 */
    STEPPER_DIR_BACKWARD = 1    /* 反向 */
} stepper_dir_t;

/* ========== 步进电机配置 ========== */

typedef struct {
    uint8_t step_pin;           /* 步进脉冲引脚 */
    uint8_t dir_pin;            /* 方向引脚 */
    uint8_t enable_pin;         /* 使能引脚 */
    uint8_t invert_step;        /* 步进脉冲反相 */
    uint8_t invert_dir;         /* 方向反相 */
    uint8_t invert_enable;      /* 使能反相 */
} stepper_config_t;

/* ========== 步进运动参数 ========== */

typedef struct {
    int32_t position;           /* 当前位置（步数） */
    int32_t target;             /* 目标位置（步数） */
    uint32_t interval;          /* 步进间隔（时钟周期） */
    uint32_t count;             /* 剩余步数 */
    int8_t dir;                 /* 当前方向 */
} stepper_move_t;

/* ========== 步进电机接口 ========== */

/**
 * @brief  初始化步进电机模块
 * @retval 0 成功，负数 失败
 */
int stepper_init(void);

/**
 * @brief  配置单个步进电机
 * @param  id 电机 ID
 * @param  config 配置参数
 * @retval 0 成功，负数 失败
 */
int stepper_config(stepper_id_t id, const stepper_config_t* config);

/**
 * @brief  使能步进电机
 * @param  id 电机 ID
 * @param  enable 1 使能，0 禁用
 */
void stepper_enable(stepper_id_t id, int enable);

/**
 * @brief  设置步进方向
 * @param  id 电机 ID
 * @param  dir 方向
 */
void stepper_set_dir(stepper_id_t id, stepper_dir_t dir);

/**
 * @brief  产生一个步进脉冲
 * @param  id 电机 ID
 */
void stepper_step(stepper_id_t id);

/**
 * @brief  获取当前位置（步数）
 * @param  id 电机 ID
 * @retval 当前位置
 */
int32_t stepper_get_position(stepper_id_t id);

/**
 * @brief  设置当前位置（步数）
 * @param  id 电机 ID
 * @param  position 位置值
 */
void stepper_set_position(stepper_id_t id, int32_t position);

/**
 * @brief  添加步进运动
 * @param  id 电机 ID
 * @param  move 运动参数
 * @retval 0 成功，负数 失败
 */
int stepper_add_move(stepper_id_t id, const stepper_move_t* move);

/**
 * @brief  停止步进电机
 * @param  id 电机 ID
 */
void stepper_stop(stepper_id_t id);

/**
 * @brief  停止所有步进电机
 */
void stepper_stop_all(void);

/**
 * @brief  检查步进电机是否在运动
 * @param  id 电机 ID
 * @retval 1 运动中，0 停止
 */
int stepper_is_moving(stepper_id_t id);

/**
 * @brief  步进电机定时器回调（内部使用）
 * @param  waketime 唤醒时间
 * @retval 下次唤醒时间
 */
sched_time_t stepper_timer_callback(sched_time_t waketime);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_H */
