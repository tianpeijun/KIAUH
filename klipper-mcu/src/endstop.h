/**
 * @file    endstop.h
 * @brief   限位开关检测接口
 * 
 * 复用自 Klipper src/endstop.c
 * 提供限位开关状态检测和归零停止功能
 */

#ifndef ENDSTOP_H
#define ENDSTOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stepper.h"

/* ========== 限位开关 ID ========== */

typedef enum {
    ENDSTOP_X = 0,
    ENDSTOP_Y,
    ENDSTOP_Z,
    ENDSTOP_COUNT
} endstop_id_t;

/* ========== 限位开关配置 ========== */

typedef struct {
    uint8_t pin;                /* GPIO 引脚 */
    uint8_t invert;             /* 电平反相 */
    stepper_id_t stepper_id;    /* 关联的步进电机 */
} endstop_config_t;

/* ========== 回调函数类型 ========== */

/**
 * @brief  限位开关触发回调函数类型
 * @param  id 限位开关 ID
 * @param  arg 用户参数
 */
typedef void (*endstop_callback_fn_t)(endstop_id_t id, void* arg);

/* ========== 限位开关接口 ========== */

/**
 * @brief  初始化限位开关模块
 * @retval 0 成功，负数 失败
 */
int endstop_init(void);

/**
 * @brief  配置限位开关
 * @param  id 限位开关 ID
 * @param  config 配置参数
 * @retval 0 成功，负数 失败
 */
int endstop_config(endstop_id_t id, const endstop_config_t* config);

/**
 * @brief  读取限位开关状态
 * @param  id 限位开关 ID
 * @retval 1 触发，0 未触发，负数 错误
 */
int endstop_get_state(endstop_id_t id);

/**
 * @brief  检查限位开关是否已触发
 * @param  id 限位开关 ID
 * @retval 1 已触发，0 未触发
 */
int endstop_is_triggered(endstop_id_t id);

/**
 * @brief  启用归零模式
 * @param  id 限位开关 ID
 * @note   归零模式下，触发时自动停止关联的步进电机
 */
void endstop_home_start(endstop_id_t id);

/**
 * @brief  结束归零模式
 * @param  id 限位开关 ID
 */
void endstop_home_end(endstop_id_t id);

/**
 * @brief  设置触发回调函数
 * @param  id 限位开关 ID
 * @param  callback 回调函数
 * @param  arg 回调参数
 */
void endstop_set_callback(endstop_id_t id, endstop_callback_fn_t callback, void* arg);

#ifdef __cplusplus
}
#endif

#endif /* ENDSTOP_H */
