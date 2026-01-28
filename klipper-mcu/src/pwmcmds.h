/**
 * @file    pwmcmds.h
 * @brief   PWM 命令处理接口
 * 
 * 复用自 Klipper src/pwmcmds.c
 * 提供 PWM 输出控制功能（用于加热器和风扇）
 */

#ifndef PWMCMDS_H
#define PWMCMDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== PWM 通道 ID ========== */

typedef enum {
    PWM_CHANNEL_HEATER_HOTEND = 0,  /* 热端加热器 */
    PWM_CHANNEL_HEATER_BED,         /* 热床加热器 */
    PWM_CHANNEL_FAN_PART,           /* 模型风扇 */
    PWM_CHANNEL_FAN_HOTEND,         /* 热端风扇 */
    PWM_CHANNEL_COUNT
} pwm_channel_t;

/* ========== PWM 配置 ========== */

typedef struct {
    uint8_t pin;                /* GPIO 引脚 */
    uint32_t cycle_time;        /* PWM 周期（时钟周期） */
    uint16_t max_value;         /* 最大值（分辨率） */
    uint8_t invert;             /* 输出反相 */
    uint8_t use_hardware;       /* 使用硬件 PWM */
} pwm_config_t;

/* ========== PWM 接口 ========== */

/**
 * @brief  初始化 PWM 模块
 * @retval 0 成功，负数 失败
 */
int pwm_init(void);

/**
 * @brief  配置 PWM 通道
 * @param  id PWM 通道 ID
 * @param  config 配置参数
 * @retval 0 成功，负数 失败
 */
int pwm_config(pwm_channel_t id, const pwm_config_t* config);

/**
 * @brief  启用 PWM 通道
 * @param  id PWM 通道 ID
 * @param  enable 1 启用，0 禁用
 */
void pwm_enable(pwm_channel_t id, int enable);

/**
 * @brief  设置 PWM 占空比
 * @param  id PWM 通道 ID
 * @param  value 占空比值（0-255）
 */
void pwm_set_value(pwm_channel_t id, uint16_t value);

/**
 * @brief  设置 PWM 占空比（浮点数）
 * @param  id PWM 通道 ID
 * @param  duty 占空比（0.0 - 1.0）
 */
void pwm_set_duty(pwm_channel_t id, float duty);

/**
 * @brief  获取 PWM 当前值
 * @param  id PWM 通道 ID
 * @retval 当前占空比值，负数表示错误
 */
int32_t pwm_get_value(pwm_channel_t id);

/**
 * @brief  获取 PWM 占空比（浮点数）
 * @param  id PWM 通道 ID
 * @retval 占空比（0.0 - 1.0），负数表示错误
 */
float pwm_get_duty(pwm_channel_t id);

#ifdef __cplusplus
}
#endif

#endif /* PWMCMDS_H */
