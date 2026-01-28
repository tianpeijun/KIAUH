/**
 * @file    pwmcmds.c
 * @brief   PWM 命令处理实现
 * 
 * 复用自 Klipper src/pwmcmds.c
 * 提供 PWM 输出控制功能（用于加热器和风扇）
 */

#include "pwmcmds.h"
#include "sched.h"
#include <stddef.h>

/* ========== 私有类型定义 ========== */

/* PWM 通道状态 */
typedef struct {
    uint8_t pin;                /* GPIO 引脚 */
    uint8_t configured;         /* 是否已配置 */
    uint8_t enabled;            /* 是否启用 */
    uint16_t value;             /* 当前占空比值 */
    uint16_t max_value;         /* 最大值（分辨率） */
    uint32_t cycle_time;        /* PWM 周期（时钟周期） */
    uint8_t invert;             /* 输出反相 */
} pwm_state_t;

/* ========== 私有变量 ========== */

/* PWM 通道状态数组 */
static pwm_state_t s_pwm_channels[PWM_CHANNEL_COUNT];

/* 软件 PWM 定时器 */
static sched_timer_t s_pwm_timer;

/* 软件 PWM 状态 */
static uint8_t s_soft_pwm_enabled = 0;

/* 默认 PWM 周期（约 1ms，1kHz） */
#define PWM_DEFAULT_CYCLE_TIME  1000

/* PWM 分辨率（8 位） */
#define PWM_RESOLUTION          8
#define PWM_MAX_VALUE           ((1 << PWM_RESOLUTION) - 1)

/* ========== HAL 接口 ========== */

/* TODO: 需要 HAL 层实现这些函数 */
extern void gpio_out_setup(uint8_t pin, uint8_t value);
extern void gpio_out_write(uint8_t pin, uint8_t value);
extern void pwm_setup(uint8_t pin, uint32_t cycle_time, uint8_t value);
extern void pwm_write(uint8_t pin, uint8_t value);

/* ========== 私有函数 ========== */

/**
 * @brief  软件 PWM 定时器回调
 * @param  waketime 唤醒时间
 * @retval 下次唤醒时间
 * @note   用于不支持硬件 PWM 的引脚
 */
static sched_time_t pwm_timer_callback(sched_time_t waketime)
{
    int i;
    static uint8_t s_pwm_counter = 0;
    
    /* 更新 PWM 计数器 (自动溢出回绕) */
    s_pwm_counter++;
    
    /* 遍历所有 PWM 通道 */
    for (i = 0; i < PWM_CHANNEL_COUNT; i++) {
        pwm_state_t* pwm = &s_pwm_channels[i];
        uint8_t output;
        
        if (!pwm->configured || !pwm->enabled) {
            continue;
        }
        
        /* 计算输出电平 */
        output = (s_pwm_counter < pwm->value) ? 1 : 0;
        
        if (pwm->invert) {
            output = !output;
        }
        
        gpio_out_write(pwm->pin, output);
    }
    
    /* 返回下次更新时间 */
    return waketime + (PWM_DEFAULT_CYCLE_TIME / PWM_MAX_VALUE);
}

/* ========== 公共接口实现 ========== */

/**
 * @brief  初始化 PWM 模块
 * @retval 0 成功，负数 失败
 */
int pwm_init(void)
{
    int i;
    
    /* 清零所有状态 */
    for (i = 0; i < PWM_CHANNEL_COUNT; i++) {
        s_pwm_channels[i].pin = 0;
        s_pwm_channels[i].configured = 0;
        s_pwm_channels[i].enabled = 0;
        s_pwm_channels[i].value = 0;
        s_pwm_channels[i].max_value = PWM_MAX_VALUE;
        s_pwm_channels[i].cycle_time = PWM_DEFAULT_CYCLE_TIME;
        s_pwm_channels[i].invert = 0;
    }
    
    /* 初始化软件 PWM 定时器 */
    s_pwm_timer.func = pwm_timer_callback;
    s_pwm_timer.waketime = 0;
    s_pwm_timer.next = NULL;
    s_soft_pwm_enabled = 0;
    
    return 0;
}

/**
 * @brief  配置 PWM 通道
 * @param  id PWM 通道 ID
 * @param  config 配置参数
 * @retval 0 成功，负数 失败
 */
int pwm_config(pwm_channel_t id, const pwm_config_t* config)
{
    pwm_state_t* pwm;
    
    if (id >= PWM_CHANNEL_COUNT || config == NULL) {
        return -1;
    }
    
    pwm = &s_pwm_channels[id];
    pwm->pin = config->pin;
    pwm->cycle_time = config->cycle_time;
    pwm->max_value = config->max_value;
    pwm->invert = config->invert;
    pwm->configured = 1;
    
    /* 配置 GPIO 为输出 */
    gpio_out_setup(config->pin, config->invert ? 1 : 0);
    
    /* 如果支持硬件 PWM，配置硬件 */
    if (config->use_hardware) {
        pwm_setup(config->pin, config->cycle_time, 0);
    }
    
    return 0;
}

/**
 * @brief  启用 PWM 通道
 * @param  id PWM 通道 ID
 * @param  enable 1 启用，0 禁用
 */
void pwm_enable(pwm_channel_t id, int enable)
{
    pwm_state_t* pwm;
    
    if (id >= PWM_CHANNEL_COUNT) {
        return;
    }
    
    pwm = &s_pwm_channels[id];
    pwm->enabled = enable ? 1 : 0;
    
    /* 如果禁用，输出低电平 */
    if (!enable) {
        gpio_out_write(pwm->pin, pwm->invert ? 1 : 0);
    }
    
    /* 检查是否需要启动软件 PWM */
    if (enable && !s_soft_pwm_enabled) {
        s_soft_pwm_enabled = 1;
        s_pwm_timer.waketime = sched_get_time() + 1;
        sched_add_timer(&s_pwm_timer);
    }
}

/**
 * @brief  设置 PWM 占空比
 * @param  id PWM 通道 ID
 * @param  value 占空比值（0-255）
 */
void pwm_set_value(pwm_channel_t id, uint16_t value)
{
    pwm_state_t* pwm;
    
    if (id >= PWM_CHANNEL_COUNT) {
        return;
    }
    
    pwm = &s_pwm_channels[id];
    
    /* 限幅 */
    if (value > pwm->max_value) {
        value = pwm->max_value;
    }
    
    pwm->value = value;
    
    /* 如果使用硬件 PWM，直接写入 */
    /* pwm_write(pwm->pin, value); */
}

/**
 * @brief  设置 PWM 占空比（浮点数）
 * @param  id PWM 通道 ID
 * @param  duty 占空比（0.0 - 1.0）
 */
void pwm_set_duty(pwm_channel_t id, float duty)
{
    pwm_state_t* pwm;
    uint16_t value;
    
    if (id >= PWM_CHANNEL_COUNT) {
        return;
    }
    
    pwm = &s_pwm_channels[id];
    
    /* 限幅 */
    if (duty < 0.0f) {
        duty = 0.0f;
    } else if (duty > 1.0f) {
        duty = 1.0f;
    }
    
    /* 转换为整数值 */
    value = (uint16_t)(duty * pwm->max_value);
    
    pwm_set_value(id, value);
}

/**
 * @brief  获取 PWM 当前值
 * @param  id PWM 通道 ID
 * @retval 当前占空比值，负数表示错误
 */
int32_t pwm_get_value(pwm_channel_t id)
{
    if (id >= PWM_CHANNEL_COUNT) {
        return -1;
    }
    
    if (!s_pwm_channels[id].configured) {
        return -2;
    }
    
    return s_pwm_channels[id].value;
}

/**
 * @brief  获取 PWM 占空比（浮点数）
 * @param  id PWM 通道 ID
 * @retval 占空比（0.0 - 1.0），负数表示错误
 */
float pwm_get_duty(pwm_channel_t id)
{
    pwm_state_t* pwm;
    
    if (id >= PWM_CHANNEL_COUNT) {
        return -1.0f;
    }
    
    pwm = &s_pwm_channels[id];
    if (!pwm->configured) {
        return -1.0f;
    }
    
    return (float)pwm->value / (float)pwm->max_value;
}
