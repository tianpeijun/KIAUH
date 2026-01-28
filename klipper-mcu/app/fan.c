/**
 * @file    fan.c
 * @brief   风扇控制实现
 * 
 * 移植自 klippy/extras/fan.py
 * 实现风扇 PWM 控制功能
 * 
 * 验收标准: 3.4.1 - 移植 klippy/extras/fan.py 逻辑
 *          3.4.2 - 支持 M106/M107 指令
 */

#include "fan.h"
#include "config.h"
#include "src/pwmcmds.h"
#include "board/gpio.h"

/* ========== PWM 配置参数 ========== */

/* PWM 周期 (约 25kHz，适合大多数风扇) */
#define FAN_PWM_CYCLE_TIME      40

/* PWM 分辨率 (8 位) */
#define FAN_PWM_MAX_VALUE       255

/* ========== 私有变量 ========== */

/* 风扇状态 */
typedef struct {
    float speed;                /* 当前速度 (0.0 - 1.0) */
    uint8_t initialized;        /* 初始化标志 */
    uint8_t pwm_enabled;        /* PWM 输出已启用 */
} fan_state_t;

/* 风扇到 PWM 通道的映射 */
static const pwm_channel_t s_fan_pwm_channel[FAN_COUNT] = {
    PWM_CHANNEL_FAN_PART,       /* FAN_PART */
    PWM_CHANNEL_FAN_HOTEND      /* FAN_HOTEND */
};

/* 风扇 GPIO 引脚配置 */
static const uint8_t s_fan_pins[FAN_COUNT] = {
    FAN_PART_PIN,               /* FAN_PART */
    FAN_HOTEND_PIN              /* FAN_HOTEND */
};

/* 风扇状态数组 */
static fan_state_t s_fan_state[FAN_COUNT];

/* 模块初始化标志 */
static uint8_t s_module_initialized = 0;

/* ========== 私有函数 ========== */

/**
 * @brief   设置风扇 PWM 输出
 * @param   id      风扇 ID
 * @param   duty    占空比 (0.0 - 1.0)
 */
static void
fan_set_pwm(fan_id_t id, float duty)
{
    pwm_channel_t pwm_ch;
    
    if (id >= FAN_COUNT) {
        return;
    }
    
    pwm_ch = s_fan_pwm_channel[id];
    
    /* 限制占空比范围 */
    if (duty < FAN_SPEED_MIN) {
        duty = FAN_SPEED_MIN;
    } else if (duty > FAN_SPEED_MAX) {
        duty = FAN_SPEED_MAX;
    }
    
    /* 设置 PWM 占空比 */
    pwm_set_duty(pwm_ch, duty);
}

/* ========== 公共函数 ========== */

/**
 * @brief   初始化风扇模块
 */
void
fan_init(void)
{
    pwm_config_t pwm_cfg;
    uint8_t i;
    
    if (s_module_initialized) {
        return;
    }
    
    /* 初始化 PWM 子系统 (如果尚未初始化) */
    pwm_init();
    
    /* 配置每个风扇的 PWM 输出 */
    for (i = 0; i < FAN_COUNT; i++) {
        /* 配置 PWM 通道 */
        pwm_cfg.pin = s_fan_pins[i];
        pwm_cfg.cycle_time = FAN_PWM_CYCLE_TIME;
        pwm_cfg.max_value = FAN_PWM_MAX_VALUE;
        pwm_cfg.invert = 0;             /* 不反相 */
        pwm_cfg.use_hardware = 0;       /* 使用软件 PWM */
        pwm_config(s_fan_pwm_channel[i], &pwm_cfg);
        
        /* 初始化风扇状态 */
        s_fan_state[i].speed = 0.0f;
        s_fan_state[i].initialized = 1;
        s_fan_state[i].pwm_enabled = 0;
    }
    
    s_module_initialized = 1;
}

/**
 * @brief   设置风扇速度
 * 
 * 验收标准: 3.4.2 - 支持 M106/M107 指令
 */
void
fan_set_speed(fan_id_t id, float speed)
{
    if (id >= FAN_COUNT) {
        return;
    }
    
    /* 确保模块已初始化 */
    if (!s_module_initialized) {
        fan_init();
    }
    
    /* 限制速度范围 */
    if (speed < FAN_SPEED_MIN) {
        speed = FAN_SPEED_MIN;
    } else if (speed > FAN_SPEED_MAX) {
        speed = FAN_SPEED_MAX;
    }
    
    /* 保存速度设置 */
    s_fan_state[id].speed = speed;
    
    /* 如果速度为 0，关闭 PWM */
    if (speed <= 0.0f) {
        fan_set_pwm(id, 0.0f);
        pwm_enable(s_fan_pwm_channel[id], 0);
        s_fan_state[id].pwm_enabled = 0;
    } else {
        /* 启用 PWM 输出 */
        if (!s_fan_state[id].pwm_enabled) {
            pwm_enable(s_fan_pwm_channel[id], 1);
            s_fan_state[id].pwm_enabled = 1;
        }
        
        /* 设置 PWM 占空比 */
        fan_set_pwm(id, speed);
    }
}

/**
 * @brief   获取风扇速度
 */
float
fan_get_speed(fan_id_t id)
{
    if (id >= FAN_COUNT) {
        return -1.0f;
    }
    
    /* 确保模块已初始化 */
    if (!s_module_initialized) {
        fan_init();
    }
    
    return s_fan_state[id].speed;
}

#ifdef TEST_BUILD
/**
 * @brief   重置风扇模块状态 (仅用于测试)
 * 
 * 此函数仅在测试构建中可用，用于在测试之间重置模块状态。
 */
void
fan_reset_for_test(void)
{
    uint8_t i;
    
    s_module_initialized = 0;
    
    for (i = 0; i < FAN_COUNT; i++) {
        s_fan_state[i].speed = 0.0f;
        s_fan_state[i].initialized = 0;
        s_fan_state[i].pwm_enabled = 0;
    }
}
#endif /* TEST_BUILD */
