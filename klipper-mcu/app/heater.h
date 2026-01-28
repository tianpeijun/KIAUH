/**
 * @file    heater.h
 * @brief   温度 PID 控制器接口
 * 
 * 移植自 klippy/extras/heaters.py
 * 提供温度读取和 PID 控制功能
 */

#ifndef HEATER_H
#define HEATER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 加热器定义 ========== */

/**
 * @brief   加热器 ID 枚举
 */
typedef enum {
    HEATER_HOTEND = 0,          /* 热端 */
    HEATER_BED,                 /* 热床 */
    HEATER_COUNT
} heater_id_t;

/**
 * @brief   PID 参数结构体
 */
typedef struct {
    float kp;                   /* 比例系数 */
    float ki;                   /* 积分系数 */
    float kd;                   /* 微分系数 */
} pid_params_t;

/**
 * @brief   加热器配置结构体
 */
typedef struct {
    uint8_t adc_channel;        /* ADC 通道 */
    uint8_t pwm_pin;            /* PWM 引脚 */
    float max_power;            /* 最大功率 0-1 */
    pid_params_t pid;           /* PID 参数 */
} heater_config_t;

/* ========== 温度常量 ========== */

/* 温度范围限制 */
#define HEATER_TEMP_MIN         0.0f        /* 最低温度 °C */
#define HEATER_TEMP_MAX         300.0f      /* 最高温度 °C */
#define HEATER_TEMP_INVALID     -999.0f     /* 无效温度标记 */

/* 温度稳定判断阈值 */
#define HEATER_TEMP_TOLERANCE   3.0f        /* 温度稳定容差 ±3°C */

/* ========== 公共函数 ========== */

/**
 * @brief   初始化加热器模块
 * 
 * 初始化 ADC 通道用于温度传感器读取。
 * 必须在使用其他加热器函数之前调用。
 */
void heater_init(void);

/**
 * @brief   设置目标温度
 * @param   id      加热器 ID
 * @param   target  目标温度 (°C)
 * 
 * 设置加热器的目标温度，PID 控制器将调节输出以达到目标。
 */
void heater_set_temp(heater_id_t id, float target);

/**
 * @brief   获取当前温度
 * @param   id      加热器 ID
 * @return  当前温度 (°C)，错误时返回 HEATER_TEMP_INVALID
 * 
 * 读取 ADC 值并通过 NTC 热敏电阻查表转换为温度。
 * 精度要求: ±2°C
 */
float heater_get_temp(heater_id_t id);

/**
 * @brief   获取目标温度
 * @param   id      加热器 ID
 * @return  目标温度 (°C)
 */
float heater_get_target(heater_id_t id);

/**
 * @brief   检查是否达到目标温度
 * @param   id      加热器 ID
 * @return  1 = 已达到目标温度 (±3°C)，0 = 未达到
 */
int heater_is_at_target(heater_id_t id);

/**
 * @brief   温度控制周期任务
 * 
 * 执行 PID 计算和 PWM 输出更新。
 * 应在主循环中以 100ms 间隔调用。
 */
void heater_task(void);

/**
 * @brief   获取当前 PID 输出
 * @param   id      加热器 ID
 * @return  PID 输出 (0.0 - 1.0)，错误时返回 -1.0
 * 
 * 用于调试和监控 PID 控制器状态。
 */
float heater_get_output(heater_id_t id);

#ifdef TEST_BUILD
/**
 * @brief   重置加热器模块状态 (仅用于测试)
 */
void heater_reset_for_test(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* HEATER_H */
