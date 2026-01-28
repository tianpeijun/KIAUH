/**
 * @file    fan.h
 * @brief   风扇控制接口
 * 
 * 移植自 klippy/extras/fan.py
 * 提供风扇 PWM 控制功能
 * 
 * 验收标准: 3.4.1 - 移植 klippy/extras/fan.py 逻辑
 *          3.4.2 - 支持 M106/M107 指令
 */

#ifndef FAN_H
#define FAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 风扇定义 ========== */

/**
 * @brief   风扇 ID 枚举
 */
typedef enum {
    FAN_PART = 0,               /* 模型风扇 (零件冷却风扇) */
    FAN_HOTEND,                 /* 热端风扇 (散热风扇) */
    FAN_COUNT
} fan_id_t;

/* ========== 风扇速度常量 ========== */

/* 风扇速度范围 */
#define FAN_SPEED_MIN           0.0f        /* 最小速度 (关闭) */
#define FAN_SPEED_MAX           1.0f        /* 最大速度 (全速) */

/* ========== 公共函数 ========== */

/**
 * @brief   初始化风扇模块
 * 
 * 初始化 PWM 通道用于风扇控制。
 * 必须在使用其他风扇函数之前调用。
 */
void fan_init(void);

/**
 * @brief   设置风扇速度
 * @param   id      风扇 ID
 * @param   speed   速度 (0.0 - 1.0)
 * 
 * 设置风扇的 PWM 占空比。
 * - 0.0 = 关闭
 * - 1.0 = 全速
 * 
 * 对应 M106 S<value> 指令，其中 S 参数 (0-255) 
 * 需要转换为 0.0-1.0 范围。
 */
void fan_set_speed(fan_id_t id, float speed);

/**
 * @brief   获取风扇速度
 * @param   id      风扇 ID
 * @return  当前速度 (0.0 - 1.0)，错误时返回 -1.0
 * 
 * 返回当前设置的风扇速度。
 */
float fan_get_speed(fan_id_t id);

#ifdef TEST_BUILD
/**
 * @brief   重置风扇模块状态 (仅用于测试)
 */
void fan_reset_for_test(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* FAN_H */
