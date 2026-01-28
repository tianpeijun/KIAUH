/**
 * @file    config.h
 * @brief   打印机配置参数
 * 
 * Klipper MCU 移植项目 - 编译时配置文件
 * 定义引脚映射和运动/温度控制参数
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ========== 步进电机引脚 ========== */
#define STEPPER_X_STEP_PIN      GPIO_PA0
#define STEPPER_X_DIR_PIN       GPIO_PA1
#define STEPPER_X_ENABLE_PIN    GPIO_PA2

#define STEPPER_Y_STEP_PIN      GPIO_PA3
#define STEPPER_Y_DIR_PIN       GPIO_PA4
#define STEPPER_Y_ENABLE_PIN    GPIO_PA5

#define STEPPER_Z_STEP_PIN      GPIO_PA6
#define STEPPER_Z_DIR_PIN       GPIO_PA7
#define STEPPER_Z_ENABLE_PIN    GPIO_PB0

#define STEPPER_E_STEP_PIN      GPIO_PB1
#define STEPPER_E_DIR_PIN       GPIO_PB2
#define STEPPER_E_ENABLE_PIN    GPIO_PB3

/* ========== 限位开关引脚 ========== */
#define ENDSTOP_X_PIN           GPIO_PC0
#define ENDSTOP_Y_PIN           GPIO_PC1
#define ENDSTOP_Z_PIN           GPIO_PC2

/* ========== 温度传感器 ========== */
#define TEMP_HOTEND_ADC_CH      0
#define TEMP_BED_ADC_CH         1

/* ========== 加热器 PWM ========== */
#define HEATER_HOTEND_PIN       GPIO_PB4
#define HEATER_BED_PIN          GPIO_PB5

/* ========== 风扇 PWM ========== */
#define FAN_PART_PIN            GPIO_PB6
#define FAN_HOTEND_PIN          GPIO_PB7

/* ========== 运动参数 ========== */
#define STEPS_PER_MM_X          80.0f
#define STEPS_PER_MM_Y          80.0f
#define STEPS_PER_MM_Z          400.0f
#define STEPS_PER_MM_E          93.0f

#define MAX_VELOCITY            200.0f      /* mm/s */
#define MAX_ACCEL               3000.0f     /* mm/s² */

#define X_MIN                   0.0f
#define X_MAX                   220.0f
#define Y_MIN                   0.0f
#define Y_MAX                   220.0f
#define Z_MIN                   0.0f
#define Z_MAX                   250.0f

/* ========== PID 参数 ========== */
#define HOTEND_PID_KP           22.2f
#define HOTEND_PID_KI           1.08f
#define HOTEND_PID_KD           114.0f

#define BED_PID_KP              54.0f
#define BED_PID_KI              0.5f
#define BED_PID_KD              200.0f

/* ========== 串口配置 ========== */
#define SERIAL_BAUD             115200

#endif /* CONFIG_H */
