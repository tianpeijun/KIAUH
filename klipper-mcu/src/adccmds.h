/**
 * @file    adccmds.h
 * @brief   ADC 命令处理接口
 * 
 * 复用自 Klipper src/adccmds.c
 * 提供 ADC 采样和温度读取功能
 */

#ifndef ADCCMDS_H
#define ADCCMDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== ADC 通道 ID ========== */

typedef enum {
    ADC_CHANNEL_HOTEND = 0,     /* 热端温度 */
    ADC_CHANNEL_BED,            /* 热床温度 */
    ADC_CHANNEL_COUNT
} adc_channel_t;

/* ========== ADC 配置 ========== */

typedef struct {
    uint8_t channel;            /* 硬件 ADC 通道号 */
    uint16_t min_value;         /* 最小有效值 */
    uint16_t max_value;         /* 最大有效值 */
} adc_config_t;

/* ========== 回调函数类型 ========== */

/**
 * @brief  ADC 采样完成回调函数类型
 * @param  id ADC 通道 ID
 * @param  value ADC 原始值
 * @param  arg 用户参数
 */
typedef void (*adc_callback_fn_t)(adc_channel_t id, uint16_t value, void* arg);

/* ========== ADC 接口 ========== */

/**
 * @brief  初始化 ADC 命令模块
 * @retval 0 成功，负数 失败
 */
int adccmds_init(void);

/**
 * @brief  配置 ADC 通道
 * @param  id ADC 通道 ID
 * @param  config 配置参数
 * @retval 0 成功，负数 失败
 */
int adc_config(adc_channel_t id, const adc_config_t* config);

/**
 * @brief  启用 ADC 通道
 * @param  id ADC 通道 ID
 * @param  enable 1 启用，0 禁用
 */
void adc_enable(adc_channel_t id, int enable);

/**
 * @brief  读取 ADC 原始值
 * @param  id ADC 通道 ID
 * @retval ADC 原始值（0-4095），负数表示错误
 */
int32_t adc_get_value(adc_channel_t id);

/**
 * @brief  读取 ADC 电压值
 * @param  id ADC 通道 ID
 * @param  vref 参考电压（mV）
 * @retval 电压值（mV），负数表示错误
 */
int32_t adc_get_voltage(adc_channel_t id, uint16_t vref);

/**
 * @brief  设置采样完成回调
 * @param  id ADC 通道 ID
 * @param  callback 回调函数
 * @param  arg 回调参数
 */
void adc_set_callback(adc_channel_t id, adc_callback_fn_t callback, void* arg);

/**
 * @brief  触发单次 ADC 采样
 * @param  id ADC 通道 ID
 * @retval ADC 原始值，负数表示错误
 */
int32_t adc_sample_now(adc_channel_t id);

#ifdef __cplusplus
}
#endif

#endif /* ADCCMDS_H */
