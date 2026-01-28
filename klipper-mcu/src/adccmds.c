/**
 * @file    adccmds.c
 * @brief   ADC 命令处理实现
 * 
 * 复用自 Klipper src/adccmds.c
 * 提供 ADC 采样和温度读取功能
 */

#include "adccmds.h"
#include "sched.h"
#include <stddef.h>

/* ========== 私有类型定义 ========== */

/* ADC 通道状态 */
typedef struct {
    uint8_t channel;            /* ADC 通道号 */
    uint8_t configured;         /* 是否已配置 */
    uint8_t enabled;            /* 是否启用 */
    uint16_t value;             /* 最新采样值 */
    uint16_t min_value;         /* 最小值（用于范围检查） */
    uint16_t max_value;         /* 最大值（用于范围检查） */
    adc_callback_fn_t callback; /* 采样完成回调 */
    void* callback_arg;         /* 回调参数 */
} adc_state_t;

/* ========== 私有变量 ========== */

/* ADC 通道状态数组 */
static adc_state_t s_adc_channels[ADC_CHANNEL_COUNT];

/* ADC 采样定时器 */
static sched_timer_t s_adc_timer;

/* 采样间隔（时钟周期，约 100ms） */
#define ADC_SAMPLE_INTERVAL     100000

/* ADC 分辨率 */
#define ADC_RESOLUTION          12
#define ADC_MAX_VALUE           ((1 << ADC_RESOLUTION) - 1)

/* ========== HAL 接口 ========== */

/* TODO: 需要 HAL 层实现这些函数 */
extern void adc_setup(uint8_t channel);
extern uint16_t adc_read(uint8_t channel);

/* ========== 私有函数 ========== */

/**
 * @brief  ADC 采样定时器回调
 * @param  waketime 唤醒时间
 * @retval 下次唤醒时间
 */
static sched_time_t adc_timer_callback(sched_time_t waketime)
{
    int i;
    uint16_t value;
    
    /* 遍历所有 ADC 通道 */
    for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
        adc_state_t* adc = &s_adc_channels[i];
        
        if (!adc->configured || !adc->enabled) {
            continue;
        }
        
        /* 读取 ADC 值 */
        value = adc_read(adc->channel);
        adc->value = value;
        
        /* 调用回调函数 */
        if (adc->callback != NULL) {
            adc->callback((adc_channel_t)i, value, adc->callback_arg);
        }
    }
    
    /* 返回下次采样时间 */
    return waketime + ADC_SAMPLE_INTERVAL;
}

/* ========== 公共接口实现 ========== */

/**
 * @brief  初始化 ADC 命令模块
 * @retval 0 成功，负数 失败
 */
int adccmds_init(void)
{
    int i;
    
    /* 清零所有状态 */
    for (i = 0; i < ADC_CHANNEL_COUNT; i++) {
        s_adc_channels[i].channel = 0;
        s_adc_channels[i].configured = 0;
        s_adc_channels[i].enabled = 0;
        s_adc_channels[i].value = 0;
        s_adc_channels[i].min_value = 0;
        s_adc_channels[i].max_value = ADC_MAX_VALUE;
        s_adc_channels[i].callback = NULL;
        s_adc_channels[i].callback_arg = NULL;
    }
    
    /* 初始化采样定时器 */
    s_adc_timer.func = adc_timer_callback;
    s_adc_timer.waketime = sched_get_time() + ADC_SAMPLE_INTERVAL;
    s_adc_timer.next = NULL;
    
    /* 启动采样定时器 */
    sched_add_timer(&s_adc_timer);
    
    return 0;
}

/**
 * @brief  配置 ADC 通道
 * @param  id ADC 通道 ID
 * @param  config 配置参数
 * @retval 0 成功，负数 失败
 */
int adc_config(adc_channel_t id, const adc_config_t* config)
{
    adc_state_t* adc;
    
    if (id >= ADC_CHANNEL_COUNT || config == NULL) {
        return -1;
    }
    
    adc = &s_adc_channels[id];
    adc->channel = config->channel;
    adc->min_value = config->min_value;
    adc->max_value = config->max_value;
    adc->configured = 1;
    
    /* 配置 HAL 层 ADC */
    adc_setup(config->channel);
    
    return 0;
}

/**
 * @brief  启用 ADC 通道
 * @param  id ADC 通道 ID
 * @param  enable 1 启用，0 禁用
 */
void adc_enable(adc_channel_t id, int enable)
{
    if (id >= ADC_CHANNEL_COUNT) {
        return;
    }
    
    s_adc_channels[id].enabled = enable ? 1 : 0;
}

/**
 * @brief  读取 ADC 原始值
 * @param  id ADC 通道 ID
 * @retval ADC 原始值（0-4095），负数表示错误
 */
int32_t adc_get_value(adc_channel_t id)
{
    if (id >= ADC_CHANNEL_COUNT) {
        return -1;
    }
    
    if (!s_adc_channels[id].configured) {
        return -2;
    }
    
    return s_adc_channels[id].value;
}

/**
 * @brief  读取 ADC 电压值
 * @param  id ADC 通道 ID
 * @param  vref 参考电压（mV）
 * @retval 电压值（mV），负数表示错误
 */
int32_t adc_get_voltage(adc_channel_t id, uint16_t vref)
{
    int32_t value;
    
    value = adc_get_value(id);
    if (value < 0) {
        return value;
    }
    
    /* 计算电压：voltage = value * vref / ADC_MAX_VALUE */
    return (value * vref) / ADC_MAX_VALUE;
}

/**
 * @brief  设置采样完成回调
 * @param  id ADC 通道 ID
 * @param  callback 回调函数
 * @param  arg 回调参数
 */
void adc_set_callback(adc_channel_t id, adc_callback_fn_t callback, void* arg)
{
    if (id >= ADC_CHANNEL_COUNT) {
        return;
    }
    
    s_adc_channels[id].callback = callback;
    s_adc_channels[id].callback_arg = arg;
}

/**
 * @brief  触发单次 ADC 采样
 * @param  id ADC 通道 ID
 * @retval ADC 原始值，负数表示错误
 */
int32_t adc_sample_now(adc_channel_t id)
{
    adc_state_t* adc;
    uint16_t value;
    
    if (id >= ADC_CHANNEL_COUNT) {
        return -1;
    }
    
    adc = &s_adc_channels[id];
    if (!adc->configured) {
        return -2;
    }
    
    /* 立即读取 ADC */
    value = adc_read(adc->channel);
    adc->value = value;
    
    return value;
}
