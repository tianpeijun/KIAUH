/**
 * @file    heater.c
 * @brief   温度 PID 控制器实现
 * 
 * 移植自 klippy/extras/heaters.py
 * 实现温度读取和 PID 控制功能
 * 
 * 验收标准: 3.1.1 - 3.1.3 (温度读取)
 *          3.2.1 - 3.2.3 (PID 控制)
 *          3.3.1, 3.3.2 (加热器控制)
 */

#include "heater.h"
#include "config.h"
#include "src/stm32/adc.h"
#include "src/pwmcmds.h"
#include "board/gpio.h"
#include <math.h>

/* ========== PID 控制参数 ========== */

/* PID 控制周期 (100ms = 0.1s) */
#define PID_DT                  0.1f

/* 积分限幅 (防止积分饱和) */
#define PID_INTEGRAL_MAX        100.0f

/* 目标温度变化阈值 (超过此值重置 PID 状态) */
#define PID_TARGET_CHANGE_THRESHOLD  10.0f

/* ========== NTC 热敏电阻参数 ========== */

/*
 * 100K NTC 热敏电阻参数 (Beta = 3950)
 * 
 * 使用 Steinhart-Hart 简化公式:
 * 1/T = 1/T0 + (1/B) * ln(R/R0)
 * 
 * 其中:
 * - T0 = 298.15K (25°C)
 * - R0 = 100K (25°C 时的电阻)
 * - B = 3950 (Beta 值)
 */
#define NTC_BETA                3950.0f     /* Beta 值 */
#define NTC_R0                  100000.0f   /* 25°C 时的电阻 (100K) */
#define NTC_T0                  298.15f     /* 参考温度 (25°C = 298.15K) */

/* 分压电阻值 (上拉电阻) */
#define NTC_PULLUP_R            4700.0f     /* 4.7K 上拉电阻 */

/* ADC 参考电压 */
#define ADC_VREF                3.3f        /* 3.3V 参考电压 */

/* ========== NTC 温度查表 ========== */

/*
 * NTC 100K (Beta=3950) 温度查表
 * 
 * 表格格式: { ADC值, 温度(°C) }
 * ADC 值基于 12 位分辨率 (0-4095)
 * 假设使用 4.7K 上拉电阻，3.3V 参考电压
 * 
 * 计算公式:
 * R_ntc = R_pullup * ADC / (4095 - ADC)
 * T = 1 / (1/T0 + ln(R_ntc/R0)/B) - 273.15
 */
typedef struct {
    uint16_t adc;               /* ADC 值 */
    int16_t temp;               /* 温度 (°C * 10，用于提高精度) */
} ntc_table_entry_t;

/* NTC 温度查表 (从高温到低温排列，ADC 值从小到大) */
static const ntc_table_entry_t s_ntc_table[] = {
    /* ADC,   Temp*10 */
    {   23,    3000 },          /* 300°C */
    {   31,    2900 },          /* 290°C */
    {   41,    2800 },          /* 280°C */
    {   54,    2700 },          /* 270°C */
    {   71,    2600 },          /* 260°C */
    {   93,    2500 },          /* 250°C */
    {  120,    2400 },          /* 240°C */
    {  154,    2300 },          /* 230°C */
    {  196,    2200 },          /* 220°C */
    {  248,    2100 },          /* 210°C */
    {  311,    2000 },          /* 200°C */
    {  386,    1900 },          /* 190°C */
    {  475,    1800 },          /* 180°C */
    {  578,    1700 },          /* 170°C */
    {  696,    1600 },          /* 160°C */
    {  829,    1500 },          /* 150°C */
    {  976,    1400 },          /* 140°C */
    { 1136,    1300 },          /* 130°C */
    { 1307,    1200 },          /* 120°C */
    { 1486,    1100 },          /* 110°C */
    { 1670,    1000 },          /* 100°C */
    { 1855,     900 },          /*  90°C */
    { 2037,     800 },          /*  80°C */
    { 2213,     700 },          /*  70°C */
    { 2379,     600 },          /*  60°C */
    { 2534,     500 },          /*  50°C */
    { 2676,     400 },          /*  40°C */
    { 2804,     300 },          /*  30°C */
    { 2918,     200 },          /*  20°C */
    { 3018,     100 },          /*  10°C */
    { 3105,       0 },          /*   0°C */
    { 3180,    -100 },          /* -10°C */
    { 3244,    -200 },          /* -20°C */
};

#define NTC_TABLE_SIZE          (sizeof(s_ntc_table) / sizeof(s_ntc_table[0]))

/* ========== 私有变量 ========== */

/* 加热器状态 */
typedef struct {
    float current_temp;         /* 当前温度 */
    float target_temp;          /* 目标温度 */
    float prev_error;           /* 上次误差 (PID 用) */
    float integral;             /* 积分累计 (PID 用) */
    float output;               /* PWM 输出 (0-1) */
    uint8_t initialized;        /* 初始化标志 */
    uint8_t pwm_enabled;        /* PWM 输出已启用 */
} heater_state_t;

/* 加热器到 PWM 通道的映射 */
static const pwm_channel_t s_heater_pwm_channel[HEATER_COUNT] = {
    PWM_CHANNEL_HEATER_HOTEND,  /* HEATER_HOTEND */
    PWM_CHANNEL_HEATER_BED      /* HEATER_BED */
};

/* 加热器配置 */
static const heater_config_t s_heater_config[HEATER_COUNT] = {
    /* HEATER_HOTEND */
    {
        .adc_channel = TEMP_HOTEND_ADC_CH,
        .pwm_pin = HEATER_HOTEND_PIN,
        .max_power = 1.0f,
        .pid = {
            .kp = HOTEND_PID_KP,
            .ki = HOTEND_PID_KI,
            .kd = HOTEND_PID_KD
        }
    },
    /* HEATER_BED */
    {
        .adc_channel = TEMP_BED_ADC_CH,
        .pwm_pin = HEATER_BED_PIN,
        .max_power = 1.0f,
        .pid = {
            .kp = BED_PID_KP,
            .ki = BED_PID_KI,
            .kd = BED_PID_KD
        }
    }
};

/* 加热器状态数组 */
static heater_state_t s_heater_state[HEATER_COUNT];

/* 模块初始化标志 */
static uint8_t s_module_initialized = 0;

/* ========== 私有函数 ========== */

/**
 * @brief   通过查表将 ADC 值转换为温度
 * @param   adc_value   ADC 读数 (0-4095)
 * @return  温度 (°C)，错误时返回 HEATER_TEMP_INVALID
 * 
 * 使用线性插值在查表中查找温度值。
 * 精度要求: ±2°C
 */
static float
ntc_adc_to_temp(int32_t adc_value)
{
    /* 检查 ADC 值有效性 */
    if (adc_value < 0 || adc_value > ADC_MAX_VALUE) {
        return HEATER_TEMP_INVALID;
    }
    
    /* 检查是否超出表格范围 */
    if (adc_value < (int32_t)s_ntc_table[0].adc) {
        /* ADC 值太小，温度超过最高值 */
        return HEATER_TEMP_MAX;
    }
    
    if (adc_value > (int32_t)s_ntc_table[NTC_TABLE_SIZE - 1].adc) {
        /* ADC 值太大，温度低于最低值 */
        return HEATER_TEMP_MIN;
    }
    
    /* 在表格中查找并线性插值 */
    for (uint32_t i = 0; i < NTC_TABLE_SIZE - 1; i++) {
        if (adc_value >= (int32_t)s_ntc_table[i].adc && 
            adc_value <= (int32_t)s_ntc_table[i + 1].adc) {
            /* 找到区间，进行线性插值 */
            int32_t adc_low = s_ntc_table[i].adc;
            int32_t adc_high = s_ntc_table[i + 1].adc;
            int32_t temp_low = s_ntc_table[i].temp;
            int32_t temp_high = s_ntc_table[i + 1].temp;
            
            /* 线性插值计算 */
            float ratio = (float)(adc_value - adc_low) / (float)(adc_high - adc_low);
            float temp = (float)temp_low + ratio * (float)(temp_high - temp_low);
            
            /* 转换为实际温度 (表中存储的是 temp * 10) */
            return temp / 10.0f;
        }
    }
    
    /* 不应该到达这里 */
    return HEATER_TEMP_INVALID;
}

/**
 * @brief   使用 Steinhart-Hart 公式计算温度 (备用方法)
 * @param   adc_value   ADC 读数 (0-4095)
 * @return  温度 (°C)，错误时返回 HEATER_TEMP_INVALID
 * 
 * 使用简化的 Beta 公式:
 * 1/T = 1/T0 + (1/B) * ln(R/R0)
 * 
 * 注意: 此函数作为备用方法保留，当前使用查表法 (ntc_adc_to_temp)
 */
#if 0  /* 备用方法，当前未使用 */
static float
ntc_adc_to_temp_formula(int32_t adc_value)
{
    /* 检查 ADC 值有效性 */
    if (adc_value <= 0 || adc_value >= ADC_MAX_VALUE) {
        return HEATER_TEMP_INVALID;
    }
    
    /* 计算 NTC 电阻值 */
    /* 分压公式: V_adc = V_ref * R_ntc / (R_pullup + R_ntc) */
    /* 解出: R_ntc = R_pullup * ADC / (ADC_MAX - ADC) */
    float r_ntc = NTC_PULLUP_R * (float)adc_value / (float)(ADC_MAX_VALUE - adc_value);
    
    /* 使用 Beta 公式计算温度 */
    /* 1/T = 1/T0 + (1/B) * ln(R/R0) */
    float ln_r = logf(r_ntc / NTC_R0);
    float inv_t = (1.0f / NTC_T0) + (ln_r / NTC_BETA);
    
    /* 转换为摄氏度 */
    float temp_k = 1.0f / inv_t;
    float temp_c = temp_k - 273.15f;
    
    /* 限制温度范围 */
    if (temp_c < HEATER_TEMP_MIN) {
        temp_c = HEATER_TEMP_MIN;
    } else if (temp_c > HEATER_TEMP_MAX) {
        temp_c = HEATER_TEMP_MAX;
    }
    
    return temp_c;
}
#endif  /* 备用方法 */

/**
 * @brief   获取 ADC 通道对应的 GPIO 引脚
 * @param   channel ADC 通道号
 * @return  GPIO 引脚编码
 */
static uint8_t
get_adc_gpio(uint8_t channel)
{
    /* ADC 通道 0-7 对应 PA0-PA7 */
    if (channel <= 7) {
        return GPIO(GPIO_PORT_A, channel);
    }
    /* ADC 通道 8-9 对应 PB0-PB1 */
    if (channel <= 9) {
        return GPIO(GPIO_PORT_B, channel - 8);
    }
    /* ADC 通道 10-15 对应 PC0-PC5 */
    if (channel <= 15) {
        return GPIO(GPIO_PORT_C, channel - 10);
    }
    
    return GPIO_INVALID;
}

/**
 * @brief   PID 控制器更新
 * @param   state   加热器状态
 * @param   params  PID 参数
 * @param   current_temp 当前温度
 * @param   dt      时间间隔 (秒)
 * @return  PID 输出 (0.0 - 1.0)
 * 
 * 实现带积分限幅 (anti-windup) 的 PID 算法
 * 
 * 验收标准: 3.2.1 - 移植 klippy/extras/heaters.py 的 ControlPID 类
 *          3.2.2 - 支持 Kp/Ki/Kd 配置
 */
static float
pid_update(heater_state_t* state, const pid_params_t* params, 
           float current_temp, float dt)
{
    float error;
    float derivative;
    float output;
    
    /* 计算误差 */
    error = state->target_temp - current_temp;
    
    /* 积分项 (带 anti-windup) */
    state->integral += error * dt;
    
    /* 积分限幅 - 防止积分饱和 */
    if (state->integral > PID_INTEGRAL_MAX) {
        state->integral = PID_INTEGRAL_MAX;
    } else if (state->integral < -PID_INTEGRAL_MAX) {
        state->integral = -PID_INTEGRAL_MAX;
    }
    
    /* 微分项 */
    derivative = (error - state->prev_error) / dt;
    state->prev_error = error;
    
    /* PID 输出计算 */
    output = params->kp * error 
           + params->ki * state->integral 
           + params->kd * derivative;
    
    /* 输出限幅 (0.0 - 1.0) */
    if (output < 0.0f) {
        output = 0.0f;
        /* 当输出饱和且误差为负时，停止积分累积 (anti-windup) */
        if (error < 0.0f && state->integral < 0.0f) {
            state->integral -= error * dt;
        }
    } else if (output > 1.0f) {
        output = 1.0f;
        /* 当输出饱和且误差为正时，停止积分累积 (anti-windup) */
        if (error > 0.0f && state->integral > 0.0f) {
            state->integral -= error * dt;
        }
    }
    
    state->output = output;
    return output;
}

/**
 * @brief   设置加热器 PWM 输出
 * @param   id      加热器 ID
 * @param   duty    占空比 (0.0 - 1.0)
 * 
 * 验收标准: 3.3.1 - 复用 Klipper src/pwmcmds.c
 */
static void
heater_set_pwm(heater_id_t id, float duty)
{
    pwm_channel_t pwm_ch;
    float max_power;
    
    if (id >= HEATER_COUNT) {
        return;
    }
    
    pwm_ch = s_heater_pwm_channel[id];
    max_power = s_heater_config[id].max_power;
    
    /* 限制输出不超过最大功率 */
    if (duty > max_power) {
        duty = max_power;
    }
    
    /* 设置 PWM 占空比 */
    pwm_set_duty(pwm_ch, duty);
}

/* ========== 公共函数 ========== */

/**
 * @brief   初始化加热器模块
 */
void
heater_init(void)
{
    pwm_config_t pwm_cfg;
    
    if (s_module_initialized) {
        return;
    }
    
    /* 初始化 ADC 子系统 */
    adc_init();
    
    /* 初始化 PWM 子系统 */
    pwm_init();
    
    /* 配置每个加热器的 ADC 通道和 PWM 输出 */
    for (uint8_t i = 0; i < HEATER_COUNT; i++) {
        uint8_t gpio = get_adc_gpio(s_heater_config[i].adc_channel);
        
        if (gpio != GPIO_INVALID) {
            /* 配置 ADC 通道，使用较长的采样时间以提高精度 */
            adc_setup(gpio, ADC_SAMPLETIME_480CYCLES);
        }
        
        /* 配置 PWM 通道 */
        pwm_cfg.pin = s_heater_config[i].pwm_pin;
        pwm_cfg.cycle_time = 1000;      /* 1kHz PWM 频率 */
        pwm_cfg.max_value = 255;        /* 8 位分辨率 */
        pwm_cfg.invert = 0;             /* 不反相 */
        pwm_cfg.use_hardware = 0;       /* 使用软件 PWM */
        pwm_config(s_heater_pwm_channel[i], &pwm_cfg);
        
        /* 初始化加热器状态 */
        s_heater_state[i].current_temp = 0.0f;
        s_heater_state[i].target_temp = 0.0f;
        s_heater_state[i].prev_error = 0.0f;
        s_heater_state[i].integral = 0.0f;
        s_heater_state[i].output = 0.0f;
        s_heater_state[i].initialized = 1;
        s_heater_state[i].pwm_enabled = 0;
    }
    
    s_module_initialized = 1;
}

/**
 * @brief   设置目标温度
 * 
 * 验收标准: 3.3.2 - 支持 M104/M109 指令
 */
void
heater_set_temp(heater_id_t id, float target)
{
    float old_target;
    float target_diff;
    
    if (id >= HEATER_COUNT) {
        return;
    }
    
    /* 限制目标温度范围 */
    if (target < HEATER_TEMP_MIN) {
        target = HEATER_TEMP_MIN;
    } else if (target > HEATER_TEMP_MAX) {
        target = HEATER_TEMP_MAX;
    }
    
    /* 保存旧目标温度 */
    old_target = s_heater_state[id].target_temp;
    
    /* 计算目标温度变化量 */
    target_diff = target - old_target;
    if (target_diff < 0.0f) {
        target_diff = -target_diff;
    }
    
    /* 设置新目标温度 */
    s_heater_state[id].target_temp = target;
    
    /* 如果目标温度变化较大，重置 PID 状态 */
    if (target_diff > PID_TARGET_CHANGE_THRESHOLD) {
        s_heater_state[id].integral = 0.0f;
        s_heater_state[id].prev_error = 0.0f;
    }
    
    /* 如果目标温度为 0，关闭加热器 */
    if (target <= 0.0f) {
        s_heater_state[id].integral = 0.0f;
        s_heater_state[id].prev_error = 0.0f;
        s_heater_state[id].output = 0.0f;
        
        /* 关闭 PWM 输出 */
        heater_set_pwm(id, 0.0f);
        pwm_enable(s_heater_pwm_channel[id], 0);
        s_heater_state[id].pwm_enabled = 0;
    } else {
        /* 启用 PWM 输出 */
        if (!s_heater_state[id].pwm_enabled) {
            pwm_enable(s_heater_pwm_channel[id], 1);
            s_heater_state[id].pwm_enabled = 1;
        }
    }
}

/**
 * @brief   获取当前温度
 */
float
heater_get_temp(heater_id_t id)
{
    if (id >= HEATER_COUNT) {
        return HEATER_TEMP_INVALID;
    }
    
    /* 确保模块已初始化 */
    if (!s_module_initialized) {
        heater_init();
    }
    
    /* 获取 ADC 通道对应的 GPIO */
    uint8_t gpio = get_adc_gpio(s_heater_config[id].adc_channel);
    
    if (gpio == GPIO_INVALID) {
        return HEATER_TEMP_INVALID;
    }
    
    /* 读取 ADC 值 */
    int32_t adc_value = adc_read(gpio);
    
    if (adc_value < 0) {
        /* ADC 读取错误 */
        return HEATER_TEMP_INVALID;
    }
    
    /* 使用查表法转换为温度 */
    float temp = ntc_adc_to_temp(adc_value);
    
    /* 更新当前温度状态 */
    if (temp != HEATER_TEMP_INVALID) {
        s_heater_state[id].current_temp = temp;
    }
    
    return temp;
}

/**
 * @brief   获取目标温度
 */
float
heater_get_target(heater_id_t id)
{
    if (id >= HEATER_COUNT) {
        return 0.0f;
    }
    
    return s_heater_state[id].target_temp;
}

/**
 * @brief   检查是否达到目标温度
 */
int
heater_is_at_target(heater_id_t id)
{
    if (id >= HEATER_COUNT) {
        return 0;
    }
    
    float current = heater_get_temp(id);
    float target = s_heater_state[id].target_temp;
    
    /* 如果目标温度为 0，认为已达到 */
    if (target <= 0.0f) {
        return 1;
    }
    
    /* 检查当前温度是否在目标温度 ±3°C 范围内 */
    float diff = current - target;
    if (diff < 0.0f) {
        diff = -diff;
    }
    
    return (diff <= HEATER_TEMP_TOLERANCE) ? 1 : 0;
}

/**
 * @brief   温度控制周期任务
 * 
 * 执行 PID 计算和 PWM 输出更新。
 * 应在主循环中以 100ms 间隔调用。
 * 
 * 验收标准: 3.2.1 - 移植 klippy/extras/heaters.py 的 ControlPID 类
 *          3.2.3 - 温度稳定后波动 < ±3°C
 */
void
heater_task(void)
{
    float current_temp;
    float output;
    
    if (!s_module_initialized) {
        return;
    }
    
    /* 遍历所有加热器 */
    for (uint8_t i = 0; i < HEATER_COUNT; i++) {
        /* 读取当前温度 */
        current_temp = heater_get_temp((heater_id_t)i);
        
        /* 检查温度读取是否有效 */
        if (current_temp == HEATER_TEMP_INVALID) {
            /* 温度读取错误，关闭加热器以确保安全 */
            heater_set_pwm((heater_id_t)i, 0.0f);
            continue;
        }
        
        /* 如果目标温度为 0，跳过 PID 计算 */
        if (s_heater_state[i].target_temp <= 0.0f) {
            s_heater_state[i].output = 0.0f;
            heater_set_pwm((heater_id_t)i, 0.0f);
            continue;
        }
        
        /* 执行 PID 计算 */
        output = pid_update(&s_heater_state[i], 
                           &s_heater_config[i].pid,
                           current_temp, 
                           PID_DT);
        
        /* 设置 PWM 输出 */
        heater_set_pwm((heater_id_t)i, output);
    }
}

/**
 * @brief   获取当前 PID 输出
 * @param   id      加热器 ID
 * @return  PID 输出 (0.0 - 1.0)，错误时返回 -1.0
 */
float
heater_get_output(heater_id_t id)
{
    if (id >= HEATER_COUNT) {
        return -1.0f;
    }
    
    return s_heater_state[id].output;
}

#ifdef TEST_BUILD
/**
 * @brief   重置加热器模块状态 (仅用于测试)
 * 
 * 此函数仅在测试构建中可用，用于在测试之间重置模块状态。
 */
void
heater_reset_for_test(void)
{
    s_module_initialized = 0;
    
    for (uint8_t i = 0; i < HEATER_COUNT; i++) {
        s_heater_state[i].current_temp = 0.0f;
        s_heater_state[i].target_temp = 0.0f;
        s_heater_state[i].prev_error = 0.0f;
        s_heater_state[i].integral = 0.0f;
        s_heater_state[i].output = 0.0f;
        s_heater_state[i].initialized = 0;
        s_heater_state[i].pwm_enabled = 0;
    }
}
#endif /* TEST_BUILD */
