/**
 * @file    test_heater.c
 * @brief   温度读取和 PID 控制单元测试
 * 
 * 测试 heater.c 的温度读取和 PID 控制功能
 * 使用主机编译器 (gcc) 编译运行
 * 
 * 编译: gcc -o test_heater test_heater.c ../app/heater.c stubs.c -I../app -I.. -I../src -DTEST_BUILD -lm
 * 运行: ./test_heater
 * 
 * 验收标准: 3.1.1 - 3.1.3 (温度读取)
 *          3.2.1 - 3.2.3 (PID 控制)
 *          3.3.1, 3.3.2 (加热器控制)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "heater.h"

/* ========== 测试框架 ========== */

static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
        return 0; \
    } \
} while (0)

#define TEST_ASSERT_EQ(a, b, msg) do { \
    if ((a) != (b)) { \
        printf("  FAIL: %s - expected %d, got %d (line %d)\n", msg, (int)(b), (int)(a), __LINE__); \
        return 0; \
    } \
} while (0)

#define TEST_ASSERT_FLOAT_EQ(a, b, tol, msg) do { \
    if (fabsf((a) - (b)) > (tol)) { \
        printf("  FAIL: %s - expected %.2f, got %.2f (line %d)\n", msg, (float)(b), (float)(a), __LINE__); \
        return 0; \
    } \
} while (0)

#define RUN_TEST(test_func) do { \
    g_tests_run++; \
    printf("Running %s...\n", #test_func); \
    if (test_func()) { \
        g_tests_passed++; \
        printf("  PASS\n"); \
    } else { \
        g_tests_failed++; \
    } \
} while (0)

/* ========== ADC 模拟桩 ========== */

/* 模拟 ADC 值 (用于测试) */
static int32_t s_mock_adc_values[16] = {0};
static int s_adc_initialized = 0;

/* ADC 桩函数 */
void adc_init(void)
{
    s_adc_initialized = 1;
    for (int i = 0; i < 16; i++) {
        s_mock_adc_values[i] = 2048;  /* 默认中间值 */
    }
}

int adc_setup(uint8_t gpio, int sample_time)
{
    (void)gpio;
    (void)sample_time;
    if (!s_adc_initialized) {
        adc_init();
    }
    return 0;
}

int32_t adc_read(uint8_t gpio)
{
    /* 从 GPIO 提取通道号 */
    uint8_t port = (gpio >> 4) & 0x0F;
    uint8_t pin = gpio & 0x0F;
    int channel = -1;
    
    /* Port A: PA0-PA7 -> Channel 0-7 */
    if (port == 0 && pin <= 7) {
        channel = pin;
    }
    /* Port B: PB0-PB1 -> Channel 8-9 */
    else if (port == 1 && pin <= 1) {
        channel = 8 + pin;
    }
    /* Port C: PC0-PC5 -> Channel 10-15 */
    else if (port == 2 && pin <= 5) {
        channel = 10 + pin;
    }
    
    if (channel >= 0 && channel < 16) {
        return s_mock_adc_values[channel];
    }
    return -1;
}

int32_t adc_read_channel(uint8_t channel)
{
    if (channel < 16) {
        return s_mock_adc_values[channel];
    }
    return -1;
}

/* 测试辅助函数: 设置模拟 ADC 值 */
void test_set_adc_value(uint8_t channel, int32_t value)
{
    if (channel < 16) {
        s_mock_adc_values[channel] = value;
    }
}

/* ========== GPIO 桩函数 ========== */

void gpio_analog_setup(uint8_t gpio)
{
    (void)gpio;
}

void gpio_out_setup(uint8_t pin, uint8_t value)
{
    (void)pin;
    (void)value;
}

void gpio_out_write(uint8_t pin, uint8_t value)
{
    (void)pin;
    (void)value;
}

void pwm_setup(uint8_t pin, uint32_t cycle_time, uint8_t value)
{
    (void)pin;
    (void)cycle_time;
    (void)value;
}

void pwm_write(uint8_t pin, uint8_t value)
{
    (void)pin;
    (void)value;
}

/* ========== PWM 模拟桩 ========== */

#define PWM_CHANNEL_COUNT 4

typedef struct {
    uint8_t pin;
    uint8_t configured;
    uint8_t enabled;
    float duty;
} mock_pwm_state_t;

static mock_pwm_state_t s_mock_pwm[PWM_CHANNEL_COUNT];

int pwm_init(void)
{
    for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
        s_mock_pwm[i].pin = 0;
        s_mock_pwm[i].configured = 0;
        s_mock_pwm[i].enabled = 0;
        s_mock_pwm[i].duty = 0.0f;
    }
    return 0;
}

typedef struct {
    uint8_t pin;
    uint32_t cycle_time;
    uint16_t max_value;
    uint8_t invert;
    uint8_t use_hardware;
} pwm_config_t;

int pwm_config(int id, const pwm_config_t* config)
{
    if (id >= 0 && id < PWM_CHANNEL_COUNT && config != NULL) {
        s_mock_pwm[id].pin = config->pin;
        s_mock_pwm[id].configured = 1;
    }
    return 0;
}

void pwm_enable(int id, int enable)
{
    if (id >= 0 && id < PWM_CHANNEL_COUNT) {
        s_mock_pwm[id].enabled = enable ? 1 : 0;
    }
}

void pwm_set_duty(int id, float duty)
{
    if (id >= 0 && id < PWM_CHANNEL_COUNT) {
        if (duty < 0.0f) {
            duty = 0.0f;
        } else if (duty > 1.0f) {
            duty = 1.0f;
        }
        s_mock_pwm[id].duty = duty;
    }
}

/* 测试辅助函数: 获取 PWM 占空比 */
float test_get_pwm_duty(int id)
{
    if (id >= 0 && id < PWM_CHANNEL_COUNT) {
        return s_mock_pwm[id].duty;
    }
    return -1.0f;
}

/* 测试辅助函数: 检查 PWM 是否启用 */
int test_is_pwm_enabled(int id)
{
    if (id >= 0 && id < PWM_CHANNEL_COUNT) {
        return s_mock_pwm[id].enabled;
    }
    return 0;
}

/* 测试辅助函数: 重置所有模拟状态 */
void test_reset_all(void)
{
    /* 重置加热器模块状态 */
    heater_reset_for_test();
    
    /* 重置 ADC 模拟状态 */
    s_adc_initialized = 0;
    adc_init();
    
    /* 重置 PWM 模拟状态 */
    pwm_init();
}

/* ========== 调度器桩函数 ========== */

typedef struct {
    void* func;
    uint32_t waketime;
    void* next;
} sched_timer_t;

uint32_t sched_get_time(void)
{
    return 0;
}

void sched_add_timer(sched_timer_t* timer)
{
    (void)timer;
}

/* ========== 测试用例 ========== */

/**
 * @brief   测试 heater_init 初始化
 */
static int
test_heater_init(void)
{
    heater_init();
    
    /* 初始化后应该能正常读取温度 */
    float temp = heater_get_temp(HEATER_HOTEND);
    TEST_ASSERT(temp != HEATER_TEMP_INVALID, "should be able to read temperature after init");
    
    return 1;
}

/**
 * @brief   测试无效加热器 ID 处理
 */
static int
test_invalid_heater_id(void)
{
    float temp;
    
    /* 测试无效 ID */
    temp = heater_get_temp(HEATER_COUNT);
    TEST_ASSERT_FLOAT_EQ(temp, HEATER_TEMP_INVALID, 0.1f, "invalid ID should return HEATER_TEMP_INVALID");
    
    temp = heater_get_temp((heater_id_t)99);
    TEST_ASSERT_FLOAT_EQ(temp, HEATER_TEMP_INVALID, 0.1f, "out of range ID should return HEATER_TEMP_INVALID");
    
    /* 测试 heater_get_target 无效 ID */
    float target = heater_get_target(HEATER_COUNT);
    TEST_ASSERT_FLOAT_EQ(target, 0.0f, 0.1f, "invalid ID should return 0 for target");
    
    /* 测试 heater_is_at_target 无效 ID */
    int at_target = heater_is_at_target(HEATER_COUNT);
    TEST_ASSERT_EQ(at_target, 0, "invalid ID should return 0 for is_at_target");
    
    return 1;
}

/**
 * @brief   测试 NTC 温度查表 - 室温范围
 * @note    验收标准: 3.1.2 - 支持 NTC 热敏电阻
 *          验收标准: 3.1.3 - 温度精度 ±2°C
 */
static int
test_ntc_room_temperature(void)
{
    heater_init();
    
    /* 设置 ADC 值对应约 30°C (查表值 2804) */
    test_set_adc_value(0, 2804);  /* Channel 0 = HOTEND */
    
    float temp = heater_get_temp(HEATER_HOTEND);
    
    /* 验证温度在 30°C ±2°C 范围内 */
    TEST_ASSERT(temp >= 28.0f && temp <= 32.0f, 
                "room temperature should be around 30°C (±2°C)");
    
    printf("    ADC=2804 -> Temp=%.2f°C (expected ~30°C)\n", temp);
    
    /* 测试 20°C (查表值 2918) */
    test_set_adc_value(0, 2918);
    temp = heater_get_temp(HEATER_HOTEND);
    TEST_ASSERT(temp >= 18.0f && temp <= 22.0f, 
                "20°C should be within ±2°C");
    printf("    ADC=2918 -> Temp=%.2f°C (expected ~20°C)\n", temp);
    
    return 1;
}

/**
 * @brief   测试 NTC 温度查表 - 高温范围 (打印温度)
 * @note    验收标准: 3.1.3 - 温度精度 ±2°C
 */
static int
test_ntc_high_temperature(void)
{
    heater_init();
    
    /* 测试 200°C (查表值约 311) */
    test_set_adc_value(0, 311);
    float temp = heater_get_temp(HEATER_HOTEND);
    TEST_ASSERT(temp >= 198.0f && temp <= 202.0f, 
                "200°C should be within ±2°C");
    printf("    ADC=311 -> Temp=%.2f°C (expected ~200°C)\n", temp);
    
    /* 测试 150°C (查表值约 829) */
    test_set_adc_value(0, 829);
    temp = heater_get_temp(HEATER_HOTEND);
    TEST_ASSERT(temp >= 148.0f && temp <= 152.0f, 
                "150°C should be within ±2°C");
    printf("    ADC=829 -> Temp=%.2f°C (expected ~150°C)\n", temp);
    
    /* 测试 100°C (查表值约 1670) */
    test_set_adc_value(0, 1670);
    temp = heater_get_temp(HEATER_HOTEND);
    TEST_ASSERT(temp >= 98.0f && temp <= 102.0f, 
                "100°C should be within ±2°C");
    printf("    ADC=1670 -> Temp=%.2f°C (expected ~100°C)\n", temp);
    
    return 1;
}

/**
 * @brief   测试 NTC 温度查表 - 低温范围
 * @note    验收标准: 3.1.3 - 温度精度 ±2°C
 */
static int
test_ntc_low_temperature(void)
{
    heater_init();
    
    /* 测试 0°C (查表值约 3105) */
    test_set_adc_value(0, 3105);
    float temp = heater_get_temp(HEATER_HOTEND);
    TEST_ASSERT(temp >= -2.0f && temp <= 2.0f, 
                "0°C should be within ±2°C");
    printf("    ADC=3105 -> Temp=%.2f°C (expected ~0°C)\n", temp);
    
    /* 测试 10°C (查表值约 3018) */
    test_set_adc_value(0, 3018);
    temp = heater_get_temp(HEATER_HOTEND);
    TEST_ASSERT(temp >= 8.0f && temp <= 12.0f, 
                "10°C should be within ±2°C");
    printf("    ADC=3018 -> Temp=%.2f°C (expected ~10°C)\n", temp);
    
    return 1;
}

/**
 * @brief   测试 NTC 温度查表 - 线性插值
 * @note    验收标准: 3.1.3 - 温度精度 ±2°C
 */
static int
test_ntc_interpolation(void)
{
    heater_init();
    
    /* 测试两个表格点之间的值 */
    /* 100°C (ADC=1670) 和 110°C (ADC=1486) 之间 */
    /* 中间值约 1578 应该对应约 105°C */
    test_set_adc_value(0, 1578);
    float temp = heater_get_temp(HEATER_HOTEND);
    TEST_ASSERT(temp >= 103.0f && temp <= 107.0f, 
                "interpolated 105°C should be within ±2°C");
    printf("    ADC=1578 -> Temp=%.2f°C (expected ~105°C)\n", temp);
    
    /* 50°C (ADC=2534) 和 60°C (ADC=2379) 之间 */
    /* 中间值约 2456 应该对应约 55°C */
    test_set_adc_value(0, 2456);
    temp = heater_get_temp(HEATER_HOTEND);
    TEST_ASSERT(temp >= 53.0f && temp <= 57.0f, 
                "interpolated 55°C should be within ±2°C");
    printf("    ADC=2456 -> Temp=%.2f°C (expected ~55°C)\n", temp);
    
    return 1;
}

/**
 * @brief   测试温度范围边界
 */
static int
test_temperature_bounds(void)
{
    heater_init();
    
    /* 测试超高温 (ADC 值很小) */
    test_set_adc_value(0, 10);  /* 超出表格范围 */
    float temp = heater_get_temp(HEATER_HOTEND);
    TEST_ASSERT(temp <= HEATER_TEMP_MAX, "should not exceed max temperature");
    printf("    ADC=10 -> Temp=%.2f°C (clamped to max)\n", temp);
    
    /* 测试超低温 (ADC 值很大) */
    test_set_adc_value(0, 4000);  /* 超出表格范围 */
    temp = heater_get_temp(HEATER_HOTEND);
    TEST_ASSERT(temp >= HEATER_TEMP_MIN, "should not go below min temperature");
    printf("    ADC=4000 -> Temp=%.2f°C (clamped to min)\n", temp);
    
    return 1;
}

/**
 * @brief   测试热床温度读取
 * @note    验收标准: 3.1.1 - 复用 Klipper src/adccmds.c
 */
static int
test_bed_temperature(void)
{
    heater_init();
    
    /* 设置热床 ADC 值 (Channel 1) 对应约 60°C */
    test_set_adc_value(1, 2379);
    
    float temp = heater_get_temp(HEATER_BED);
    TEST_ASSERT(temp >= 58.0f && temp <= 62.0f, 
                "bed temperature should be around 60°C (±2°C)");
    printf("    Bed ADC=2379 -> Temp=%.2f°C (expected ~60°C)\n", temp);
    
    return 1;
}

/**
 * @brief   测试目标温度设置和获取
 */
static int
test_target_temperature(void)
{
    heater_init();
    
    /* 设置目标温度 */
    heater_set_temp(HEATER_HOTEND, 200.0f);
    float target = heater_get_target(HEATER_HOTEND);
    TEST_ASSERT_FLOAT_EQ(target, 200.0f, 0.1f, "target should be 200°C");
    
    /* 设置热床目标温度 */
    heater_set_temp(HEATER_BED, 60.0f);
    target = heater_get_target(HEATER_BED);
    TEST_ASSERT_FLOAT_EQ(target, 60.0f, 0.1f, "bed target should be 60°C");
    
    /* 测试温度范围限制 */
    heater_set_temp(HEATER_HOTEND, 500.0f);  /* 超过最大值 */
    target = heater_get_target(HEATER_HOTEND);
    TEST_ASSERT(target <= HEATER_TEMP_MAX, "target should be clamped to max");
    
    heater_set_temp(HEATER_HOTEND, -50.0f);  /* 低于最小值 */
    target = heater_get_target(HEATER_HOTEND);
    TEST_ASSERT(target >= HEATER_TEMP_MIN, "target should be clamped to min");
    
    return 1;
}

/**
 * @brief   测试 heater_is_at_target 功能
 */
static int
test_is_at_target(void)
{
    heater_init();
    
    /* 设置目标温度为 100°C */
    heater_set_temp(HEATER_HOTEND, 100.0f);
    
    /* 设置当前温度为 100°C (ADC=1670) */
    test_set_adc_value(0, 1670);
    int at_target = heater_is_at_target(HEATER_HOTEND);
    TEST_ASSERT_EQ(at_target, 1, "should be at target when temp matches");
    
    /* 设置当前温度为约 102°C (在 ±3°C 范围内) */
    /* 100°C = ADC 1670, 110°C = ADC 1486 */
    /* 102°C 约等于 ADC 1633 (线性插值) */
    test_set_adc_value(0, 1633);
    float temp = heater_get_temp(HEATER_HOTEND);
    printf("    ADC=1633 -> Temp=%.2f°C (testing ±3°C tolerance)\n", temp);
    at_target = heater_is_at_target(HEATER_HOTEND);
    TEST_ASSERT_EQ(at_target, 1, "should be at target within ±3°C tolerance");
    
    /* 设置当前温度为 90°C (超出 ±3°C 范围) */
    test_set_adc_value(0, 1855);  /* 约 90°C */
    at_target = heater_is_at_target(HEATER_HOTEND);
    TEST_ASSERT_EQ(at_target, 0, "should not be at target when diff > 3°C");
    
    /* 测试目标温度为 0 的情况 */
    heater_set_temp(HEATER_HOTEND, 0.0f);
    at_target = heater_is_at_target(HEATER_HOTEND);
    TEST_ASSERT_EQ(at_target, 1, "should be at target when target is 0");
    
    return 1;
}

/**
 * @brief   测试 heater_task 周期任务
 */
static int
test_heater_task(void)
{
    heater_init();
    
    /* 设置一些 ADC 值 */
    test_set_adc_value(0, 1670);  /* 热端约 100°C */
    test_set_adc_value(1, 2379);  /* 热床约 60°C */
    
    /* 调用周期任务 */
    heater_task();
    
    /* 验证温度已更新 */
    float hotend_temp = heater_get_temp(HEATER_HOTEND);
    float bed_temp = heater_get_temp(HEATER_BED);
    
    TEST_ASSERT(hotend_temp >= 98.0f && hotend_temp <= 102.0f, 
                "hotend temp should be updated");
    TEST_ASSERT(bed_temp >= 58.0f && bed_temp <= 62.0f, 
                "bed temp should be updated");
    
    return 1;
}

/**
 * @brief   测试多次初始化 (幂等性)
 */
static int
test_multiple_init(void)
{
    /* 多次调用 init 不应该出错 */
    heater_init();
    heater_init();
    heater_init();
    
    /* 应该仍然能正常工作 */
    test_set_adc_value(0, 2804);
    float temp = heater_get_temp(HEATER_HOTEND);
    TEST_ASSERT(temp != HEATER_TEMP_INVALID, "should work after multiple inits");
    
    return 1;
}

/* ========== PID 控制测试用例 ========== */

/**
 * @brief   测试 PID 输出范围
 * @note    验收标准: 3.2.1 - 移植 klippy/extras/heaters.py 的 ControlPID 类
 *          正确性属性 P5: PID 输出在 [0, max_power] 范围内
 */
static int
test_pid_output_range(void)
{
    test_reset_all();
    heater_init();
    
    /* 设置目标温度为 200°C */
    heater_set_temp(HEATER_HOTEND, 200.0f);
    
    /* 设置当前温度为 30°C (远低于目标) */
    test_set_adc_value(0, 2804);  /* 约 30°C */
    
    /* 执行 PID 计算 */
    heater_task();
    
    /* 获取 PID 输出 */
    float output = heater_get_output(HEATER_HOTEND);
    
    /* 验证输出在 0-1 范围内 */
    TEST_ASSERT(output >= 0.0f, "PID output should be >= 0");
    TEST_ASSERT(output <= 1.0f, "PID output should be <= 1");
    
    printf("    Target=200°C, Current=30°C -> Output=%.3f (should be high)\n", output);
    
    /* 当温度远低于目标时，输出应该接近最大值 */
    TEST_ASSERT(output > 0.5f, "output should be high when temp is far below target");
    
    return 1;
}

/**
 * @brief   测试 PID 输出为零当目标为零
 * @note    验收标准: 3.2.1 - PID 控制器
 */
static int
test_pid_zero_target(void)
{
    test_reset_all();
    heater_init();
    
    /* 设置目标温度为 0 */
    heater_set_temp(HEATER_HOTEND, 0.0f);
    
    /* 设置当前温度为 100°C */
    test_set_adc_value(0, 1670);  /* 约 100°C */
    
    /* 执行 PID 计算 */
    heater_task();
    
    /* 获取 PID 输出 */
    float output = heater_get_output(HEATER_HOTEND);
    
    /* 目标为 0 时，输出应该为 0 */
    TEST_ASSERT_FLOAT_EQ(output, 0.0f, 0.01f, "output should be 0 when target is 0");
    
    printf("    Target=0°C, Current=100°C -> Output=%.3f (should be 0)\n", output);
    
    return 1;
}

/**
 * @brief   测试 PID 输出当温度高于目标
 * @note    验收标准: 3.2.1 - PID 控制器
 */
static int
test_pid_overshoot(void)
{
    test_reset_all();
    heater_init();
    
    /* 设置目标温度为 100°C */
    heater_set_temp(HEATER_HOTEND, 100.0f);
    
    /* 设置当前温度为 150°C (高于目标) */
    test_set_adc_value(0, 829);  /* 约 150°C */
    
    /* 执行 PID 计算 */
    heater_task();
    
    /* 获取 PID 输出 */
    float output = heater_get_output(HEATER_HOTEND);
    
    /* 当温度高于目标时，输出应该为 0 */
    TEST_ASSERT_FLOAT_EQ(output, 0.0f, 0.01f, "output should be 0 when temp > target");
    
    printf("    Target=100°C, Current=150°C -> Output=%.3f (should be 0)\n", output);
    
    return 1;
}

/**
 * @brief   测试 PID 输出当温度接近目标
 * @note    验收标准: 3.2.3 - 温度稳定后波动 < ±3°C
 */
static int
test_pid_near_target(void)
{
    test_reset_all();
    heater_init();
    
    /* 设置目标温度为 100°C */
    heater_set_temp(HEATER_HOTEND, 100.0f);
    
    /* 设置当前温度为 98°C (略低于目标) */
    /* 100°C = ADC 1670, 90°C = ADC 1855 */
    /* 98°C 约等于 ADC 1707 */
    test_set_adc_value(0, 1707);
    
    /* 执行 PID 计算 */
    heater_task();
    
    /* 获取 PID 输出 */
    float output = heater_get_output(HEATER_HOTEND);
    
    /* 当温度接近目标时，输出应该是中等值 */
    TEST_ASSERT(output >= 0.0f && output <= 1.0f, "output should be in valid range");
    
    float temp = heater_get_temp(HEATER_HOTEND);
    printf("    Target=100°C, Current=%.1f°C -> Output=%.3f\n", temp, output);
    
    return 1;
}

/**
 * @brief   测试 PWM 输出启用/禁用
 * @note    验收标准: 3.3.1 - 复用 Klipper src/pwmcmds.c
 */
static int
test_pwm_enable_disable(void)
{
    test_reset_all();
    heater_init();
    
    /* 设置目标温度为 200°C - 应该启用 PWM */
    heater_set_temp(HEATER_HOTEND, 200.0f);
    TEST_ASSERT_EQ(test_is_pwm_enabled(0), 1, "PWM should be enabled when target > 0");
    
    /* 设置目标温度为 0 - 应该禁用 PWM */
    heater_set_temp(HEATER_HOTEND, 0.0f);
    TEST_ASSERT_EQ(test_is_pwm_enabled(0), 0, "PWM should be disabled when target = 0");
    
    return 1;
}

/**
 * @brief   测试 PID 状态重置当目标变化较大
 * @note    验收标准: 3.2.2 - 支持 Kp/Ki/Kd 配置
 */
static int
test_pid_reset_on_large_change(void)
{
    test_reset_all();
    heater_init();
    
    /* 设置初始目标温度 */
    heater_set_temp(HEATER_HOTEND, 100.0f);
    
    /* 设置当前温度为 80°C */
    test_set_adc_value(0, 2037);  /* 约 80°C */
    
    /* 执行多次 PID 计算以累积积分项 */
    for (int i = 0; i < 10; i++) {
        heater_task();
    }
    
    float output_before = heater_get_output(HEATER_HOTEND);
    printf("    After 10 iterations at 100°C target: Output=%.3f\n", output_before);
    
    /* 大幅改变目标温度 (超过 10°C 阈值) */
    heater_set_temp(HEATER_HOTEND, 200.0f);
    
    /* 执行一次 PID 计算 */
    heater_task();
    
    float output_after = heater_get_output(HEATER_HOTEND);
    printf("    After changing target to 200°C: Output=%.3f\n", output_after);
    
    /* 输出应该仍然在有效范围内 */
    TEST_ASSERT(output_after >= 0.0f && output_after <= 1.0f, 
                "output should be in valid range after target change");
    
    return 1;
}

/**
 * @brief   测试热床 PID 控制
 * @note    验收标准: 3.2.2 - 支持 Kp/Ki/Kd 配置
 */
static int
test_bed_pid_control(void)
{
    test_reset_all();
    heater_init();
    
    /* 设置热床目标温度为 60°C */
    heater_set_temp(HEATER_BED, 60.0f);
    
    /* 设置热床当前温度为 30°C */
    test_set_adc_value(1, 2804);  /* 约 30°C */
    
    /* 执行 PID 计算 */
    heater_task();
    
    /* 获取热床 PID 输出 */
    float output = heater_get_output(HEATER_BED);
    
    /* 验证输出在有效范围内 */
    TEST_ASSERT(output >= 0.0f && output <= 1.0f, "bed output should be in valid range");
    
    /* 当温度低于目标时，输出应该大于 0 */
    TEST_ASSERT(output > 0.0f, "bed output should be > 0 when temp < target");
    
    printf("    Bed Target=60°C, Current=30°C -> Output=%.3f\n", output);
    
    return 1;
}

/**
 * @brief   测试 heater_get_output 函数
 */
static int
test_heater_get_output(void)
{
    test_reset_all();
    heater_init();
    
    /* 测试有效 ID */
    float output = heater_get_output(HEATER_HOTEND);
    TEST_ASSERT(output >= -0.01f, "valid ID should return valid output");
    
    output = heater_get_output(HEATER_BED);
    TEST_ASSERT(output >= -0.01f, "valid ID should return valid output");
    
    /* 测试无效 ID */
    output = heater_get_output(HEATER_COUNT);
    TEST_ASSERT_FLOAT_EQ(output, -1.0f, 0.01f, "invalid ID should return -1");
    
    output = heater_get_output((heater_id_t)99);
    TEST_ASSERT_FLOAT_EQ(output, -1.0f, 0.01f, "out of range ID should return -1");
    
    return 1;
}

/* ========== 主函数 ========== */

int
main(void)
{
    printf("========================================\n");
    printf("  Heater Temperature & PID Control Tests\n");
    printf("  验收标准: 3.1.1 - 3.1.3 (温度读取)\n");
    printf("            3.2.1 - 3.2.3 (PID 控制)\n");
    printf("            3.3.1, 3.3.2 (加热器控制)\n");
    printf("========================================\n\n");
    
    /* 运行测试 */
    printf("--- Initialization Tests ---\n");
    RUN_TEST(test_heater_init);
    RUN_TEST(test_invalid_heater_id);
    RUN_TEST(test_multiple_init);
    
    printf("\n--- NTC Temperature Conversion Tests ---\n");
    RUN_TEST(test_ntc_room_temperature);
    RUN_TEST(test_ntc_high_temperature);
    RUN_TEST(test_ntc_low_temperature);
    RUN_TEST(test_ntc_interpolation);
    RUN_TEST(test_temperature_bounds);
    
    printf("\n--- Heater Functionality Tests ---\n");
    RUN_TEST(test_bed_temperature);
    RUN_TEST(test_target_temperature);
    RUN_TEST(test_is_at_target);
    RUN_TEST(test_heater_task);
    
    printf("\n--- PID Control Tests (验收标准 3.2.1-3.2.3) ---\n");
    RUN_TEST(test_pid_output_range);
    RUN_TEST(test_pid_zero_target);
    RUN_TEST(test_pid_overshoot);
    RUN_TEST(test_pid_near_target);
    RUN_TEST(test_pid_reset_on_large_change);
    RUN_TEST(test_bed_pid_control);
    RUN_TEST(test_heater_get_output);
    
    printf("\n--- PWM Control Tests (验收标准 3.3.1, 3.3.2) ---\n");
    RUN_TEST(test_pwm_enable_disable);
    
    /* 输出结果 */
    printf("\n========================================\n");
    printf("  Test Results\n");
    printf("========================================\n");
    printf("Total:  %d\n", g_tests_run);
    printf("Passed: %d\n", g_tests_passed);
    printf("Failed: %d\n", g_tests_failed);
    printf("========================================\n");
    
    return (g_tests_failed > 0) ? 1 : 0;
}
