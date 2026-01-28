/**
 * @file    test_fan.c
 * @brief   风扇控制模块单元测试
 * 
 * 测试 fan.c 的功能:
 * - fan_init() 初始化
 * - fan_set_speed() 速度设置
 * - fan_get_speed() 速度获取
 * - PWM 输出验证
 * 
 * 验收标准: 3.4.1, 3.4.2
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "fan.h"

/* ========== 测试框架 ========== */

static int s_tests_run = 0;
static int s_tests_passed = 0;
static int s_tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
        return 0; \
    } \
} while (0)

#define TEST_ASSERT_FLOAT_EQ(a, b, eps, msg) do { \
    float diff = (a) - (b); \
    if (diff < 0) diff = -diff; \
    if (diff > (eps)) { \
        printf("  FAIL: %s (expected %.4f, got %.4f, line %d)\n", \
               msg, (double)(b), (double)(a), __LINE__); \
        return 0; \
    } \
} while (0)

#define RUN_TEST(test_func) do { \
    printf("Running %s...\n", #test_func); \
    s_tests_run++; \
    fan_reset_for_test(); \
    if (test_func()) { \
        printf("  PASS\n"); \
        s_tests_passed++; \
    } else { \
        s_tests_failed++; \
    } \
} while (0)

/* ========== PWM 模拟 ========== */

/* PWM 通道状态 (用于验证) */
typedef struct {
    uint8_t configured;
    uint8_t enabled;
    float duty;
    uint8_t pin;
} mock_pwm_state_t;

#define MOCK_PWM_CHANNEL_COUNT  4
static mock_pwm_state_t s_mock_pwm[MOCK_PWM_CHANNEL_COUNT];

/* PWM 模拟函数 */
int pwm_init(void)
{
    memset(s_mock_pwm, 0, sizeof(s_mock_pwm));
    return 0;
}

int pwm_config(int id, const void* config)
{
    typedef struct {
        uint8_t pin;
        uint32_t cycle_time;
        uint16_t max_value;
        uint8_t invert;
        uint8_t use_hardware;
    } pwm_config_t;
    
    const pwm_config_t* cfg = (const pwm_config_t*)config;
    
    if (id >= MOCK_PWM_CHANNEL_COUNT || config == NULL) {
        return -1;
    }
    
    s_mock_pwm[id].configured = 1;
    s_mock_pwm[id].pin = cfg->pin;
    s_mock_pwm[id].duty = 0.0f;
    s_mock_pwm[id].enabled = 0;
    
    return 0;
}

void pwm_enable(int id, int enable)
{
    if (id >= MOCK_PWM_CHANNEL_COUNT) {
        return;
    }
    s_mock_pwm[id].enabled = enable ? 1 : 0;
}

void pwm_set_duty(int id, float duty)
{
    if (id >= MOCK_PWM_CHANNEL_COUNT) {
        return;
    }
    s_mock_pwm[id].duty = duty;
}

float pwm_get_duty(int id)
{
    if (id >= MOCK_PWM_CHANNEL_COUNT) {
        return -1.0f;
    }
    return s_mock_pwm[id].duty;
}

/* GPIO 模拟函数 */
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

/* ========== 测试用例 ========== */

/**
 * @brief   测试风扇初始化
 */
static int
test_fan_init(void)
{
    /* 初始化风扇模块 */
    fan_init();
    
    /* 验证 PWM 通道已配置 */
    /* PWM_CHANNEL_FAN_PART = 2, PWM_CHANNEL_FAN_HOTEND = 3 */
    TEST_ASSERT(s_mock_pwm[2].configured == 1, "FAN_PART PWM should be configured");
    TEST_ASSERT(s_mock_pwm[3].configured == 1, "FAN_HOTEND PWM should be configured");
    
    /* 验证初始速度为 0 */
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 0.0f, 0.001f, 
                         "FAN_PART initial speed should be 0");
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_HOTEND), 0.0f, 0.001f, 
                         "FAN_HOTEND initial speed should be 0");
    
    return 1;
}

/**
 * @brief   测试设置风扇速度
 */
static int
test_fan_set_speed(void)
{
    fan_init();
    
    /* 设置 FAN_PART 速度为 50% */
    fan_set_speed(FAN_PART, 0.5f);
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 0.5f, 0.001f, 
                         "FAN_PART speed should be 0.5");
    TEST_ASSERT(s_mock_pwm[2].enabled == 1, "FAN_PART PWM should be enabled");
    TEST_ASSERT_FLOAT_EQ(s_mock_pwm[2].duty, 0.5f, 0.001f, 
                         "FAN_PART PWM duty should be 0.5");
    
    /* 设置 FAN_HOTEND 速度为 100% */
    fan_set_speed(FAN_HOTEND, 1.0f);
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_HOTEND), 1.0f, 0.001f, 
                         "FAN_HOTEND speed should be 1.0");
    TEST_ASSERT(s_mock_pwm[3].enabled == 1, "FAN_HOTEND PWM should be enabled");
    TEST_ASSERT_FLOAT_EQ(s_mock_pwm[3].duty, 1.0f, 0.001f, 
                         "FAN_HOTEND PWM duty should be 1.0");
    
    return 1;
}

/**
 * @brief   测试关闭风扇 (速度为 0)
 */
static int
test_fan_turn_off(void)
{
    fan_init();
    
    /* 先开启风扇 */
    fan_set_speed(FAN_PART, 0.8f);
    TEST_ASSERT(s_mock_pwm[2].enabled == 1, "FAN_PART PWM should be enabled");
    
    /* 关闭风扇 */
    fan_set_speed(FAN_PART, 0.0f);
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 0.0f, 0.001f, 
                         "FAN_PART speed should be 0");
    TEST_ASSERT(s_mock_pwm[2].enabled == 0, "FAN_PART PWM should be disabled");
    TEST_ASSERT_FLOAT_EQ(s_mock_pwm[2].duty, 0.0f, 0.001f, 
                         "FAN_PART PWM duty should be 0");
    
    return 1;
}

/**
 * @brief   测试速度范围限制
 */
static int
test_fan_speed_clamping(void)
{
    fan_init();
    
    /* 测试超过最大值 */
    fan_set_speed(FAN_PART, 1.5f);
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 1.0f, 0.001f, 
                         "Speed should be clamped to 1.0");
    
    /* 测试负值 */
    fan_set_speed(FAN_PART, -0.5f);
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 0.0f, 0.001f, 
                         "Speed should be clamped to 0.0");
    
    return 1;
}

/**
 * @brief   测试无效风扇 ID
 */
static int
test_fan_invalid_id(void)
{
    fan_init();
    
    /* 设置无效 ID 不应崩溃 */
    fan_set_speed((fan_id_t)99, 0.5f);
    
    /* 获取无效 ID 应返回 -1.0 */
    float speed = fan_get_speed((fan_id_t)99);
    TEST_ASSERT_FLOAT_EQ(speed, -1.0f, 0.001f, 
                         "Invalid ID should return -1.0");
    
    return 1;
}

/**
 * @brief   测试 M106 风扇速度转换 (0-255 到 0.0-1.0)
 * 
 * 验收标准: 3.4.2 - 支持 M106/M107 指令
 */
static int
test_fan_m106_conversion(void)
{
    fan_init();
    
    /* M106 S0 -> 0.0 */
    fan_set_speed(FAN_PART, 0.0f / 255.0f);
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 0.0f, 0.001f, 
                         "M106 S0 should be 0.0");
    
    /* M106 S127 -> ~0.498 */
    fan_set_speed(FAN_PART, 127.0f / 255.0f);
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 127.0f / 255.0f, 0.001f, 
                         "M106 S127 should be ~0.498");
    
    /* M106 S255 -> 1.0 */
    fan_set_speed(FAN_PART, 255.0f / 255.0f);
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 1.0f, 0.001f, 
                         "M106 S255 should be 1.0");
    
    return 1;
}

/**
 * @brief   测试 M107 关闭风扇
 * 
 * 验收标准: 3.4.2 - 支持 M106/M107 指令
 */
static int
test_fan_m107(void)
{
    fan_init();
    
    /* 先开启风扇 */
    fan_set_speed(FAN_PART, 1.0f);
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 1.0f, 0.001f, 
                         "Fan should be at full speed");
    
    /* M107 关闭风扇 (设置速度为 0) */
    fan_set_speed(FAN_PART, 0.0f);
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 0.0f, 0.001f, 
                         "M107 should turn off fan");
    TEST_ASSERT(s_mock_pwm[2].enabled == 0, "PWM should be disabled after M107");
    
    return 1;
}

/**
 * @brief   测试多次速度变化
 */
static int
test_fan_speed_changes(void)
{
    fan_init();
    
    /* 多次改变速度 */
    fan_set_speed(FAN_PART, 0.25f);
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 0.25f, 0.001f, "Speed 1");
    
    fan_set_speed(FAN_PART, 0.75f);
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 0.75f, 0.001f, "Speed 2");
    
    fan_set_speed(FAN_PART, 0.5f);
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 0.5f, 0.001f, "Speed 3");
    
    /* 验证 PWM 占空比跟随变化 */
    TEST_ASSERT_FLOAT_EQ(s_mock_pwm[2].duty, 0.5f, 0.001f, 
                         "PWM duty should follow speed");
    
    return 1;
}

/**
 * @brief   测试自动初始化
 */
static int
test_fan_auto_init(void)
{
    /* 不调用 fan_init()，直接使用 */
    fan_set_speed(FAN_PART, 0.5f);
    
    /* 应该自动初始化并正常工作 */
    TEST_ASSERT_FLOAT_EQ(fan_get_speed(FAN_PART), 0.5f, 0.001f, 
                         "Auto-init should work");
    
    return 1;
}

/* ========== 主函数 ========== */

int
main(void)
{
    printf("========================================\n");
    printf("Fan Module Unit Tests\n");
    printf("========================================\n\n");
    
    /* 运行测试 */
    RUN_TEST(test_fan_init);
    RUN_TEST(test_fan_set_speed);
    RUN_TEST(test_fan_turn_off);
    RUN_TEST(test_fan_speed_clamping);
    RUN_TEST(test_fan_invalid_id);
    RUN_TEST(test_fan_m106_conversion);
    RUN_TEST(test_fan_m107);
    RUN_TEST(test_fan_speed_changes);
    RUN_TEST(test_fan_auto_init);
    
    /* 输出结果 */
    printf("\n========================================\n");
    printf("Results: %d/%d passed", s_tests_passed, s_tests_run);
    if (s_tests_failed > 0) {
        printf(" (%d failed)", s_tests_failed);
    }
    printf("\n========================================\n");
    
    return (s_tests_failed > 0) ? 1 : 0;
}
