/**
 * @file    test_toolhead.c
 * @brief   运动规划器单元测试
 * 
 * 测试 toolhead.c 的基础功能:
 * - toolhead_init() 初始化
 * - toolhead_get_position() 位置获取
 * - toolhead_set_position() 位置设置
 * - trapq 运动队列初始化
 * 
 * 使用主机编译器 (gcc) 编译运行
 * 
 * 编译: make test_toolhead
 * 运行: ./test_toolhead
 * 
 * @note    验收标准: 2.3.1 - 2.3.3
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "toolhead.h"
#include "chelper/trapq.h"

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
        printf("  FAIL: %s - expected %d, got %d (line %d)\n", \
               msg, (int)(b), (int)(a), __LINE__); \
        return 0; \
    } \
} while (0)

#define TEST_ASSERT_DOUBLE_EQ(a, b, msg) do { \
    if (fabs((a) - (b)) > 0.0001) { \
        printf("  FAIL: %s - expected %.4f, got %.4f (line %d)\n", \
               msg, (double)(b), (double)(a), __LINE__); \
        return 0; \
    } \
} while (0)

#define TEST_ASSERT_FLOAT_EQ(a, b, msg) do { \
    if (fabsf((float)(a) - (float)(b)) > 0.001f) { \
        printf("  FAIL: %s - expected %.3f, got %.3f (line %d)\n", \
               msg, (float)(b), (float)(a), __LINE__); \
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

/* ========== 测试用例 ========== */

/**
 * @brief   测试 toolhead_init 初始化
 * @note    验收标准: 2.3.1 - 复用 Klipper klippy/chelper/kin_cartesian.c
 */
static int
test_toolhead_init(void)
{
    /* 初始化 toolhead */
    toolhead_init();
    
    /* 验证 trapq 已分配 */
    struct trapq *p_tq = toolhead_get_trapq();
    TEST_ASSERT(p_tq != NULL, "trapq should be allocated after init");
    
    /* 验证初始位置为零 */
    struct coord pos;
    int ret = toolhead_get_position(&pos);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "get_position should return OK");
    TEST_ASSERT_DOUBLE_EQ(pos.x, 0.0, "initial X should be 0");
    TEST_ASSERT_DOUBLE_EQ(pos.y, 0.0, "initial Y should be 0");
    TEST_ASSERT_DOUBLE_EQ(pos.z, 0.0, "initial Z should be 0");
    TEST_ASSERT_DOUBLE_EQ(pos.e, 0.0, "initial E should be 0");
    
    /* 验证初始打印时间为零 */
    double print_time = toolhead_get_print_time();
    TEST_ASSERT_DOUBLE_EQ(print_time, 0.0, "initial print_time should be 0");
    
    return 1;
}

/**
 * @brief   测试 toolhead_get_position 空指针处理
 */
static int
test_get_position_null(void)
{
    int ret = toolhead_get_position(NULL);
    TEST_ASSERT_EQ(ret, TOOLHEAD_ERR_NULL, "NULL pointer should return TOOLHEAD_ERR_NULL");
    
    return 1;
}

/**
 * @brief   测试 toolhead_set_position 空指针处理
 */
static int
test_set_position_null(void)
{
    int ret = toolhead_set_position(NULL);
    TEST_ASSERT_EQ(ret, TOOLHEAD_ERR_NULL, "NULL pointer should return TOOLHEAD_ERR_NULL");
    
    return 1;
}

/**
 * @brief   测试位置设置和获取
 * @note    验收标准: 2.3.2 - 移植 klippy/kinematics/cartesian.py 配置逻辑
 */
static int
test_position_set_get(void)
{
    struct coord set_pos, get_pos;
    int ret;
    
    /* 设置位置 */
    set_pos.x = 100.0;
    set_pos.y = 50.0;
    set_pos.z = 25.0;
    set_pos.e = 10.0;
    
    ret = toolhead_set_position(&set_pos);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "set_position should return OK");
    
    /* 获取位置并验证 */
    ret = toolhead_get_position(&get_pos);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "get_position should return OK");
    TEST_ASSERT_DOUBLE_EQ(get_pos.x, 100.0, "X should be 100");
    TEST_ASSERT_DOUBLE_EQ(get_pos.y, 50.0, "Y should be 50");
    TEST_ASSERT_DOUBLE_EQ(get_pos.z, 25.0, "Z should be 25");
    TEST_ASSERT_DOUBLE_EQ(get_pos.e, 10.0, "E should be 10");
    
    return 1;
}

/**
 * @brief   测试负数坐标
 */
static int
test_negative_position(void)
{
    struct coord set_pos, get_pos;
    int ret;
    
    /* 设置负数位置 */
    set_pos.x = -50.5;
    set_pos.y = -25.25;
    set_pos.z = -10.0;
    set_pos.e = -5.0;
    
    ret = toolhead_set_position(&set_pos);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "set_position with negative values should return OK");
    
    /* 获取位置并验证 */
    ret = toolhead_get_position(&get_pos);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "get_position should return OK");
    TEST_ASSERT_DOUBLE_EQ(get_pos.x, -50.5, "X should be -50.5");
    TEST_ASSERT_DOUBLE_EQ(get_pos.y, -25.25, "Y should be -25.25");
    TEST_ASSERT_DOUBLE_EQ(get_pos.z, -10.0, "Z should be -10");
    TEST_ASSERT_DOUBLE_EQ(get_pos.e, -5.0, "E should be -5");
    
    return 1;
}

/**
 * @brief   测试零位置
 */
static int
test_zero_position(void)
{
    struct coord set_pos, get_pos;
    int ret;
    
    /* 先设置非零位置 */
    set_pos.x = 100.0;
    set_pos.y = 100.0;
    set_pos.z = 100.0;
    set_pos.e = 100.0;
    toolhead_set_position(&set_pos);
    
    /* 设置零位置 */
    set_pos.x = 0.0;
    set_pos.y = 0.0;
    set_pos.z = 0.0;
    set_pos.e = 0.0;
    
    ret = toolhead_set_position(&set_pos);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "set_position to zero should return OK");
    
    /* 获取位置并验证 */
    ret = toolhead_get_position(&get_pos);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "get_position should return OK");
    TEST_ASSERT_DOUBLE_EQ(get_pos.x, 0.0, "X should be 0");
    TEST_ASSERT_DOUBLE_EQ(get_pos.y, 0.0, "Y should be 0");
    TEST_ASSERT_DOUBLE_EQ(get_pos.z, 0.0, "Z should be 0");
    TEST_ASSERT_DOUBLE_EQ(get_pos.e, 0.0, "E should be 0");
    
    return 1;
}

/**
 * @brief   测试大坐标值
 */
static int
test_large_position(void)
{
    struct coord set_pos, get_pos;
    int ret;
    
    /* 设置大坐标值 */
    set_pos.x = 10000.0;
    set_pos.y = 10000.0;
    set_pos.z = 5000.0;
    set_pos.e = 50000.0;
    
    ret = toolhead_set_position(&set_pos);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "set_position with large values should return OK");
    
    /* 获取位置并验证 */
    ret = toolhead_get_position(&get_pos);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "get_position should return OK");
    TEST_ASSERT_DOUBLE_EQ(get_pos.x, 10000.0, "X should be 10000");
    TEST_ASSERT_DOUBLE_EQ(get_pos.y, 10000.0, "Y should be 10000");
    TEST_ASSERT_DOUBLE_EQ(get_pos.z, 5000.0, "Z should be 5000");
    TEST_ASSERT_DOUBLE_EQ(get_pos.e, 50000.0, "E should be 50000");
    
    return 1;
}

/**
 * @brief   测试小数精度
 * @note    验收标准: 2.3.3 - 支持 steps/mm 配置
 */
static int
test_decimal_precision(void)
{
    struct coord set_pos, get_pos;
    int ret;
    
    /* 设置高精度小数位置 */
    set_pos.x = 123.456;
    set_pos.y = 78.901;
    set_pos.z = 45.678;
    set_pos.e = 12.345;
    
    ret = toolhead_set_position(&set_pos);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "set_position with decimals should return OK");
    
    /* 获取位置并验证精度 */
    ret = toolhead_get_position(&get_pos);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "get_position should return OK");
    TEST_ASSERT_DOUBLE_EQ(get_pos.x, 123.456, "X should be 123.456");
    TEST_ASSERT_DOUBLE_EQ(get_pos.y, 78.901, "Y should be 78.901");
    TEST_ASSERT_DOUBLE_EQ(get_pos.z, 45.678, "Z should be 45.678");
    TEST_ASSERT_DOUBLE_EQ(get_pos.e, 12.345, "E should be 12.345");
    
    return 1;
}

/**
 * @brief   测试 trapq 初始化
 */
static int
test_trapq_init(void)
{
    /* 重新初始化 */
    toolhead_init();
    
    /* 获取 trapq */
    struct trapq *p_tq = toolhead_get_trapq();
    TEST_ASSERT(p_tq != NULL, "trapq should not be NULL");
    
    /* 验证队列为空 */
    int has_moves = toolhead_has_moves();
    TEST_ASSERT_EQ(has_moves, 0, "trapq should be empty after init");
    
    return 1;
}

/**
 * @brief   测试运动配置获取
 */
static int
test_config_get(void)
{
    toolhead_config_t config;
    int ret;
    
    ret = toolhead_get_config(&config);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "get_config should return OK");
    
    /* 验证默认配置值 (来自 config.h) */
    TEST_ASSERT_FLOAT_EQ(config.max_velocity, 200.0f, "max_velocity should be 200");
    TEST_ASSERT_FLOAT_EQ(config.max_accel, 3000.0f, "max_accel should be 3000");
    
    return 1;
}

/**
 * @brief   测试运动配置获取空指针
 */
static int
test_config_get_null(void)
{
    int ret = toolhead_get_config(NULL);
    TEST_ASSERT_EQ(ret, TOOLHEAD_ERR_NULL, "NULL pointer should return TOOLHEAD_ERR_NULL");
    
    return 1;
}

/**
 * @brief   测试运动配置设置
 */
static int
test_config_set(void)
{
    toolhead_config_t set_config, get_config;
    int ret;
    
    /* 设置新配置 */
    set_config.max_velocity = 300.0f;
    set_config.max_accel = 5000.0f;
    set_config.max_accel_to_decel = 2500.0f;
    set_config.square_corner_velocity = 10.0f;
    
    ret = toolhead_set_config(&set_config);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "set_config should return OK");
    
    /* 获取并验证 */
    ret = toolhead_get_config(&get_config);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "get_config should return OK");
    TEST_ASSERT_FLOAT_EQ(get_config.max_velocity, 300.0f, "max_velocity should be 300");
    TEST_ASSERT_FLOAT_EQ(get_config.max_accel, 5000.0f, "max_accel should be 5000");
    TEST_ASSERT_FLOAT_EQ(get_config.max_accel_to_decel, 2500.0f, "max_accel_to_decel should be 2500");
    TEST_ASSERT_FLOAT_EQ(get_config.square_corner_velocity, 10.0f, "square_corner_velocity should be 10");
    
    return 1;
}

/**
 * @brief   测试运动配置设置空指针
 */
static int
test_config_set_null(void)
{
    int ret = toolhead_set_config(NULL);
    TEST_ASSERT_EQ(ret, TOOLHEAD_ERR_NULL, "NULL pointer should return TOOLHEAD_ERR_NULL");
    
    return 1;
}

/**
 * @brief   测试 toolhead_home 基础功能
 * @note    归零后位置应该是回退距离 (HOMING_RETRACT = 5.0)
 */
static int
test_home_basic(void)
{
    struct coord pos;
    int ret;
    
    /* 先设置非零位置 */
    pos.x = 100.0;
    pos.y = 100.0;
    pos.z = 100.0;
    pos.e = 100.0;
    toolhead_set_position(&pos);
    
    /* 归零 X 轴 */
    ret = toolhead_home(AXIS_X_MASK);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "home X should return OK");
    
    /* 验证 X 归零后回退到 5.0，其他轴不变 */
    toolhead_get_position(&pos);
    TEST_ASSERT_DOUBLE_EQ(pos.x, 5.0, "X should be 5.0 after home (retract distance)");
    TEST_ASSERT_DOUBLE_EQ(pos.y, 100.0, "Y should remain 100");
    TEST_ASSERT_DOUBLE_EQ(pos.z, 100.0, "Z should remain 100");
    
    return 1;
}

/**
 * @brief   测试多轴归零
 * @note    归零后位置应该是回退距离 (HOMING_RETRACT = 5.0)
 */
static int
test_home_multiple_axes(void)
{
    struct coord pos;
    int ret;
    
    /* 先设置非零位置 */
    pos.x = 100.0;
    pos.y = 100.0;
    pos.z = 100.0;
    pos.e = 100.0;
    toolhead_set_position(&pos);
    
    /* 归零 X 和 Y 轴 */
    ret = toolhead_home(AXIS_X_MASK | AXIS_Y_MASK);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "home X Y should return OK");
    
    /* 验证 X Y 归零后回退到 5.0，Z 不变 */
    toolhead_get_position(&pos);
    TEST_ASSERT_DOUBLE_EQ(pos.x, 5.0, "X should be 5.0 after home");
    TEST_ASSERT_DOUBLE_EQ(pos.y, 5.0, "Y should be 5.0 after home");
    TEST_ASSERT_DOUBLE_EQ(pos.z, 100.0, "Z should remain 100");
    
    return 1;
}

/**
 * @brief   测试全轴归零
 * @note    归零后位置应该是回退距离 (HOMING_RETRACT = 5.0)
 */
static int
test_home_all_axes(void)
{
    struct coord pos;
    int ret;
    
    /* 先设置非零位置 */
    pos.x = 100.0;
    pos.y = 100.0;
    pos.z = 100.0;
    pos.e = 100.0;
    toolhead_set_position(&pos);
    
    /* 归零所有轴 */
    ret = toolhead_home(AXIS_ALL_MASK);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "home all should return OK");
    
    /* 验证所有轴归零后回退到 5.0 */
    toolhead_get_position(&pos);
    TEST_ASSERT_DOUBLE_EQ(pos.x, 5.0, "X should be 5.0 after home");
    TEST_ASSERT_DOUBLE_EQ(pos.y, 5.0, "Y should be 5.0 after home");
    TEST_ASSERT_DOUBLE_EQ(pos.z, 5.0, "Z should be 5.0 after home");
    
    return 1;
}

/**
 * @brief   测试 toolhead_move 空指针处理
 */
static int
test_move_null(void)
{
    int ret = toolhead_move(NULL, 100.0f);
    TEST_ASSERT_EQ(ret, TOOLHEAD_ERR_NULL, "NULL pointer should return TOOLHEAD_ERR_NULL");
    
    return 1;
}

/**
 * @brief   测试 toolhead_move 基础功能 (位置更新)
 */
static int
test_move_basic(void)
{
    struct coord end_pos, get_pos;
    int ret;
    
    /* 初始化 */
    toolhead_init();
    
    /* 设置目标位置 */
    end_pos.x = 100.0;
    end_pos.y = 50.0;
    end_pos.z = 25.0;
    end_pos.e = 10.0;
    
    /* 执行移动 */
    ret = toolhead_move(&end_pos, 100.0f);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "move should return OK");
    
    /* 验证命令位置已更新 */
    ret = toolhead_get_position(&get_pos);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "get_position should return OK");
    TEST_ASSERT_DOUBLE_EQ(get_pos.x, 100.0, "X should be 100");
    TEST_ASSERT_DOUBLE_EQ(get_pos.y, 50.0, "Y should be 50");
    TEST_ASSERT_DOUBLE_EQ(get_pos.z, 25.0, "Z should be 25");
    TEST_ASSERT_DOUBLE_EQ(get_pos.e, 10.0, "E should be 10");
    
    return 1;
}

/**
 * @brief   测试连续运动
 * @note    验收标准: 2.2.1 - 复用 Klipper klippy/chelper/trapq.c 梯形队列
 */
static int
test_move_sequence(void)
{
    struct coord pos, get_pos;
    int ret;
    
    /* 初始化 */
    toolhead_init();
    
    /* 设置起始位置 */
    pos.x = 0.0;
    pos.y = 0.0;
    pos.z = 0.0;
    pos.e = 0.0;
    toolhead_set_position(&pos);
    
    /* 执行一系列运动 */
    pos.x = 10.0;
    ret = toolhead_move(&pos, 50.0f);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "move 1 should return OK");
    
    pos.x = 20.0;
    pos.y = 10.0;
    ret = toolhead_move(&pos, 50.0f);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "move 2 should return OK");
    
    pos.x = 30.0;
    pos.y = 20.0;
    pos.z = 5.0;
    ret = toolhead_move(&pos, 50.0f);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "move 3 should return OK");
    
    /* 验证最终位置 */
    ret = toolhead_get_position(&get_pos);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "get_position should return OK");
    TEST_ASSERT_DOUBLE_EQ(get_pos.x, 30.0, "X should be 30");
    TEST_ASSERT_DOUBLE_EQ(get_pos.y, 20.0, "Y should be 20");
    TEST_ASSERT_DOUBLE_EQ(get_pos.z, 5.0, "Z should be 5");
    
    return 1;
}

/**
 * @brief   测试零距离运动 (应该被忽略)
 */
static int
test_move_zero_distance(void)
{
    struct coord pos;
    int ret;
    
    /* 初始化 */
    toolhead_init();
    
    /* 设置位置 */
    pos.x = 50.0;
    pos.y = 50.0;
    pos.z = 50.0;
    pos.e = 0.0;
    toolhead_set_position(&pos);
    
    /* 移动到相同位置 (零距离) */
    ret = toolhead_move(&pos, 100.0f);
    TEST_ASSERT_EQ(ret, TOOLHEAD_OK, "zero distance move should return OK");
    
    return 1;
}

/**
 * @brief   测试超出限位的运动
 */
static int
test_move_out_of_bounds(void)
{
    struct coord pos;
    int ret;
    
    /* 初始化 */
    toolhead_init();
    
    /* 设置起始位置 */
    pos.x = 0.0;
    pos.y = 0.0;
    pos.z = 0.0;
    pos.e = 0.0;
    toolhead_set_position(&pos);
    
    /* 尝试移动到超出 X 最大限位的位置 */
    pos.x = 300.0;  /* X_MAX = 220 */
    ret = toolhead_move(&pos, 100.0f);
    TEST_ASSERT_EQ(ret, TOOLHEAD_ERR_LIMIT, "out of bounds move should return TOOLHEAD_ERR_LIMIT");
    
    return 1;
}

/**
 * @brief   测试 toolhead_wait_moves 和 toolhead_flush
 */
static int
test_wait_and_flush(void)
{
    struct coord pos;
    
    /* 初始化 */
    toolhead_init();
    
    /* 添加一些运动 */
    pos.x = 10.0;
    pos.y = 10.0;
    pos.z = 0.0;
    pos.e = 0.0;
    toolhead_move(&pos, 50.0f);
    
    pos.x = 20.0;
    pos.y = 20.0;
    toolhead_move(&pos, 50.0f);
    
    /* 刷新队列 */
    toolhead_flush();
    
    /* 等待运动完成 */
    toolhead_wait_moves();
    
    /* 验证队列为空 */
    int has_moves = toolhead_has_moves();
    TEST_ASSERT_EQ(has_moves, 0, "queue should be empty after wait_moves");
    
    return 1;
}

/**
 * @brief   测试运动完成回调
 */
static int s_callback_called = 0;
static void test_move_callback(void *arg)
{
    (void)arg;
    s_callback_called = 1;
}

static int
test_move_complete_callback(void)
{
    struct coord pos;
    
    /* 初始化 */
    toolhead_init();
    s_callback_called = 0;
    
    /* 设置回调 */
    toolhead_set_move_complete_callback(test_move_callback, NULL);
    
    /* 添加运动 */
    pos.x = 10.0;
    pos.y = 10.0;
    pos.z = 0.0;
    pos.e = 0.0;
    toolhead_move(&pos, 50.0f);
    
    /* 等待完成 */
    toolhead_wait_moves();
    
    /* 验证回调被调用 */
    TEST_ASSERT_EQ(s_callback_called, 1, "callback should be called after wait_moves");
    
    return 1;
}

/**
 * @brief   测试重复初始化
 */
static int
test_reinit(void)
{
    struct coord pos;
    
    /* 第一次初始化 */
    toolhead_init();
    
    /* 设置位置 */
    pos.x = 100.0;
    pos.y = 100.0;
    pos.z = 100.0;
    pos.e = 100.0;
    toolhead_set_position(&pos);
    
    /* 再次初始化 (应该被忽略) */
    toolhead_init();
    
    /* 验证位置保持不变 */
    toolhead_get_position(&pos);
    TEST_ASSERT_DOUBLE_EQ(pos.x, 100.0, "X should remain 100 after reinit");
    TEST_ASSERT_DOUBLE_EQ(pos.y, 100.0, "Y should remain 100 after reinit");
    
    return 1;
}

/* ========== 主函数 ========== */

int
main(void)
{
    printf("========================================\n");
    printf("  Toolhead Unit Tests\n");
    printf("  验收标准: 2.3.1 - 2.3.3\n");
    printf("========================================\n\n");
    
    /* 初始化 */
    toolhead_init();
    
    /* 运行初始化测试 */
    printf("--- Initialization Tests ---\n");
    RUN_TEST(test_toolhead_init);
    RUN_TEST(test_trapq_init);
    
    /* 运行位置测试 */
    printf("\n--- Position Tests ---\n");
    RUN_TEST(test_get_position_null);
    RUN_TEST(test_set_position_null);
    RUN_TEST(test_position_set_get);
    RUN_TEST(test_negative_position);
    RUN_TEST(test_zero_position);
    RUN_TEST(test_large_position);
    RUN_TEST(test_decimal_precision);
    
    /* 运行配置测试 */
    printf("\n--- Configuration Tests ---\n");
    RUN_TEST(test_config_get);
    RUN_TEST(test_config_get_null);
    RUN_TEST(test_config_set);
    RUN_TEST(test_config_set_null);
    
    /* 运行归零测试 */
    printf("\n--- Homing Tests ---\n");
    RUN_TEST(test_home_basic);
    RUN_TEST(test_home_multiple_axes);
    RUN_TEST(test_home_all_axes);
    
    /* 运行运动测试 */
    printf("\n--- Move Tests ---\n");
    RUN_TEST(test_move_null);
    RUN_TEST(test_move_basic);
    RUN_TEST(test_move_sequence);
    RUN_TEST(test_move_zero_distance);
    RUN_TEST(test_move_out_of_bounds);
    RUN_TEST(test_wait_and_flush);
    RUN_TEST(test_move_complete_callback);
    
    /* 运行其他测试 */
    printf("\n--- Other Tests ---\n");
    RUN_TEST(test_reinit);
    
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
