/**
 * @file    stubs.c
 * @brief   测试环境的桩函数
 * 
 * 提供硬件相关函数的模拟实现，用于主机测试环境
 */

#include <stdint.h>
#include <stddef.h>

/* ========== 调度器桩函数 ========== */

static uint32_t s_mock_time = 0;

uint32_t sched_get_time(void)
{
    return s_mock_time++;
}

void sched_main(void)
{
    /* 模拟调度器主循环 - 什么都不做 */
    s_mock_time += 100;
}

int sched_init(void)
{
    s_mock_time = 0;
    return 0;
}

/* ========== 步进电机桩函数 ========== */

static int32_t s_stepper_pos[4] = {0, 0, 0, 0};
static int s_stepper_moving[4] = {0, 0, 0, 0};

int stepper_init(void)
{
    for (int i = 0; i < 4; i++) {
        s_stepper_pos[i] = 0;
        s_stepper_moving[i] = 0;
    }
    return 0;
}

void stepper_stop(int id)
{
    if (id >= 0 && id < 4) {
        s_stepper_moving[id] = 0;
    }
}

void stepper_stop_all(void)
{
    for (int i = 0; i < 4; i++) {
        s_stepper_moving[i] = 0;
    }
}

int stepper_is_moving(int id)
{
    if (id >= 0 && id < 4) {
        return s_stepper_moving[id];
    }
    return 0;
}

int32_t stepper_get_position(int id)
{
    if (id >= 0 && id < 4) {
        return s_stepper_pos[id];
    }
    return 0;
}

void stepper_set_position(int id, int32_t pos)
{
    if (id >= 0 && id < 4) {
        s_stepper_pos[id] = pos;
    }
}

/* ========== 限位开关桩函数 ========== */

static int s_endstop_triggered[3] = {0, 0, 0};
static int s_endstop_homing[3] = {0, 0, 0};
static void (*s_endstop_callback[3])(int, void*) = {NULL, NULL, NULL};
static void *s_endstop_callback_arg[3] = {NULL, NULL, NULL};

int endstop_init(void)
{
    for (int i = 0; i < 3; i++) {
        s_endstop_triggered[i] = 0;
        s_endstop_homing[i] = 0;
        s_endstop_callback[i] = NULL;
        s_endstop_callback_arg[i] = NULL;
    }
    return 0;
}

int endstop_is_triggered(int id)
{
    if (id >= 0 && id < 3) {
        /* 模拟: 在归零模式下，经过一段时间后触发 */
        if (s_endstop_homing[id]) {
            s_endstop_triggered[id] = 1;
            if (s_endstop_callback[id] != NULL) {
                s_endstop_callback[id](id, s_endstop_callback_arg[id]);
            }
        }
        return s_endstop_triggered[id];
    }
    return 0;
}

void endstop_home_start(int id)
{
    if (id >= 0 && id < 3) {
        s_endstop_homing[id] = 1;
        s_endstop_triggered[id] = 0;
    }
}

void endstop_home_end(int id)
{
    if (id >= 0 && id < 3) {
        s_endstop_homing[id] = 0;
    }
}

void endstop_set_callback(int id, void (*callback)(int, void*), void *arg)
{
    if (id >= 0 && id < 3) {
        s_endstop_callback[id] = callback;
        s_endstop_callback_arg[id] = arg;
    }
}

/* 测试辅助函数: 手动触发限位开关 */
void test_trigger_endstop(int id)
{
    if (id >= 0 && id < 3) {
        s_endstop_triggered[id] = 1;
    }
}

void test_reset_endstop(int id)
{
    if (id >= 0 && id < 3) {
        s_endstop_triggered[id] = 0;
    }
}
