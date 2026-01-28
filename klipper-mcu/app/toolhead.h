/**
 * @file    toolhead.h
 * @brief   运动规划器接口
 * 
 * Klipper MCU 移植项目 - 运动规划模块
 * 移植自 klippy/toolhead.py，核心是 Move 类和 LookAheadQueue
 * 
 * 功能:
 * - 位置管理 (get/set position)
 * - 运动规划 (move, home)
 * - 运动队列管理 (wait, flush)
 * 
 * @note    验收标准: 2.3.1 - 2.3.3
 */

#ifndef TOOLHEAD_H
#define TOOLHEAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "chelper/trapq.h"

/* ========== 错误码定义 ========== */

#define TOOLHEAD_OK             0       /* 成功 */
#define TOOLHEAD_ERR_NULL       (-1)    /* 空指针 */
#define TOOLHEAD_ERR_LIMIT      (-2)    /* 超出限位 */
#define TOOLHEAD_ERR_QUEUE      (-3)    /* 队列满 */
#define TOOLHEAD_ERR_BUSY       (-4)    /* 正在运动 */
#define TOOLHEAD_ERR_HOMING     (-5)    /* 归零失败 */

/* ========== 轴掩码定义 ========== */

#define AXIS_X_MASK             (1U << 0)   /* X 轴 */
#define AXIS_Y_MASK             (1U << 1)   /* Y 轴 */
#define AXIS_Z_MASK             (1U << 2)   /* Z 轴 */
#define AXIS_E_MASK             (1U << 3)   /* E 轴 (挤出机) */
#define AXIS_ALL_MASK           (AXIS_X_MASK | AXIS_Y_MASK | AXIS_Z_MASK)

/* ========== 运动参数配置 ========== */

/**
 * @brief   运动参数配置结构体
 */
typedef struct {
    float max_velocity;             /* 最大速度 mm/s */
    float max_accel;                /* 最大加速度 mm/s² */
    float max_accel_to_decel;       /* 减速加速度 mm/s² */
    float square_corner_velocity;   /* 拐角速度 mm/s */
} toolhead_config_t;

/* ========== 回调函数类型 ========== */

/**
 * @brief   运动完成回调函数类型
 * @param   arg 用户参数
 */
typedef void (*toolhead_callback_fn_t)(void *arg);

/* ========== 公有函数声明 ========== */

/**
 * @brief   初始化运动规划器
 * 
 * 初始化:
 * - trapq 运动队列内存池
 * - 分配主运动队列
 * - 设置默认运动参数
 * - 清零当前位置
 * 
 * @note    必须在使用其他 toolhead 函数前调用
 */
void toolhead_init(void);

/**
 * @brief   获取当前位置
 * @param   p_pos   输出位置结构体指针
 * @retval  TOOLHEAD_OK 成功
 * @retval  TOOLHEAD_ERR_NULL 空指针
 * 
 * 返回当前工具头位置 (X, Y, Z, E)
 */
int toolhead_get_position(struct coord *p_pos);

/**
 * @brief   设置当前位置
 * @param   p_pos   输入位置结构体指针
 * @retval  TOOLHEAD_OK 成功
 * @retval  TOOLHEAD_ERR_NULL 空指针
 * 
 * 设置当前工具头位置，不产生运动。
 * 用于归零后设置原点或 G92 命令。
 */
int toolhead_set_position(const struct coord *p_pos);

/**
 * @brief   添加直线运动
 * @param   p_end_pos   目标位置
 * @param   speed       运动速度 (mm/s)
 * @retval  TOOLHEAD_OK 成功
 * @retval  TOOLHEAD_ERR_NULL 空指针
 * @retval  TOOLHEAD_ERR_LIMIT 超出限位
 * @retval  TOOLHEAD_ERR_QUEUE 队列满
 * 
 * 添加从当前位置到目标位置的直线运动。
 * 自动计算梯形加速度曲线。
 */
int toolhead_move(const struct coord *p_end_pos, float speed);

/**
 * @brief   归零指定轴
 * @param   axes_mask   轴掩码 (AXIS_X_MASK | AXIS_Y_MASK | AXIS_Z_MASK)
 * @retval  TOOLHEAD_OK 成功
 * @retval  TOOLHEAD_ERR_HOMING 归零失败
 * 
 * 执行归零序列:
 * 1. 向限位开关方向运动
 * 2. 检测到限位后停止
 * 3. 设置位置为 0
 */
int toolhead_home(uint8_t axes_mask);

/**
 * @brief   等待所有运动完成
 * 
 * 阻塞直到运动队列为空。
 */
void toolhead_wait_moves(void);

/**
 * @brief   刷新运动队列
 * 
 * 强制执行队列中的所有运动。
 * 用于确保运动在特定时间点完成。
 */
void toolhead_flush(void);

/**
 * @brief   检查是否有待执行的运动
 * @retval  1 有待执行运动
 * @retval  0 队列为空
 */
int toolhead_has_moves(void);

/**
 * @brief   获取运动队列指针
 * @return  trapq 指针，用于高级操作
 * 
 * @note    仅供内部使用或调试
 */
struct trapq *toolhead_get_trapq(void);

/**
 * @brief   获取当前打印时间
 * @return  当前打印时间 (秒)
 */
double toolhead_get_print_time(void);

/**
 * @brief   获取运动配置
 * @param   p_config    输出配置结构体指针
 * @retval  TOOLHEAD_OK 成功
 * @retval  TOOLHEAD_ERR_NULL 空指针
 */
int toolhead_get_config(toolhead_config_t *p_config);

/**
 * @brief   设置运动配置
 * @param   p_config    输入配置结构体指针
 * @retval  TOOLHEAD_OK 成功
 * @retval  TOOLHEAD_ERR_NULL 空指针
 */
int toolhead_set_config(const toolhead_config_t *p_config);

/**
 * @brief   设置运动完成回调
 * @param   callback    回调函数
 * @param   arg         回调参数
 * 
 * 当所有运动完成时调用此回调函数。
 */
void toolhead_set_move_complete_callback(toolhead_callback_fn_t callback, void *arg);

#ifdef __cplusplus
}
#endif

#endif /* TOOLHEAD_H */
