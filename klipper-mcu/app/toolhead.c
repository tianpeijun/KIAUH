/**
 * @file    toolhead.c
 * @brief   运动规划器实现
 * 
 * Klipper MCU 移植项目 - 运动规划模块
 * 移植自 klippy/toolhead.py
 * 
 * 本文件实现:
 * - 任务 4.1: 基础初始化和位置管理
 * - 任务 4.2: 运动规划 (梯形加速度曲线, 前瞻队列)
 * - 任务 4.3: 归零逻辑 (限位开关检测)
 * - 任务 4.4: 辅助功能 (等待完成, 刷新队列)
 * 
 * @note    验收标准: 2.2.1-2.2.4, 2.3.1-2.3.3, 2.4.1-2.4.2
 */

#include "toolhead.h"
#include "config.h"
#include "chelper/trapq.h"
#include "chelper/itersolve.h"
#include "src/endstop.h"
#include "src/stepper.h"
#include "src/sched.h"
#include <string.h>
#include <math.h>

/* ========== 常量定义 ========== */

/** 最小运动距离 (mm) */
#define MIN_MOVE_DISTANCE       0.000001

/** 最小运动时间 (秒) */
#define MIN_MOVE_TIME           0.000001

/** 归零速度 (mm/s) */
#define HOMING_SPEED            10.0

/** 归零回退距离 (mm) */
#define HOMING_RETRACT          5.0

/** 归零超时 (秒) */
#define HOMING_TIMEOUT          30.0

/** 前瞻队列大小 */
#define LOOKAHEAD_SIZE          16

/** 轴数量 */
#define NUM_AXES                4

/* ========== 私有类型定义 ========== */

/**
 * @brief   前瞻运动段
 * 
 * 用于前瞻队列中存储待处理的运动
 */
typedef struct {
    struct coord start_pos;     /* 起始位置 */
    struct coord end_pos;       /* 结束位置 */
    double distance;            /* 运动距离 */
    double max_velocity;        /* 最大速度 */
    double max_start_v;         /* 最大起始速度 */
    double max_cruise_v;        /* 最大巡航速度 */
    double max_end_v;           /* 最大结束速度 */
    double start_v;             /* 实际起始速度 */
    double cruise_v;            /* 实际巡航速度 */
    double end_v;               /* 实际结束速度 */
    uint8_t valid;              /* 有效标志 */
} lookahead_move_t;

/**
 * @brief   归零状态
 */
typedef enum {
    HOME_STATE_IDLE = 0,        /* 空闲 */
    HOME_STATE_FAST,            /* 快速归零 */
    HOME_STATE_RETRACT,         /* 回退 */
    HOME_STATE_SLOW,            /* 慢速归零 */
    HOME_STATE_DONE,            /* 完成 */
    HOME_STATE_ERROR            /* 错误 */
} home_state_t;

/**
 * @brief   归零上下文
 */
typedef struct {
    home_state_t state;         /* 当前状态 */
    uint8_t axis_mask;          /* 归零轴掩码 */
    double start_time;          /* 开始时间 */
    uint8_t triggered;          /* 触发标志 */
} home_context_t;

/* ========== 私有变量 ========== */

/** 当前工具头位置 */
static struct coord s_current_pos;

/** 命令位置 (G-code 命令的目标位置) */
static struct coord s_commanded_pos;

/** 运动队列指针 */
static struct trapq *s_p_trapq = NULL;

/** 当前打印时间 (秒) */
static double s_print_time = 0.0;

/** 运动配置参数 */
static toolhead_config_t s_config;

/** 初始化完成标志 */
static uint8_t s_initialized = 0;

/** 前瞻队列 */
static lookahead_move_t s_lookahead[LOOKAHEAD_SIZE];
static int s_lookahead_head = 0;
static int s_lookahead_tail = 0;
static int s_lookahead_count = 0;

/** 步进运动学 */
static struct stepper_kinematics *s_steppers[NUM_AXES];

/** 归零上下文 */
static home_context_t s_home_ctx;

/** 运动完成回调 */
static toolhead_callback_fn_t s_move_complete_cb = NULL;
static void *s_move_complete_arg = NULL;

/** 轴限位 */
static double s_min_pos[NUM_AXES];
static double s_max_pos[NUM_AXES];

/** steps_per_mm 配置 */
static double s_steps_per_mm[NUM_AXES];

/* ========== 外部函数声明 ========== */

/* 笛卡尔运动学设置函数 */
extern void cartesian_stepper_setup(struct stepper_kinematics *sk, int axis,
                                    double steps_per_mm);
extern double cartesian_calc_direction(const struct coord *start,
                                       const struct coord *end,
                                       struct coord *axes_r);

/* ========== 私有函数声明 ========== */

static void config_init_defaults(void);
static void coord_clear(struct coord *p_pos);
static void coord_copy(struct coord *p_dst, const struct coord *p_src);
static double calc_move_distance(const struct coord *start,
                                 const struct coord *end);
static void calc_trapezoidal_profile(double distance, double start_v,
                                     double cruise_v, double end_v,
                                     double accel,
                                     double *accel_t, double *cruise_t,
                                     double *decel_t);
static int lookahead_push(const lookahead_move_t *move);
static int lookahead_pop(lookahead_move_t *move);
static void lookahead_flush(void);
static void lookahead_process(void);
static double calc_junction_velocity(const struct coord *prev_dir,
                                     const struct coord *next_dir,
                                     double max_v);
static void generate_steps(double flush_time);
static void home_endstop_callback(endstop_id_t id, void *arg);

/* ========== 私有函数实现 ========== */

/**
 * @brief   初始化默认配置
 */
static void
config_init_defaults(void)
{
    s_config.max_velocity = MAX_VELOCITY;
    s_config.max_accel = MAX_ACCEL;
    s_config.max_accel_to_decel = MAX_ACCEL * 0.5f;
    s_config.square_corner_velocity = 5.0f;
    
    /* 初始化轴限位 */
    s_min_pos[0] = X_MIN;
    s_min_pos[1] = Y_MIN;
    s_min_pos[2] = Z_MIN;
    s_min_pos[3] = -1e9;  /* E 轴无限位 */
    
    s_max_pos[0] = X_MAX;
    s_max_pos[1] = Y_MAX;
    s_max_pos[2] = Z_MAX;
    s_max_pos[3] = 1e9;   /* E 轴无限位 */
    
    /* 初始化 steps_per_mm */
    s_steps_per_mm[0] = STEPS_PER_MM_X;
    s_steps_per_mm[1] = STEPS_PER_MM_Y;
    s_steps_per_mm[2] = STEPS_PER_MM_Z;
    s_steps_per_mm[3] = STEPS_PER_MM_E;
}

/**
 * @brief   清零位置
 */
static void
coord_clear(struct coord *p_pos)
{
    p_pos->x = 0.0;
    p_pos->y = 0.0;
    p_pos->z = 0.0;
    p_pos->e = 0.0;
}

/**
 * @brief   复制位置
 */
static void
coord_copy(struct coord *p_dst, const struct coord *p_src)
{
    p_dst->x = p_src->x;
    p_dst->y = p_src->y;
    p_dst->z = p_src->z;
    p_dst->e = p_src->e;
}

/**
 * @brief   计算运动距离
 * @param   start   起始位置
 * @param   end     结束位置
 * @return  运动距离 (mm)
 */
static double
calc_move_distance(const struct coord *start, const struct coord *end)
{
    double dx = end->x - start->x;
    double dy = end->y - start->y;
    double dz = end->z - start->z;
    double de = end->e - start->e;
    
    return sqrt(dx * dx + dy * dy + dz * dz + de * de);
}

/**
 * @brief   计算梯形速度曲线参数
 * 
 * 根据距离、起始/结束速度和加速度计算三个阶段的时间:
 * - 加速阶段 (accel_t)
 * - 匀速阶段 (cruise_t)
 * - 减速阶段 (decel_t)
 * 
 * @param   distance    运动距离
 * @param   start_v     起始速度
 * @param   cruise_v    巡航速度
 * @param   end_v       结束速度
 * @param   accel       加速度
 * @param   accel_t     输出: 加速时间
 * @param   cruise_t    输出: 匀速时间
 * @param   decel_t     输出: 减速时间
 */
static void
calc_trapezoidal_profile(double distance, double start_v, double cruise_v,
                         double end_v, double accel,
                         double *accel_t, double *cruise_t, double *decel_t)
{
    /* 计算加速和减速距离 */
    double accel_dist = 0.0;
    double decel_dist = 0.0;
    
    if (cruise_v > start_v) {
        /* 需要加速 */
        *accel_t = (cruise_v - start_v) / accel;
        accel_dist = (start_v + cruise_v) * 0.5 * (*accel_t);
    } else {
        *accel_t = 0.0;
    }
    
    if (cruise_v > end_v) {
        /* 需要减速 */
        *decel_t = (cruise_v - end_v) / accel;
        decel_dist = (cruise_v + end_v) * 0.5 * (*decel_t);
    } else {
        *decel_t = 0.0;
    }
    
    /* 检查是否有足够距离达到巡航速度 */
    double cruise_dist = distance - accel_dist - decel_dist;
    
    if (cruise_dist < 0.0) {
        /* 距离不够，需要降低巡航速度 */
        /* 使用公式: v^2 = v0^2 + 2*a*d 求解峰值速度 */
        double peak_v_sq = (start_v * start_v + end_v * end_v) * 0.5 +
                          accel * distance;
        double peak_v = sqrt(peak_v_sq);
        
        if (peak_v < start_v) {
            peak_v = start_v;
        }
        if (peak_v < end_v) {
            peak_v = end_v;
        }
        
        /* 重新计算时间 */
        if (peak_v > start_v) {
            *accel_t = (peak_v - start_v) / accel;
        } else {
            *accel_t = 0.0;
        }
        
        if (peak_v > end_v) {
            *decel_t = (peak_v - end_v) / accel;
        } else {
            *decel_t = 0.0;
        }
        
        *cruise_t = 0.0;
    } else {
        /* 有匀速阶段 */
        *cruise_t = cruise_dist / cruise_v;
    }
}

/**
 * @brief   计算结点速度 (junction velocity)
 * 
 * 根据两个运动段的方向变化计算允许的最大结点速度。
 * 方向变化越大，结点速度越低。
 * 
 * @param   prev_dir    前一段运动方向 (归一化)
 * @param   next_dir    下一段运动方向 (归一化)
 * @param   max_v       最大允许速度
 * @return  结点速度
 */
static double
calc_junction_velocity(const struct coord *prev_dir,
                       const struct coord *next_dir,
                       double max_v)
{
    /* 计算方向向量的点积 */
    double dot = prev_dir->x * next_dir->x +
                 prev_dir->y * next_dir->y +
                 prev_dir->z * next_dir->z;
    
    /* 点积范围 [-1, 1]，1 表示同向，-1 表示反向 */
    if (dot < -0.999) {
        /* 几乎反向，需要完全停止 */
        return 0.0;
    }
    
    if (dot > 0.999) {
        /* 几乎同向，可以保持最大速度 */
        return max_v;
    }
    
    /* 计算转角的一半的正弦值 */
    /* sin(theta/2) = sqrt((1 - cos(theta)) / 2) */
    double sin_half_theta = sqrt((1.0 - dot) * 0.5);
    
    /* 使用拐角速度公式 */
    /* v_junction = sqrt(accel * deviation / sin(theta/2)) */
    /* 其中 deviation 是允许的偏差距离 */
    double deviation = s_config.square_corner_velocity * 
                       s_config.square_corner_velocity / s_config.max_accel;
    
    double junction_v = sqrt(s_config.max_accel * deviation / sin_half_theta);
    
    if (junction_v > max_v) {
        junction_v = max_v;
    }
    
    return junction_v;
}

/**
 * @brief   向前瞻队列添加运动
 */
static int
lookahead_push(const lookahead_move_t *move)
{
    if (s_lookahead_count >= LOOKAHEAD_SIZE) {
        return -1;  /* 队列满 */
    }
    
    s_lookahead[s_lookahead_tail] = *move;
    s_lookahead_tail = (s_lookahead_tail + 1) % LOOKAHEAD_SIZE;
    s_lookahead_count++;
    
    return 0;
}

/**
 * @brief   从前瞻队列取出运动
 */
static int
lookahead_pop(lookahead_move_t *move)
{
    if (s_lookahead_count == 0) {
        return -1;  /* 队列空 */
    }
    
    *move = s_lookahead[s_lookahead_head];
    s_lookahead_head = (s_lookahead_head + 1) % LOOKAHEAD_SIZE;
    s_lookahead_count--;
    
    return 0;
}

/**
 * @brief   处理前瞻队列
 * 
 * 实现前瞻算法:
 * 1. 反向遍历，计算每段运动的最大结束速度
 * 2. 正向遍历，计算每段运动的实际速度
 * 3. 将处理好的运动添加到 trapq
 */
static void
lookahead_process(void)
{
    if (s_lookahead_count == 0) {
        return;
    }
    
    /* 反向遍历: 计算最大结束速度 */
    /* 最后一段运动的结束速度为 0 (或下一段的起始速度) */
    int idx = (s_lookahead_tail - 1 + LOOKAHEAD_SIZE) % LOOKAHEAD_SIZE;
    s_lookahead[idx].max_end_v = 0.0;
    
    for (int i = s_lookahead_count - 1; i > 0; i--) {
        int prev_idx = (idx - 1 + LOOKAHEAD_SIZE) % LOOKAHEAD_SIZE;
        lookahead_move_t *curr = &s_lookahead[idx];
        lookahead_move_t *prev = &s_lookahead[prev_idx];
        
        /* 计算从当前段结束速度反推的最大起始速度 */
        /* v_start^2 = v_end^2 + 2 * a * d */
        double max_start_v_sq = curr->max_end_v * curr->max_end_v +
                                2.0 * s_config.max_accel * curr->distance;
        double max_start_v = sqrt(max_start_v_sq);
        
        if (max_start_v > curr->max_cruise_v) {
            max_start_v = curr->max_cruise_v;
        }
        
        curr->max_start_v = max_start_v;
        
        /* 计算结点速度 */
        struct coord prev_dir, curr_dir;
        cartesian_calc_direction(&prev->start_pos, &prev->end_pos, &prev_dir);
        cartesian_calc_direction(&curr->start_pos, &curr->end_pos, &curr_dir);
        
        double junction_v = calc_junction_velocity(&prev_dir, &curr_dir,
                                                   max_start_v);
        
        if (junction_v < max_start_v) {
            curr->max_start_v = junction_v;
        }
        
        prev->max_end_v = curr->max_start_v;
        
        idx = prev_idx;
    }
    
    /* 正向遍历: 计算实际速度并添加到 trapq */
    idx = s_lookahead_head;
    double prev_end_v = 0.0;
    
    for (int i = 0; i < s_lookahead_count; i++) {
        lookahead_move_t *move = &s_lookahead[idx];
        
        /* 起始速度 = 前一段的结束速度 */
        move->start_v = prev_end_v;
        
        /* 检查是否能达到巡航速度 */
        /* v_cruise^2 = v_start^2 + 2 * a * d_accel */
        double max_cruise_v_sq = move->start_v * move->start_v +
                                 2.0 * s_config.max_accel * move->distance;
        double max_cruise_v = sqrt(max_cruise_v_sq);
        
        if (max_cruise_v > move->max_cruise_v) {
            max_cruise_v = move->max_cruise_v;
        }
        
        move->cruise_v = max_cruise_v;
        
        /* 计算结束速度 */
        /* v_end^2 = v_cruise^2 - 2 * a * d_decel */
        double max_end_v_sq = move->cruise_v * move->cruise_v -
                              2.0 * s_config.max_accel_to_decel * move->distance;
        double max_end_v = (max_end_v_sq > 0.0) ? sqrt(max_end_v_sq) : 0.0;
        
        if (max_end_v > move->max_end_v) {
            max_end_v = move->max_end_v;
        }
        
        move->end_v = max_end_v;
        prev_end_v = move->end_v;
        
        idx = (idx + 1) % LOOKAHEAD_SIZE;
    }
}

/**
 * @brief   刷新前瞻队列到 trapq
 * 
 * 将前瞻队列中的运动转换为 trapq 运动段
 */
static void
lookahead_flush(void)
{
    lookahead_move_t move;
    
    while (lookahead_pop(&move) == 0) {
        /* 计算梯形曲线参数 */
        double accel_t, cruise_t, decel_t;
        calc_trapezoidal_profile(move.distance, move.start_v, move.cruise_v,
                                 move.end_v, s_config.max_accel,
                                 &accel_t, &cruise_t, &decel_t);
        
        /* 计算方向向量 */
        struct coord axes_r;
        cartesian_calc_direction(&move.start_pos, &move.end_pos, &axes_r);
        
        /* 添加到 trapq */
        trapq_append(s_p_trapq, s_print_time,
                     accel_t, cruise_t, decel_t,
                     &move.start_pos, &axes_r,
                     move.start_v, move.cruise_v, s_config.max_accel);
        
        /* 更新打印时间 */
        s_print_time += accel_t + cruise_t + decel_t;
        
        /* 更新当前位置 */
        coord_copy(&s_current_pos, &move.end_pos);
    }
}

/**
 * @brief   生成步进时序
 * @param   flush_time  刷新时间
 */
static void
generate_steps(double flush_time)
{
    for (int i = 0; i < NUM_AXES; i++) {
        if (s_steppers[i] != NULL) {
            itersolve_generate_steps(s_steppers[i], flush_time);
        }
    }
}

/**
 * @brief   归零限位开关回调
 */
static void
home_endstop_callback(endstop_id_t id, void *arg)
{
    (void)arg;
    
    /* 标记触发 */
    s_home_ctx.triggered = 1;
    
    /* 停止对应的步进电机 */
    switch (id) {
    case ENDSTOP_X:
        stepper_stop(STEPPER_X);
        break;
    case ENDSTOP_Y:
        stepper_stop(STEPPER_Y);
        break;
    case ENDSTOP_Z:
        stepper_stop(STEPPER_Z);
        break;
    default:
        break;
    }
}

/* ========== 公有函数实现 ========== */

void
toolhead_init(void)
{
    /* 防止重复初始化 */
    if (s_initialized) {
        return;
    }
    
    /* 初始化 trapq 内存池 */
    trapq_pool_init();
    
    /* 初始化 itersolve 内存池 */
    itersolve_pool_init();
    
    /* 分配运动队列 */
    s_p_trapq = trapq_alloc();
    if (s_p_trapq == NULL) {
        /* 内存池分配失败 - 严重错误 */
        return;
    }
    
    /* 初始化默认配置 */
    config_init_defaults();
    
    /* 分配并配置步进运动学 */
    for (int i = 0; i < NUM_AXES; i++) {
        s_steppers[i] = itersolve_alloc();
        if (s_steppers[i] != NULL) {
            cartesian_stepper_setup(s_steppers[i], i, s_steps_per_mm[i]);
            itersolve_set_trapq(s_steppers[i], s_p_trapq);
        }
    }
    
    /* 初始化位置为零 */
    coord_clear(&s_current_pos);
    coord_clear(&s_commanded_pos);
    
    /* 初始化打印时间 */
    s_print_time = 0.0;
    
    /* 初始化前瞻队列 */
    s_lookahead_head = 0;
    s_lookahead_tail = 0;
    s_lookahead_count = 0;
    
    /* 初始化归零上下文 */
    s_home_ctx.state = HOME_STATE_IDLE;
    s_home_ctx.axis_mask = 0;
    s_home_ctx.triggered = 0;
    
    /* 标记初始化完成 */
    s_initialized = 1;
}

int
toolhead_get_position(struct coord *p_pos)
{
    /* 参数检查 */
    if (p_pos == NULL) {
        return TOOLHEAD_ERR_NULL;
    }
    
    /* 复制命令位置 */
    coord_copy(p_pos, &s_commanded_pos);
    
    return TOOLHEAD_OK;
}

int
toolhead_set_position(const struct coord *p_pos)
{
    /* 参数检查 */
    if (p_pos == NULL) {
        return TOOLHEAD_ERR_NULL;
    }
    
    /* 设置当前位置和命令位置 */
    coord_copy(&s_current_pos, p_pos);
    coord_copy(&s_commanded_pos, p_pos);
    
    /* 更新步进运动学位置 */
    for (int i = 0; i < NUM_AXES; i++) {
        if (s_steppers[i] != NULL) {
            double pos = 0.0;
            switch (i) {
            case 0: pos = p_pos->x * s_steps_per_mm[0]; break;
            case 1: pos = p_pos->y * s_steps_per_mm[1]; break;
            case 2: pos = p_pos->z * s_steps_per_mm[2]; break;
            case 3: pos = p_pos->e * s_steps_per_mm[3]; break;
            }
            itersolve_set_position(s_steppers[i], pos);
        }
    }
    
    return TOOLHEAD_OK;
}

int
toolhead_move(const struct coord *p_end_pos, float speed)
{
    /* 参数检查 */
    if (p_end_pos == NULL) {
        return TOOLHEAD_ERR_NULL;
    }
    
    if (s_p_trapq == NULL) {
        return TOOLHEAD_ERR_QUEUE;
    }
    
    /* 计算运动距离 */
    double distance = calc_move_distance(&s_commanded_pos, p_end_pos);
    
    /* 忽略零距离运动 */
    if (distance < MIN_MOVE_DISTANCE) {
        return TOOLHEAD_OK;
    }
    
    /* 限制速度 */
    double max_v = (double)speed;
    if (max_v > s_config.max_velocity) {
        max_v = s_config.max_velocity;
    }
    if (max_v < 0.001) {
        max_v = s_config.max_velocity;
    }
    
    /* 检查位置限位 */
    if (p_end_pos->x < s_min_pos[0] || p_end_pos->x > s_max_pos[0] ||
        p_end_pos->y < s_min_pos[1] || p_end_pos->y > s_max_pos[1] ||
        p_end_pos->z < s_min_pos[2] || p_end_pos->z > s_max_pos[2]) {
        return TOOLHEAD_ERR_LIMIT;
    }
    
    /* 创建前瞻运动段 */
    lookahead_move_t move;
    coord_copy(&move.start_pos, &s_commanded_pos);
    coord_copy(&move.end_pos, p_end_pos);
    move.distance = distance;
    move.max_velocity = max_v;
    move.max_cruise_v = max_v;
    move.max_start_v = max_v;
    move.max_end_v = max_v;
    move.start_v = 0.0;
    move.cruise_v = max_v;
    move.end_v = 0.0;
    move.valid = 1;
    
    /* 添加到前瞻队列 */
    if (lookahead_push(&move) != 0) {
        /* 队列满，先刷新 */
        lookahead_process();
        lookahead_flush();
        
        /* 重试 */
        if (lookahead_push(&move) != 0) {
            return TOOLHEAD_ERR_QUEUE;
        }
    }
    
    /* 更新命令位置 */
    coord_copy(&s_commanded_pos, p_end_pos);
    
    /* 如果队列接近满，处理前瞻 */
    if (s_lookahead_count >= LOOKAHEAD_SIZE - 2) {
        lookahead_process();
        
        /* 保留最后几个运动用于前瞻 */
        while (s_lookahead_count > 2) {
            lookahead_move_t m;
            if (lookahead_pop(&m) == 0) {
                /* 计算梯形曲线参数 */
                double accel_t, cruise_t, decel_t;
                calc_trapezoidal_profile(m.distance, m.start_v, m.cruise_v,
                                         m.end_v, s_config.max_accel,
                                         &accel_t, &cruise_t, &decel_t);
                
                /* 计算方向向量 */
                struct coord axes_r;
                cartesian_calc_direction(&m.start_pos, &m.end_pos, &axes_r);
                
                /* 添加到 trapq */
                trapq_append(s_p_trapq, s_print_time,
                             accel_t, cruise_t, decel_t,
                             &m.start_pos, &axes_r,
                             m.start_v, m.cruise_v, s_config.max_accel);
                
                /* 更新打印时间 */
                s_print_time += accel_t + cruise_t + decel_t;
                
                /* 更新当前位置 */
                coord_copy(&s_current_pos, &m.end_pos);
            }
        }
        
        /* 生成步进时序 */
        generate_steps(s_print_time);
    }
    
    return TOOLHEAD_OK;
}

int
toolhead_home(uint8_t axes_mask)
{
    /* 等待当前运动完成 */
    toolhead_wait_moves();
    
    /* 设置归零回调 */
    if (axes_mask & AXIS_X_MASK) {
        endstop_set_callback(ENDSTOP_X, home_endstop_callback, NULL);
        endstop_home_start(ENDSTOP_X);
    }
    if (axes_mask & AXIS_Y_MASK) {
        endstop_set_callback(ENDSTOP_Y, home_endstop_callback, NULL);
        endstop_home_start(ENDSTOP_Y);
    }
    if (axes_mask & AXIS_Z_MASK) {
        endstop_set_callback(ENDSTOP_Z, home_endstop_callback, NULL);
        endstop_home_start(ENDSTOP_Z);
    }
    
    /* 初始化归零上下文 */
    s_home_ctx.state = HOME_STATE_FAST;
    s_home_ctx.axis_mask = axes_mask;
    s_home_ctx.triggered = 0;
    s_home_ctx.start_time = s_print_time;
    
    /* 快速归零: 向负方向运动直到触发限位 */
    struct coord home_target;
    coord_copy(&home_target, &s_commanded_pos);
    
    /* 设置归零目标 (向负方向移动最大行程) */
    if (axes_mask & AXIS_X_MASK) {
        home_target.x = s_min_pos[0] - 10.0;  /* 超出限位一点 */
    }
    if (axes_mask & AXIS_Y_MASK) {
        home_target.y = s_min_pos[1] - 10.0;
    }
    if (axes_mask & AXIS_Z_MASK) {
        home_target.z = s_min_pos[2] - 10.0;
    }
    
    /* 临时禁用限位检查，执行归零运动 */
    double saved_min[NUM_AXES];
    for (int i = 0; i < NUM_AXES; i++) {
        saved_min[i] = s_min_pos[i];
        s_min_pos[i] = -1e9;
    }
    
    /* 执行快速归零运动 */
    toolhead_move(&home_target, HOMING_SPEED * 2.0f);
    
    /* 刷新并等待 */
    toolhead_flush();
    
    /* 等待限位触发或超时 */
    double timeout_time = s_print_time + HOMING_TIMEOUT;
    while (!s_home_ctx.triggered && s_print_time < timeout_time) {
        /* 在实际硬件上，这里会等待中断 */
        /* 模拟环境下直接检查限位状态 */
        if (axes_mask & AXIS_X_MASK) {
            if (endstop_is_triggered(ENDSTOP_X)) {
                s_home_ctx.triggered = 1;
            }
        }
        if (axes_mask & AXIS_Y_MASK) {
            if (endstop_is_triggered(ENDSTOP_Y)) {
                s_home_ctx.triggered = 1;
            }
        }
        if (axes_mask & AXIS_Z_MASK) {
            if (endstop_is_triggered(ENDSTOP_Z)) {
                s_home_ctx.triggered = 1;
            }
        }
        
        /* 短暂延时 */
        sched_main();
    }
    
    /* 停止所有运动 */
    stepper_stop_all();
    
    /* 恢复限位 */
    for (int i = 0; i < NUM_AXES; i++) {
        s_min_pos[i] = saved_min[i];
    }
    
    /* 检查是否超时 */
    if (!s_home_ctx.triggered) {
        s_home_ctx.state = HOME_STATE_ERROR;
        
        /* 结束归零模式 */
        if (axes_mask & AXIS_X_MASK) {
            endstop_home_end(ENDSTOP_X);
        }
        if (axes_mask & AXIS_Y_MASK) {
            endstop_home_end(ENDSTOP_Y);
        }
        if (axes_mask & AXIS_Z_MASK) {
            endstop_home_end(ENDSTOP_Z);
        }
        
        return TOOLHEAD_ERR_HOMING;
    }
    
    /* 回退一小段距离 */
    s_home_ctx.state = HOME_STATE_RETRACT;
    s_home_ctx.triggered = 0;
    
    struct coord retract_target;
    coord_copy(&retract_target, &s_commanded_pos);
    
    if (axes_mask & AXIS_X_MASK) {
        retract_target.x = HOMING_RETRACT;
    }
    if (axes_mask & AXIS_Y_MASK) {
        retract_target.y = HOMING_RETRACT;
    }
    if (axes_mask & AXIS_Z_MASK) {
        retract_target.z = HOMING_RETRACT;
    }
    
    /* 设置当前位置为 0 */
    struct coord zero_pos;
    coord_clear(&zero_pos);
    if (axes_mask & AXIS_X_MASK) {
        s_commanded_pos.x = 0.0;
        s_current_pos.x = 0.0;
    }
    if (axes_mask & AXIS_Y_MASK) {
        s_commanded_pos.y = 0.0;
        s_current_pos.y = 0.0;
    }
    if (axes_mask & AXIS_Z_MASK) {
        s_commanded_pos.z = 0.0;
        s_current_pos.z = 0.0;
    }
    
    /* 执行回退运动 */
    toolhead_move(&retract_target, HOMING_SPEED);
    toolhead_flush();
    toolhead_wait_moves();
    
    /* 结束归零模式 */
    if (axes_mask & AXIS_X_MASK) {
        endstop_home_end(ENDSTOP_X);
    }
    if (axes_mask & AXIS_Y_MASK) {
        endstop_home_end(ENDSTOP_Y);
    }
    if (axes_mask & AXIS_Z_MASK) {
        endstop_home_end(ENDSTOP_Z);
    }
    
    s_home_ctx.state = HOME_STATE_DONE;
    
    return TOOLHEAD_OK;
}

void
toolhead_wait_moves(void)
{
    /* 处理并刷新前瞻队列 */
    lookahead_process();
    lookahead_flush();
    
    /* 生成所有步进时序 */
    generate_steps(s_print_time);
    
    /* 等待所有步进电机停止 */
    while (stepper_is_moving(STEPPER_X) ||
           stepper_is_moving(STEPPER_Y) ||
           stepper_is_moving(STEPPER_Z) ||
           stepper_is_moving(STEPPER_E)) {
        sched_main();
    }
    
    /* 同步当前位置和命令位置 */
    coord_copy(&s_current_pos, &s_commanded_pos);
    
    /* 调用完成回调 */
    if (s_move_complete_cb != NULL) {
        s_move_complete_cb(s_move_complete_arg);
    }
}

void
toolhead_flush(void)
{
    /* 处理前瞻队列 */
    lookahead_process();
    
    /* 刷新到 trapq */
    lookahead_flush();
    
    /* 生成步进时序 */
    generate_steps(s_print_time);
    
    /* 清理历史运动 */
    if (s_p_trapq != NULL) {
        trapq_finalize_moves(s_p_trapq, s_print_time);
        trapq_free_moves(s_p_trapq, s_print_time - 1.0);
    }
}

int
toolhead_has_moves(void)
{
    /* 检查前瞻队列 */
    if (s_lookahead_count > 0) {
        return 1;
    }
    
    /* 检查 trapq */
    if (s_p_trapq != NULL && trapq_has_moves(s_p_trapq)) {
        return 1;
    }
    
    return 0;
}

struct trapq *
toolhead_get_trapq(void)
{
    return s_p_trapq;
}

double
toolhead_get_print_time(void)
{
    return s_print_time;
}

int
toolhead_get_config(toolhead_config_t *p_config)
{
    /* 参数检查 */
    if (p_config == NULL) {
        return TOOLHEAD_ERR_NULL;
    }
    
    /* 复制配置 */
    memcpy(p_config, &s_config, sizeof(toolhead_config_t));
    
    return TOOLHEAD_OK;
}

int
toolhead_set_config(const toolhead_config_t *p_config)
{
    /* 参数检查 */
    if (p_config == NULL) {
        return TOOLHEAD_ERR_NULL;
    }
    
    /* 复制配置 */
    memcpy(&s_config, p_config, sizeof(toolhead_config_t));
    
    return TOOLHEAD_OK;
}

void
toolhead_set_move_complete_callback(toolhead_callback_fn_t callback, void *arg)
{
    s_move_complete_cb = callback;
    s_move_complete_arg = arg;
}
