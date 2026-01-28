/**
 * @file    gcode.h
 * @brief   G-code 解析器接口
 * 
 * Klipper MCU 移植项目 - G-code 解析模块
 * 移植自 klippy/gcode.py，简化为只支持 MVP 指令
 * 
 * 支持的指令:
 * - G0/G1: 直线运动 (X Y Z E F)
 * - G28: 归零 (X Y Z 可选)
 * - G90/G91: 绝对/相对坐标模式
 * - M104/M109: 热端温度设置/等待
 * - M106/M107: 风扇控制
 * - M114: 位置查询
 * 
 * @note    验收标准: 4.1.1 - 4.1.7
 */

#ifndef GCODE_H
#define GCODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 错误码定义 ========== */

#define GCODE_OK                0       /* 成功 */
#define GCODE_ERR_NULL          (-1)    /* 空指针 */
#define GCODE_ERR_EMPTY         (-2)    /* 空行 */
#define GCODE_ERR_COMMENT       (-3)    /* 注释行 */
#define GCODE_ERR_INVALID       (-4)    /* 无效命令 */
#define GCODE_ERR_PARAM         (-5)    /* 参数错误 */
#define GCODE_ERR_UNKNOWN       (-6)    /* 未知命令 */

/* ========== 坐标模式 ========== */

typedef enum {
    GCODE_MODE_ABSOLUTE = 0,    /* G90: 绝对坐标模式 */
    GCODE_MODE_RELATIVE         /* G91: 相对坐标模式 */
} gcode_mode_t;

/* ========== G-code 命令结构 ========== */

/**
 * @brief   G-code 命令结构体
 * 
 * 存储解析后的 G-code 命令及其参数
 * 使用位域标记参数是否存在
 */
typedef struct {
    char cmd;                   /* 命令字母 (G/M) */
    int code;                   /* 命令编号 */
    float x, y, z, e;           /* 坐标参数 */
    float f;                    /* 进给速度 (mm/min) */
    float s;                    /* S 参数 (温度/风扇) */
    uint8_t has_x : 1;          /* X 参数存在标志 */
    uint8_t has_y : 1;          /* Y 参数存在标志 */
    uint8_t has_z : 1;          /* Z 参数存在标志 */
    uint8_t has_e : 1;          /* E 参数存在标志 */
    uint8_t has_f : 1;          /* F 参数存在标志 */
    uint8_t has_s : 1;          /* S 参数存在标志 */
    uint8_t reserved : 2;       /* 保留位 */
} gcode_cmd_t;

/* ========== 公有函数声明 ========== */

/**
 * @brief   初始化 G-code 解析器
 * 
 * 初始化解析器状态:
 * - 设置默认坐标模式为绝对模式 (G90)
 * - 清零当前位置
 */
void gcode_init(void);

/**
 * @brief   解析单行 G-code
 * @param   line    输入的 G-code 行 (以 '\0' 结尾)
 * @param   p_cmd   输出的命令结构体指针
 * @retval  GCODE_OK 成功
 * @retval  GCODE_ERR_NULL 空指针
 * @retval  GCODE_ERR_EMPTY 空行
 * @retval  GCODE_ERR_COMMENT 注释行 (以 ; 开头)
 * @retval  GCODE_ERR_INVALID 无效命令格式
 * @retval  GCODE_ERR_UNKNOWN 未知命令
 * 
 * 解析 G-code 行并填充命令结构体
 * 支持的格式: G0, G1, G28, G90, G91, M104, M109, M106, M107, M114
 */
int gcode_parse_line(const char *line, gcode_cmd_t *p_cmd);

/**
 * @brief   获取当前坐标模式
 * @retval  GCODE_MODE_ABSOLUTE 绝对坐标模式
 * @retval  GCODE_MODE_RELATIVE 相对坐标模式
 */
gcode_mode_t gcode_get_mode(void);

/**
 * @brief   设置坐标模式
 * @param   mode    坐标模式
 */
void gcode_set_mode(gcode_mode_t mode);

/**
 * @brief   清空命令结构体
 * @param   p_cmd   命令结构体指针
 * 
 * 将命令结构体所有字段清零
 */
void gcode_cmd_clear(gcode_cmd_t *p_cmd);

/**
 * @brief   处理串口输入 (在主循环调用)
 * 
 * 从串口读取一行 G-code，解析并执行。
 * 非阻塞函数，如果没有完整行则立即返回。
 * 
 * @note    应在主循环中周期性调用
 */
void gcode_process(void);

/**
 * @brief   执行 G-code 命令
 * @param   p_cmd   解析后的命令结构体指针
 * @retval  GCODE_OK 成功
 * @retval  GCODE_ERR_NULL 空指针
 * @retval  GCODE_ERR_UNKNOWN 未知命令
 * 
 * 根据命令类型分发到相应的处理函数:
 * - G0/G1: 调用 toolhead_move()
 * - G28: 调用 toolhead_home()
 * - G90/G91: 设置坐标模式
 * - M104/M109: 调用 heater_set_temp()
 * - M106/M107: 调用 fan_set_speed()
 * - M114: 查询并输出当前位置
 */
int gcode_execute(const gcode_cmd_t *p_cmd);

/**
 * @brief   发送响应消息
 * @param   msg     响应消息字符串 (以 '\0' 结尾)
 * 
 * 通过串口发送响应消息给主机。
 * 自动添加换行符。
 */
void gcode_respond(const char *msg);

/**
 * @brief   发送格式化响应消息
 * @param   fmt     格式字符串 (printf 风格)
 * @param   ...     格式参数
 * 
 * 支持的格式: %d, %u, %x, %s, %c
 * 自动添加换行符。
 */
void gcode_respond_fmt(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* GCODE_H */
