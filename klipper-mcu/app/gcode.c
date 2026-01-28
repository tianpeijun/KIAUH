/**
 * @file    gcode.c
 * @brief   G-code 解析器实现
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

#include "gcode.h"
#include <stddef.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

/* 条件编译: 仅在 MCU 构建时包含串口头文件 */
#ifndef TEST_BUILD
#include "src/stm32/serial.h"
#endif

/* ========== 私有变量 ========== */

/* 当前坐标模式 */
static gcode_mode_t s_coord_mode = GCODE_MODE_ABSOLUTE;

/* 当前位置 (用于相对坐标计算和 M114 查询) */
static float s_pos_x = 0.0f;
static float s_pos_y = 0.0f;
static float s_pos_z = 0.0f;
static float s_pos_e = 0.0f;

/* 当前进给速度 (mm/min) */
static float s_feedrate = 3000.0f;

/* 行缓冲区 (仅在 MCU 构建时使用) */
#define GCODE_LINE_BUFFER_SIZE  128
#ifndef TEST_BUILD
static char s_line_buffer[GCODE_LINE_BUFFER_SIZE];
#endif

/* ========== 弱符号声明 (后续任务实现) ========== */

/*
 * 使用弱符号允许在没有实现的情况下编译通过
 * 实际功能将在后续任务中实现
 */

/* Toolhead 运动接口 */
__attribute__((weak)) int toolhead_move(float x, float y, float z, float e, float speed)
{
    (void)x; (void)y; (void)z; (void)e; (void)speed;
    return 0;  /* 默认返回成功 */
}

__attribute__((weak)) int toolhead_home(uint8_t axes_mask)
{
    (void)axes_mask;
    return 0;  /* 默认返回成功 */
}

__attribute__((weak)) void toolhead_get_position(float *p_x, float *p_y, float *p_z, float *p_e)
{
    /* 默认返回内部跟踪的位置 */
    if (p_x != NULL) { *p_x = s_pos_x; }
    if (p_y != NULL) { *p_y = s_pos_y; }
    if (p_z != NULL) { *p_z = s_pos_z; }
    if (p_e != NULL) { *p_e = s_pos_e; }
}

__attribute__((weak)) void toolhead_wait_moves(void)
{
    /* 默认空实现 */
}

/* Heater 温度接口 */
__attribute__((weak)) void heater_set_temp(int id, float temp)
{
    (void)id; (void)temp;
}

__attribute__((weak)) float heater_get_temp(int id)
{
    (void)id;
    return 0.0f;  /* 默认返回 0 */
}

__attribute__((weak)) float heater_get_target(int id)
{
    (void)id;
    return 0.0f;  /* 默认返回 0 */
}

__attribute__((weak)) int heater_is_at_target(int id)
{
    (void)id;
    return 1;  /* 默认返回已达到目标 */
}

/* Fan 风扇接口 */
__attribute__((weak)) void fan_set_speed(int id, float speed)
{
    (void)id; (void)speed;
}

__attribute__((weak)) float fan_get_speed(int id)
{
    (void)id;
    return 0.0f;  /* 默认返回 0 */
}

/* 加热器 ID 定义 */
#define HEATER_HOTEND   0
#define HEATER_BED      1

/* 风扇 ID 定义 */
#define FAN_PART        0
#define FAN_HOTEND      1

/* 归零轴掩码 */
#define HOME_X_AXIS     (1 << 0)
#define HOME_Y_AXIS     (1 << 1)
#define HOME_Z_AXIS     (1 << 2)

/* ========== 私有函数声明 ========== */

static int parse_command(const char *line, gcode_cmd_t *p_cmd);
static int parse_parameters(const char *line, gcode_cmd_t *p_cmd);
static const char *skip_whitespace(const char *str);
static int parse_float(const char *str, float *p_value, const char **p_end);
static int is_supported_gcode(char cmd, int code);

/* ========== 公有函数实现 ========== */

/**
 * @brief   初始化 G-code 解析器
 */
void
gcode_init(void)
{
    /* 设置默认坐标模式为绝对模式 */
    s_coord_mode = GCODE_MODE_ABSOLUTE;
}

/**
 * @brief   解析单行 G-code
 */
int
gcode_parse_line(const char *line, gcode_cmd_t *p_cmd)
{
    const char *p_pos;
    int ret;
    
    /* 参数检查 */
    if ((line == NULL) || (p_cmd == NULL)) {
        return GCODE_ERR_NULL;
    }
    
    /* 清空命令结构体 */
    gcode_cmd_clear(p_cmd);
    
    /* 跳过行首空白 */
    p_pos = skip_whitespace(line);
    
    /* 检查空行 */
    if ((*p_pos == '\0') || (*p_pos == '\n') || (*p_pos == '\r')) {
        return GCODE_ERR_EMPTY;
    }
    
    /* 检查注释行 (以 ; 开头) */
    if (*p_pos == ';') {
        return GCODE_ERR_COMMENT;
    }
    
    /* 解析命令字母和编号 */
    ret = parse_command(p_pos, p_cmd);
    if (ret != GCODE_OK) {
        return ret;
    }
    
    /* 检查是否为支持的命令 */
    if (!is_supported_gcode(p_cmd->cmd, p_cmd->code)) {
        return GCODE_ERR_UNKNOWN;
    }
    
    /* 解析参数 */
    ret = parse_parameters(p_pos, p_cmd);
    if (ret != GCODE_OK) {
        return ret;
    }
    
    return GCODE_OK;
}

/**
 * @brief   获取当前坐标模式
 */
gcode_mode_t
gcode_get_mode(void)
{
    return s_coord_mode;
}

/**
 * @brief   设置坐标模式
 */
void
gcode_set_mode(gcode_mode_t mode)
{
    s_coord_mode = mode;
}

/**
 * @brief   清空命令结构体
 */
void
gcode_cmd_clear(gcode_cmd_t *p_cmd)
{
    if (p_cmd != NULL) {
        memset(p_cmd, 0, sizeof(gcode_cmd_t));
    }
}

/* ========== 私有函数实现 ========== */

/**
 * @brief   跳过空白字符
 * @param   str     输入字符串
 * @retval  指向第一个非空白字符的指针
 */
static const char *
skip_whitespace(const char *str)
{
    while ((*str == ' ') || (*str == '\t')) {
        str++;
    }
    return str;
}

/**
 * @brief   解析浮点数
 * @param   str         输入字符串
 * @param   p_value     输出浮点值
 * @param   p_end       输出解析结束位置
 * @retval  0 成功，-1 失败
 * 
 * 简单的浮点数解析，支持:
 * - 整数: 123
 * - 小数: 123.456
 * - 负数: -123.456
 */
static int
parse_float(const char *str, float *p_value, const char **p_end)
{
    float result = 0.0f;
    float fraction = 0.0f;
    float divisor = 10.0f;
    int negative = 0;
    int has_digits = 0;
    int in_fraction = 0;
    
    /* 跳过空白 */
    str = skip_whitespace(str);
    
    /* 检查负号 */
    if (*str == '-') {
        negative = 1;
        str++;
    } else if (*str == '+') {
        str++;
    }
    
    /* 解析数字 */
    while (1) {
        if ((*str >= '0') && (*str <= '9')) {
            has_digits = 1;
            if (in_fraction) {
                fraction += (float)(*str - '0') / divisor;
                divisor *= 10.0f;
            } else {
                result = result * 10.0f + (float)(*str - '0');
            }
            str++;
        } else if ((*str == '.') && (!in_fraction)) {
            in_fraction = 1;
            str++;
        } else {
            break;
        }
    }
    
    /* 检查是否有有效数字 */
    if (!has_digits) {
        return -1;
    }
    
    /* 合并整数和小数部分 */
    result += fraction;
    
    /* 应用符号 */
    if (negative) {
        result = -result;
    }
    
    *p_value = result;
    if (p_end != NULL) {
        *p_end = str;
    }
    
    return 0;
}

/**
 * @brief   解析命令字母和编号
 * @param   line    输入行
 * @param   p_cmd   输出命令结构体
 * @retval  GCODE_OK 成功
 * @retval  GCODE_ERR_INVALID 无效命令格式
 */
static int
parse_command(const char *line, gcode_cmd_t *p_cmd)
{
    const char *p_pos = line;
    char cmd_char;
    int code = 0;
    
    /* 跳过空白 */
    p_pos = skip_whitespace(p_pos);
    
    /* 获取命令字母 (G 或 M) */
    cmd_char = (char)toupper((unsigned char)*p_pos);
    if ((cmd_char != 'G') && (cmd_char != 'M')) {
        return GCODE_ERR_INVALID;
    }
    p_pos++;
    
    /* 解析命令编号 */
    while ((*p_pos >= '0') && (*p_pos <= '9')) {
        code = code * 10 + (*p_pos - '0');
        p_pos++;
    }
    
    /* 存储命令 */
    p_cmd->cmd = cmd_char;
    p_cmd->code = code;
    
    return GCODE_OK;
}

/**
 * @brief   解析命令参数
 * @param   line    输入行
 * @param   p_cmd   命令结构体 (已包含命令字母和编号)
 * @retval  GCODE_OK 成功
 * @retval  GCODE_ERR_PARAM 参数错误
 * 
 * 解析 X, Y, Z, E, F, S 参数
 */
static int
parse_parameters(const char *line, gcode_cmd_t *p_cmd)
{
    const char *p_pos = line;
    char param_char;
    float value;
    
    /* 跳过命令部分 (G/M + 数字) */
    p_pos = skip_whitespace(p_pos);
    if ((*p_pos == 'G') || (*p_pos == 'g') || 
        (*p_pos == 'M') || (*p_pos == 'm')) {
        p_pos++;
        while ((*p_pos >= '0') && (*p_pos <= '9')) {
            p_pos++;
        }
    }
    
    /* 解析参数 */
    while (*p_pos != '\0') {
        /* 跳过空白 */
        p_pos = skip_whitespace(p_pos);
        
        /* 检查行尾或注释 */
        if ((*p_pos == '\0') || (*p_pos == '\n') || 
            (*p_pos == '\r') || (*p_pos == ';')) {
            break;
        }
        
        /* 获取参数字母 */
        param_char = (char)toupper((unsigned char)*p_pos);
        p_pos++;
        
        /* 解析参数值 */
        if (parse_float(p_pos, &value, &p_pos) != 0) {
            /* 某些参数可能没有值 (如 G28 X Y Z) */
            /* 对于 G28，X/Y/Z 参数不需要值 */
            if ((p_cmd->cmd == 'G') && (p_cmd->code == 28)) {
                value = 0.0f;
            } else {
                /* 跳过无效参数 */
                continue;
            }
        }
        
        /* 存储参数值 */
        switch (param_char) {
            case 'X':
                p_cmd->x = value;
                p_cmd->has_x = 1;
                break;
            case 'Y':
                p_cmd->y = value;
                p_cmd->has_y = 1;
                break;
            case 'Z':
                p_cmd->z = value;
                p_cmd->has_z = 1;
                break;
            case 'E':
                p_cmd->e = value;
                p_cmd->has_e = 1;
                break;
            case 'F':
                p_cmd->f = value;
                p_cmd->has_f = 1;
                break;
            case 'S':
                p_cmd->s = value;
                p_cmd->has_s = 1;
                break;
            default:
                /* 忽略未知参数 */
                break;
        }
    }
    
    return GCODE_OK;
}

/**
 * @brief   检查是否为支持的 G-code 命令
 * @param   cmd     命令字母 (G/M)
 * @param   code    命令编号
 * @retval  1 支持，0 不支持
 */
static int
is_supported_gcode(char cmd, int code)
{
    if (cmd == 'G') {
        switch (code) {
            case 0:     /* G0: 快速移动 */
            case 1:     /* G1: 直线插补 */
            case 28:    /* G28: 归零 */
            case 90:    /* G90: 绝对坐标 */
            case 91:    /* G91: 相对坐标 */
                return 1;
            default:
                return 0;
        }
    } else if (cmd == 'M') {
        switch (code) {
            case 104:   /* M104: 设置热端温度 */
            case 109:   /* M109: 等待热端温度 */
            case 106:   /* M106: 设置风扇速度 */
            case 107:   /* M107: 关闭风扇 */
            case 114:   /* M114: 查询位置 */
                return 1;
            default:
                return 0;
        }
    }
    
    return 0;
}

/* ========== 命令执行函数 ========== */

/**
 * @brief   处理 G0/G1 直线运动命令
 * @param   p_cmd   命令结构体
 * @retval  0 成功
 * 
 * @note    验收标准: 4.1.2 - 支持 G0/G1 (直线运动)
 */
static int
execute_g0_g1(const gcode_cmd_t *p_cmd)
{
    float target_x, target_y, target_z, target_e;
    float speed;
    
    /* 获取当前位置 */
    toolhead_get_position(&target_x, &target_y, &target_z, &target_e);
    
    /* 计算目标位置 */
    if (s_coord_mode == GCODE_MODE_ABSOLUTE) {
        /* 绝对坐标模式 */
        if (p_cmd->has_x) { target_x = p_cmd->x; }
        if (p_cmd->has_y) { target_y = p_cmd->y; }
        if (p_cmd->has_z) { target_z = p_cmd->z; }
        if (p_cmd->has_e) { target_e = p_cmd->e; }
    } else {
        /* 相对坐标模式 */
        if (p_cmd->has_x) { target_x += p_cmd->x; }
        if (p_cmd->has_y) { target_y += p_cmd->y; }
        if (p_cmd->has_z) { target_z += p_cmd->z; }
        if (p_cmd->has_e) { target_e += p_cmd->e; }
    }
    
    /* 更新进给速度 (F 参数单位是 mm/min) */
    if (p_cmd->has_f) {
        s_feedrate = p_cmd->f;
    }
    
    /* 转换为 mm/s */
    speed = s_feedrate / 60.0f;
    
    /* 执行运动 */
    toolhead_move(target_x, target_y, target_z, target_e, speed);
    
    /* 更新内部位置跟踪 */
    s_pos_x = target_x;
    s_pos_y = target_y;
    s_pos_z = target_z;
    s_pos_e = target_e;
    
    return 0;
}

/**
 * @brief   处理 G28 归零命令
 * @param   p_cmd   命令结构体
 * @retval  0 成功
 * 
 * @note    验收标准: 4.1.3 - 支持 G28 (归零)
 */
static int
execute_g28(const gcode_cmd_t *p_cmd)
{
    uint8_t axes_mask = 0;
    
    /* 确定要归零的轴 */
    if (p_cmd->has_x) {
        axes_mask |= HOME_X_AXIS;
    }
    if (p_cmd->has_y) {
        axes_mask |= HOME_Y_AXIS;
    }
    if (p_cmd->has_z) {
        axes_mask |= HOME_Z_AXIS;
    }
    
    /* 如果没有指定轴，则归零所有轴 */
    if (axes_mask == 0) {
        axes_mask = HOME_X_AXIS | HOME_Y_AXIS | HOME_Z_AXIS;
    }
    
    /* 执行归零 */
    toolhead_home(axes_mask);
    
    /* 更新内部位置跟踪 */
    if (axes_mask & HOME_X_AXIS) { s_pos_x = 0.0f; }
    if (axes_mask & HOME_Y_AXIS) { s_pos_y = 0.0f; }
    if (axes_mask & HOME_Z_AXIS) { s_pos_z = 0.0f; }
    
    return 0;
}

/**
 * @brief   处理 G90 绝对坐标模式命令
 * @retval  0 成功
 * 
 * @note    验收标准: 4.1.4 - 支持 G90/G91 (坐标模式)
 */
static int
execute_g90(void)
{
    s_coord_mode = GCODE_MODE_ABSOLUTE;
    return 0;
}

/**
 * @brief   处理 G91 相对坐标模式命令
 * @retval  0 成功
 * 
 * @note    验收标准: 4.1.4 - 支持 G90/G91 (坐标模式)
 */
static int
execute_g91(void)
{
    s_coord_mode = GCODE_MODE_RELATIVE;
    return 0;
}

/**
 * @brief   处理 M104 设置热端温度命令
 * @param   p_cmd   命令结构体
 * @retval  0 成功
 * 
 * @note    验收标准: 4.1.5 - 支持 M104/M109 (热端温度)
 */
static int
execute_m104(const gcode_cmd_t *p_cmd)
{
    if (p_cmd->has_s) {
        heater_set_temp(HEATER_HOTEND, p_cmd->s);
    }
    return 0;
}

/**
 * @brief   处理 M109 等待热端温度命令
 * @param   p_cmd   命令结构体
 * @retval  0 成功
 * 
 * @note    验收标准: 4.1.5 - 支持 M104/M109 (热端温度)
 */
static int
execute_m109(const gcode_cmd_t *p_cmd)
{
    /* 先设置目标温度 */
    if (p_cmd->has_s) {
        heater_set_temp(HEATER_HOTEND, p_cmd->s);
    }
    
    /* 等待温度达到目标 */
    /* 注意: 在实际实现中，这应该是非阻塞的 */
    /* 当前使用弱符号实现，直接返回 */
    while (!heater_is_at_target(HEATER_HOTEND)) {
        /* 在实际实现中，这里应该调用调度器 */
        /* 并周期性检查温度 */
        break;  /* 临时: 避免死循环 */
    }
    
    return 0;
}

/**
 * @brief   处理 M106 设置风扇速度命令
 * @param   p_cmd   命令结构体
 * @retval  0 成功
 * 
 * @note    验收标准: 4.1.6 - 支持 M106/M107 (风扇)
 */
static int
execute_m106(const gcode_cmd_t *p_cmd)
{
    float speed = 1.0f;  /* 默认全速 */
    
    if (p_cmd->has_s) {
        /* S 参数范围 0-255，转换为 0.0-1.0 */
        speed = p_cmd->s / 255.0f;
        if (speed > 1.0f) {
            speed = 1.0f;
        } else if (speed < 0.0f) {
            speed = 0.0f;
        }
    }
    
    fan_set_speed(FAN_PART, speed);
    return 0;
}

/**
 * @brief   处理 M107 关闭风扇命令
 * @retval  0 成功
 * 
 * @note    验收标准: 4.1.6 - 支持 M106/M107 (风扇)
 */
static int
execute_m107(void)
{
    fan_set_speed(FAN_PART, 0.0f);
    return 0;
}

/**
 * @brief   处理 M114 位置查询命令
 * @retval  0 成功
 * 
 * @note    验收标准: 4.1.7 - 支持 M114 (位置查询)
 */
static int
execute_m114(void)
{
    float x, y, z, e;
    
    /* 获取当前位置 */
    toolhead_get_position(&x, &y, &z, &e);
    
    /* 输出位置信息 */
    /* 格式: X:0.00 Y:0.00 Z:0.00 E:0.00 */
#ifndef TEST_BUILD
    serial_printf("X:%.2d.%02d Y:%.2d.%02d Z:%.2d.%02d E:%.2d.%02d\r\n",
                  (int)x, (int)((x - (int)x) * 100),
                  (int)y, (int)((y - (int)y) * 100),
                  (int)z, (int)((z - (int)z) * 100),
                  (int)e, (int)((e - (int)e) * 100));
#endif
    
    return 0;
}

/* ========== 公有函数实现 (命令执行) ========== */

/**
 * @brief   执行 G-code 命令
 */
int
gcode_execute(const gcode_cmd_t *p_cmd)
{
    int ret = GCODE_OK;
    
    /* 参数检查 */
    if (p_cmd == NULL) {
        return GCODE_ERR_NULL;
    }
    
    /* 根据命令类型分发 */
    if (p_cmd->cmd == 'G') {
        switch (p_cmd->code) {
            case 0:     /* G0: 快速移动 */
            case 1:     /* G1: 直线插补 */
                ret = execute_g0_g1(p_cmd);
                break;
                
            case 28:    /* G28: 归零 */
                ret = execute_g28(p_cmd);
                break;
                
            case 90:    /* G90: 绝对坐标 */
                ret = execute_g90();
                break;
                
            case 91:    /* G91: 相对坐标 */
                ret = execute_g91();
                break;
                
            default:
                ret = GCODE_ERR_UNKNOWN;
                break;
        }
    } else if (p_cmd->cmd == 'M') {
        switch (p_cmd->code) {
            case 104:   /* M104: 设置热端温度 */
                ret = execute_m104(p_cmd);
                break;
                
            case 109:   /* M109: 等待热端温度 */
                ret = execute_m109(p_cmd);
                break;
                
            case 106:   /* M106: 设置风扇速度 */
                ret = execute_m106(p_cmd);
                break;
                
            case 107:   /* M107: 关闭风扇 */
                ret = execute_m107();
                break;
                
            case 114:   /* M114: 查询位置 */
                ret = execute_m114();
                break;
                
            default:
                ret = GCODE_ERR_UNKNOWN;
                break;
        }
    } else {
        ret = GCODE_ERR_UNKNOWN;
    }
    
    return ret;
}

/**
 * @brief   发送响应消息
 */
void
gcode_respond(const char *msg)
{
    if (msg == NULL) {
        return;
    }
    
#ifndef TEST_BUILD
    serial_puts(msg);
    serial_puts("\r\n");
#else
    /* 测试构建时不输出 */
    (void)msg;
#endif
}

/**
 * @brief   发送格式化响应消息
 */
void
gcode_respond_fmt(const char *fmt, ...)
{
    if (fmt == NULL) {
        return;
    }
    
#ifndef TEST_BUILD
    va_list args;
    va_start(args, fmt);
    
    /* 使用简单的格式化输出 */
    /* 注意: serial_printf 不支持 va_list，需要手动处理 */
    /* 这里简化处理，直接输出格式字符串 */
    serial_puts(fmt);
    serial_puts("\r\n");
    
    va_end(args);
#else
    /* 测试构建时不输出 */
    (void)fmt;
#endif
}

/**
 * @brief   处理串口输入
 * 
 * 从串口读取一行 G-code，解析并执行。
 * 非阻塞函数，如果没有完整行则立即返回。
 */
void
gcode_process(void)
{
#ifndef TEST_BUILD
    gcode_cmd_t cmd;
    int ret;
    int line_len;
    
    /* 检查是否有完整行可用 */
    if (!serial_line_available()) {
        return;
    }
    
    /* 读取一行 */
    line_len = serial_readline(s_line_buffer, GCODE_LINE_BUFFER_SIZE);
    if (line_len <= 0) {
        return;
    }
    
    /* 解析 G-code */
    ret = gcode_parse_line(s_line_buffer, &cmd);
    
    /* 处理解析结果 */
    switch (ret) {
        case GCODE_OK:
            /* 执行命令 */
            ret = gcode_execute(&cmd);
            if (ret == GCODE_OK) {
                gcode_respond("ok");
            } else {
                gcode_respond("error: execution failed");
            }
            break;
            
        case GCODE_ERR_EMPTY:
        case GCODE_ERR_COMMENT:
            /* 空行或注释，发送 ok */
            gcode_respond("ok");
            break;
            
        case GCODE_ERR_UNKNOWN:
            gcode_respond("error: unknown command");
            break;
            
        case GCODE_ERR_INVALID:
            gcode_respond("error: invalid command");
            break;
            
        default:
            gcode_respond("error: parse error");
            break;
    }
#endif
}
