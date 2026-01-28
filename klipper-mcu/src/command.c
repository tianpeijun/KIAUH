/**
 * @file    command.c
 * @brief   命令处理实现
 * 
 * 复用自 Klipper src/command.c
 * 提供命令注册和分发功能
 */

#include "command.h"
#include "sched.h"
#include <stdarg.h>
#include <string.h>

/* ========== 私有常量 ========== */

/* 最大注册命令数 */
#define CMD_MAX_HANDLERS        32

/* 响应缓冲区大小 */
#define CMD_TX_BUF_SIZE         256

/* ========== 私有变量 ========== */

/* 命令处理函数表 */
static cmd_desc_t s_cmd_handlers[CMD_MAX_HANDLERS];
static uint8_t s_cmd_count = 0;

/* 发送缓冲区 */
static uint8_t s_tx_buf[CMD_TX_BUF_SIZE];
static size_t s_tx_len = 0;

/* ========== HAL 接口 ========== */

/* TODO: 需要 HAL 层实现这些函数 */
extern void serial_write(const uint8_t* data, size_t len);

/* ========== 公共接口实现 ========== */

/**
 * @brief  初始化命令处理模块
 * @retval 0 成功，负数 失败
 */
int command_init(void)
{
    /* 清零命令表 */
    memset(s_cmd_handlers, 0, sizeof(s_cmd_handlers));
    s_cmd_count = 0;
    
    /* 清零发送缓冲区 */
    memset(s_tx_buf, 0, sizeof(s_tx_buf));
    s_tx_len = 0;
    
    return 0;
}

/**
 * @brief  注册命令处理函数
 * @param  desc 命令描述结构指针
 * @retval 0 成功，负数 失败
 */
int command_register(const cmd_desc_t* desc)
{
    if (desc == NULL) {
        return -1;
    }
    
    if (s_cmd_count >= CMD_MAX_HANDLERS) {
        return -2;
    }
    
    /* 检查是否已注册 */
    for (uint8_t i = 0; i < s_cmd_count; i++) {
        if (s_cmd_handlers[i].id == desc->id) {
            return -3;  /* 已存在 */
        }
    }
    
    /* 添加到命令表 */
    s_cmd_handlers[s_cmd_count] = *desc;
    s_cmd_count++;
    
    return 0;
}

/**
 * @brief  处理接收到的命令
 * @param  data 命令数据
 * @param  len 数据长度
 * @retval 0 成功，负数 失败
 */
int command_process(const uint8_t* data, size_t len)
{
    cmd_id_t cmd_id;
    cmd_args_t args;
    
    if (data == NULL || len == 0) {
        return -1;
    }
    
    /* 提取命令 ID */
    cmd_id = data[0];
    
    /* 设置参数 */
    args.data = data + 1;
    args.len = len - 1;
    
    /* 查找处理函数 */
    for (uint8_t i = 0; i < s_cmd_count; i++) {
        if (s_cmd_handlers[i].id == cmd_id) {
            if (s_cmd_handlers[i].handler != NULL) {
                s_cmd_handlers[i].handler(&args);
                return 0;
            }
        }
    }
    
    return -2;  /* 未找到处理函数 */
}

/**
 * @brief  发送响应数据
 * @param  data 响应数据
 * @param  len 数据长度
 * @retval 0 成功，负数 失败
 */
int command_send_response(const uint8_t* data, size_t len)
{
    if (data == NULL || len == 0) {
        return -1;
    }
    
    if (len > CMD_TX_BUF_SIZE) {
        return -2;
    }
    
    /* 发送数据 */
    serial_write(data, len);
    
    return 0;
}

/**
 * @brief  发送字符串响应
 * @param  str 响应字符串
 * @retval 0 成功，负数 失败
 */
int command_send_string(const char* str)
{
    if (str == NULL) {
        return -1;
    }
    
    return command_send_response((const uint8_t*)str, strlen(str));
}

/* ========== 编码/解码工具实现 ========== */

/**
 * @brief  从参数中读取 uint8
 * @param  args 参数结构
 * @param  offset 偏移量
 * @retval 读取的值
 */
uint8_t cmd_decode_u8(const cmd_args_t* args, size_t offset)
{
    if (args == NULL || args->data == NULL) {
        return 0;
    }
    
    if (offset >= args->len) {
        return 0;
    }
    
    return args->data[offset];
}

/**
 * @brief  从参数中读取 uint16（小端序）
 * @param  args 参数结构
 * @param  offset 偏移量
 * @retval 读取的值
 */
uint16_t cmd_decode_u16(const cmd_args_t* args, size_t offset)
{
    if (args == NULL || args->data == NULL) {
        return 0;
    }
    
    if (offset + 1 >= args->len) {
        return 0;
    }
    
    return (uint16_t)args->data[offset] |
           ((uint16_t)args->data[offset + 1] << 8);
}

/**
 * @brief  从参数中读取 uint32（小端序）
 * @param  args 参数结构
 * @param  offset 偏移量
 * @retval 读取的值
 */
uint32_t cmd_decode_u32(const cmd_args_t* args, size_t offset)
{
    if (args == NULL || args->data == NULL) {
        return 0;
    }
    
    if (offset + 3 >= args->len) {
        return 0;
    }
    
    return (uint32_t)args->data[offset] |
           ((uint32_t)args->data[offset + 1] << 8) |
           ((uint32_t)args->data[offset + 2] << 16) |
           ((uint32_t)args->data[offset + 3] << 24);
}

/**
 * @brief  从参数中读取 int32（小端序）
 * @param  args 参数结构
 * @param  offset 偏移量
 * @retval 读取的值
 */
int32_t cmd_decode_i32(const cmd_args_t* args, size_t offset)
{
    return (int32_t)cmd_decode_u32(args, offset);
}

/**
 * @brief  编码 uint8 到缓冲区
 * @param  buf 缓冲区
 * @param  value 值
 * @retval 写入的字节数
 */
size_t cmd_encode_u8(uint8_t* buf, uint8_t value)
{
    if (buf == NULL) {
        return 0;
    }
    
    buf[0] = value;
    return 1;
}

/**
 * @brief  编码 uint16 到缓冲区（小端序）
 * @param  buf 缓冲区
 * @param  value 值
 * @retval 写入的字节数
 */
size_t cmd_encode_u16(uint8_t* buf, uint16_t value)
{
    if (buf == NULL) {
        return 0;
    }
    
    buf[0] = (uint8_t)(value & 0xFF);
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
    return 2;
}

/**
 * @brief  编码 uint32 到缓冲区（小端序）
 * @param  buf 缓冲区
 * @param  value 值
 * @retval 写入的字节数
 */
size_t cmd_encode_u32(uint8_t* buf, uint32_t value)
{
    if (buf == NULL) {
        return 0;
    }
    
    buf[0] = (uint8_t)(value & 0xFF);
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
    buf[2] = (uint8_t)((value >> 16) & 0xFF);
    buf[3] = (uint8_t)((value >> 24) & 0xFF);
    return 4;
}

/**
 * @brief  编码 int32 到缓冲区（小端序）
 * @param  buf 缓冲区
 * @param  value 值
 * @retval 写入的字节数
 */
size_t cmd_encode_i32(uint8_t* buf, int32_t value)
{
    return cmd_encode_u32(buf, (uint32_t)value);
}

/* ========== 调试输出实现 ========== */

/**
 * @brief  输出调试信息
 * @param  fmt 格式字符串
 * @param  ... 可变参数
 */
void command_debug(const char* fmt, ...)
{
    char buf[128];
    va_list args;
    int len;
    
    if (fmt == NULL) {
        return;
    }
    
    va_start(args, fmt);
    
    /* 格式化字符串 */
    /* 注意：嵌入式环境可能需要使用简化版的 vsnprintf */
    extern int vsnprintf(char* str, size_t size, const char* format, va_list ap);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    
    va_end(args);
    
    if (len > 0) {
        command_send_string(buf);
    }
}
