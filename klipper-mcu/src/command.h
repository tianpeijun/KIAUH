/**
 * @file    command.h
 * @brief   命令处理接口
 * 
 * 复用自 Klipper src/command.h
 * 提供命令注册和分发功能
 */

#ifndef COMMAND_H
#define COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* ========== 命令类型定义 ========== */

/* 命令 ID 类型 */
typedef uint8_t cmd_id_t;

/* 命令参数结构 */
typedef struct {
    const uint8_t* data;        /* 参数数据指针 */
    size_t len;                 /* 参数数据长度 */
} cmd_args_t;

/* 命令处理函数类型 */
typedef void (*cmd_handler_fn_t)(const cmd_args_t* args);

/* 命令描述结构 */
typedef struct {
    cmd_id_t id;                /* 命令 ID */
    const char* name;           /* 命令名称 */
    cmd_handler_fn_t handler;   /* 处理函数 */
} cmd_desc_t;

/* ========== 响应缓冲区 ========== */

/* 响应缓冲区大小 */
#define CMD_RESPONSE_MAX_SIZE   256

/* ========== 命令接口 ========== */

/**
 * @brief  初始化命令处理模块
 * @retval 0 成功，负数 失败
 */
int command_init(void);

/**
 * @brief  注册命令处理函数
 * @param  desc 命令描述结构指针
 * @retval 0 成功，负数 失败
 */
int command_register(const cmd_desc_t* desc);

/**
 * @brief  处理接收到的命令
 * @param  data 命令数据
 * @param  len 数据长度
 * @retval 0 成功，负数 失败
 */
int command_process(const uint8_t* data, size_t len);

/**
 * @brief  发送响应数据
 * @param  data 响应数据
 * @param  len 数据长度
 * @retval 0 成功，负数 失败
 */
int command_send_response(const uint8_t* data, size_t len);

/**
 * @brief  发送字符串响应
 * @param  str 响应字符串
 * @retval 0 成功，负数 失败
 */
int command_send_string(const char* str);

/* ========== 编码/解码工具 ========== */

/**
 * @brief  从参数中读取 uint8
 * @param  args 参数结构
 * @param  offset 偏移量
 * @retval 读取的值
 */
uint8_t cmd_decode_u8(const cmd_args_t* args, size_t offset);

/**
 * @brief  从参数中读取 uint16
 * @param  args 参数结构
 * @param  offset 偏移量
 * @retval 读取的值
 */
uint16_t cmd_decode_u16(const cmd_args_t* args, size_t offset);

/**
 * @brief  从参数中读取 uint32
 * @param  args 参数结构
 * @param  offset 偏移量
 * @retval 读取的值
 */
uint32_t cmd_decode_u32(const cmd_args_t* args, size_t offset);

/**
 * @brief  从参数中读取 int32
 * @param  args 参数结构
 * @param  offset 偏移量
 * @retval 读取的值
 */
int32_t cmd_decode_i32(const cmd_args_t* args, size_t offset);

/**
 * @brief  编码 uint8 到缓冲区
 * @param  buf 缓冲区
 * @param  value 值
 * @retval 写入的字节数
 */
size_t cmd_encode_u8(uint8_t* buf, uint8_t value);

/**
 * @brief  编码 uint16 到缓冲区
 * @param  buf 缓冲区
 * @param  value 值
 * @retval 写入的字节数
 */
size_t cmd_encode_u16(uint8_t* buf, uint16_t value);

/**
 * @brief  编码 uint32 到缓冲区
 * @param  buf 缓冲区
 * @param  value 值
 * @retval 写入的字节数
 */
size_t cmd_encode_u32(uint8_t* buf, uint32_t value);

/**
 * @brief  编码 int32 到缓冲区
 * @param  buf 缓冲区
 * @param  value 值
 * @retval 写入的字节数
 */
size_t cmd_encode_i32(uint8_t* buf, int32_t value);

/* ========== 调试输出 ========== */

/**
 * @brief  输出调试信息
 * @param  fmt 格式字符串
 * @param  ... 可变参数
 */
void command_debug(const char* fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* COMMAND_H */
