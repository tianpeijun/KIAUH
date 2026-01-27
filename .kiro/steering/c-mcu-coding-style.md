---
inclusion: fileMatch
fileMatchPattern: ['**/*.c', '**/*.h', '**/*.cpp', '**/*.hpp']
---

# C & MCU 编写规范

本规范适用于 C/C++ 和嵌入式 MCU 开发，旨在提高代码可读性、可维护性和可移植性。

## 核心原则

- 低耦合，可重用，参数化，注释全
- 使用 C99 标准（某些仅支持 C98 的 IDE 除外）
- 一个 tab 等于四个空格
- 代码洁癖是好习惯

## 命名规范

### 通用规则
- 文件名：全小写
- 变量和函数：小写 + 下划线分隔，遵循 `属什么_是什么_做什么` 形式
- 宏定义：全大写 + 下划线分隔
- 避免使用单字节命名（循环变量 i, j, k 除外）
- 避免以双下划线或单下划线开头（系统保留）

### 变量命名
```c
int local_var;           /* 局部变量：小写下划线 */
int g_system_state;      /* 全局变量：g_ 前缀 */
static int s_counter;    /* 静态变量：s_ 前缀 */
int *p_buffer;           /* 指针变量：p_ 前缀 */
#define MAX_SIZE 100     /* 常量/宏：全大写 */
```

### 函数命名
```c
void uart_init(void);           /* 模块前缀 + 动作 */
int get_value(void);            /* 动词 + 名词 */
bool is_ready(void);            /* 布尔返回：is_/has_ 前缀 */
```

### 类型命名
```c
typedef struct { ... } uart_config_t;    /* 结构体类型：_t 后缀 */
typedef enum { ... } state_enum_t;       /* 枚举类型：_enum_t 后缀 */
typedef union { ... } data_union_t;      /* 联合类型：_union_t 后缀 */
typedef void (*callback_fn_t)(int);      /* 函数指针类型：_fn_t 后缀 */
```

### 互斥词组
使用成对的互斥词命名相反操作：
- add/remove, begin/end, create/destroy, insert/delete
- open/close, lock/unlock, start/stop, send/receive
- get/set, enable/disable, show/hide, min/max

## 代码格式

### 缩进与空格
```c
/* 运算符两侧加空格 */
a = b + c;

/* 逗号后加空格 */
func(a, b, c);

/* 指针星号靠近类型名 */
int* ptr;
char* str;

/* 多指针定义时星号靠近变量名 */
int *ptr1, *ptr2, *ptr3;
```

### 大括号风格（K&R）
```c
if (condition) {
    /* code */
} else {
    /* code */
}

/* 函数定义 */
void function_name(void)
{
    /* code */
}

/* switch 语句 */
switch (value) {
    case 0:
        do_something();
        break;
    default:
        break;
}
```

### 控制语句
- 总是使用大括号，即使只有一行
- if 必带 else，switch 必带 default
- 无限循环用 `for (;;)` 而非 `while (1)`
- 多条件判断每个条件用括号括起来

```c
if ((a == 1) && (b == 2)) {
    /* code */
} else {
    /* code */
}
```

### 行宽与换行
- 每行不超过 80-120 字符
- 长表达式在运算符后换行

## 函数规范

### 函数设计原则
- 一个函数只做一件事
- 局部变量不超过 5-10 个
- 嵌套层数不超过 4 层
- 避免递归，用循环替代
- 形参超过 5 个时考虑用结构体打包

### 参数检查
```c
int process_data(const uint8_t* buf, size_t len)
{
    /* 参数合法性检查 */
    if (buf == NULL) {
        return -1;
    }
    if (len == 0 || len > MAX_LEN) {
        return -2;
    }
    /* 处理逻辑 */
    return 0;
}
```

### 返回值约定
- 返回 0 表示成功
- 返回负数或正数表示错误码
- 可用枚举定义错误类型

### 函数声明
```c
void sys_init(void);                    /* 无参数明确用 void */
int sys_read(uint8_t* buf, size_t len); /* 数组参数传长度 */
```

## 变量规范

### 类型定义
使用 `<stdint.h>` 中的固定宽度类型：
```c
uint8_t, uint16_t, uint32_t
int8_t, int16_t, int32_t
```

### const 使用
```c
const uint8_t value;           /* 值不可变 */
const char* p;                 /* 指向内容不可变 */
char* const p;                 /* 指针地址不可变 */
const char* const p;           /* 都不可变 */
```

### volatile 使用
用于中断共享变量、寄存器访问：
```c
volatile uint32_t g_tick_count;
volatile uint8_t* const REG_STATUS = (uint8_t*)0x40000000;
```

### 作用域控制
- 文件内私有变量/函数用 `static` 修饰
- 公有变量声明用 `extern`
- 尽量缩小变量作用域

## 注释规范

### Doxygen 格式
```c
/**
 * @file    filename.c
 * @brief   简要描述
 * @author  作者
 * @date    日期
 */

/**
 * @brief  函数简要描述
 * @param  buf 数据缓冲区指针
 * @param  len 数据长度
 * @retval 0 成功，负数 失败
 */
int process_data(uint8_t* buf, size_t len);
```

### 注释原则
- 注释写"为什么"而非"做什么"
- 字母数字两侧空一格
- 使用 `/* */` 而非 `//`

## MCU 开发规范

### 寄存器操作
```c
#define BIT_SET(reg, bit)    ((reg) |= (1U << (bit)))
#define BIT_CLR(reg, bit)    ((reg) &= ~(1U << (bit)))
#define BIT_GET(reg, bit)    (((reg) >> (bit)) & 1U)
#define BIT_TOGGLE(reg, bit) ((reg) ^= (1U << (bit)))
```

### 中断处理
- 中断服务函数尽量简短
- 共享变量用 `volatile` 修饰
- 避免在中断中阻塞
- 关键代码段使用临界区保护
- 不要在中断中喂狗

### 外设驱动接口
```c
int xxx_init(xxx_config_t* config);
int xxx_deinit(void);
int xxx_read(uint8_t* buf, size_t len);
int xxx_write(const uint8_t* buf, size_t len);
```

### 通信外设
- 收发使用中断或 DMA
- 使用环形缓冲区
- 依时间间隔区分帧

## 安全编码

### 防御性编程
- 函数入口检查参数有效性
- 使用 `assert()` 检查假设条件
- 返回值必须检查
- 避免魔数，使用宏定义
- 避免"死等"，设置超时退出

```c
/* 避免死等 */
uint32_t timeout = 1000;
while (!flag && timeout--) {
    delay_ms(1);
}
if (timeout == 0) {
    return -ETIMEDOUT;
}
```

### 类型安全
- 使用固定宽度类型
- 注意有符号/无符号转换
- 避免隐式类型转换
- 使用 `sizeof` 而非硬编码大小

### 内存安全
- 嵌入式环境避免动态内存分配
- 使用静态分配或内存池
- 检查数组边界
- 指针使用前检查 NULL
- `malloc`/`free` 成对使用

### 关键数据保护
- 关键数据多份备份
- 使用"表决法"读取
- 周期性刷新寄存器配置

## 头文件规范

```c
#ifndef MODULE_NAME_H
#define MODULE_NAME_H

#ifdef __cplusplus
extern "C" {
#endif

/* 头文件内容 */

#ifdef __cplusplus
}
#endif

#endif /* MODULE_NAME_H */
```

### 头文件原则
- 只引用必要的头文件
- 头文件尽量无依赖
- 一个模块可多个 .c 对应一个 .h

## 宏定义规范

### 宏函数
```c
/* 所有参数和结果用括号保护 */
#define MAX(x, y)    (((x) > (y)) ? (x) : (y))
#define MIN(x, y)    (((x) < (y)) ? (x) : (y))

/* 多语句用 do-while(0) */
#define SET_POINT(p, x, y)  do { \
    (p)->px = (x);              \
    (p)->py = (y);              \
} while (0)
```

### 常用宏
```c
#define ARR_SIZE(a)     (sizeof(a) / sizeof((a)[0]))
#define UNUSED(x)       ((void)(x))
#define ROUNDUP(a, sz)  (((a) + (sz) - 1) / (sz) * (sz))
```

## 编译与调试

### 编译选项
- 开启所有警告：`-Wall -Wextra`
- 将警告视为错误：`-Werror`

### 调试宏
```c
#ifdef DEBUG
    #define DBG_PRINT(fmt, ...) printf("[%s:%d] " fmt, __FILE__, __LINE__, ##__VA_ARGS__)
#else
    #define DBG_PRINT(fmt, ...)
#endif
```

## 模块化设计

- 高内聚、低耦合
- 接口与实现分离
- 使用 `static` 限制内部函数可见性
- 避免全局变量，使用访问函数
- 功能剪裁用 `_config.h` 文件集中管理
