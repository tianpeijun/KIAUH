# Klipper MCU 移植设计文档 (MVP)

## 1. 架构概述

### 1.1 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        应用层 (新增/移植)                         │
├─────────────────────────────────────────────────────────────────┤
│  main.c          │  gcode.c        │  toolhead.c                │
│  主循环入口       │  G-code 解析    │  运动规划                   │
├──────────────────┼─────────────────┼────────────────────────────┤
│  heater.c        │  fan.c          │  config.h                  │
│  温度 PID 控制    │  风扇控制       │  配置参数                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     运动库层 (复用 chelper)                       │
├─────────────────────────────────────────────────────────────────┤
│  trapq.c         │  itersolve.c    │  kin_cartesian.c           │
│  梯形运动队列     │  迭代求解器     │  笛卡尔运动学               │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     MCU 固件层 (复用 src/)                        │
├─────────────────────────────────────────────────────────────────┤
│  sched.c         │  stepper.c      │  endstop.c                 │
│  任务调度器       │  步进电机驱动   │  限位开关                   │
├──────────────────┼─────────────────┼────────────────────────────┤
│  adccmds.c       │  pwmcmds.c      │  command.c                 │
│  ADC 命令        │  PWM 命令       │  命令处理                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     HAL 层 (复用 src/stm32/)                      │
├─────────────────────────────────────────────────────────────────┤
│  stm32f4.c       │  gpio.c         │  adc.c                     │
│  芯片初始化       │  GPIO 驱动      │  ADC 驱动                   │
├──────────────────┼─────────────────┼────────────────────────────┤
│  serial.c        │  timer.c        │  irq.c                     │
│  串口驱动        │  定时器驱动     │  中断管理                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                        硬件 (STM32F407)                          │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 目录结构

```
klipper-mcu/
├── Makefile                    # 构建脚本
├── linker.ld                   # 链接脚本
├── config.h                    # 配置参数
│
├── src/                        # 复用 Klipper src/
│   ├── sched.c/h               # 调度器
│   ├── stepper.c/h             # 步进驱动
│   ├── endstop.c               # 限位开关
│   ├── adccmds.c               # ADC
│   ├── pwmcmds.c               # PWM
│   ├── command.c/h             # 命令处理
│   └── stm32/                  # STM32 HAL
│       ├── stm32f4.c           # 芯片初始化
│       ├── gpio.c/h            # GPIO
│       ├── adc.c               # ADC
│       └── serial.c            # 串口
│
├── chelper/                    # 复用 Klipper chelper/
│   ├── trapq.c/h               # 梯形队列
│   ├── itersolve.c/h           # 迭代求解
│   ├── kin_cartesian.c         # 笛卡尔运动学
│   └── list.h                  # 链表工具
│
├── app/                        # 新增/移植代码
│   ├── main.c                  # 主程序入口
│   ├── gcode.c/h               # G-code 解析 (移植)
│   ├── toolhead.c/h            # 运动规划 (移植)
│   ├── heater.c/h              # 温度控制 (移植)
│   └── fan.c/h                 # 风扇控制 (移植)
│
└── board/                      # 板级配置
    └── stm32f407/
        └── board.h             # 引脚定义
```

## 2. 模块设计

### 2.1 主循环 (main.c)

```c
/**
 * @file    main.c
 * @brief   程序入口和主循环
 */

#include "sched.h"
#include "gcode.h"
#include "toolhead.h"
#include "heater.h"

int main(void)
{
    /* 硬件初始化 */
    board_init();
    sched_init();
    
    /* 模块初始化 */
    toolhead_init();
    heater_init();
    gcode_init();
    
    /* 主循环 */
    for (;;) {
        sched_main();           /* 调度器主循环 */
        gcode_process();        /* 处理 G-code 输入 */
    }
    
    return 0;
}
```

### 2.2 G-code 解析器 (gcode.c)

移植自 `klippy/gcode.py`，简化为只支持 MVP 指令。

```c
/**
 * @file    gcode.h
 * @brief   G-code 解析器接口
 */

#ifndef GCODE_H
#define GCODE_H

#include <stdint.h>

/* G-code 命令结构 */
typedef struct {
    char cmd;                   /* 命令字母 (G/M) */
    int code;                   /* 命令编号 */
    float x, y, z, e;           /* 坐标参数 */
    float f;                    /* 进给速度 */
    float s;                    /* S 参数 (温度/风扇) */
    uint8_t has_x : 1;          /* 参数存在标志 */
    uint8_t has_y : 1;
    uint8_t has_z : 1;
    uint8_t has_e : 1;
    uint8_t has_f : 1;
    uint8_t has_s : 1;
} gcode_cmd_t;

/* 初始化 */
void gcode_init(void);

/* 处理输入 (在主循环调用) */
void gcode_process(void);

/* 解析单行 G-code */
int gcode_parse_line(const char* line, gcode_cmd_t* cmd);

/* 执行命令 */
int gcode_execute(const gcode_cmd_t* cmd);

/* 响应输出 */
void gcode_respond(const char* msg);

#endif /* GCODE_H */
```

**支持的指令**:
| 指令 | 功能 | 参数 |
|------|------|------|
| G0/G1 | 直线运动 | X Y Z E F |
| G28 | 归零 | X Y Z (可选) |
| G90 | 绝对坐标模式 | - |
| G91 | 相对坐标模式 | - |
| M104 | 设置热端温度 | S |
| M109 | 等待热端温度 | S |
| M106 | 设置风扇速度 | S (0-255) |
| M107 | 关闭风扇 | - |
| M114 | 查询位置 | - |

### 2.3 运动规划器 (toolhead.c)

移植自 `klippy/toolhead.py`，核心是 `Move` 类和 `LookAheadQueue`。

```c
/**
 * @file    toolhead.h
 * @brief   运动规划器接口
 */

#ifndef TOOLHEAD_H
#define TOOLHEAD_H

#include "trapq.h"

/* 运动参数配置 */
typedef struct {
    float max_velocity;         /* 最大速度 mm/s */
    float max_accel;            /* 最大加速度 mm/s² */
    float max_accel_to_decel;   /* 减速加速度 */
    float square_corner_velocity; /* 拐角速度 */
} toolhead_config_t;

/* 初始化 */
void toolhead_init(void);

/* 获取当前位置 */
void toolhead_get_position(struct coord* pos);

/* 设置当前位置 */
void toolhead_set_position(const struct coord* pos);

/* 添加运动 (G0/G1) */
int toolhead_move(const struct coord* end_pos, float speed);

/* 归零 (G28) */
int toolhead_home(uint8_t axes_mask);

/* 等待运动完成 */
void toolhead_wait_moves(void);

/* 刷新运动队列 */
void toolhead_flush(void);

#endif /* TOOLHEAD_H */
```

**核心数据结构** (复用 `chelper/trapq.h`):

```c
/* 运动段 - 来自 trapq.h */
struct move {
    double print_time, move_t;      /* 时间 */
    double start_v, half_accel;     /* 速度/加速度 */
    struct coord start_pos, axes_r; /* 起点/方向 */
    struct list_node node;
};

/* 梯形队列 */
struct trapq {
    struct list_head moves, history;
};
```

### 2.4 温度控制 (heater.c)

移植自 `klippy/extras/heaters.py`。

```c
/**
 * @file    heater.h
 * @brief   温度 PID 控制器接口
 */

#ifndef HEATER_H
#define HEATER_H

#include <stdint.h>

/* 加热器 ID */
typedef enum {
    HEATER_HOTEND = 0,          /* 热端 */
    HEATER_BED,                 /* 热床 */
    HEATER_COUNT
} heater_id_t;

/* PID 参数 */
typedef struct {
    float kp;
    float ki;
    float kd;
} pid_params_t;

/* 加热器配置 */
typedef struct {
    uint8_t adc_channel;        /* ADC 通道 */
    uint8_t pwm_pin;            /* PWM 引脚 */
    float max_power;            /* 最大功率 0-1 */
    pid_params_t pid;           /* PID 参数 */
} heater_config_t;

/* 初始化 */
void heater_init(void);

/* 设置目标温度 */
void heater_set_temp(heater_id_t id, float target);

/* 获取当前温度 */
float heater_get_temp(heater_id_t id);

/* 获取目标温度 */
float heater_get_target(heater_id_t id);

/* 检查是否达到目标温度 */
int heater_is_at_target(heater_id_t id);

/* 温度控制周期任务 (100ms 调用一次) */
void heater_task(void);

#endif /* HEATER_H */
```

**PID 算法**:

```c
/* PID 控制器状态 */
typedef struct {
    float target_temp;
    float prev_error;
    float integral;
    float output;
} pid_state_t;

/* PID 计算 */
static float pid_update(pid_state_t* state, const pid_params_t* params, 
                        float current_temp, float dt)
{
    float error = state->target_temp - current_temp;
    
    /* 积分项 */
    state->integral += error * dt;
    
    /* 微分项 */
    float derivative = (error - state->prev_error) / dt;
    state->prev_error = error;
    
    /* PID 输出 */
    state->output = params->kp * error 
                  + params->ki * state->integral 
                  + params->kd * derivative;
    
    /* 限幅 */
    if (state->output < 0.0f) {
        state->output = 0.0f;
    } else if (state->output > 1.0f) {
        state->output = 1.0f;
    }
    
    return state->output;
}
```

### 2.5 风扇控制 (fan.c)

移植自 `klippy/extras/fan.py`。

```c
/**
 * @file    fan.h
 * @brief   风扇控制接口
 */

#ifndef FAN_H
#define FAN_H

#include <stdint.h>

/* 风扇 ID */
typedef enum {
    FAN_PART = 0,               /* 模型风扇 */
    FAN_HOTEND,                 /* 热端风扇 */
    FAN_COUNT
} fan_id_t;

/* 初始化 */
void fan_init(void);

/* 设置风扇速度 (0.0 - 1.0) */
void fan_set_speed(fan_id_t id, float speed);

/* 获取风扇速度 */
float fan_get_speed(fan_id_t id);

#endif /* FAN_H */
```

### 2.6 配置管理 (config.h)

编译时配置，定义引脚和参数。

```c
/**
 * @file    config.h
 * @brief   打印机配置参数
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ========== 步进电机引脚 ========== */
#define STEPPER_X_STEP_PIN      GPIO_PA0
#define STEPPER_X_DIR_PIN       GPIO_PA1
#define STEPPER_X_ENABLE_PIN    GPIO_PA2

#define STEPPER_Y_STEP_PIN      GPIO_PA3
#define STEPPER_Y_DIR_PIN       GPIO_PA4
#define STEPPER_Y_ENABLE_PIN    GPIO_PA5

#define STEPPER_Z_STEP_PIN      GPIO_PA6
#define STEPPER_Z_DIR_PIN       GPIO_PA7
#define STEPPER_Z_ENABLE_PIN    GPIO_PB0

#define STEPPER_E_STEP_PIN      GPIO_PB1
#define STEPPER_E_DIR_PIN       GPIO_PB2
#define STEPPER_E_ENABLE_PIN    GPIO_PB3

/* ========== 限位开关引脚 ========== */
#define ENDSTOP_X_PIN           GPIO_PC0
#define ENDSTOP_Y_PIN           GPIO_PC1
#define ENDSTOP_Z_PIN           GPIO_PC2

/* ========== 温度传感器 ========== */
#define TEMP_HOTEND_ADC_CH      0
#define TEMP_BED_ADC_CH         1

/* ========== 加热器 PWM ========== */
#define HEATER_HOTEND_PIN       GPIO_PB4
#define HEATER_BED_PIN          GPIO_PB5

/* ========== 风扇 PWM ========== */
#define FAN_PART_PIN            GPIO_PB6
#define FAN_HOTEND_PIN          GPIO_PB7

/* ========== 运动参数 ========== */
#define STEPS_PER_MM_X          80.0f
#define STEPS_PER_MM_Y          80.0f
#define STEPS_PER_MM_Z          400.0f
#define STEPS_PER_MM_E          93.0f

#define MAX_VELOCITY            200.0f      /* mm/s */
#define MAX_ACCEL               3000.0f     /* mm/s² */

#define X_MIN                   0.0f
#define X_MAX                   220.0f
#define Y_MIN                   0.0f
#define Y_MAX                   220.0f
#define Z_MIN                   0.0f
#define Z_MAX                   250.0f

/* ========== PID 参数 ========== */
#define HOTEND_PID_KP           22.2f
#define HOTEND_PID_KI           1.08f
#define HOTEND_PID_KD           114.0f

#define BED_PID_KP              54.0f
#define BED_PID_KI              0.5f
#define BED_PID_KD              200.0f

/* ========== 串口配置 ========== */
#define SERIAL_BAUD             115200

#endif /* CONFIG_H */
```

## 3. 数据流

### 3.1 G-code 执行流程

```
串口输入 "G1 X100 Y50 F3000"
         │
         ▼
┌─────────────────┐
│  gcode_parse()  │  解析字符串
└────────┬────────┘
         │ gcode_cmd_t
         ▼
┌─────────────────┐
│ gcode_execute() │  分发命令
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ toolhead_move() │  添加运动
└────────┬────────┘
         │ struct move
         ▼
┌─────────────────┐
│  trapq_append() │  加入队列
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  itersolve()    │  计算步进时序
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  stepper ISR    │  输出脉冲
└─────────────────┘
```

### 3.2 温度控制流程

```
┌─────────────────┐
│  ADC 采样中断   │  10Hz
└────────┬────────┘
         │ raw ADC value
         ▼
┌─────────────────┐
│  温度转换       │  查表/公式
└────────┬────────┘
         │ float temp
         ▼
┌─────────────────┐
│  PID 计算       │  100ms 周期
└────────┬────────┘
         │ float output (0-1)
         ▼
┌─────────────────┐
│  PWM 输出       │  设置占空比
└─────────────────┘
```

## 4. 内存布局

### 4.1 RAM 分配 (< 40KB)

| 区域 | 大小 | 用途 |
|------|------|------|
| 栈 | 4KB | 函数调用栈 |
| 运动队列 | 8KB | 32 个 move 结构体 (每个 ~256B) |
| 串口缓冲 | 1KB | 接收 + 发送缓冲 |
| ADC 缓冲 | 256B | DMA 缓冲 |
| 全局变量 | ~2KB | 状态变量 |
| **预留** | ~25KB | 后续扩展 |

### 4.2 ROM 分配 (< 200KB)

| 区域 | 大小 | 用途 |
|------|------|------|
| 向量表 | 1KB | 中断向量 |
| 代码段 | ~150KB | 程序代码 |
| 只读数据 | ~10KB | 常量、温度表 |
| **预留** | ~40KB | 后续扩展 |

## 5. 接口适配

### 5.1 chelper 适配

原 `chelper` 代码为 Linux 编译，需要适配：

1. 移除 `__visible` 宏 (用于 Python FFI)
2. 替换 `malloc/free` 为静态内存池
3. 移除 Python 相关头文件

```c
/* chelper 适配层 */
#ifdef MCU_BUILD
    #define __visible
    #define malloc(size)    mem_pool_alloc(size)
    #define free(ptr)       mem_pool_free(ptr)
#endif
```

### 5.2 src/ 适配

原 `src/` 代码依赖主机通信协议，需要适配：

1. 移除 `DECL_COMMAND` 宏 (主机命令注册)
2. 直接调用函数而非通过命令协议
3. 保留调度器和步进驱动核心逻辑

## 6. 构建系统

### 6.1 Makefile

```makefile
# 工具链
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

# 目标
TARGET = klipper-mcu
MCU = STM32F407VG

# 编译选项
CFLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS += -O2 -g -Wall -Werror
CFLAGS += -DMCU_BUILD -DSTM32F407xx

# 源文件
SRCS = app/main.c app/gcode.c app/toolhead.c app/heater.c app/fan.c
SRCS += src/sched.c src/stepper.c src/endstop.c
SRCS += src/stm32/stm32f4.c src/stm32/gpio.c src/stm32/adc.c src/stm32/serial.c
SRCS += chelper/trapq.c chelper/itersolve.c chelper/kin_cartesian.c

# 构建目标
all: $(TARGET).bin

$(TARGET).elf: $(SRCS)
	$(CC) $(CFLAGS) -T linker.ld -o $@ $^

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f $(TARGET).elf $(TARGET).bin

flash: $(TARGET).bin
	st-flash write $< 0x08000000

.PHONY: all clean flash
```

## 7. 正确性属性

### 7.1 运动控制属性

| ID | 属性 | 验证方式 |
|----|------|----------|
| P1 | 步进脉冲间隔 >= 配置的最小间隔 | 示波器测量 |
| P2 | 运动速度不超过 max_velocity | 日志分析 |
| P3 | 加速度不超过 max_accel | 日志分析 |
| P4 | 限位触发时立即停止 | 功能测试 |

### 7.2 温度控制属性

| ID | 属性 | 验证方式 |
|----|------|----------|
| P5 | PID 输出在 [0, max_power] 范围内 | 单元测试 |
| P6 | 温度稳定后波动 < ±3°C | 功能测试 |
| P7 | 温度读取误差 < ±2°C | 校准测试 |

### 7.3 G-code 解析属性

| ID | 属性 | 验证方式 |
|----|------|----------|
| P8 | 有效 G-code 正确解析 | 单元测试 |
| P9 | 无效 G-code 返回错误 | 单元测试 |
| P10 | 注释行被忽略 | 单元测试 |
