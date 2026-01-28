# Klipper MCU 移植任务列表 (MVP)

## 任务概览

- **总任务数**: 6 个主任务，24 个子任务
- **预计工期**: 4-6 周
- **代码量**: ~3,500 行新增 C 代码 + ~19,000 行复用代码

---

## 任务列表

### 1. 项目初始化
- [x] 1.1 创建项目目录结构
  - 创建 `klipper-mcu/` 目录
  - 创建子目录: `src/`, `src/stm32/`, `chelper/`, `app/`, `board/`
  - 创建 `config.h` 配置文件模板
  - **验证**: 目录结构符合 design.md 规范
  - **验收标准**: 5.1.1

- [x] 1.2 复制 Klipper 可复用代码
  - 从 Klipper 仓库复制 `src/sched.c`, `src/sched.h`
  - 复制 `src/stepper.c`, `src/stepper.h`
  - 复制 `src/endstop.c`, `src/endstop.h`
  - 复制 `src/adccmds.c`, `src/adccmds.h`, `src/pwmcmds.c`, `src/pwmcmds.h`
  - 复制 `src/command.c`, `src/command.h`
  - **验证**: 文件完整复制
  - **验收标准**: 1.2.1, 2.1.1, 2.4.1, 3.1.1, 3.3.1

- [x] 1.3 复制 STM32 HAL 代码
  - 复制 `src/stm32/stm32f4.c` (时钟初始化)
  - 复制 `src/stm32/gpio.c`, `src/stm32/gpio.h`
  - 创建 `src/stm32/internal.h` (内部定义)
  - 创建 `board/irq.h` (中断接口)
  - 创建 `board/misc.h` (杂项接口)
  - **验证**: HAL 文件完整
  - **验收标准**: 1.1.1, 1.1.2, 1.1.5

- [x] 1.4 复制 chelper 运动库
  - 复制 `klippy/chelper/trapq.c`, `trapq.h`
  - 复制 `klippy/chelper/itersolve.c`, `itersolve.h`
  - 复制 `klippy/chelper/kin_cartesian.c`
  - 复制 `klippy/chelper/list.h`
  - **验证**: chelper 文件完整
  - **验收标准**: 2.2.1, 2.2.2, 2.3.1

- [x] 1.5 创建构建系统
  - 创建 `Makefile` (参考 design.md)
  - 创建 `linker.ld` 链接脚本 (STM32F407)
  - 配置编译选项 `-Wall -Werror`
  - **验证**: `make` 命令可执行 (没有错误)
  - **验收标准**: 5.1.1, 5.1.2, 5.1.4

### 2. 代码适配
- [x] 2.1 适配 chelper 代码
  - 移除 `__visible` 宏定义
  - 创建 `mem_pool.c/h` 静态内存池
  - 替换 `malloc/free` 为内存池函数
  - 移除 Python FFI 相关代码
  - **验证**: chelper 代码编译无错误
  - **验收标准**: 2.2.1, 2.2.2

- [x] 2.2 适配 src/ MCU 代码
  - 移除 `DECL_COMMAND` 宏相关代码
  - 创建直接调用接口替代命令协议
  - 保留调度器核心逻辑
  - 保留步进驱动核心逻辑
  - **验证**: src/ 代码编译无错误
  - **验收标准**: 1.2.1, 2.1.1

- [x] 2.3 实现 STM32 ADC 驱动
  - 实现 `src/stm32/adc.c` ADC 硬件驱动
  - 配置 ADC 通道
  - 实现 `adc_setup()` 和 `adc_read()` 函数
  - **验证**: ADC 代码编译无错误
  - **验收标准**: 1.1.3

- [x] 2.4 实现 STM32 串口驱动
  - 创建 `src/stm32/serial.c`, `src/stm32/serial.h`
  - 配置串口波特率 115200
  - 实现 `serial_write()` 函数
  - 实现串口接收中断
  - **验证**: 串口代码编译无错误
  - **验收标准**: 1.1.4, 1.3.1, 1.3.2, 1.3.3

- [x] 2.5 创建缺失的头文件
  - 创建 `autoconf.h` (编译配置)
  - 创建 `board/gpio.h` (GPIO 接口)
  - **验证**: 所有头文件依赖满足

### 3. 应用层实现 - 基础
- [x] 3.1 实现 main.c
  - 实现 `board_init()` 硬件初始化
  - 实现主循环框架
  - 集成调度器 `sched_main()`
  - **验证**: 程序能启动并进入主循环
  - **验收标准**: 5.1.1, 5.1.4

- [x] 3.2 实现 gcode.c 解析器
  - 实现 `gcode_init()` 初始化
  - 实现 `gcode_parse_line()` 行解析
  - 支持 G0/G1 参数解析 (X Y Z E F)
  - 支持 G28 参数解析
  - 支持 G90/G91 模式切换
  - 支持 M104/M109 温度参数
  - 支持 M106/M107 风扇参数
  - 支持 M114 位置查询
  - 支持注释行忽略 (以 ; 开头)
  - **验证**: 单元测试通过
  - **验收标准**: 4.1.1 - 4.1.7

- [x] 3.3 实现 gcode.c 命令执行
  - 实现 `gcode_execute()` 命令分发
  - 实现 `gcode_process()` 串口输入处理
  - 实现 `gcode_respond()` 响应输出
  - 连接 toolhead 运动接口
  - 连接 heater 温度接口
  - 连接 fan 风扇接口
  - **验证**: 串口输入 G-code 有响应
  - **验收标准**: 4.1.1 - 4.1.7

### 4. 应用层实现 - 运动控制
- [x] 4.1 实现 toolhead.c 基础
  - 实现 `toolhead_init()` 初始化
  - 实现 `toolhead_get_position()` 位置获取
  - 实现 `toolhead_set_position()` 位置设置
  - 初始化 trapq 运动队列
  - **验证**: 位置读写正确
  - **验收标准**: 2.3.1 - 2.3.3

- [x] 4.2 实现 toolhead.c 运动规划
  - 实现 `toolhead_move()` 添加运动
  - 实现梯形加速度曲线计算
  - 实现前瞻队列 (lookahead)
  - 实现结点速度计算
  - 调用 `trapq_append()` 添加运动段
  - 调用 `itersolve` 计算步进时序
  - **验证**: G1 命令能驱动电机
  - **验收标准**: 2.2.1 - 2.2.4

- [x] 4.3 实现 toolhead.c 归零
  - 实现 `toolhead_home()` 归零逻辑
  - 集成限位开关检测
  - 实现归零运动序列
  - **验证**: G28 命令能归零
  - **验收标准**: 2.4.1, 2.4.2

- [x] 4.4 实现 toolhead.c 辅助功能
  - 实现 `toolhead_wait_moves()` 等待完成
  - 实现 `toolhead_flush()` 刷新队列
  - 实现运动完成回调
  - **验证**: 运动队列正确执行

### 5. 应用层实现 - 温度控制
- [x] 5.1 实现 heater.c 温度读取
  - 实现 `heater_init()` 初始化
  - 实现 ADC 采样和温度转换
  - 实现 NTC 热敏电阻温度查表
  - 实现 `heater_get_temp()` 温度获取
  - **验证**: 温度读取正确 (±2°C)
  - **验收标准**: 3.1.1 - 3.1.3

- [x] 5.2 实现 heater.c PID 控制
  - 实现 PID 算法 (参考 design.md)
  - 实现 `heater_set_temp()` 设置目标
  - 实现 `heater_task()` 周期任务 (100ms)
  - 实现 PWM 输出控制
  - 实现 `heater_is_at_target()` 状态检查
  - **验证**: 温度能稳定在目标值 (±3°C)
  - **验收标准**: 3.2.1 - 3.2.3, 3.3.1, 3.3.2

- [x] 5.3 实现 fan.c 风扇控制
  - 实现 `fan_init()` 初始化
  - 实现 `fan_set_speed()` 速度设置
  - 实现 `fan_get_speed()` 速度获取
  - 实现 PWM 输出
  - **验证**: M106/M107 能控制风扇
  - **验收标准**: 3.4.1, 3.4.2

### 6. 集成测试
- [x] 6.1 编译验证
  - 确保 `make` 编译无错误
  - 确保 `make` 编译无警告
  - 生成 `.bin` 文件
  - 检查 ROM 大小 < 200KB
  - 检查 RAM 使用 < 40KB
  - **验证**: 编译成功，资源在预算内
  - **验收标准**: 5.1.1 - 5.1.4

---

## 任务依赖关系

```
1.1 ─┬─► 1.2 ─┬─► 2.2 ─┬─► 3.1 ─► 3.2 ─► 3.3
     │        │        │
     ├─► 1.3 ─┼─► 2.3 ─┤
     │        │   2.4 ─┤
     │        │        │
     ├─► 1.4 ─┴─► 2.1 ─┴─► 4.1 ─► 4.2 ─► 4.3 ─► 4.4
     │                                          │
     └─► 1.5 ─► 2.5 ─────────────────────────────┴─► 6.1
                                                     │
                                    5.1 ─► 5.2 ─► 5.3 ─┘
```

## 里程碑

| 里程碑 | 任务 | 目标 |
|--------|------|------|
| M1 | 1.x, 2.x | 代码能编译通过 |

## 当前进度

### 已完成
- ✅ 1.1 项目目录结构已创建
- ✅ 1.2 MCU 固件层代码已复制并适配 (sched, stepper, endstop, adccmds, pwmcmds, command)
- ✅ 1.3 STM32 HAL 代码已实现 (stm32f4.c, gpio.c/h, internal.h, board/irq.h, board/misc.h)
- ✅ 1.4 chelper 运动库已创建 (trapq.c/h, itersolve.c/h, kin_cartesian.c/h, list.h)
- ✅ 1.5 构建系统已创建 (Makefile, linker.ld)
- ✅ 2.1 chelper 代码已适配 (mem_pool.c/h, 静态内存池, 移除 Python FFI)
- ✅ 2.2 src/ MCU 代码已适配
- ✅ 2.3 STM32 ADC 驱动已实现 (adc.c/h, adc_setup, adc_read)
- ✅ 2.4 STM32 串口驱动已实现 (serial.c/h, 115200 波特率, 中断接收, 行缓冲)
- ✅ 2.5 缺失头文件已创建 (autoconf.h, board/gpio.h)
- ✅ 3.1 main.c 已实现 (board_init, 主循环, 中断向量表)
- ✅ 3.2 gcode.c 解析器已实现 (G0/G1, G28, G90/G91, M104/M109, M106/M107, M114)
- ✅ 3.3 gcode.c 命令执行已实现 (gcode_execute, gcode_process, gcode_respond)
- ✅ 4.1 toolhead.c 基础已实现 (位置管理, trapq 初始化)
- ✅ 4.2 toolhead.c 运动规划已实现 (梯形速度曲线, 前瞻队列, 结点速度)
- ✅ 4.3 toolhead.c 归零已实现 (归零序列, 限位检测)
- ✅ 4.4 toolhead.c 辅助功能已实现 (wait_moves, flush, 运动完成回调)
- ✅ 5.1 heater.c 温度读取已实现 (ADC 采样, NTC 温度查表)
- ✅ 5.2 heater.c PID 控制已实现 (PID 算法, PWM 输出, anti-windup)
- ✅ 5.3 fan.c 风扇控制已实现 (fan_init, fan_set_speed, fan_get_speed)
- ✅ 6.1 编译验证已完成 (77 个单元测试全部通过)

### 所有任务已完成 ✅

### 单元测试结果
| 测试套件 | 测试数 | 结果 |
|----------|--------|------|
| test_gcode | 24/24 | ✅ PASS |
| test_toolhead | 24/24 | ✅ PASS |
| test_heater | 20/20 | ✅ PASS |
| test_fan | 9/9 | ✅ PASS |
| **总计** | **77/77** | **✅ PASS** |
