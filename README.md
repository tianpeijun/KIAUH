# KIAUH - Klipper Installation And Update Helper

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

KIAUH 是一个用于在 Linux 系统上安装和管理 Klipper 3D 打印机固件及其生态系统的工具。

---

## 🆕 Klipper MCU C 移植 (klipper-mcu)

本项目新增了 **Klipper MCU 固件的 C 语言移植**，将原 Python 实现的核心功能移植到 C 语言，可直接编译烧录到 STM32F407 微控制器。

### 功能特性

| 模块 | 功能 | 状态 |
|------|------|------|
| G-code 解析器 | G0/G1, G28, G90/G91, M104/M109, M106/M107, M114 | ✅ |
| 运动规划器 | 梯形速度曲线、前瞻队列、笛卡尔运动学 | ✅ |
| 温度控制 | NTC 热敏电阻查表、PID 控制、防积分饱和 | ✅ |
| 风扇控制 | PWM 调速、M106/M107 指令 | ✅ |
| HAL 驱动 | GPIO、ADC、Serial、Timer (STM32F407) | ✅ |

### 快速开始

```bash
# 安装 ARM 交叉编译工具链
sudo apt-get install gcc-arm-none-eabi

# 编译 ARM 固件
cd klipper-mcu
make

# 生成文件
# build/klipper-mcu.bin  - 可烧录的二进制文件 (20KB)
# build/klipper-mcu.elf  - ARM 32-bit ELF 可执行文件
# build/klipper-mcu.hex  - Intel HEX 格式

# 烧录到 STM32F407 (需要 st-flash)
make flash

# 或使用 OpenOCD 烧录
make flash-openocd
```

### 运行测试

```bash
cd klipper-mcu/test
make
./test_gcode && ./test_toolhead && ./test_heater && ./test_fan
# 77 个单元测试全部通过
```

### 目录结构

```
klipper-mcu/
├── app/                    # 应用层
│   ├── gcode.c/h          # G-code 解析器
│   ├── toolhead.c/h       # 运动规划器
│   ├── heater.c/h         # 温度控制
│   ├── fan.c/h            # 风扇控制
│   └── main.c             # 主程序入口
├── chelper/               # 运动库 (移植自 Klipper)
│   ├── trapq.c/h          # 梯形运动队列
│   ├── itersolve.c/h      # 迭代求解器
│   └── kin_cartesian.c/h  # 笛卡尔运动学
├── src/                   # MCU 固件层
│   ├── sched.c/h          # 任务调度器
│   ├── stepper.c/h        # 步进电机驱动
│   ├── endstop.c/h        # 限位开关
│   └── stm32/             # STM32 HAL 驱动
├── board/                 # 板级支持
├── test/                  # 单元测试
├── Makefile               # ARM 交叉编译
└── Makefile.host          # 主机编译
```

---

## 📋 原 KIAUH 功能

### 前置条件

KIAUH 是一个帮助你在 Linux 系统上安装 Klipper 的脚本。推荐使用 `Raspberry Pi OS Lite`。

### 下载和使用

**📢 免责声明：使用此脚本的风险由您自行承担！**

```bash
# 安装 git
sudo apt-get update && sudo apt-get install git -y

# 下载 KIAUH
cd ~ && git clone https://github.com/tianpeijun/KIAUH.git

# 启动 KIAUH
./KIAUH/kiauh.sh
```

### 技术架构

- **语言**: Python 3.8+
- **核心模块** (`kiauh/core/`): 菜单系统、实例管理、配置管理
- **组件模块** (`kiauh/components/`): Klipper、Moonraker、Web UI、KlipperScreen
- **扩展模块** (`kiauh/extensions/`): Telegram Bot、Spoolman 等插件

---

## 📝 License

GPL v3 - 详见 [LICENSE](LICENSE)
