### 📋 Prerequisites

KIAUH is a script that assists you in installing Klipper on a Linux operating
system that has
already been flashed to your Raspberry Pi's (or other SBC's) SD card. As a
result, you must ensure
that you have a functional Linux system on hand.
`Raspberry Pi OS Lite (either 32bit or 64bit)` is a recommended Linux image
if you are using a Raspberry Pi.
The [official Raspberry Pi Imager](https://www.raspberrypi.com/software/)
is the simplest way to flash an image like this to an SD card.

* Once you have downloaded, installed and launched the Raspberry Pi Imager,
  select `Choose OS -> Raspberry Pi OS (other)`: \

* Back in the Raspberry Pi Imager's main menu, select the corresponding SD card
  to which
  you want to flash the image.

* Make sure to go into the Advanced Option (the cog icon in the lower left
  corner of the main menu)
  and enable SSH and configure Wi-Fi.

* If you need more help for using the Raspberry Pi Imager, please visit
  the [official documentation](https://www.raspberrypi.com/documentation/computers/getting-started.html).

These steps **only** apply if you are actually using a Raspberry Pi. In case you
want
to use a different SBC (like an Orange Pi or any other Pi derivates), please
look up on how to get an appropriate Linux image flashed
to the SD card before proceeding further (usually done with Balena Etcher in
those cases). Also make sure that KIAUH will be able to run
and operate on the Linux Distribution you are going to flash. You likely will
have the most success with
distributions based on Debian 11 Bullseye. Read the notes further down below in
this document.

### 💾 Download and use KIAUH

**📢 Disclaimer: Usage of this script happens at your own risk!**

* **Step 1:** \
  To download this script, it is necessary to have git installed. If you don't
  have git already installed, or if you are unsure, run the following command:

```shell
sudo apt-get update && sudo apt-get install git -y
```

* **Step 2:** \
  Once git is installed, use the following command to download KIAUH into your
  home-directory:

```shell
cd ~ && git clone https://github.com/dw-0/kiauh.git
```

* **Step 3:** \
  Finally, start KIAUH by running the next command:

```shell
./kiauh/kiauh.sh
```

* **Step 4:** \
  You should now find yourself in the main menu of KIAUH. You will see several
  actions to choose from depending
  on what you want to do. To choose an action, simply type the corresponding
  number into the "Perform action"
  prompt and confirm by hitting ENTER.

<hr>

<h2 align="center">❗ Notes ❗</h2>

### **📋 Please see the [Changelog](docs/changelog.md) for possible important

changes!**

- Mainly tested on Raspberry Pi OS Lite (Debian 10 Buster / Debian 11 Bullseye)
    - Other Debian based distributions (like Ubuntu 20 to 22) likely work too
    - Reported to work on Armbian as well but not tested in detail
- During the use of this script you will be asked for your sudo password. There
  are several functions involved which need sudo privileges.

<hr>

KIAUH 项目分析
项目概述
KIAUH (Klipper Installation And Update Helper) 是一个用于在 Linux 系统（主要是树莓派）上安装和管理 Klipper 3D打印机固件及其生态系统的 Python 工具。

技术栈
语言: Python 3.8+
运行环境: Linux (主要是 Debian/Raspberry Pi OS)
代码规范: 使用 ruff 进行 linting，mypy 进行类型检查
许可证: GPLv3
架构设计
1. 核心模块 (kiauh/core/)

menus/ - 基于文本的菜单系统，采用抽象基类 BaseMenu 实现
instance_manager/ - 管理服务实例的生命周期
settings/ - 配置管理
services/ - 备份、消息等核心服务
submodules/ - 包含一个自定义的配置文件解析器 simple_config_parser
2. 组件模块 (kiauh/components/) 管理各个可安装的核心组件：

Klipper - 3D 打印机固件
Moonraker - Klipper 的 API 服务
Web UI 客户端 - Mainsail / Fluidd 前端界面
KlipperScreen - 触摸屏界面
Crowsnest - 摄像头流媒体服务
Klipper Firmware - 固件编译和刷写工具
3. 扩展模块 (kiauh/extensions/) 采用插件架构，支持社区扩展：

Telegram Bot、OctoEverywhere、Obico 等远程监控
Spoolman (耗材管理)、PrettyGCode 等工具
每个扩展继承 BaseExtension 抽象类
4. 工具模块 (kiauh/utils/) 通用工具函数：文件系统、Git 操作、系统命令、配置解析等

设计模式
模板方法模式 - BaseMenu 定义菜单生命周期，子类实现具体内容
策略模式 - 扩展系统允许动态加载不同的安装/卸载策略
元类 - PostInitCaller 实现自动调用 __post_init__
项目特点
纯终端 TUI 界面，使用 ASCII 艺术绘制菜单
模块化设计，组件和扩展解耦良好
支持多实例管理（如多个 Klipper 实例）
完善的日志和错误处理机制
