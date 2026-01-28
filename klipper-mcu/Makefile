# =============================================================================
# Klipper MCU 移植项目 - Makefile
# 目标: STM32F407VG (Cortex-M4 with FPU)
# =============================================================================

# -----------------------------------------------------------------------------
# 工具链配置
# -----------------------------------------------------------------------------
PREFIX      = arm-none-eabi-
CC          = $(PREFIX)gcc
AS          = $(PREFIX)gcc -x assembler-with-cpp
CP          = $(PREFIX)objcopy
SZ          = $(PREFIX)size
HEX         = $(CP) -O ihex
BIN         = $(CP) -O binary -S

# -----------------------------------------------------------------------------
# 目标配置
# -----------------------------------------------------------------------------
TARGET      = klipper-mcu
MCU         = STM32F407VG

# -----------------------------------------------------------------------------
# 目录配置
# -----------------------------------------------------------------------------
BUILD_DIR   = build
SRC_DIR     = src
CHELPER_DIR = chelper
APP_DIR     = app
BOARD_DIR   = board

# -----------------------------------------------------------------------------
# MCU 配置 (STM32F407VG - Cortex-M4 with FPU)
# -----------------------------------------------------------------------------
CPU         = -mcpu=cortex-m4
FPU         = -mfpu=fpv4-sp-d16
FLOAT_ABI   = -mfloat-abi=hard
MCU_FLAGS   = $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

# -----------------------------------------------------------------------------
# 源文件
# -----------------------------------------------------------------------------

# MCU 固件层 (src/)
SRC_SRCS    = \
    $(SRC_DIR)/sched.c \
    $(SRC_DIR)/stepper.c \
    $(SRC_DIR)/endstop.c \
    $(SRC_DIR)/adccmds.c \
    $(SRC_DIR)/pwmcmds.c \
    $(SRC_DIR)/command.c

# STM32 HAL 层 (src/stm32/)
STM32_SRCS  = \
    $(SRC_DIR)/stm32/stm32f4.c \
    $(SRC_DIR)/stm32/gpio.c \
    $(SRC_DIR)/stm32/adc.c \
    $(SRC_DIR)/stm32/serial.c

# 运动库层 (chelper/)
CHELPER_SRCS = \
    $(CHELPER_DIR)/mem_pool.c \
    $(CHELPER_DIR)/trapq.c \
    $(CHELPER_DIR)/itersolve.c \
    $(CHELPER_DIR)/kin_cartesian.c

# 应用层 (app/) - 排除 main_host.c (仅用于主机编译)
APP_SRCS    = \
    $(APP_DIR)/main.c \
    $(APP_DIR)/gcode.c \
    $(APP_DIR)/toolhead.c \
    $(APP_DIR)/heater.c \
    $(APP_DIR)/fan.c

# 汇总所有源文件
C_SOURCES   = $(SRC_SRCS) $(STM32_SRCS) $(CHELPER_SRCS) $(APP_SRCS)

# 启动文件 (后续添加)
ASM_SOURCES =

# -----------------------------------------------------------------------------
# 头文件路径
# -----------------------------------------------------------------------------
C_INCLUDES  = \
    -I. \
    -I$(SRC_DIR) \
    -I$(SRC_DIR)/stm32 \
    -I$(CHELPER_DIR) \
    -I$(APP_DIR) \
    -I$(BOARD_DIR)

# -----------------------------------------------------------------------------
# 编译选项
# -----------------------------------------------------------------------------

# C 编译选项
CFLAGS      = $(MCU_FLAGS)
CFLAGS     += -O2 -g
CFLAGS     += -Wall -Werror -Wextra
CFLAGS     += -Wno-unused-parameter
CFLAGS     += -fdata-sections -ffunction-sections
CFLAGS     += -std=c99
CFLAGS     += $(C_INCLUDES)

# 预处理器定义
CFLAGS     += -DMCU_BUILD
CFLAGS     += -DSTM32F407xx
CFLAGS     += -DSTM32F4

# 汇编选项
ASFLAGS     = $(MCU_FLAGS)
ASFLAGS    += -Wall -Werror

# -----------------------------------------------------------------------------
# 链接选项
# -----------------------------------------------------------------------------
LDSCRIPT    = linker.ld

LDFLAGS     = $(MCU_FLAGS)
LDFLAGS    += -specs=nano.specs
LDFLAGS    += -T$(LDSCRIPT)
LDFLAGS    += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref
LDFLAGS    += -Wl,--gc-sections
LDFLAGS    += -lm -lc -lnosys

# -----------------------------------------------------------------------------
# 构建目标
# -----------------------------------------------------------------------------

# 默认目标
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin size

# 目标文件列表
OBJECTS     = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS    += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# 创建构建目录
$(BUILD_DIR):
	mkdir -p $@

# 编译 C 文件
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "[CC] $<"
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

# 编译汇编文件
$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "[AS] $<"
	@$(AS) -c $(ASFLAGS) $< -o $@

# 链接
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile $(LDSCRIPT)
	@echo "[LD] $@"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@

# 生成 HEX 文件
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf | $(BUILD_DIR)
	@echo "[HEX] $@"
	@$(HEX) $< $@

# 生成 BIN 文件
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf | $(BUILD_DIR)
	@echo "[BIN] $@"
	@$(BIN) $< $@

# 显示大小信息
size: $(BUILD_DIR)/$(TARGET).elf
	@echo ""
	@echo "========== 内存使用情况 =========="
	@$(SZ) $<
	@echo ""

# -----------------------------------------------------------------------------
# 烧录目标
# -----------------------------------------------------------------------------

# 使用 st-flash 烧录 (ST-Link)
flash: $(BUILD_DIR)/$(TARGET).bin
	@echo "[FLASH] 烧录到 0x08000000..."
	st-flash write $< 0x08000000

# 使用 OpenOCD 烧录
flash-openocd: $(BUILD_DIR)/$(TARGET).elf
	@echo "[FLASH] 使用 OpenOCD 烧录..."
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
		-c "program $< verify reset exit"

# 擦除芯片
erase:
	@echo "[ERASE] 擦除芯片..."
	st-flash erase

# -----------------------------------------------------------------------------
# 调试目标
# -----------------------------------------------------------------------------

# 启动 GDB 服务器
debug-server:
	@echo "[DEBUG] 启动 OpenOCD GDB 服务器..."
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

# 启动 GDB 客户端
debug: $(BUILD_DIR)/$(TARGET).elf
	@echo "[DEBUG] 启动 GDB..."
	$(PREFIX)gdb -ex "target remote localhost:3333" $<

# -----------------------------------------------------------------------------
# 清理目标
# -----------------------------------------------------------------------------

# 清理构建文件
clean:
	@echo "[CLEAN] 清理构建目录..."
	rm -rf $(BUILD_DIR)

# 完全清理
distclean: clean
	@echo "[DISTCLEAN] 完全清理..."
	rm -f tags cscope.*

# -----------------------------------------------------------------------------
# 辅助目标
# -----------------------------------------------------------------------------

# 显示源文件列表
info:
	@echo "========== 项目信息 =========="
	@echo "目标: $(TARGET)"
	@echo "MCU: $(MCU)"
	@echo ""
	@echo "C 源文件:"
	@for src in $(C_SOURCES); do echo "  $$src"; done
	@echo ""
	@echo "头文件路径:"
	@for inc in $(C_INCLUDES); do echo "  $$inc"; done

# 生成 tags 文件 (用于编辑器)
tags:
	@echo "[TAGS] 生成 tags..."
	ctags -R .

# 检查工具链
check-tools:
	@echo "========== 工具链检查 =========="
	@which $(CC) > /dev/null 2>&1 && echo "[OK] $(CC)" || echo "[MISSING] $(CC)"
	@which $(CP) > /dev/null 2>&1 && echo "[OK] $(CP)" || echo "[MISSING] $(CP)"
	@which $(SZ) > /dev/null 2>&1 && echo "[OK] $(SZ)" || echo "[MISSING] $(SZ)"
	@which st-flash > /dev/null 2>&1 && echo "[OK] st-flash" || echo "[MISSING] st-flash (可选)"
	@which openocd > /dev/null 2>&1 && echo "[OK] openocd" || echo "[MISSING] openocd (可选)"

# 伪目标声明
.PHONY: all clean distclean flash flash-openocd erase debug debug-server size info tags check-tools

