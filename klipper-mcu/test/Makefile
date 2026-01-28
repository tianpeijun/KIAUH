# =============================================================================
# Klipper MCU Unit Tests Makefile
# 使用主机编译器 (gcc) 编译运行
# =============================================================================

CC          = gcc
CFLAGS      = -Wall -Wextra -g -std=c99
CFLAGS     += -I../app -I.. -I../chelper -I../src
CFLAGS     += -DTEST_BUILD

# 测试目标
TEST_GCODE    = test_gcode
TEST_TOOLHEAD = test_toolhead
TEST_HEATER   = test_heater
TEST_FAN      = test_fan

# 源文件
TEST_GCODE_SRCS    = test_gcode.c ../app/gcode.c
TEST_TOOLHEAD_SRCS = test_toolhead.c ../app/toolhead.c \
                     ../chelper/trapq.c ../chelper/itersolve.c \
                     ../chelper/kin_cartesian.c \
                     stubs.c
TEST_HEATER_SRCS   = test_heater.c ../app/heater.c
TEST_FAN_SRCS      = test_fan.c ../app/fan.c

# 默认目标
all: $(TEST_GCODE) $(TEST_TOOLHEAD) $(TEST_HEATER) $(TEST_FAN)

# 编译 G-code 测试
$(TEST_GCODE): $(TEST_GCODE_SRCS)
	$(CC) $(CFLAGS) -o $@ $^ -lm

# 编译 Toolhead 测试
$(TEST_TOOLHEAD): $(TEST_TOOLHEAD_SRCS)
	$(CC) $(CFLAGS) -o $@ $^ -lm

# 编译 Heater 测试
$(TEST_HEATER): $(TEST_HEATER_SRCS)
	$(CC) $(CFLAGS) -o $@ $^ -lm

# 编译 Fan 测试
$(TEST_FAN): $(TEST_FAN_SRCS)
	$(CC) $(CFLAGS) -o $@ $^ -lm

# 运行所有测试
test: $(TEST_GCODE) $(TEST_TOOLHEAD) $(TEST_HEATER) $(TEST_FAN)
	@echo "========== Running G-code Tests =========="
	./$(TEST_GCODE)
	@echo ""
	@echo "========== Running Toolhead Tests =========="
	./$(TEST_TOOLHEAD)
	@echo ""
	@echo "========== Running Heater Tests =========="
	./$(TEST_HEATER)
	@echo ""
	@echo "========== Running Fan Tests =========="
	./$(TEST_FAN)

# 运行单个测试
test-gcode: $(TEST_GCODE)
	./$(TEST_GCODE)

test-toolhead: $(TEST_TOOLHEAD)
	./$(TEST_TOOLHEAD)

test-heater: $(TEST_HEATER)
	./$(TEST_HEATER)

test-fan: $(TEST_FAN)
	./$(TEST_FAN)

# 清理
clean:
	rm -f $(TEST_GCODE) $(TEST_TOOLHEAD) $(TEST_HEATER) $(TEST_FAN)

.PHONY: all test test-gcode test-toolhead test-heater test-fan clean
