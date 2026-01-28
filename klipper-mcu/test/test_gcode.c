/**
 * @file    test_gcode.c
 * @brief   G-code 解析器单元测试
 * 
 * 测试 gcode.c 的解析功能
 * 使用主机编译器 (gcc) 编译运行
 * 
 * 编译: gcc -o test_gcode test_gcode.c ../app/gcode.c -I../app -I.. -DTEST_BUILD
 * 运行: ./test_gcode
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "gcode.h"

/* ========== 测试框架 ========== */

static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
        return 0; \
    } \
} while (0)

#define TEST_ASSERT_EQ(a, b, msg) do { \
    if ((a) != (b)) { \
        printf("  FAIL: %s - expected %d, got %d (line %d)\n", msg, (int)(b), (int)(a), __LINE__); \
        return 0; \
    } \
} while (0)

#define TEST_ASSERT_FLOAT_EQ(a, b, msg) do { \
    if (fabsf((a) - (b)) > 0.001f) { \
        printf("  FAIL: %s - expected %.3f, got %.3f (line %d)\n", msg, (float)(b), (float)(a), __LINE__); \
        return 0; \
    } \
} while (0)

#define RUN_TEST(test_func) do { \
    g_tests_run++; \
    printf("Running %s...\n", #test_func); \
    if (test_func()) { \
        g_tests_passed++; \
        printf("  PASS\n"); \
    } else { \
        g_tests_failed++; \
    } \
} while (0)

/* ========== 测试用例 ========== */

/**
 * @brief   测试 gcode_init 初始化
 */
static int
test_gcode_init(void)
{
    gcode_init();
    TEST_ASSERT_EQ(gcode_get_mode(), GCODE_MODE_ABSOLUTE, "default mode should be absolute");
    return 1;
}

/**
 * @brief   测试空指针处理
 */
static int
test_null_pointer(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    ret = gcode_parse_line(NULL, &cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_NULL, "NULL line should return GCODE_ERR_NULL");
    
    ret = gcode_parse_line("G0 X10", NULL);
    TEST_ASSERT_EQ(ret, GCODE_ERR_NULL, "NULL cmd should return GCODE_ERR_NULL");
    
    return 1;
}

/**
 * @brief   测试空行处理
 */
static int
test_empty_line(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    ret = gcode_parse_line("", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_EMPTY, "empty string should return GCODE_ERR_EMPTY");
    
    ret = gcode_parse_line("   ", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_EMPTY, "whitespace only should return GCODE_ERR_EMPTY");
    
    ret = gcode_parse_line("\n", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_EMPTY, "newline only should return GCODE_ERR_EMPTY");
    
    ret = gcode_parse_line("\r\n", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_EMPTY, "CRLF only should return GCODE_ERR_EMPTY");
    
    return 1;
}

/**
 * @brief   测试注释行处理 (以 ; 开头)
 * @note    验收标准: 注释行忽略
 */
static int
test_comment_line(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    ret = gcode_parse_line("; this is a comment", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_COMMENT, "comment line should return GCODE_ERR_COMMENT");
    
    ret = gcode_parse_line(";G0 X10", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_COMMENT, "comment starting with ; should be ignored");
    
    ret = gcode_parse_line("  ; indented comment", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_COMMENT, "indented comment should be ignored");
    
    return 1;
}

/**
 * @brief   测试 G0/G1 直线运动解析
 * @note    验收标准: 4.1.2 - 支持 G0/G1 (直线运动)
 */
static int
test_g0_g1_parsing(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 测试 G0 快速移动 */
    ret = gcode_parse_line("G0 X100 Y50 Z10", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G0 should parse successfully");
    TEST_ASSERT_EQ(cmd.cmd, 'G', "command should be G");
    TEST_ASSERT_EQ(cmd.code, 0, "code should be 0");
    TEST_ASSERT(cmd.has_x, "should have X");
    TEST_ASSERT(cmd.has_y, "should have Y");
    TEST_ASSERT(cmd.has_z, "should have Z");
    TEST_ASSERT_FLOAT_EQ(cmd.x, 100.0f, "X should be 100");
    TEST_ASSERT_FLOAT_EQ(cmd.y, 50.0f, "Y should be 50");
    TEST_ASSERT_FLOAT_EQ(cmd.z, 10.0f, "Z should be 10");
    
    /* 测试 G1 直线插补 */
    ret = gcode_parse_line("G1 X50.5 Y25.25 E1.5 F3000", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G1 should parse successfully");
    TEST_ASSERT_EQ(cmd.cmd, 'G', "command should be G");
    TEST_ASSERT_EQ(cmd.code, 1, "code should be 1");
    TEST_ASSERT(cmd.has_x, "should have X");
    TEST_ASSERT(cmd.has_y, "should have Y");
    TEST_ASSERT(cmd.has_e, "should have E");
    TEST_ASSERT(cmd.has_f, "should have F");
    TEST_ASSERT_FLOAT_EQ(cmd.x, 50.5f, "X should be 50.5");
    TEST_ASSERT_FLOAT_EQ(cmd.y, 25.25f, "Y should be 25.25");
    TEST_ASSERT_FLOAT_EQ(cmd.e, 1.5f, "E should be 1.5");
    TEST_ASSERT_FLOAT_EQ(cmd.f, 3000.0f, "F should be 3000");
    
    /* 测试小写命令 */
    ret = gcode_parse_line("g1 x10 y20", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "lowercase g1 should parse successfully");
    TEST_ASSERT_EQ(cmd.cmd, 'G', "command should be G (uppercase)");
    TEST_ASSERT_EQ(cmd.code, 1, "code should be 1");
    
    /* 测试负数坐标 */
    ret = gcode_parse_line("G1 X-10.5 Y-20.25", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "negative coordinates should parse");
    TEST_ASSERT_FLOAT_EQ(cmd.x, -10.5f, "X should be -10.5");
    TEST_ASSERT_FLOAT_EQ(cmd.y, -20.25f, "Y should be -20.25");
    
    return 1;
}

/**
 * @brief   测试 G28 归零解析
 * @note    验收标准: 4.1.3 - 支持 G28 (归零)
 */
static int
test_g28_parsing(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 测试 G28 全部归零 */
    ret = gcode_parse_line("G28", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G28 should parse successfully");
    TEST_ASSERT_EQ(cmd.cmd, 'G', "command should be G");
    TEST_ASSERT_EQ(cmd.code, 28, "code should be 28");
    TEST_ASSERT(!cmd.has_x, "should not have X");
    TEST_ASSERT(!cmd.has_y, "should not have Y");
    TEST_ASSERT(!cmd.has_z, "should not have Z");
    
    /* 测试 G28 X 单轴归零 */
    ret = gcode_parse_line("G28 X", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G28 X should parse successfully");
    TEST_ASSERT(cmd.has_x, "should have X flag");
    TEST_ASSERT(!cmd.has_y, "should not have Y");
    TEST_ASSERT(!cmd.has_z, "should not have Z");
    
    /* 测试 G28 X Y 多轴归零 */
    ret = gcode_parse_line("G28 X Y", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G28 X Y should parse successfully");
    TEST_ASSERT(cmd.has_x, "should have X flag");
    TEST_ASSERT(cmd.has_y, "should have Y flag");
    TEST_ASSERT(!cmd.has_z, "should not have Z");
    
    /* 测试 G28 X Y Z 全轴归零 */
    ret = gcode_parse_line("G28 X Y Z", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G28 X Y Z should parse successfully");
    TEST_ASSERT(cmd.has_x, "should have X flag");
    TEST_ASSERT(cmd.has_y, "should have Y flag");
    TEST_ASSERT(cmd.has_z, "should have Z flag");
    
    return 1;
}

/**
 * @brief   测试 G90/G91 坐标模式切换
 * @note    验收标准: 4.1.4 - 支持 G90/G91 (坐标模式)
 */
static int
test_g90_g91_parsing(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 测试 G90 绝对坐标模式 */
    ret = gcode_parse_line("G90", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G90 should parse successfully");
    TEST_ASSERT_EQ(cmd.cmd, 'G', "command should be G");
    TEST_ASSERT_EQ(cmd.code, 90, "code should be 90");
    
    /* 测试 G91 相对坐标模式 */
    ret = gcode_parse_line("G91", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G91 should parse successfully");
    TEST_ASSERT_EQ(cmd.cmd, 'G', "command should be G");
    TEST_ASSERT_EQ(cmd.code, 91, "code should be 91");
    
    /* 测试模式设置和获取 */
    gcode_set_mode(GCODE_MODE_ABSOLUTE);
    TEST_ASSERT_EQ(gcode_get_mode(), GCODE_MODE_ABSOLUTE, "mode should be absolute");
    
    gcode_set_mode(GCODE_MODE_RELATIVE);
    TEST_ASSERT_EQ(gcode_get_mode(), GCODE_MODE_RELATIVE, "mode should be relative");
    
    return 1;
}

/**
 * @brief   测试 M104/M109 温度参数解析
 * @note    验收标准: 4.1.5 - 支持 M104/M109 (热端温度)
 */
static int
test_m104_m109_parsing(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 测试 M104 设置温度 */
    ret = gcode_parse_line("M104 S200", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M104 should parse successfully");
    TEST_ASSERT_EQ(cmd.cmd, 'M', "command should be M");
    TEST_ASSERT_EQ(cmd.code, 104, "code should be 104");
    TEST_ASSERT(cmd.has_s, "should have S parameter");
    TEST_ASSERT_FLOAT_EQ(cmd.s, 200.0f, "S should be 200");
    
    /* 测试 M109 等待温度 */
    ret = gcode_parse_line("M109 S210", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M109 should parse successfully");
    TEST_ASSERT_EQ(cmd.cmd, 'M', "command should be M");
    TEST_ASSERT_EQ(cmd.code, 109, "code should be 109");
    TEST_ASSERT(cmd.has_s, "should have S parameter");
    TEST_ASSERT_FLOAT_EQ(cmd.s, 210.0f, "S should be 210");
    
    /* 测试小数温度 */
    ret = gcode_parse_line("M104 S195.5", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M104 with decimal should parse");
    TEST_ASSERT_FLOAT_EQ(cmd.s, 195.5f, "S should be 195.5");
    
    return 1;
}

/**
 * @brief   测试 M106/M107 风扇参数解析
 * @note    验收标准: 4.1.6 - 支持 M106/M107 (风扇)
 */
static int
test_m106_m107_parsing(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 测试 M106 设置风扇速度 */
    ret = gcode_parse_line("M106 S255", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M106 should parse successfully");
    TEST_ASSERT_EQ(cmd.cmd, 'M', "command should be M");
    TEST_ASSERT_EQ(cmd.code, 106, "code should be 106");
    TEST_ASSERT(cmd.has_s, "should have S parameter");
    TEST_ASSERT_FLOAT_EQ(cmd.s, 255.0f, "S should be 255");
    
    /* 测试 M106 半速 */
    ret = gcode_parse_line("M106 S127", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M106 half speed should parse");
    TEST_ASSERT_FLOAT_EQ(cmd.s, 127.0f, "S should be 127");
    
    /* 测试 M107 关闭风扇 */
    ret = gcode_parse_line("M107", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M107 should parse successfully");
    TEST_ASSERT_EQ(cmd.cmd, 'M', "command should be M");
    TEST_ASSERT_EQ(cmd.code, 107, "code should be 107");
    
    return 1;
}

/**
 * @brief   测试 M114 位置查询解析
 * @note    验收标准: 4.1.7 - 支持 M114 (位置查询)
 */
static int
test_m114_parsing(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    ret = gcode_parse_line("M114", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M114 should parse successfully");
    TEST_ASSERT_EQ(cmd.cmd, 'M', "command should be M");
    TEST_ASSERT_EQ(cmd.code, 114, "code should be 114");
    
    return 1;
}

/**
 * @brief   测试行内注释处理
 */
static int
test_inline_comment(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 测试行内注释 */
    ret = gcode_parse_line("G1 X100 Y50 ; move to position", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "inline comment should be ignored");
    TEST_ASSERT_EQ(cmd.cmd, 'G', "command should be G");
    TEST_ASSERT_EQ(cmd.code, 1, "code should be 1");
    TEST_ASSERT(cmd.has_x, "should have X");
    TEST_ASSERT(cmd.has_y, "should have Y");
    TEST_ASSERT_FLOAT_EQ(cmd.x, 100.0f, "X should be 100");
    TEST_ASSERT_FLOAT_EQ(cmd.y, 50.0f, "Y should be 50");
    
    return 1;
}

/**
 * @brief   测试未知命令处理
 */
static int
test_unknown_command(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 测试不支持的 G 命令 */
    ret = gcode_parse_line("G99", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_UNKNOWN, "G99 should return GCODE_ERR_UNKNOWN");
    
    /* 测试不支持的 M 命令 */
    ret = gcode_parse_line("M999", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_UNKNOWN, "M999 should return GCODE_ERR_UNKNOWN");
    
    return 1;
}

/**
 * @brief   测试无效命令格式
 */
static int
test_invalid_command(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 测试无效命令字母 */
    ret = gcode_parse_line("X100", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_INVALID, "X100 should return GCODE_ERR_INVALID");
    
    ret = gcode_parse_line("T0", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_INVALID, "T0 should return GCODE_ERR_INVALID");
    
    return 1;
}

/**
 * @brief   测试 gcode_cmd_clear 函数
 */
static int
test_cmd_clear(void)
{
    gcode_cmd_t cmd;
    
    /* 设置一些值 */
    cmd.cmd = 'G';
    cmd.code = 1;
    cmd.x = 100.0f;
    cmd.has_x = 1;
    
    /* 清空 */
    gcode_cmd_clear(&cmd);
    
    TEST_ASSERT_EQ(cmd.cmd, 0, "cmd should be 0");
    TEST_ASSERT_EQ(cmd.code, 0, "code should be 0");
    TEST_ASSERT_FLOAT_EQ(cmd.x, 0.0f, "x should be 0");
    TEST_ASSERT(!cmd.has_x, "has_x should be 0");
    
    /* 测试 NULL 指针 (不应崩溃) */
    gcode_cmd_clear(NULL);
    
    return 1;
}

/**
 * @brief   测试带空格的命令
 */
static int
test_whitespace_handling(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 测试前导空格 */
    ret = gcode_parse_line("  G1 X100", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "leading spaces should be handled");
    TEST_ASSERT_EQ(cmd.code, 1, "code should be 1");
    
    /* 测试多个空格分隔 */
    ret = gcode_parse_line("G1  X100   Y50", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "multiple spaces should be handled");
    TEST_ASSERT_FLOAT_EQ(cmd.x, 100.0f, "X should be 100");
    TEST_ASSERT_FLOAT_EQ(cmd.y, 50.0f, "Y should be 50");
    
    /* 测试 Tab 分隔 */
    ret = gcode_parse_line("G1\tX100\tY50", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "tabs should be handled");
    TEST_ASSERT_FLOAT_EQ(cmd.x, 100.0f, "X should be 100");
    TEST_ASSERT_FLOAT_EQ(cmd.y, 50.0f, "Y should be 50");
    
    return 1;
}

/* ========== 命令执行测试 ========== */

/**
 * @brief   测试 gcode_execute 空指针处理
 */
static int
test_execute_null_pointer(void)
{
    int ret;
    
    ret = gcode_execute(NULL);
    TEST_ASSERT_EQ(ret, GCODE_ERR_NULL, "NULL cmd should return GCODE_ERR_NULL");
    
    return 1;
}

/**
 * @brief   测试 G0/G1 命令执行
 * @note    验收标准: 4.1.2 - 支持 G0/G1 (直线运动)
 */
static int
test_execute_g0_g1(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 初始化 */
    gcode_init();
    
    /* 解析并执行 G1 命令 */
    ret = gcode_parse_line("G1 X100 Y50 Z10 F3000", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G1 should parse successfully");
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G1 should execute successfully");
    
    /* 测试 G0 命令 */
    ret = gcode_parse_line("G0 X0 Y0 Z0", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G0 should parse successfully");
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G0 should execute successfully");
    
    return 1;
}

/**
 * @brief   测试 G28 归零命令执行
 * @note    验收标准: 4.1.3 - 支持 G28 (归零)
 */
static int
test_execute_g28(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 测试全轴归零 */
    ret = gcode_parse_line("G28", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G28 should parse successfully");
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G28 should execute successfully");
    
    /* 测试单轴归零 */
    ret = gcode_parse_line("G28 X", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G28 X should parse successfully");
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G28 X should execute successfully");
    
    /* 测试多轴归零 */
    ret = gcode_parse_line("G28 X Y", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G28 X Y should parse successfully");
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G28 X Y should execute successfully");
    
    return 1;
}

/**
 * @brief   测试 G90/G91 坐标模式切换执行
 * @note    验收标准: 4.1.4 - 支持 G90/G91 (坐标模式)
 */
static int
test_execute_g90_g91(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 初始化 - 默认绝对模式 */
    gcode_init();
    TEST_ASSERT_EQ(gcode_get_mode(), GCODE_MODE_ABSOLUTE, "default mode should be absolute");
    
    /* 切换到相对模式 */
    ret = gcode_parse_line("G91", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G91 should parse successfully");
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G91 should execute successfully");
    TEST_ASSERT_EQ(gcode_get_mode(), GCODE_MODE_RELATIVE, "mode should be relative after G91");
    
    /* 切换回绝对模式 */
    ret = gcode_parse_line("G90", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G90 should parse successfully");
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "G90 should execute successfully");
    TEST_ASSERT_EQ(gcode_get_mode(), GCODE_MODE_ABSOLUTE, "mode should be absolute after G90");
    
    return 1;
}

/**
 * @brief   测试 M104/M109 温度命令执行
 * @note    验收标准: 4.1.5 - 支持 M104/M109 (热端温度)
 */
static int
test_execute_m104_m109(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 测试 M104 设置温度 */
    ret = gcode_parse_line("M104 S200", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M104 should parse successfully");
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M104 should execute successfully");
    
    /* 测试 M109 等待温度 */
    ret = gcode_parse_line("M109 S210", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M109 should parse successfully");
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M109 should execute successfully");
    
    return 1;
}

/**
 * @brief   测试 M106/M107 风扇命令执行
 * @note    验收标准: 4.1.6 - 支持 M106/M107 (风扇)
 */
static int
test_execute_m106_m107(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    /* 测试 M106 设置风扇速度 */
    ret = gcode_parse_line("M106 S255", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M106 should parse successfully");
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M106 should execute successfully");
    
    /* 测试 M106 半速 */
    ret = gcode_parse_line("M106 S127", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M106 half speed should parse successfully");
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M106 half speed should execute successfully");
    
    /* 测试 M107 关闭风扇 */
    ret = gcode_parse_line("M107", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M107 should parse successfully");
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M107 should execute successfully");
    
    return 1;
}

/**
 * @brief   测试 M114 位置查询命令执行
 * @note    验收标准: 4.1.7 - 支持 M114 (位置查询)
 */
static int
test_execute_m114(void)
{
    gcode_cmd_t cmd;
    int ret;
    
    ret = gcode_parse_line("M114", &cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M114 should parse successfully");
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_OK, "M114 should execute successfully");
    
    return 1;
}

/**
 * @brief   测试未知命令执行
 */
static int
test_execute_unknown_command(void)
{
    gcode_cmd_t cmd;
    
    /* 手动构造一个未知命令 */
    gcode_cmd_clear(&cmd);
    cmd.cmd = 'G';
    cmd.code = 999;
    
    int ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_UNKNOWN, "unknown G command should return GCODE_ERR_UNKNOWN");
    
    /* 测试未知 M 命令 */
    cmd.cmd = 'M';
    cmd.code = 999;
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_UNKNOWN, "unknown M command should return GCODE_ERR_UNKNOWN");
    
    /* 测试未知命令字母 */
    cmd.cmd = 'X';
    cmd.code = 0;
    
    ret = gcode_execute(&cmd);
    TEST_ASSERT_EQ(ret, GCODE_ERR_UNKNOWN, "unknown command letter should return GCODE_ERR_UNKNOWN");
    
    return 1;
}

/**
 * @brief   测试 gcode_respond 函数
 */
static int
test_gcode_respond(void)
{
    /* 测试正常响应 (在 TEST_BUILD 模式下不会实际输出) */
    gcode_respond("ok");
    gcode_respond("error: test");
    
    /* 测试 NULL 指针 (不应崩溃) */
    gcode_respond(NULL);
    
    return 1;
}

/* ========== 主函数 ========== */

int
main(void)
{
    printf("========================================\n");
    printf("  G-code Parser Unit Tests\n");
    printf("========================================\n\n");
    
    /* 初始化 */
    gcode_init();
    
    /* 运行解析测试 */
    printf("--- Parsing Tests ---\n");
    RUN_TEST(test_gcode_init);
    RUN_TEST(test_null_pointer);
    RUN_TEST(test_empty_line);
    RUN_TEST(test_comment_line);
    RUN_TEST(test_g0_g1_parsing);
    RUN_TEST(test_g28_parsing);
    RUN_TEST(test_g90_g91_parsing);
    RUN_TEST(test_m104_m109_parsing);
    RUN_TEST(test_m106_m107_parsing);
    RUN_TEST(test_m114_parsing);
    RUN_TEST(test_inline_comment);
    RUN_TEST(test_unknown_command);
    RUN_TEST(test_invalid_command);
    RUN_TEST(test_cmd_clear);
    RUN_TEST(test_whitespace_handling);
    
    /* 运行执行测试 */
    printf("\n--- Execution Tests ---\n");
    RUN_TEST(test_execute_null_pointer);
    RUN_TEST(test_execute_g0_g1);
    RUN_TEST(test_execute_g28);
    RUN_TEST(test_execute_g90_g91);
    RUN_TEST(test_execute_m104_m109);
    RUN_TEST(test_execute_m106_m107);
    RUN_TEST(test_execute_m114);
    RUN_TEST(test_execute_unknown_command);
    RUN_TEST(test_gcode_respond);
    
    /* 输出结果 */
    printf("\n========================================\n");
    printf("  Test Results\n");
    printf("========================================\n");
    printf("Total:  %d\n", g_tests_run);
    printf("Passed: %d\n", g_tests_passed);
    printf("Failed: %d\n", g_tests_failed);
    printf("========================================\n");
    
    return (g_tests_failed > 0) ? 1 : 0;
}
