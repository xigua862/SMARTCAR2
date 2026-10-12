/**
  ******************************************************************************
  * @file    test_parse.c
  * @brief   命令解析的 PC 端单元测试（不接板子就能跑）
  *
  * 用法（在 bcar 目录下，用任意 PC 版 gcc）：
  *     gcc -I src/app -o build_test_parse.exe Tools/test_parse.c src/app/str_util.c
  *     ./build_test_parse.exe
  *
  * 为什么值得：命令行解析是最容易写错的一层，而它又是"调车时唯一的手"。
  *   ★这里抓出过一个真 bug：先截断命令词再取参数 → "MOTOR FWD 500" 永远解析失败。
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
#include "str_util.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(int cond, const char *what)
{
    if (cond) {
        g_pass++;
    } else {
        g_fail++;
        printf("  FAIL: %s\n", what);
    }
}

/* 复制一份到可写缓冲：解析函数会就地改字符串 */
static void run_motor(const char *text, int expect_ok, int side, int dir, unsigned mag)
{
    char buf[64];
    su_motor_t m;
    int rc;

    strncpy(buf, text, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    su_upper(buf);

    rc = (int)su_parse_motor(buf, &m);
    if (!expect_ok) {
        check(rc == 0, text);
        return;
    }
    check(rc == 1, text);
    if (rc != 1) {
        return;
    }
    check((int)m.has_side == (side >= 0 ? 1 : 0), text);
    if (side >= 0) {
        check((int)m.side == side, text);
    }
    check((int)m.dir == dir, text);
    check(m.mag == mag, text);
}

int main(void)
{
    printf("=== str_util 单测 ===\n");

    /* ---- su_streq ---- */
    check(su_streq("MOTOR", "MOTOR") == 1, "streq 相等");
    check(su_streq("MOTOR", "MOTO") == 0, "streq 长短不同");
    check(su_streq("M", "MOTOR") == 0, "streq 前缀不算相等");
    check(su_streq("", "") == 1, "streq 空串");

    /* ---- su_upper ---- */
    {
        char b[16];
        strcpy(b, "mOtOr");
        su_upper(b);
        check(su_streq(b, "MOTOR") == 1, "upper 混合大小写");
    }

    /* ---- su_parse_u32 ---- */
    {
        uint32_t v = 0;
        check(su_parse_u32("500", &v) == 1 && v == 500, "u32 500");
        check(su_parse_u32("0", &v) == 1 && v == 0, "u32 0");
        check(su_parse_u32("1000", &v) == 1 && v == 1000, "u32 1000");
        check(su_parse_u32("", &v) == 0, "u32 空串应失败");
        check(su_parse_u32("500x", &v) == 0, "u32 尾随字符应失败");
        check(su_parse_u32("99999999999", &v) == 0, "u32 溢出应失败");
    }

    /* ---- su_word ---- */
    {
        char b[8];
        check(su_word("MOTOR FWD", b, sizeof(b)) == 5 && su_streq(b, "MOTOR") == 1, "word 取词");
        check(su_word("TOOLONGWORD", b, sizeof(b)) == 0, "word 放不下应失败");
    }

    /* ---- su_arg（不改缓冲） ---- */
    {
        char b[32];
        strcpy(b, "MOTOR L FWD 500");
        check(su_streq(su_arg(b, 0), "MOTOR L FWD 500") == 1, "arg0 指向整行");
        check(su_streq(su_arg(b, 1), "L FWD 500") == 1, "arg1");
        check(su_streq(su_arg(b, 2), "FWD 500") == 1, "arg2");
        check(su_streq(su_arg(b, 3), "500") == 1, "arg3");
        check(su_arg(b, 4) == 0, "arg4 应为空");
        check(su_streq(b, "MOTOR L FWD 500") == 1, "arg 不该改缓冲");
    }

    /* ---- su_parse_motor：两参数形式 ---- */
    printf("--- MOTOR 两参数形式 ---\n");
    run_motor("MOTOR FWD 500", 1, -1, SU_DIR_FWD, 500);
    run_motor("MOTOR REV 1000", 1, -1, SU_DIR_REV, 1000);
    run_motor("MOTOR BRAKE 0", 1, -1, SU_DIR_BRAKE, 0);
    run_motor("MOTOR COAST 0", 1, -1, SU_DIR_COAST, 0);
    run_motor("motor fwd 500", 1, -1, SU_DIR_FWD, 500);     /* 小写也行 */

    /* ---- su_parse_motor：三参数形式 ---- */
    printf("--- MOTOR 三参数形式 ---\n");
    run_motor("MOTOR L FWD 500", 1, 0, SU_DIR_FWD, 500);
    run_motor("MOTOR R REV 800", 1, 1, SU_DIR_REV, 800);
    run_motor("MOTOR L BRAKE 0", 1, 0, SU_DIR_BRAKE, 0);
    run_motor("MOTOR R COAST 0", 1, 1, SU_DIR_COAST, 0);

    /* ---- 非法输入必须被拒 ---- */
    printf("--- 非法输入 ---\n");
    run_motor("MOTOR", 0, -1, 0, 0);                  /* 缺参数 */
    run_motor("MOTOR FWD", 0, -1, 0, 0);              /* 缺力度 */
    run_motor("MOTOR XXX 500", 0, -1, 0, 0);          /* 方向词非法 */
    run_motor("MOTOR FWD 1001", 0, -1, 0, 0);         /* 力度越界 */
    run_motor("MOTOR L FWD", 0, -1, 0, 0);            /* 三参数缺一 */
    run_motor("MOTOR L 500", 0, -1, 0, 0);            /* L 后面跟数字，不是方向 */
    run_motor("MOTOR FWD 500x", 0, -1, 0, 0);         /* 力度带尾巴 */

    /* ---- 边界：mag 恰好 1000 / 0 ---- */
    run_motor("MOTOR FWD 1000", 1, -1, SU_DIR_FWD, 1000);
    run_motor("MOTOR FWD 0", 1, -1, SU_DIR_FWD, 0);

    /* ---- 多空格 / 前导空格要能容忍 ---- */
    run_motor("  MOTOR   FWD    500  ", 1, -1, SU_DIR_FWD, 500);

    printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
