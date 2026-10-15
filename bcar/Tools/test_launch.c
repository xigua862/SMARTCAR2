/**
  ******************************************************************************
  * @file    test_launch.c
  * @brief   发车读秒时序器的 PC 端单元测试
  *
  * 用法（在 bcar 目录下，或用 Tools\run_pc_tests.py）：
  *     gcc -I src/app -o build_test_launch.exe Tools/test_launch.c src/app/launch.c
  *
  * ★最要紧的两条：
  *   ① 3 秒里必须恰好出"短短长"三声，且顺序对、不重复报；
  *   ② 到 3000ms 必须给出 DONE（漏拍也不能漏起跑）。
  ******************************************************************************
  */
#include <stdio.h>
#include "launch.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(int cond, const char *what)
{
    if (cond) { g_pass++; }
    else { g_fail++; printf("  FAIL: %s\n", what); }
}

static const char *ev_name(launch_ev_t e)
{
    switch (e) {
    case LAUNCH_EV_NONE:       return "NONE";
    case LAUNCH_EV_BEEP_SHORT: return "SHORT";
    case LAUNCH_EV_BEEP_LONG:  return "LONG";
    case LAUNCH_EV_DONE:       return "DONE";
    default:                   return "?";
    }
}

int main(void)
{
    printf("=== 发车读秒时序器单测（3 秒 / 短短长 / 到点起跑）===\n");

    /* ---- 1. 没读秒时什么都不报 ---- */
    {
        launch_t l;
        launch_abort(&l);              /* 显式置成"没在读秒" */
        check(launch_step(&l, 0u) == LAUNCH_EV_NONE, "未读秒：step 报 NONE");
        check(launch_active(&l) == 0u, "未读秒：active=0");
        check(launch_elapsed_ms(&l, 12345u) == 0u, "未读秒：elapsed 报 0");
    }

    /* ---- 2. 第 1 声：刚开始就短鸣（不是等 1 秒才响）---- */
    {
        launch_t l;
        launch_begin(&l, 1000u);
        check(launch_active(&l) == 1u, "begin 后：active=1");
        check(launch_step(&l, 1000u) == LAUNCH_EV_BEEP_SHORT, "t+0：第 1 声 = 短鸣");
        check(launch_step(&l, 1000u) == LAUNCH_EV_NONE, "同一时刻再调：不重复报");
        check(launch_step(&l, 1005u) == LAUNCH_EV_NONE, "t+5ms：还是不报");
    }

    /* ---- 3. 第 2、3 声的时间点与"短短长"顺序 ---- */
    {
        launch_t l;
        launch_begin(&l, 0u);
        check(launch_step(&l, 0u)    == LAUNCH_EV_BEEP_SHORT, "t+0：短");
        check(launch_step(&l, 999u)  == LAUNCH_EV_NONE,        "t+999ms：还没到第 2 声");
        check(launch_step(&l, 1000u) == LAUNCH_EV_BEEP_SHORT,  "t+1000ms：第 2 声 = 短");
        check(launch_step(&l, 1999u) == LAUNCH_EV_NONE,        "t+1999ms：还没到第 3 声");
        check(launch_step(&l, 2000u) == LAUNCH_EV_BEEP_LONG,   "★t+2000ms：第 3 声 = 长");
        check(launch_step(&l, 2999u) == LAUNCH_EV_NONE,        "t+2999ms：三声已完，安静等起跑");
        check(launch_step(&l, 3000u) == LAUNCH_EV_DONE,        "★t+3000ms：DONE（起跑）");
        check(launch_active(&l) == 0u, "DONE 后：active 自动归 0");
        check(launch_step(&l, 3005u) == LAUNCH_EV_NONE, "DONE 之后：不再重复报");
        check(launch_step(&l, 99999u) == LAUNCH_EV_NONE, "很久之后：仍然是 NONE");
    }

    /* ---- 4. 5ms 拍走完整一遍：事件序列必须正好是 短 短 长 DONE ---- */
    {
        launch_t l;
        uint32_t t;
        int n_short = 0, n_long = 0, n_done = 0, n_other = 0;

        launch_begin(&l, 500u);
        for (t = 500u; t <= 500u + LAUNCH_TOTAL_MS + 50u; t += 5u) {
            launch_ev_t e = launch_step(&l, t);
            if (e == LAUNCH_EV_BEEP_SHORT)      { n_short++; }
            else if (e == LAUNCH_EV_BEEP_LONG)  { n_long++; }
            else if (e == LAUNCH_EV_DONE)       { n_done++; }
        }
        check(n_short == 2, "★整遍：恰好短鸣 2 声");
        check(n_long == 1,  "★整遍：恰好长鸣 1 声");
        check(n_done == 1,  "★整遍：DONE 恰好 1 次（不重复起跑）");
        (void)n_other;
    }

    /* ---- 5. 中途取消（按 PB0）：掐掉读秒，之后不再报 ---- */
    {
        launch_t l;
        launch_begin(&l, 0u);
        check(launch_step(&l, 0u) == LAUNCH_EV_BEEP_SHORT, "取消前：第 1 声照报");
        launch_abort(&l);
        check(launch_active(&l) == 0u, "取消后：active=0");
        check(launch_step(&l, 1000u) == LAUNCH_EV_NONE, "取消后：不再报任何声");
        check(launch_step(&l, 5000u) == LAUNCH_EV_NONE, "取消后：也不会冒出 DONE");

        /* 取消后重新发车，还得能完整走一遍（照 5ms 拍那样逐秒调）*/
        launch_begin(&l, 10000u);
        check(launch_step(&l, 10000u) == LAUNCH_EV_BEEP_SHORT, "重发：第 1 声回来了");
        check(launch_step(&l, 11000u) == LAUNCH_EV_BEEP_SHORT, "重发：第 2 声（短）");
        check(launch_step(&l, 12000u) == LAUNCH_EV_BEEP_LONG,  "重发：第 3 声（长）");
        check(launch_step(&l, 13000u) == LAUNCH_EV_DONE,       "重发：到点 DONE");
    }

    /* ---- 6. 主循环卡住漏拍：直接跳到 3500ms 也必须给 DONE（不能杵着不起跑）---- */
    {
        launch_t l;
        launch_begin(&l, 0u);
        check(launch_step(&l, 3500u) == LAUNCH_EV_DONE, "★漏拍到 3500ms：仍然给 DONE");
        check(launch_active(&l) == 0u, "漏拍 DONE 后：active=0");
    }

    /* ---- 7. 漏拍后追赶：一拍一拍补（每拍最多一件事）---- */
    {
        launch_t l;
        launch_begin(&l, 0u);
        check(launch_step(&l, 1500u) == LAUNCH_EV_BEEP_SHORT, "漏到 1500ms：先补第 1 声");
        check(launch_step(&l, 1500u) == LAUNCH_EV_BEEP_SHORT, "同一拍再调：补第 2 声");
        check(launch_step(&l, 1500u) == LAUNCH_EV_NONE,       "补完了：不越过 2000ms 那一档");
        check(launch_step(&l, 2100u) == LAUNCH_EV_BEEP_LONG,  "到 2100ms：第 3 声（长）");
        check(launch_step(&l, 3000u) == LAUNCH_EV_DONE,       "到 3000ms：DONE");
    }

    /* ---- 8. elapsed 读数 + 时间戳回绕（49.7 天）也要算得对 ---- */
    {
        launch_t l;
        launch_begin(&l, 100u);
        check(launch_elapsed_ms(&l, 1100u) == 1000u, "elapsed = 1000ms");

        launch_begin(&l, 0xFFFFFF00u);                     /* 快到回绕点了 */
        check(launch_elapsed_ms(&l, 0x00000100u) == 512u, "★回绕后 elapsed 仍算对（512ms）");
        check(launch_step(&l, 0x00000100u) == LAUNCH_EV_BEEP_SHORT, "回绕后：第 1 声照报");
    }

    /* ---- 9. 空指针不许崩（防御）---- */
    {
        launch_t l;
        launch_begin(0, 0u);
        launch_abort(0);
        check(launch_active(0) == 0u, "空指针：active 返回 0");
        check(launch_step(0, 0u) == LAUNCH_EV_NONE, "空指针：step 返回 NONE");
        check(launch_elapsed_ms(0, 0u) == 0u, "空指针：elapsed 返回 0");
        (void)l;
    }

    printf("  通过 %d / 失败 %d\n", g_pass, g_fail);
    (void)ev_name;   /* 调试时想打事件名再用 */
    return g_fail;
}
