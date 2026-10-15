/**
  ******************************************************************************
  * @file    test_speed_pi.c
  * @brief   速度环 PI 的 PC 端单元测试
  *
  * 用法（在 bcar 目录下，或用 Tools\run_pc_tests.py）：
  *     gcc -I src/control -o build_test_speed_pi.exe Tools/test_speed_pi.c ^
  *         src/control/speed_pi.c -lm
  *
  * ★最要紧的两条：
  *   ① 车被"静摩擦"按住（输出很大也不动）时，积分要能把输出顶到破摩擦为止（不白绕、不爆炸）；
  *   ② 到了目标速度要稳住、不再往上顶（抗积分饱和）。
  ******************************************************************************
  */
#include <stdio.h>
#include <math.h>
#include "speed_pi.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(int cond, const char *what)
{
    if (cond) { g_pass++; }
    else { g_fail++; printf("  FAIL: %s\n", what); }
}

static void check_near(float got, float want, float tol, const char *what)
{
    if (fabsf(got - want) <= tol) { g_pass++; }
    else { g_fail++; printf("  FAIL: %s（得到 %.2f，期望 %.2f ±%.2f）\n", what, got, want, tol); }
}

/* 假车模型：占空比 out 要超过 friction 才动；速度 = (out - friction) * k */
static float plant(float out, float friction, float k)
{
    float v = (out - friction) * k;
    return (v < 0.0f) ? 0.0f : v;
}

int main(void)
{
    printf("=== 速度环 PI 单测 ===\n");

    /* ---- 1. 输出下限：起步时不许低于 out_min ---- */
    {
        speed_pi_t s;
        float out;
        speed_pi_init(&s, 1.0f, 5.0f, 380.0f, 1000.0f, 380.0f);
        out = speed_pi_step(&s, 300.0f, 0.0f, 0.005f);
        check(out >= 380.0f, "输出下限：起步第一拍就不低于 380");
    }

    /* ---- 2. ★破静摩擦：车不动（friction 600）时，积分要把输出顶到 >600 ---- */
    {
        speed_pi_t s;
        float out = 0.0f, v = 0.0f, out_break = 0.0f;
        int i;
        speed_pi_init(&s, 0.6f, 2.0f, 380.0f, 1000.0f, 380.0f);   /* ★增益要和 bcar_config.h 一致 */
        for (i = 0; i < 400; i++) {                 /* 2 秒 */
            out = speed_pi_step(&s, 300.0f, v, 0.005f);
            v   = plant(out, 600.0f, 0.5f);
            if (v > 0.0f && out_break == 0.0f) { out_break = out; }
        }
        check(out_break > 600.0f, "★静摩擦 600：积分把输出顶过 600（破摩擦）");
        check(v > 0.0f, "★破摩擦之后车真的动起来了");
    }

    /* ---- 3. 稳态：到了目标速度附近，输出稳住、不再无限上涨 ----
     * 注意假车的参数要能让目标速度"够得着"：v=(out-friction)*k，friction=400、k=1 ⇒
     * 到 300mm/s 需要 out=700（< 上限 1000）✓ 否则测的是"物理到不了"，不是控制器。 */
    {
        speed_pi_t s;
        float out = 0.0f, v = 0.0f, out_a = 0.0f;
        int i;
        speed_pi_init(&s, 0.6f, 2.0f, 380.0f, 1000.0f, 380.0f);   /* ★增益要和 bcar_config.h 一致 */
        for (i = 0; i < 600; i++) {                 /* 3 秒 */
            out = speed_pi_step(&s, 300.0f, v, 0.005f);
            v   = plant(out, 400.0f, 1.0f);
        }
        out_a = out;
        for (i = 0; i < 600; i++) {
            out = speed_pi_step(&s, 300.0f, v, 0.005f);
            v   = plant(out, 400.0f, 1.0f);
        }
        check_near(v, 300.0f, 30.0f, "稳态：实测速度跟到目标（±30mm/s）");
        check_near(out, out_a, 20.0f, "稳态：输出稳住了（后 3 秒不再漂）");
    }

    /* ---- 4. 抗积分饱和：输出顶到上限时，误差还在正方向也不再积分 ---- */
    {
        speed_pi_t s;
        float out;
        int i;
        speed_pi_init(&s, 1.0f, 50.0f, 380.0f, 1000.0f, 380.0f);
        for (i = 0; i < 2000; i++) {                /* 一直要求 300，但车一直不动（v=0）*/
            out = speed_pi_step(&s, 300.0f, 0.0f, 0.005f);
        }
        check_near(out, 1000.0f, 1.0f, "顶到上限就停在 1000（不会数值爆炸）");
    }

    /* ---- 5. 复位：进 RUN / 重新起步时积分清零 ---- */
    {
        speed_pi_t s;
        float out;
        speed_pi_init(&s, 0.6f, 2.0f, 380.0f, 1000.0f, 380.0f);   /* ★增益要和 bcar_config.h 一致 */
        (void)speed_pi_step(&s, 300.0f, 0.0f, 0.005f);
        (void)speed_pi_step(&s, 300.0f, 0.0f, 0.005f);
        speed_pi_reset(&s, 380.0f);
        out = speed_pi_step(&s, 300.0f, 300.0f, 0.005f);   /* 已经到目标速度 */
        check_near(out, 380.0f, 1.0f, "复位后：积分清零，输出从下限重新开始");
    }

    /* ---- 6. 反方向（车比目标快）：输出要往下收，且不低于下限 ---- */
    {
        speed_pi_t s;
        float out;
        speed_pi_init(&s, 1.2f, 6.0f, 380.0f, 1000.0f, 700.0f);
        out = speed_pi_step(&s, 300.0f, 600.0f, 0.005f);
        check(out < 700.0f, "车比目标快：输出往下收");
        check(out >= 380.0f, "但不会低于下限");
    }

    printf("  通过 %d / 失败 %d\n", g_pass, g_fail);
    return g_fail;
}
