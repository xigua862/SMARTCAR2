/**
  ******************************************************************************
  * @file    test_control.c
  * @brief   循迹控制环（转向 PD + 差速混合）的 PC 端单元测试
  *
  * 用法（在 bcar 目录下，或用 Tools\run_pc_tests.py）：
  *     gcc -I src/control -o build_test_control.exe Tools/test_control.c ^
  *         src/control/angle_pd.c src/control/mixer.c -lm
  *
  * ★最要紧的一条：**转向符号**。灰度 err 为正 = 线在车体右侧 ⇒ 必须"往右转"
  *   ⇒ 左轮快、右轮慢。这条错了，车一进 RUN 就往反方向冲出去。
  ******************************************************************************
  */
#include <stdio.h>
#include <math.h>
#include "angle_pd.h"
#include "mixer.h"

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

/* 走一套"完整链路"：err → PD → mixer */
static mix_out_t run_once(angle_pd_t *pd, float err, float base)
{
    float steer = angle_pd_step(pd, err);
    return mixer_step(base, steer, 50u);
}

int main(void)
{
    printf("=== 控制环单测（转向 PD + 差速混合）===\n");

    /* ---- 1. 零偏差 → 两轮同速、同向 ---- */
    {
        angle_pd_t pd;
        mix_out_t m;
        angle_pd_init(&pd, 25.0f, 3.0f, 600.0f);
        m = run_once(&pd, 0.0f, 400.0f);
        check(m.l.mag == m.r.mag, "零偏差：左右同速");
        check(m.l.dir == m.r.dir, "零偏差：左右同向");
        check(m.l.mag == 400u, "零偏差：力度等于 base");
    }

    /* ---- 2. ★核心：线偏右（err>0）→ 左轮快、右轮慢（往右转）---- */
    {
        angle_pd_t pd;
        mix_out_t m;
        angle_pd_init(&pd, 25.0f, 3.0f, 600.0f);
        m = run_once(&pd, 8.0f, 400.0f);     /* 线偏右 8mm */
        check(m.l.mag > m.r.mag, "★线偏右 → 左轮快、右轮慢（车往右转）");
        check(m.l.mag > 400u && m.r.mag < 400u, "★两侧一快一慢跨过 base");
    }

    /* ---- 3. 线偏左 → 反过来 ---- */
    {
        angle_pd_t pd;
        mix_out_t m;
        angle_pd_init(&pd, 25.0f, 3.0f, 600.0f);
        m = run_once(&pd, -8.0f, 400.0f);
        check(m.r.mag > m.l.mag, "线偏左 → 右轮快、左轮慢");
    }

    /* ---- 4. 转向量限幅 ---- */
    {
        angle_pd_t pd;
        float steer;
        angle_pd_init(&pd, 100.0f, 0.0f, 600.0f);
        steer = angle_pd_step(&pd, 20.0f);   /* 100×20 = 2000，远超 600 */
        check_near(steer, 600.0f, 0.01f, "转向量被限到 steer_max");
        steer = angle_pd_step(&pd, -20.0f);
        check_near(steer, -600.0f, 0.01f, "负方向同样限幅");
    }

    /* ---- 5. D 项：偏差突然变大 → 输出比只有 P 时更大 ---- */
    {
        angle_pd_t pd_p, pd_pd;
        float s_p, s_pd;
        angle_pd_init(&pd_p, 25.0f, 0.0f, 1000.0f);   /* 只有 P */
        angle_pd_init(&pd_pd, 25.0f, 5.0f, 1000.0f);  /* P + D */
        (void)angle_pd_step(&pd_p, 2.0f);
        (void)angle_pd_step(&pd_pd, 2.0f);
        s_p  = angle_pd_step(&pd_p, 8.0f);            /* 偏差从 2 跳到 8 */
        s_pd = angle_pd_step(&pd_pd, 8.0f);
        check(s_pd > s_p, "D 项在偏差突变时加大输出");
    }

    /* ---- 6. 超量程：按比例缩放，绝不超 1000，且不破坏"一快一慢" ---- */
    {
        mix_out_t m = mixer_step(900.0f, 500.0f, 50u);   /* 1400 / 400 */
        check(m.l.mag <= 1000u && m.r.mag <= 1000u, "超量程后两轮都不超 1000");
        check(m.l.mag == 1000u, "整体缩放后峰值恰好到 1000");
        check(m.l.mag > m.r.mag, "缩放后仍保持左快右慢（差速比没被破坏）");
    }

    /* ---- 7. 消隐下限：算出来很小的非零值 → 直接给 0 ---- */
    {
        mix_out_t m = mixer_step(30.0f, 0.0f, 50u);      /* 30 < 50 */
        check(m.l.mag == 0u && m.r.mag == 0u, "低于消隐下限 → 给 0（不是给一点点）");
        m = mixer_step(60.0f, 0.0f, 50u);
        check(m.l.mag == 60u, "刚好超过下限 → 正常给");
    }

    /* ---- 8. 反转：某轮算成负数 → 该轮反转，不是夹到 0 ---- */
    {
        mix_out_t m = mixer_step(100.0f, 300.0f, 50u);   /* 左 400 / 右 -200 */
        check(m.l.dir == MIX_DIR_FWD && m.l.mag == 400u, "左轮正转 400");
        check(m.r.dir == MIX_DIR_REV, "右轮算成负数 → 反转（坦克转向）");
        check(m.r.mag == 200u, "反转力度 = 绝对值");
    }

    /* ---- 9. 原地转：base=0 时也能给出纯差速 ---- */
    {
        mix_out_t m = mixer_step(0.0f, 300.0f, 50u);
        check(m.l.mag == 300u && m.r.mag == 300u, "原地转：两轮等力反向");
        check(m.l.dir != m.r.dir, "原地转：两轮转向相反");
    }

    /* ---- 10. PD 复位：丢线回来不拿旧偏差算微分 ---- */
    {
        angle_pd_t pd;
        float a, b;
        angle_pd_init(&pd, 25.0f, 5.0f, 1000.0f);
        (void)angle_pd_step(&pd, -20.0f);        /* 历史偏差 -20 */
        angle_pd_reset(&pd);                     /* 丢线回来复位 */
        a = angle_pd_step(&pd, 5.0f);            /* 复位后不该有巨大微分 */
        angle_pd_init(&pd, 25.0f, 5.0f, 1000.0f);
        b = angle_pd_step(&pd, 5.0f);            /* 干净初值 */
        check_near(a, b, 0.01f, "复位后与干净初值一致（无旧微分残留）");
    }

    printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
