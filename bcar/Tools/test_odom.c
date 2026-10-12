/**
  ******************************************************************************
  * @file    test_odom.c
  * @brief   里程/位姿解算的 PC 端单元测试
  *
  * 用法（在 bcar 目录下，用 PC 版 gcc，math 库要 -lm）：
  *     gcc -I src/middleware -o build_test_odom.exe Tools/test_odom.c src/middleware/odom.c -lm
  *     ./build_test_odom.exe
  ******************************************************************************
  */
#include <stdio.h>
#include <math.h>
#include "odom.h"

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

static void check_near(float got, float want, float tol, const char *what)
{
    if (fabsf(got - want) <= tol) {
        g_pass++;
    } else {
        g_fail++;
        printf("  FAIL: %s（得到 %.3f，期望 %.3f ±%.3f）\n", what, got, want, tol);
    }
}

/* 标定成"1 个计数 = 0.1mm"，方便手算校验 */
static odom_cfg_t cfg_01mm(void)
{
    odom_cfg_t c;
    c.mm_per_count = 0.1f;
    c.track_mm     = 110.0f;
    c.dt_s         = 0.005f;
    return c;
}

int main(void)
{
    printf("=== odom 单测 (mm_per_count=0.1, 轮距 110mm, dt=5ms) ===\n");

    /* ---- 1. 直走：左右各 1000 计数 = 各 100mm ---- */
    {
        odom_cfg_t c = cfg_01mm();
        odom_state_t s;
        odom_init(&c);
        odom_step(1000, 1000, 0, 0.0f);
        s = odom_get();
        check_near(s.x_mm, 100.0f, 0.01f, "直走 x");
        check_near(s.y_mm, 0.0f, 0.01f, "直走 y");
        check_near(s.yaw_deg, 0.0f, 0.01f, "直走 yaw");
        check_near(s.s_mm, 100.0f, 0.01f, "直走 s");
        check_near(s.v_mm_s, 20000.0f, 1.0f, "直走速度 = 100mm/0.005s");
    }

    /* ---- 2. 原地转（左右反向）：ds=0，只有 yaw 变 ---- */
    {
        odom_cfg_t c = cfg_01mm();
        odom_state_t s;
        odom_init(&c);
        /* 左轮 -550 计数(-55mm)、右轮 +550(+55mm) → 弧差 110mm = 一个轮距
         * → 差速推算转角 = 110/110 = 1 rad ≈ 57.3° */
        odom_step(-550, 550, 0, 0.0f);
        s = odom_get();
        check_near(s.x_mm, 0.0f, 0.01f, "原地转 x 不动");
        check_near(s.y_mm, 0.0f, 0.01f, "原地转 y 不动");
        check_near(s.yaw_deg, 57.2958f, 0.05f, "原地转 yaw = 1 rad = 57.3°");
        check_near(s.s_mm, 0.0f, 0.01f, "原地转 s = 0");
    }

    /* ---- 3. 陀螺优先：给了陀螺就完全按陀螺算 yaw ---- */
    {
        odom_cfg_t c = cfg_01mm();
        odom_state_t s;
        odom_init(&c);
        odom_step(1000, 1000, 1, 30.0f);   /* 陀螺说转了 30°*/
        s = odom_get();
        check_near(s.yaw_deg, 30.0f, 0.01f, "陀螺模式 yaw 用陀螺值");
        check_near(s.x_mm, 100.0f * cosf(15.0f * 3.14159265f / 180.0f), 0.05f,
                   "陀螺模式按中点法前进（15°方向）");
    }

    /* ---- 4. 走一个直角：先直走 100mm，再原地左转 90°，再直走 100mm ---- */
    {
        odom_cfg_t c = cfg_01mm();
        odom_state_t s;
        odom_init(&c);
        odom_step(1000, 1000, 0, 0.0f);          /* 直走 100mm */
        /* 原地左转 90°：dθ = 90° = π/2 rad → 弧差 = (π/2)*110 ≈ 172.8mm
         * 左右各 ±86.4mm → ±864 计数 */
        odom_step(-864, 864, 0, 0.0f);
        odom_step(1000, 1000, 0, 0.0f);          /* 再直走 100mm */
        s = odom_get();
        check_near(s.yaw_deg, 90.0f, 0.3f, "直角 yaw ≈ 90°");
        check_near(s.x_mm, 100.0f, 1.0f, "直角后 x ≈ 100（第二段朝 y 走）");
        check_near(s.y_mm, 100.0f, 1.5f, "直角后 y ≈ 100");
        check_near(s.s_mm, 200.0f, 0.5f, "累计弧长 ≈ 200（转角那段 ds≈0）");
    }

    /* ---- 5. 倒退：负计数 ---- */
    {
        odom_cfg_t c = cfg_01mm();
        odom_state_t s;
        odom_init(&c);
        odom_step(-500, -500, 0, 0.0f);
        s = odom_get();
        check_near(s.x_mm, -50.0f, 0.01f, "倒退 x 为负");
        check_near(s.s_mm, -50.0f, 0.01f, "倒退 s 累计为负");
    }

    /* ---- 6. 累加性：分 100 小步 = 一大步 ---- */
    {
        odom_cfg_t c = cfg_01mm();
        odom_state_t a, b;
        int i;
        odom_init(&c);
        odom_step(1000, 1000, 0, 0.0f);
        a = odom_get();

        odom_init(&c);
        for (i = 0; i < 100; i++) {
            odom_step(10, 10, 0, 0.0f);
        }
        b = odom_get();
        check_near(b.x_mm, a.x_mm, 0.01f, "分步 100 次 = 一次（x）");
        check_near(b.s_mm, a.s_mm, 0.01f, "分步 100 次 = 一次（s）");
    }

    /* ---- 7. 防护：轮距为 0 时不许出 NaN ---- */
    {
        odom_cfg_t c = cfg_01mm();
        odom_state_t s;
        c.track_mm = 0.0f;
        odom_init(&c);
        odom_step(-500, 500, 0, 0.0f);
        s = odom_get();
        check(s.yaw_deg == s.yaw_deg, "轮距 0 时 yaw 不是 NaN");
        check(s.x_mm == s.x_mm, "轮距 0 时 x 不是 NaN");
    }

    /* ---- 8. odom_reset 清零 ---- */
    {
        odom_cfg_t c = cfg_01mm();
        odom_state_t s;
        odom_init(&c);
        odom_step(1000, 1000, 0, 0.0f);
        odom_reset();
        s = odom_get();
        check(s.x_mm == 0.0f && s.s_mm == 0.0f && s.yaw_deg == 0.0f, "reset 清零");
    }

    printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
