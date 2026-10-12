/**
  ******************************************************************************
  * @file    test_heading.c
  * @brief   陀螺航向积分的 PC 端单元测试
  *
  * 用法（在 bcar 目录下）：
  *     gcc -I src/middleware -o build_test_heading.exe Tools/test_heading.c src/middleware/heading.c -lm
  *     ./build_test_heading.exe
  ******************************************************************************
  */
#include <stdio.h>
#include <math.h>
#include "heading.h"

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
        printf("  FAIL: %s（得到 %.4f，期望 %.4f ±%.4f）\n", what, got, want, tol);
    }
}

int main(void)
{
    printf("=== heading 单测 ===\n");

    /* ---- 1. 静止（无零偏无噪声）→ 航向不变 ---- */
    {
        int i;
        heading_init(0.0f);
        for (i = 0; i < 200; i++) {
            heading_step(0.0f, 0.005f);
        }
        check_near(heading_yaw_deg(), 0.0f, 0.0001f, "静止不动");
        check(heading_samples() == 200u, "样本计数");
    }

    /* ---- 2. 恒速 90dps 转 1 秒 → 90° ---- */
    {
        int i;
        heading_init(0.0f);
        for (i = 0; i < 200; i++) {          /* 200 × 5ms = 1s */
            heading_step(90.0f, 0.005f);
        }
        check_near(heading_yaw_deg(), 90.0f, 0.001f, "90dps 转 1 秒 = 90°");
    }

    /* ---- 3. 死区：小于死区的角速度一律当 0 ---- */
    {
        int i;
        heading_init(0.5f);                  /* 死区 0.5 dps */
        for (i = 0; i < 200; i++) {
            heading_step(0.3f, 0.005f);      /* 噪声级别 */
        }
        check_near(heading_yaw_deg(), 0.0f, 1e-6f, "死区内不积分");
        /* 超过死区就正常积分（0.6 - 0 = 0.6dps，不减去死区本身）*/
        heading_step(0.6f, 1.0f);
        check_near(heading_yaw_deg(), 0.6f, 0.001f, "超死区正常积分");
    }

    /* ---- 4. 零偏补偿：读数=零偏 → 不动 ---- */
    {
        int i;
        heading_init(0.0f);
        heading_set_bias(0.2f, -0.1f, 1.5f);
        for (i = 0; i < 200; i++) {
            heading_step(1.5f, 0.005f);      /* 正好等于零偏 */
        }
        check_near(heading_yaw_deg(), 0.0f, 1e-5f, "零偏被减掉后静止");
        check_near(heading_bias(2), 1.5f, 1e-6f, "读回零偏");
    }

    /* ---- 5. 负方向：反转 yaw 减小 ---- */
    {
        int i;
        heading_init(0.0f);
        for (i = 0; i < 200; i++) {
            heading_step(-45.0f, 0.005f);
        }
        check_near(heading_yaw_deg(), -45.0f, 0.001f, "反转 45dps 1 秒 = -45°");
    }

    /* ---- 6. 零偏标定：样本够 → 均值正确；不够 → 失败 ---- */
    {
        heading_cal_t c;
        float bias[3];
        int i;

        heading_cal_begin(&c);
        for (i = 0; i < 100; i++) {
            heading_cal_push(&c, 1.5f, -0.5f, 0.8f);
        }
        check(heading_cal_finish(&c, bias, 50u) == 1u, "样本够 → 标定成功");
        check_near(bias[0], 1.5f, 1e-5f, "标定 x 轴零偏");
        check_near(bias[1], -0.5f, 1e-5f, "标定 y 轴零偏");
        check_near(bias[2], 0.8f, 1e-5f, "标定 z 轴零偏");

        heading_cal_begin(&c);
        for (i = 0; i < 10; i++) {
            heading_cal_push(&c, 1.0f, 1.0f, 1.0f);
        }
        check(heading_cal_finish(&c, bias, 50u) == 0u, "样本不足 → 标定失败（不许用坏零偏）");

        heading_cal_begin(&c);
        check(heading_cal_finish(&c, bias, 1u) == 0u, "零样本 → 标定失败");
    }

    /* ---- 7. 标定+积分闭环：零偏 1.5dps 的传感器静止 10 秒也不飘 ---- */
    {
        int i;
        heading_cal_t c;
        float bias[3];
        heading_cal_begin(&c);
        for (i = 0; i < 100; i++) {
            heading_cal_push(&c, 0.0f, 0.0f, 1.5f);
        }
        check(heading_cal_finish(&c, bias, 50u) == 1u, "闭环标定成功");
        heading_init(0.3f);
        heading_set_bias(bias[0], bias[1], bias[2]);
        for (i = 0; i < 2000; i++) {          /* 10 秒 */
            heading_step(1.5f, 0.005f);
        }
        check_near(heading_yaw_deg(), 0.0f, 1e-4f, "标定后静止 10 秒不漂");
    }

    /* ---- 8. reset ---- */
    {
        heading_init(0.0f);
        heading_step(90.0f, 1.0f);
        heading_reset();
        check_near(heading_yaw_deg(), 0.0f, 1e-6f, "reset 清零航向");
    }

    printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
