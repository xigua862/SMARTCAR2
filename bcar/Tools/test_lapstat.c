/**
  ******************************************************************************
  * @file    test_lapstat.c
  * @brief   单圈统计累加器的 PC 端单元测试
  *
  * 用法（在 bcar 目录下）：
  *     gcc -I src/middleware -o build_test_lapstat.exe Tools/test_lapstat.c ^
  *         src/middleware/lapstat.c src/middleware/p2quant.c -lm
  ******************************************************************************
  */
#include <stdio.h>
#include <math.h>
#include "lapstat.h"

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

/* 模拟一圈：每拍走 ds mm、速度 v、偏差 err */
static void feed(lapstat_t *s, int ticks, float v, float ds, float err, int visible)
{
    int i;
    for (i = 0; i < ticks; i++) {
        lapstat_push(s, err, (uint8_t)visible, v, ds, 5u);
    }
}

int main(void)
{
    lapstat_t   s;
    lap_record_t r;

    printf("=== lapstat 单测 ===\n");

    /* ---- 1. 稳定一圈：速度恒定、偏差恒定 ---- */
    {
        lapstat_reset(&s);
        /* 200 拍 = 1000ms；每拍 6mm → 1200mm；速度 1200mm/s；偏差恒 5mm */
        feed(&s, 200, 1200.0f, 6.0f, 5.0f, 1);
        check(lapstat_finish(&s, &r) == 1u, "有效圈");
        check(r.ticks == 200u, "拍数");
        check(r.t_ms == 1000u, "用时 1000ms");
        check_near(r.s_mm, 1200.0f, 0.01f, "里程 1200mm");
        check_near(r.v_max_mm_s, 1200.0f, 0.01f, "最高速");
        check_near(r.v_avg_mm_s, 1200.0f, 0.01f, "平均速 = 1200mm/s");
        check_near(r.abs_err_mean_mm, 5.0f, 0.01f, "|偏差|均值");
        check_near(r.abs_err_p95_mm, 5.0f, 0.01f, "常数偏差的 P95");
        check(r.lost_ticks == 0u, "没有丢线");
    }

    /* ---- 2. 偏差正负都算绝对值 ---- */
    {
        lapstat_reset(&s);
        feed(&s, 100, 500.0f, 3.0f, -7.0f, 1);   /* 全是负偏差 */
        check(lapstat_finish(&s, &r) == 1u, "负偏差圈有效");
        check_near(r.abs_err_mean_mm, 7.0f, 0.01f, "负偏差取绝对值");
        check_near(r.abs_err_p95_mm, 7.0f, 0.01f, "负偏差 P95");
    }

    /* ---- 3. ★丢线拍不计入偏差统计（否则会把指标做好看）---- */
    {
        lapstat_reset(&s);
        feed(&s, 100, 800.0f, 4.0f, 3.0f, 1);    /* 100 拍看见线，偏差 3mm */
        feed(&s, 100, 800.0f, 4.0f, 0.0f, 0);    /* 100 拍丢线（err 恒 0）*/
        check(lapstat_finish(&s, &r) == 1u, "含丢线的圈有效");
        check(r.lost_ticks == 100u, "丢线拍数统计");
        check_near(r.abs_err_mean_mm, 3.0f, 0.01f, "丢线不拉低 |偏差|均值");
        check_near(r.abs_err_p95_mm, 3.0f, 0.01f, "丢线不拉低 P95");
        /* 平均速度照算（含丢线的时间）*/
        check_near(r.v_avg_mm_s, 800.0f, 0.01f, "平均速度不被丢线影响");
    }

    /* ---- 4. 速度有起伏：最高/平均要对 ---- */
    {
        lapstat_reset(&s);
        /* 100 拍 @ 1000mm/s、100 拍 @ 2000mm/s */
        feed(&s, 100, 1000.0f, 5.0f, 1.0f, 1);
        feed(&s, 100, 2000.0f, 10.0f, 1.0f, 1);
        check(lapstat_finish(&s, &r) == 1u, "变速圈有效");
        check_near(r.v_max_mm_s, 2000.0f, 0.01f, "最高速 = 2000");
        /* 里程 = 100*5 + 100*10 = 1500mm，用时 1000ms → 1500mm/s */
        check_near(r.v_avg_mm_s, 1500.0f, 0.01f, "平均速度 = 1500mm/s");
        check_near(r.s_mm, 1500.0f, 0.01f, "里程 1500mm");
    }

    /* ---- 5. 太短的一圈不算（防误触发）---- */
    {
        lapstat_reset(&s);
        feed(&s, LAPSTAT_MIN_TICKS - 1, 500.0f, 2.0f, 1.0f, 1);
        check(lapstat_finish(&s, &r) == 0u, "太短 → 无效圈");
        /* 刚好够 */
        lapstat_reset(&s);
        feed(&s, LAPSTAT_MIN_TICKS, 500.0f, 2.0f, 1.0f, 1);
        check(lapstat_finish(&s, &r) == 1u, "刚好够 → 有效圈");
    }

    /* ---- 6. P95 要能把"偶发大偏差"反映出来，但不被单次异常拉爆 ---- */
    {
        lapstat_reset(&s);
        feed(&s, 1000, 1000.0f, 5.0f, 2.0f, 1);     /* 常态 2mm */
        /* 10 拍大偏差 40mm（占 1%）—— 只在 P95 之上，不该影响 P95 太多 */
        {
            int i;
            for (i = 0; i < 10; i++) {
                lapstat_push(&s, 40.0f, 1u, 1000.0f, 5.0f, 5u);
            }
        }
        check(lapstat_finish(&s, &r) == 1u, "含尖峰圈有效");
        check(r.abs_err_p95_mm < 10.0f, "P95 不被 1% 的尖峰拉爆");
        check(r.abs_err_p95_mm >= 2.0f, "P95 至少在常态值附近");
        /* 但均值会被拉高一点（10/1010 拍 × 40mm ≈ 0.38mm 增量）*/
        check(r.abs_err_mean_mm > 2.0f, "均值能反映出尖峰存在");
    }

    /* ---- 7. 全丢线：不该出 NaN，均值给 0 ---- */
    {
        lapstat_reset(&s);
        feed(&s, 100, 300.0f, 1.0f, 0.0f, 0);
        check(lapstat_finish(&s, &r) == 1u, "全丢线仍是有效圈（用来报警）");
        check(r.lost_ticks == 100u, "全丢线拍数");
        check(r.abs_err_mean_mm == r.abs_err_mean_mm, "均值不是 NaN");
        check_near(r.abs_err_mean_mm, 0.0f, 0.001f, "无有效偏差时均值为 0");
    }

    /* ---- 8. reset 后从零开始，前一圈不影响后一圈 ---- */
    {
        lapstat_reset(&s);
        feed(&s, 200, 1000.0f, 5.0f, 20.0f, 1);
        (void)lapstat_finish(&s, &r);
        lapstat_reset(&s);
        feed(&s, 200, 1000.0f, 5.0f, 1.0f, 1);
        check(lapstat_finish(&s, &r) == 1u, "第二圈有效");
        check_near(r.abs_err_p95_mm, 1.0f, 0.05f, "第二圈 P95 从零重算（不带上一圈）");
        check(r.lost_ticks == 0u, "第二圈丢线计数从零");
    }

    printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
