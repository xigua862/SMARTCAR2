/**
  ******************************************************************************
  * @file    test_p2quant.c
  * @brief   P² 分位数估计的 PC 端单元测试
  *
  * 用法（在 bcar 目录下）：
  *     gcc -I src/middleware -o build_test_p2quant.exe Tools/test_p2quant.c src/middleware/p2quant.c -lm
  *     ./build_test_p2quant.exe
  *
  * 测试思路：造一批已知数据，用"排序后取真实分位"当标准答案，看 P² 估得准不准。
  ******************************************************************************
  */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "p2quant.h"

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

static void check_rel(float got, float want, float tol_frac, const char *what)
{
    float den = (fabsf(want) > 1e-6f) ? fabsf(want) : 1.0f;
    float rel = fabsf(got - want) / den;
    if (rel <= tol_frac) {
        g_pass++;
    } else {
        g_fail++;
        printf("  FAIL: %s（得到 %.2f，真值 %.2f，相对误差 %.2f%% > %.2f%%）\n",
               what, got, want, rel * 100.0f, tol_frac * 100.0f);
    }
}

static int cmp_float(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return (x < y) ? -1 : (x > y) ? 1 : 0;
}

/* 排序后取真实分位（线性插值）*/
static float exact_quantile(float *data, int n, float p)
{
    float idx, frac;
    int   lo;
    qsort(data, (size_t)n, sizeof(float), cmp_float);
    idx = p * (float)(n - 1);
    lo  = (int)idx;
    frac = idx - (float)lo;
    if (lo + 1 >= n) {
        return data[n - 1];
    }
    return data[lo] * (1.0f - frac) + data[lo + 1] * frac;
}

/* 确定性伪随机（不用 rand()，免得平台间结果不一致）*/
static uint32_t g_seed = 12345u;
static float rnd01(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return (float)((g_seed >> 8) & 0xFFFFu) / 65535.0f;
}

int main(void)
{
    printf("=== p2quant 单测 ===\n");

    /* ---- 1. 等差数列 1..10000：真值可手算 ---- */
    {
        p2quant_t q;
        int i;
        p2_init(&q, 0.95f);
        for (i = 1; i <= 10000; i++) {
            p2_push(&q, (float)i);
        }
        /* 真值：1..10000 的 P95 ≈ 9500.5（线性插值）*/
        check_rel(p2_value(&q), 9500.5f, 0.02f, "等差 1..10000 的 P95");
        check(p2_count(&q) == 10000u, "样本计数");
    }

    /* ---- 2. 同一个等差序列上估 P50 ---- */
    {
        p2quant_t q;
        int i;
        p2_init(&q, 0.50f);
        for (i = 1; i <= 10000; i++) {
            p2_push(&q, (float)i);
        }
        check_rel(p2_value(&q), 5000.5f, 0.02f, "等差列的 P50");
    }

    /* ---- 3. 均匀分布：和排序真值比 ---- */
    {
        enum { N = 20000 };
        static float data[N];
        p2quant_t q;
        int i;
        float truth95, truth50;

        g_seed = 12345u;
        for (i = 0; i < N; i++) {
            data[i] = rnd01();
        }
        /* 先算真值（exact_quantile 会排序，所以要在喂之前先拷一份）*/
        {
            static float copy[N];
            for (i = 0; i < N; i++) { copy[i] = data[i]; }
            truth95 = exact_quantile(copy, N, 0.95f);
            for (i = 0; i < N; i++) { copy[i] = data[i]; }
            truth50 = exact_quantile(copy, N, 0.50f);
        }

        p2_init(&q, 0.95f);
        for (i = 0; i < N; i++) {
            p2_push(&q, data[i]);
        }
        check_rel(p2_value(&q), truth95, 0.02f, "均匀分布 P95 vs 排序真值");

        p2_init(&q, 0.50f);
        for (i = 0; i < N; i++) {
            p2_push(&q, data[i]);
        }
        check_rel(p2_value(&q), truth50, 0.02f, "均匀分布 P50 vs 排序真值");
    }

    /* ---- 4. 带长尾：P99（调参器关心"最坏情况"那一档）---- */
    {
        enum { N = 20000 };
        static float data[N];
        static float copy[N];
        p2quant_t q;
        int i;
        float truth;

        g_seed = 777u;
        for (i = 0; i < N; i++) {
            /* 90% 小值 + 10% 尖峰：模拟"大部分很稳、偶尔一次大偏差"*/
            float u = rnd01();
            data[i] = (u < 0.9f) ? (u * 10.0f) : (100.0f + u * 50.0f);
        }
        for (i = 0; i < N; i++) { copy[i] = data[i]; }
        truth = exact_quantile(copy, N, 0.99f);

        p2_init(&q, 0.99f);
        for (i = 0; i < N; i++) {
            p2_push(&q, data[i]);
        }
        check_rel(p2_value(&q), truth, 0.05f, "长尾分布 P99（容差 5%）");
    }

    /* ---- 5. 单调递增：标记点必须跟得上，不能卡死 ---- */
    {
        p2quant_t q;
        int i;
        p2_init(&q, 0.95f);
        for (i = 0; i < 5000; i++) {
            p2_push(&q, (float)i);
        }
        /* 最后一拍的值应该接近最后一个样本 */
        check(p2_value(&q) > 4700.0f, "单调递增时 P95 跟得上");
        check(p2_value(&q) <= 5000.0f, "P95 不会超过最大值");
    }

    /* ---- 6. 样本不足 5 个：返回当前最大值（保守）---- */
    {
        p2quant_t q;
        p2_init(&q, 0.95f);
        check(p2_value(&q) == 0.0f, "零样本返回 0");
        p2_push(&q, 3.0f);
        p2_push(&q, 9.0f);
        p2_push(&q, 1.0f);
        check(p2_value(&q) == 9.0f, "样本<5 时返回最大值");
        check(p2_count(&q) == 3u, "不足 5 个时的计数正确");
    }

    /* ---- 7. 全部相同的值：不该出 NaN 或乱跳 ---- */
    {
        p2quant_t q;
        int i;
        p2_init(&q, 0.95f);
        for (i = 0; i < 1000; i++) {
            p2_push(&q, 42.0f);
        }
        check(p2_value(&q) == 42.0f, "常数序列 P95 恒等于该常数");
        check(p2_value(&q) == p2_value(&q), "常数序列不是 NaN");
    }

    /* ---- 8. reset 后重新统计 ---- */
    {
        p2quant_t q;
        int i;
        p2_init(&q, 0.95f);
        for (i = 0; i < 1000; i++) {
            p2_push(&q, 1000.0f);
        }
        check_rel(p2_value(&q), 1000.0f, 0.01f, "reset 前");
        p2_reset(&q);
        check(p2_count(&q) == 0u, "reset 清计数");
        for (i = 0; i < 1000; i++) {
            p2_push(&q, 5.0f);
        }
        check_rel(p2_value(&q), 5.0f, 0.01f, "reset 后按新数据统计");
    }

    /* ---- 9. 非法分位参数要兜住 ---- */
    {
        p2quant_t q;
        int i;
        p2_init(&q, 1.5f);      /* 非法 → 应该悄悄换成默认 0.95 */
        for (i = 1; i <= 10000; i++) {
            p2_push(&q, (float)i);
        }
        check(p2_value(&q) == p2_value(&q), "非法分位不产生 NaN");
        check(p2_value(&q) > 8000.0f, "非法分位按默认 0.95 处理");
    }

    printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
