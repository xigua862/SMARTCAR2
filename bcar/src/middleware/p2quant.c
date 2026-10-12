/**
  ******************************************************************************
  * @file    p2quant.c
  * @brief   P² 分位数估计实现（Jain & Chlamtac, 1985）
  *
  * 五个标记点：q[0]=最小、q[4]=最大，q[1..3] 分别代理
  *   np[1] = 1 + 2p·N   np[2] = 1 + 4p·N   np[3] = 3 + 2p·N
  * 每来一个样本：先定位落在哪个区间 → 更新 n/np → 用抛物线（必要时退化成直线）
  * 把 q[1..3] 挪一小步。
  ******************************************************************************
  */
#include "p2quant.h"

void p2_init(p2quant_t *q, float p)
{
    uint8_t i;

    if (q == 0) {
        return;
    }
    if (!(p > 0.0f && p < 1.0f)) {
        p = 0.95f;   /* 非法分位给个默认，别让后面除以 0 */
    }

    q->p = p;
    for (i = 0u; i < 5u; i++) {
        q->q[i]  = 0.0f;
        q->n[i]  = 0.0f;
        q->np[i] = 0.0f;
        q->init[i] = 0.0f;
    }
    /* 期望位置的初始值（论文式 (1)）*/
    q->np[0] = 1.0f;
    q->np[1] = 1.0f + 2.0f * p;
    q->np[2] = 1.0f + 4.0f * p;
    q->np[3] = 3.0f + 2.0f * p;
    q->np[4] = 5.0f;

    /* 每样本的期望位置增量（论文式 (2)）*/
    q->dn[0] = 0.0f;
    q->dn[1] = p * 0.5f;
    q->dn[2] = p;
    q->dn[3] = (1.0f + p) * 0.5f;
    q->dn[4] = 1.0f;

    q->init_cnt = 0u;
    q->count    = 0u;
}

void p2_reset(p2quant_t *q)
{
    float p;
    if (q == 0) {
        return;
    }
    p = q->p;
    p2_init(q, p);
}

/* 初始化：前 5 个样本排序后直接当标记点 */
static void p2_finish_init(p2quant_t *q)
{
    uint8_t i, j;
    /* 冒泡排 5 个数，比调 qsort 省得多也够用 */
    for (i = 0u; i < 5u; i++) {
        for (j = (uint8_t)(i + 1u); j < 5u; j++) {
            if (q->init[j] < q->init[i]) {
                float t = q->init[i];
                q->init[i] = q->init[j];
                q->init[j] = t;
            }
        }
    }
    for (i = 0u; i < 5u; i++) {
        q->q[i] = q->init[i];
        q->n[i] = (float)(i + 1u);
    }
}

void p2_push(p2quant_t *q, float x)
{
    uint8_t k;
    uint8_t i;

    if (q == 0) {
        return;
    }
    q->count++;

    /* 前 5 个样本只缓存，不做标记点更新 */
    if (q->init_cnt < 5u) {
        q->init[q->init_cnt] = x;
        q->init_cnt++;
        if (q->init_cnt == 5u) {
            p2_finish_init(q);
        }
        return;
    }

    /* 1) 定位落在哪个区间；落在两端就直接顶替端点 */
    if (x < q->q[0]) {
        q->q[0] = x;
        k = 0u;
    } else if (x < q->q[1]) {
        k = 0u;
    } else if (x < q->q[2]) {
        k = 1u;
    } else if (x < q->q[3]) {
        k = 2u;
    } else if (x <= q->q[4]) {
        k = 3u;
    } else {
        q->q[4] = x;
        k = 3u;
    }

    /* 2) k 右侧的所有标记点位置 +1 */
    for (i = (uint8_t)(k + 1u); i < 5u; i++) {
        q->n[i] += 1.0f;
    }

    /* 3) 期望位置全部按增量推进 */
    for (i = 0u; i < 5u; i++) {
        q->np[i] += q->dn[i];
    }

    /* 4) 中间三个标记点各挪一步（两端 q[0]/q[4] 就是极值，不用挪）*/
    for (i = 1u; i <= 3u; i++) {
        float d = q->np[i] - q->n[i];
        int8_t step = 0;

        if (d >= 1.0f && (q->n[i + 1u] - q->n[i]) > 1.0f) {
            step = 1;
        } else if (d <= -1.0f && (q->n[i - 1u] - q->n[i]) < -1.0f) {
            step = -1;
        } else {
            continue;
        }

        /* 抛物线插值（论文式 (5)）；两个分母为 0 时退化成直线 */
        {
            float dnm1 = q->n[i] - q->n[i - 1u];
            float dnp1 = q->n[i + 1u] - q->n[i];
            float denom = q->n[i + 1u] - q->n[i - 1u];

            if (denom > 0.0f && dnm1 > 0.0f && dnp1 > 0.0f) {
                float a = ((float)step) * (dnp1 + (float)step) *
                          (q->q[i + 1u] - q->q[i]) / dnp1;
                float b = ((float)step) * (dnm1 - (float)step) *
                          (q->q[i] - q->q[i - 1u]) / dnm1;
                q->q[i] += (a + b) / denom;
            } else {
                /* 退化：直接按相邻点线性走一步，别让分母 0 把值炸成 inf */
                float nb = (step > 0) ? dnp1 : dnm1;
                if (nb > 0.0f) {
                    q->q[i] += (float)step *
                               ((step > 0) ? (q->q[i + 1u] - q->q[i])
                                           : (q->q[i] - q->q[i - 1u])) / nb;
                }
            }
        }
        q->n[i] += (float)step;
    }
}

float p2_value(const p2quant_t *q)
{
    if (q == 0 || q->count == 0u) {
        return 0.0f;
    }
    if (q->init_cnt < 5u) {
        /* 样本不足：返回目前最大值（保守）*/
        float m = q->init[0];
        uint8_t i;
        for (i = 1u; i < q->init_cnt; i++) {
            if (q->init[i] > m) {
                m = q->init[i];
            }
        }
        return m;
    }
    return q->q[2];
}

uint32_t p2_count(const p2quant_t *q)
{
    return (q == 0) ? 0u : q->count;
}
