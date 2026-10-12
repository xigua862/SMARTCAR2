/**
  ******************************************************************************
  * @file    p2quant.h
  * @brief   P² 分位数估计（**纯逻辑，无硬件依赖，可在 PC 上单测**）
  *
  * 要解决什么：遥测里要报"循迹偏差的 P95"这类指标，但
  *   ① C8T6 只有 20KB RAM，装不下整圈的采样再排序（一圈 1.2 万拍 × 4 字节 = 48KB）；
  *   ② 每拍都排序更不可能。
  *   P² 算法（Jain & Chlamtac 1985）用 **5 个标记点** 在线估计任意分位数，
  *   内存 O(1)、每样本 O(1)，精度足够给调参器当判据。
  *
  * 用法：p2_init(0.95) → 每拍 p2_push(x) → 随时 p2_value() 读 P95。
  ******************************************************************************
  */
#ifndef __MW_P2QUANT_H
#define __MW_P2QUANT_H

#include <stdint.h>

typedef struct {
    float    p;          /* 目标分位（0~1），如 0.95 */
    float    q[5];       /* 5 个标记点的值 */
    float    n[5];       /* 各标记点的实际位置（第几个样本）*/
    float    np[5];      /* 各标记点的期望位置 */
    float    dn[5];      /* 每来一个样本，期望位置的增量 */
    float    init[5];    /* 初始化阶段先缓 5 个 */
    uint8_t  init_cnt;   /* 已缓几个 */
    uint32_t count;      /* 总样本数 */
} p2quant_t;

/** @brief 初始化：p 取 0.5 / 0.95 / 0.99 之类 */
void p2_init(p2quant_t *q, float p);

/** @brief 喂一个样本 */
void p2_push(p2quant_t *q, float x);

/**
  * @brief  读当前估计值
  * @note   样本不足 5 个时返回"目前见过的最大值"（保守：作为 P95 的估计里
  *         宁可偏大也不要偏小 —— 偏小会让调参器误判"很稳"）。一个样本都没有返回 0。
  */
float p2_value(const p2quant_t *q);

/** @brief 已喂入的样本数 */
uint32_t p2_count(const p2quant_t *q);

/** @brief 清空（重新统计一圈）*/
void p2_reset(p2quant_t *q);

#endif /* __MW_P2QUANT_H */
