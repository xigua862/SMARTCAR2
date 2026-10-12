/**
  ******************************************************************************
  * @file    lapstat.h
  * @brief   单圈统计累加器（**纯逻辑，无硬件依赖，可在 PC 上单测**）
  *
  * 一圈要报什么（对齐框架 §6.2 的 LAP 字段）：
  *   圈序号 / 用时 / 里程 / 平均速度 / 最高速度 / |偏差|均值 / |偏差|P95 / 丢线拍数
  *
  * 为什么要 P95 而不是"最大偏差"：最大值被单次异常（一次反光、一次震动）拉爆，
  * 拿它当判据会让调参器一直在追噪声；P95 才是"常态最坏情况"，适合当收敛判据。
  * 内存只有 O(1)（P² 算法），不存整圈数据。
  ******************************************************************************
  */
#ifndef __MW_LAPSTAT_H
#define __MW_LAPSTAT_H

#include <stdint.h>
#include "p2quant.h"

typedef struct {
    uint32_t lap_index;      /* 第几圈（从 1 开始）*/
    uint32_t t_ms;           /* 本圈用时 */
    uint32_t ticks;          /* 本圈拍数（5ms 一拍）*/
    float    s_mm;           /* 本圈里程 */
    float    v_max_mm_s;     /* 最高速度 */
    float    v_avg_mm_s;     /* 平均速度 = 里程 / 时间 */
    float    abs_err_mean_mm;/* |偏差| 均值 */
    float    abs_err_p95_mm; /* |偏差| P95（调参器的主判据）*/
    uint16_t lost_ticks;     /* 丢线拍数（越少越好，一次丢线可能就是冲出赛道）*/
} lap_record_t;

typedef struct {
    p2quant_t err_abs_p95;   /* |err| 的 P95 估计 */
    uint32_t  ticks;
    uint32_t  t_ms;
    float     s_mm;
    float     v_max;
    float     v_sum;
    float     abs_err_sum;
    uint16_t  lost_ticks;
} lapstat_t;

/* 少于这么多拍的一圈不算数（防止误触发把 0.05 秒当一圈报出去）*/
#define LAPSTAT_MIN_TICKS  20u

/** @brief 清零，准备累积新的一圈 */
void lapstat_reset(lapstat_t *s);

/**
  * @brief  每拍喂一次
  * @param  err_mm  循迹偏差（毫米；丢线时也照样传，由 visible 决定是否计入）
  * @param  visible 1 = 看到线
  * @param  v_mm_s  当前速度
  * @param  ds_mm   这一拍走过的弧长增量
  * @param  dt_ms   这一拍的时间（5ms 拍传 5）
  */
void lapstat_push(lapstat_t *s, float err_mm, uint8_t visible,
                  float v_mm_s, float ds_mm, uint32_t dt_ms);

/**
  * @brief  结算这一圈
  * @param  out 结算结果（lap_index 由调用方填）
  * @retval 1 = 有效（拍数够）；0 = 太短，不算一圈（out 不动）
  */
uint8_t lapstat_finish(const lapstat_t *s, lap_record_t *out);

#endif /* __MW_LAPSTAT_H */
