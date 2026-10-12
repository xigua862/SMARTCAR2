/**
  ******************************************************************************
  * @file    heading.h
  * @brief   陀螺航向积分（**纯逻辑，无硬件依赖，可在 PC 上单测**）
  *
  * 为什么要有"零偏"和"死区"两个东西 —— 惯导成败就在这两条：
  *   ① 零偏（bias）：芯片静止时输出也不完全是 0（零偏），直接积分会一直往一边飘。
  *      所以必须**上电静止标定**：采一两百个样本求平均，积分时先减掉它。
  *   ② 死区（dead zone）：减掉零偏后还有噪声，噪声积分 = 随机游走，车停着 yaw 也会爬。
  *      所以小于死区的角速度一律当 0。
  *
  * 本文件不许 #include 任何 stm32/HAL/驱动头文件 —— 硬纪律（保证 PC 可单测）。
  ******************************************************************************
  */
#ifndef __MW_HEADING_H
#define __MW_HEADING_H

#include <stdint.h>

typedef struct {
    float    gx_sum_f;      /* 标定累加用 */
    float    gy_sum_f;
    float    gz_sum_f;
    uint32_t n;
} heading_cal_t;

/** @brief 初始化：给死区（单位 dps）。静止噪声的典型量级是 0.1~0.5 dps */
void heading_init(float dead_zone_dps);

/** @brief 航向与累计样本清零 */
void heading_reset(void);

/** @brief 设置零偏（三个轴，dps）—— 由标定结果喂进来 */
void heading_set_bias(float gx, float gy, float gz);

/** @brief 读当前零偏（轴 0=x 1=y 2=z）*/
float heading_bias(uint8_t axis);

/** @brief 读当前死区 */
float heading_dead_zone(void);

/**
  * @brief  推进一拍（用 Z 轴角速度积分航向）
  * @param  gz_dps 陀螺 Z 轴读数（dps，未减零偏）
  * @param  dt_s   这一拍的时间（秒，5ms 拍 = 0.005）
  * @return 这一拍的航向增量 dθ（度）—— odom 直接拿它当中点法的转角
  */
float heading_step(float gz_dps, float dt_s);

/** @brief 当前航向（度，逆时针为正）*/
float heading_yaw_deg(void);

/** @brief 累计积分样本数 */
uint32_t heading_samples(void);

/* ---------------- 零偏标定（上电静止时做） ---------------- */

/** @brief 开始标定：清零累加器 */
void heading_cal_begin(heading_cal_t *c);

/** @brief 送一个静止样本（用原始 dps，别先减零偏）*/
void heading_cal_push(heading_cal_t *c, float gx, float gy, float gz);

/**
  * @brief  结束标定，算出零偏
  * @param  out          输出三个轴的零偏（dps）
  * @param  min_samples  至少要有这么多样本才算数（样本太少均值不可信）
  * @retval 1 = 标定成功；0 = 样本不足（此时 out 不动，调用方应报错）
  */
uint8_t heading_cal_finish(const heading_cal_t *c, float out[3], uint32_t min_samples);

#endif /* __MW_HEADING_H */
