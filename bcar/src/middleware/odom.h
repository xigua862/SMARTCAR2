/**
  ******************************************************************************
  * @file    odom.h
  * @brief   里程与位姿解算（**纯逻辑，无硬件依赖，可在 PC 上单测**）
  *
  * 口径（对齐框架 §4.2）：
  *   编码器增量 → 左右轮位移 → 弧长 ds + 航向增量 dθ → 位姿 (x, y, yaw)
  *   ★航向优先用陀螺积分（惯导为主）；没有陀螺时退化成差速推算（dθ = Δs_diff / 轮距）。
  *
  * 本文件不许 #include 任何 stm32/HAL/驱动头文件 —— 硬纪律，
  * 保证 Tools\test_odom.c 能用普通 gcc 直接编译运行。
  ******************************************************************************
  */
#ifndef __MW_ODOM_H
#define __MW_ODOM_H

#include <stdint.h>

typedef struct {
    float mm_per_count;   /* ★必须推车 1m 实测标定：1000 / 推 1m 的计数 */
    float track_mm;       /* 轮距（左右轮中心距）= 110mm（实测真值）*/
    float dt_s;           /* 调用周期（秒），5ms 拍 = 0.005 */
} odom_cfg_t;

typedef struct {
    float x_mm;        /* 车体位置（起点为原点，x 朝发车方向）*/
    float y_mm;
    float yaw_deg;     /* 航向角，逆时针为正，起点 0 */
    float s_mm;        /* 累计弧长（沿路径走过的距离）*/
    float v_mm_s;      /* 当前速度（用最近一拍的 ds 算）*/
} odom_state_t;

/** @brief 初始化（清零位姿 + 存配置）*/
void odom_init(const odom_cfg_t *cfg);

/**
  * @brief  推进一步
  * @param  dL_counts 左轮这一拍的计数增量（可正可负）
  * @param  dR_counts 右轮这一拍的计数增量
  * @param  has_gyro  1 = 有陀螺，用 dtheta_deg 当航向增量；0 = 用差速推算
  * @param  dtheta_deg 这一拍的航向增量（度，陀螺积分得到；has_gyro=0 时忽略）
  */
void odom_step(int16_t dL_counts, int16_t dR_counts,
               uint8_t has_gyro, float dtheta_deg);

/** @brief 读当前位姿 */
odom_state_t odom_get(void);

/** @brief 位姿清零（重新标定起点用）*/
void odom_reset(void);

#endif /* __MW_ODOM_H */
