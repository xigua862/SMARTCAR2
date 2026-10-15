/**
  ******************************************************************************
  * @file    speed_pi.h
  * @brief   速度环 PI（用编码器实测速度当反馈）—— **纯逻辑，无硬件依赖，可在 PC 上单测**
  *
  * 为什么要有它（2026-10-15 深夜加）：
  *   这台车**静摩擦特别大**：同一个占空比，起步时能转、巡航时可能就停住
  *   （用户实测："起步之后就停了"），而且换一次电池/换一段地面能跑的速度就变一次。
  *   手动调 BASE 等于每次都重新猜 —— 速度环让**积分项自动把这个差额补上**：
  *   车慢了就把占空比往上加，直到实测速度追上目标速度。
  *
  *   这就是 ADRC 想干的事（把"摩擦"当成扰动量抵消掉），但用编码器闭环做，
  *   **参数从 6 个降到 2 个**，赛前才调得动。
  *
  * 为什么输出要有个"下限"（out_min）：这台车占空比低于 ~300 根本推不动（实测），
  *   给它自由输出到 0 没有意义，还会让积分白绕。
  *
  * 抗积分饱和：条件积分 —— 输出顶到上下限时**只往"能退回来"的方向积分**。
  *
  * 本文件不许 #include 任何 stm32/HAL/驱动头文件 —— 硬纪律（保证 PC 可单测）。
  ******************************************************************************
  */
#ifndef __CTRL_SPEED_PI_H
#define __CTRL_SPEED_PI_H

#include <stdint.h>

typedef struct {
    float kp;        /* mag / (mm/s) */
    float ki;        /* mag / mm（误差按"毫米"积分，即 err×dt 的累加）*/
    float out_min;   /* 输出下限（这台车 ~300 以下推不动）*/
    float out_max;   /* 输出上限（一般 1000）*/
    float integ;     /* 积分累加器（单位：mm）*/
    float out;       /* 上一次输出 */
} speed_pi_t;

/** @brief 初始化（out_init = 从哪个输出起步，一般给 BASE 或 out_min）*/
void speed_pi_init(speed_pi_t *s, float kp, float ki, float out_min, float out_max, float out_init);

/** @brief 现场改增益 */
void speed_pi_set(speed_pi_t *s, float kp, float ki);

/** @brief 复位（清积分，输出回到 out_init）—— 进 RUN / 重新起步时用 */
void speed_pi_reset(speed_pi_t *s, float out_init);

/**
  * @brief  推进一拍
  * @param  v_target 目标速度（mm/s）
  * @param  v_meas   实测速度（mm/s，正 = 前进）
  * @param  dt_s     这一拍的时间（秒）
  * @return 这一拍该给的"基础占空比"（mag，交给差速混合）
  */
float speed_pi_step(speed_pi_t *s, float v_target, float v_meas, float dt_s);

/** @brief 读回（任一指针传 0 = 不取）*/
void speed_pi_get(const speed_pi_t *s, float *kp, float *ki, float *out);

#endif /* __CTRL_SPEED_PI_H */
