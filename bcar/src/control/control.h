/**
  ******************************************************************************
  * @file    control.h
  * @brief   循迹控制环总装（转向 PD + 差速混合 + 丢线保护）
  *
  * 这是"阶段 7 的起步版"：**速度先固定**，只闭环转向。
  * 为什么够用：明天要的是"能完整跑一圈"，不是"跑得快"。
  * 速度环（编码器闭环）等这一版跑通了再加。
  ******************************************************************************
  */
#ifndef __CTRL_CONTROL_H
#define __CTRL_CONTROL_H

#include <stdint.h>

void control_init(void);

/** @brief 进入 RUN：复位 PD、清丢线计时、并把当前 PWM 清零 */
void control_enter_run(void);

/** @brief 退出 RUN：立刻刹车 */
void control_exit_run(void);

/**
  * @brief  5ms 拍调用（只在 RUN 状态由 main 调用）
  * @param  err_mm  灰度给的线偏差（正 = 线在右侧）
  * @param  visible 1 = 看到线
  */
void control_step(float err_mm, uint8_t visible);

/** @brief 现场改速度基准（不用重烧）*/
void control_set_base(uint16_t mag);
uint16_t control_base(void);

/** @brief 现场改增益 */
void control_set_pd(float kp, float kd);
void control_get_pd(float *kp, float *kd);

/** @brief 丢线持续了多少拍（诊断：跑完看一眼就知道有没有频繁丢线）*/
uint32_t control_lost_ticks(void);

/** @brief 是否处于"丢线但还在撑"的状态（诊断用）*/
uint8_t control_lost_holding(void);

#endif /* __CTRL_CONTROL_H */
