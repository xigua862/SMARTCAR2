/**
  ******************************************************************************
  * @file    angle_pd.h
  * @brief   转向 PD（**纯逻辑，无硬件依赖，可在 PC 上单测**）
  *
  * 输入：灰度的线偏差 err_mm（正 = 线在车体右侧）
  * 输出：转向量 steer（正 = 该往右转），量纲和电机 mag 一致（0~1000 附近）
  *
  * 公式：steer = kp·err + kd·(err − err_prev)
  *   —— kd 用的是"两次采样的差值"、**不除 dt**。因为 5ms 固定周期下
  *      除以 dt 只是把系数整体放大 200 倍，现场调参时反而难估量级。
  ******************************************************************************
  */
#ifndef __CTRL_ANGLE_PD_H
#define __CTRL_ANGLE_PD_H

#include <stdint.h>

typedef struct {
    float   kp;         /* 比例：mag / mm（err=10mm、kp=25 → steer=250）*/
    float   kd;         /* 微分：mag / mm（看偏差变化，抑制过冲）*/
    float   steer_max;  /* 输出限幅（mag）*/
    float   err_prev;
    uint8_t inited;
} angle_pd_t;

/** @brief 初始化并给默认增益 */
void angle_pd_init(angle_pd_t *pd, float kp, float kd, float steer_max);

/** @brief 现场改增益（串口命令用，不用重烧）*/
void angle_pd_set(angle_pd_t *pd, float kp, float kd);

/** @brief 读回增益 */
void angle_pd_get(const angle_pd_t *pd, float *kp, float *kd, float *steer_max);

/** @brief 推进一拍，返回转向量（已限幅）*/
float angle_pd_step(angle_pd_t *pd, float err_mm);

/** @brief 清历史（丢线恢复后用，避免拿旧偏差算微分）*/
void angle_pd_reset(angle_pd_t *pd);

#endif /* __CTRL_ANGLE_PD_H */
