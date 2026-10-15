/**
  ******************************************************************************
  * @file    straight.h
  * @brief   陀螺直行（航向保持）—— **纯逻辑，无硬件依赖，可在 PC 上单测**
  *
  * 干什么用的：让车"朝一个方向直着走"，靠的是陀螺的航向，不需要看到线。
  * 三个地方要用到它：
  *   ① **发车起步**：从发车点朝 S 出去，前几厘米线没压准时也能走；
  *   ② **丢线**：线被挡住/断开（过斑马线、边角）时不立刻刹死，保持当时的航向直着找线；
  *   ③ 以后"惯导为主"那条线：直道段就是靠它保持航向，光电只负责修正。
  *
  * 符号口径（★这块最容易写反，写反了车会一进 RUN 就原地打转）：
  *   `heading_yaw_deg()` 逆时针为正（车头往左偏 = yaw 变大）；
  *   `mixer_step()` 约定**转向量 > 0 = 左轮快 = 车往右转**。
  *   ⇒ 车头已经偏左（yaw 大于目标）时，要往右打 ⇒ 转向量应为正：
  *      `steer = kp × (yaw − target)`。
  *   `sign` 这个 ±1 开关是留给现场的：万一陀螺/电机装反了，台上一条命令就能翻过来，
  *   不用回电脑改代码重烧。
  *
  * 单位：kp [转向量mag / 度]、kd [转向量mag / (度/秒)]（角速度用 deg/s 算，别用"每拍"，
  *       否则 5ms 一拍会让 D 项小 200 倍、调了没反应）。
  *
  * 本文件不许 #include 任何 stm32/HAL/驱动头文件 —— 硬纪律（保证 PC 可单测）。
  ******************************************************************************
  */
#ifndef __CTRL_STRAIGHT_H
#define __CTRL_STRAIGHT_H

#include <stdint.h>

typedef struct {
    float   kp;          /* 航向误差 → 转向量 */
    float   kd;          /* 航向角速度 → 转向量（阻尼）*/
    float   steer_max;   /* 转向量限幅 */
    float   target_deg;  /* 要保住的航向（进直行那一刻的 yaw）*/
    float   err_prev;    /* 上一拍的误差（算 D 用）*/
    int8_t  sign;        /* ±1：方向反了改它 */
    uint8_t inited;      /* 0 = 第一拍不算 D（防第一下猛拐）*/
} straight_t;

/** @brief 初始化（sign 传 +1 或 -1）*/
void straight_init(straight_t *s, float kp, float kd, float steer_max, int8_t sign);

/** @brief 现场改增益（不动其它状态）*/
void straight_set_gains(straight_t *s, float kp, float kd);

/** @brief 读回当前设置（任一指针传 0 表示不取那一项）*/
void straight_get(const straight_t *s, float *kp, float *kd, int8_t *sign, float *target);

/** @brief 现场翻方向（±1）*/
void straight_set_sign(straight_t *s, int8_t sign);

/** @brief 把目标航向设成 yaw_deg（进直行/丢线那一刻的当前航向）*/
void straight_set_target(straight_t *s, float yaw_deg);

/** @brief 当前目标航向 */
float straight_target(const straight_t *s);

/** @brief 复位（清 D 历史；换目标/刚从循迹环切过来时调，免得拿老误差算微分）*/
void straight_reset(straight_t *s);

/**
  * @brief  推进一拍，返回这一拍该给的转向量（mag，正 = 往右）
  * @param  yaw_deg 当前航向（逆时针为正）
  * @param  dt_s    这一拍的时间（秒，5ms 拍传 0.005）
  */
float straight_step(straight_t *s, float yaw_deg, float dt_s);

#endif /* __CTRL_STRAIGHT_H */
