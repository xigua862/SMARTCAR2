/**
  ******************************************************************************
  * @file    line_pos.h
  * @brief   8 路灰度 → 线位置偏差（**纯逻辑，无硬件依赖，可在 PC 上单测**）
  *
  * 算法（对齐框架 §2.5）：
  *   ① 每路各自归一化：p = (v - v_黑) / (v_白 - v_黑)，钳到 [0,1]
  *      —— 各自归一化是关键：8 个探头的一致性很差（发射管亮度、贴纸高度、
  *         喷绘布反光都不同），用统一阈值必然有的通道永远亮、有的永远灭。
  *   ② 加权质心：err = Σ(w_i · p_i) / Σ(p_i)，w_i 是该路的横向位置（mm）
  *      —— err 直接就是"线偏了多少毫米"，转向环拿它当输入，不用再标系数。
  *   ③ 丢线判据：Σp 太小 ⇒ 8 路都没看见线，必须报出去（不能瞎给 0）
  *
  * 本文件不许 #include 任何 stm32/HAL/驱动头文件 —— 硬纪律（保证 PC 可单测）。
  ******************************************************************************
  */
#ifndef __MW_LINE_POS_H
#define __MW_LINE_POS_H

#include <stdint.h>

#define LINE_CH  8u   /* 8 路，对应 PA0~PA7 */

typedef struct {
    float pos_mm[LINE_CH];   /* ★每路的横向位置（mm）。中心那两路之间为 0，
                              *   实测探头间距后填；正负方向按"车头朝前"定 */
    float black[LINE_CH];    /* 每路在**全黑**上的读数（现场教它）*/
    float white[LINE_CH];    /* 每路在**白线**上的读数（现场教它）*/
    float lost_sum_p;        /* Σp 低于这个 ⇒ 判定丢线（建议 0.3~0.6）*/
} line_pos_cfg_t;

typedef struct {
    float   err_mm;          /* 线相对车体中线的偏移：正 = 线偏右 */
    float   sum_p;           /* 线强度（归一化值之和）*/
    uint8_t visible;         /* 1 = 看到线（Σp 够大），0 = 丢线 */
    float   p[LINE_CH];      /* 每路归一化后的值（遥测/调参用）*/
} line_pos_t;

/** @brief 初始化：给几何与黑白基准 */
void line_pos_init(const line_pos_cfg_t *cfg);

/** @brief 读回配置 */
const line_pos_cfg_t *line_pos_cfg(void);

/** @brief 取可变配置（现场标定：改间距 / 改丢线阈值）*/
line_pos_cfg_t *line_pos_cfg_mut(void);

/** @brief 单独设某一路的黑/白基准（现场"教"它就是调这个）*/
void line_pos_set_ref(uint8_t ch, float black, float white);

/**
  * @brief  推进一拍：喂 8 个原始读数，出偏差
  * @param  raw8 指向 8 个 12 位 ADC 值
  * @return 本次结果（含 err / sum_p / visible / 各路 p）
  */
line_pos_t line_pos_step(const uint16_t *raw8);

#endif /* __MW_LINE_POS_H */
