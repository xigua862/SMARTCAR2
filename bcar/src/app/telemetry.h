/**
  ******************************************************************************
  * @file    telemetry.h
  * @brief   遥测：每圈结算 + 实时短行（都走串口 TX DMA，不挡控制环）
  *
  * 这一层是"调参迭代器"的嘴：它每圈要吃掉一行带指标的 LAP 记录。
  * 指标口径见 lapstat.h（|偏差|均值、|偏差|P95、丢线拍数…）。
  ******************************************************************************
  */
#ifndef __APP_TELEMETRY_H
#define __APP_TELEMETRY_H

#include <stdint.h>
#include "lapstat.h"

/** @brief 初始化（清空累加器、关实时行）*/
void telemetry_init(void);

/**
  * @brief  5ms 拍调用：累积本圈统计 + （开了实时行时）按 100ms 节奏发一行
  * @param  err_mm  循迹偏差
  * @param  visible 1 = 看到线
  * @param  v_mm_s  当前速度
  * @param  ds_mm   本拍弧长增量
  * @param  yaw_deg 当前航向（实时行里显示，便于手推车时看惯导对不对）
  */
void telemetry_tick(float err_mm, uint8_t visible, float v_mm_s,
                    float ds_mm, float yaw_deg);

/** @brief 开新圈（清零累加器，圈号 +1）*/
void telemetry_start_lap(void);

/**
  * @brief  结算当前圈：算指标 + 发一行 LAP（DMA）+ 自动开新圈
  * @retval 1 = 这一圈有效（拍数够）；0 = 太短，没算
  */
uint8_t telemetry_finish_lap(void);

/** @brief 实时短行开关（1 = 每 100ms 一行）*/
void telemetry_set_realtime(uint8_t on);
uint8_t telemetry_realtime(void);

/** @brief 最近一次结算结果 */
const lap_record_t *telemetry_last_lap(void);

/** @brief 本圈已经累积多少拍了（判断"够不够一圈"用）*/
uint32_t telemetry_lap_ticks(void);

/** @brief 当前圈号 */
uint32_t telemetry_lap_index(void);

#endif /* __APP_TELEMETRY_H */
