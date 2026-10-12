/**
  ******************************************************************************
  * @file    bsp_systick.h
  * @brief   时基：5ms 控制节拍 + 微秒级计时（给 IMU 积分 dt 用）
  ******************************************************************************
  */
#ifndef __BSP_SYSTICK_H
#define __BSP_SYSTICK_H

#include <stdint.h>

/**
  * @brief  初始化：
  *         1) HAL 的 1ms 时基照旧（HAL_Delay / 超时要用）
  *         2) 打开 DWT 周期计数器，提供 bsp_micros()（陀螺积分要准到微秒）
  * @note   5ms 节拍由 SysTick 中断里的 bsp_tick_inc_ms() 累积产生。
  */
void bsp_systick_init(void);

/** @brief 在 SysTick 中断里调用（1ms 一次）：累加毫秒、置 5ms 标志 */
void bsp_tick_inc_ms(void);

/** @brief 主循环用：取一次 5ms 标志（取走后清零）。返回 1 表示这一拍到了 */
uint8_t bsp_tick5ms_take(void);

/** @brief 上电至今的毫秒数 */
uint32_t bsp_millis(void);

/** @brief 上电至今的微秒数（DWT 实现，约 71 分钟回绕一次）*/
uint32_t bsp_micros(void);

/* 主循环里"每 500ms 翻灯 / 每 1000ms 打印"用的软定时器：
 * 传入周期，到点返回 1（自带下次到期时间，不用手算差） */
uint8_t bsp_timer_expired(uint32_t *last_ms, uint32_t period_ms);

#endif /* __BSP_SYSTICK_H */
