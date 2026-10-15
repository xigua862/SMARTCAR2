/**
  ******************************************************************************
  * @file    buzzer.h
  * @brief   蜂鸣器驱动（PB10，有源 5V，只能开/关不能调音）
  ******************************************************************************
  */
#ifndef __DRV_BUZZER_H
#define __DRV_BUZZER_H

#include <stdint.h>

void buzzer_init(void);

/** @brief 直接开/关（电平由 bcar_config.h 的 BCAR_BUZZER_ON_LEVEL 决定）*/
void buzzer_on(void);
void buzzer_off(void);

/**
  * @brief  非阻塞鸣叫：响 ms 毫秒后自动停（要在主循环里 buzzer_poll() 才会停）
  * @note   别用 HAL_Delay 阻塞式鸣叫 —— 会卡住控制环。
  */
void buzzer_beep(uint32_t ms);

/**
  * @brief  非阻塞"连鸣 n 声"：每声响 on_ms、间隔 off_ms
  * @note   测试模式进门"蜂鸣 2 声"用它；也是同一套序列状态，
  *         和 buzzer_beep 不会互相打架（后调用的覆盖前一个）。
  */
void buzzer_beeps(uint8_t n, uint16_t on_ms, uint16_t off_ms);

/** @brief 主循环每拍调用：推进鸣叫序列（自动停/下一声）*/
void buzzer_poll(void);

/** @brief 当前是否在响（1 = 响）*/
uint8_t buzzer_is_on(void);

/** @brief 鸣叫序列是否还没放完（1 = 还在响或还在等下一声）*/
uint8_t buzzer_busy(void);

#endif /* __DRV_BUZZER_H */
