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

/** @brief 主循环每拍调用：处理自动停 */
void buzzer_poll(void);

/** @brief 当前是否在响（1 = 响）*/
uint8_t buzzer_is_on(void);

#endif /* __DRV_BUZZER_H */
