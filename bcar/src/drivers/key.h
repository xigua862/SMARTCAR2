/**
  ******************************************************************************
  * @file    key.h
  * @brief   按键驱动（PC13 = 发车键，PB0 = 测试键；都接 GND + 内部上拉）
  ******************************************************************************
  */
#ifndef __DRV_KEY_H
#define __DRV_KEY_H

#include <stdint.h>

typedef enum {
    KEY_START = 0,   /* PC13 —— 正式发车（★长按 BCAR_KEY_LONG_MS 才算）*/
    KEY_TRIG  = 1,   /* PB0  —— 进测试模式（电机滑行、等串口指令）*/
    KEY_COUNT = 2
} key_id_t;

void key_init(void);

/**
  * @brief  主循环每 5ms 调一次：做消抖 + 产生"按下/松开"事件
  * @note   只在主循环跑，中断里不碰 —— 消抖逻辑不需要抢实时性，
  *         放在中断里反而会拉长 ISR。
  */
void key_poll(void);

/** @brief 取"按下"事件（取走后清零）。返回 1 表示这一拍刚按下 */
uint8_t key_take_press(key_id_t id);

/** @brief 取"松开"事件（取走后清零）*/
uint8_t key_take_release(key_id_t id);

/**
  * @brief  取"长按"事件（取走后清零）
  * @note   按住达到 BCAR_KEY_LONG_MS（默认 200ms）时**只报一次**（不是每拍都报）。
  *         车上的用法：PC13 长按 200ms = 发车；短按不响应。
  */
uint8_t key_take_long(key_id_t id);

/**
  * @brief  取"短按"事件（取走后清零）
  * @note   松手时若这一下按的时长**没到**长按门限，就在这里报一次。
  *         用途：PC13 短按时串口提示一句"发车要长按 200ms"，而不是闷着不响应。
  */
uint8_t key_take_short(key_id_t id);

/** @brief 当前是否按住（已消抖）*/
uint8_t key_is_down(key_id_t id);

/** @brief 这一下已经按住多久（毫秒）；没按住时返回 0 */
uint32_t key_hold_ms(key_id_t id);

#endif /* __DRV_KEY_H */
