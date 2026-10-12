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
    KEY_START = 0,   /* PC13 —— 正式发车 */
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

/** @brief 当前是否按住（已消抖）*/
uint8_t key_is_down(key_id_t id);

#endif /* __DRV_KEY_H */
