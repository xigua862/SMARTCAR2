/**
  ******************************************************************************
  * @file    led.h
  * @brief   状态灯驱动（PC14 / PC15 / PA12）
  ******************************************************************************
  */
#ifndef __DRV_LED_H
#define __DRV_LED_H

#include <stdint.h>

typedef enum {
    LED1 = 0,   /* PC14 —— 待机/心跳 */
    LED2 = 1,   /* PC15 —— 读秒/运行 */
    LED3 = 2,   /* PA12 —— 测试/故障 */
    LED_COUNT = 3
} led_id_t;

/** @brief 初始化三个脚为推挽输出并全灭 */
void led_init(void);

/** @brief 点亮 / 熄灭 */
void led_on(led_id_t id);
void led_off(led_id_t id);

/** @brief 翻转 */
void led_toggle(led_id_t id);

/** @brief 全灭 */
void led_all_off(void);

#endif /* __DRV_LED_H */
