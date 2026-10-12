/**
  ******************************************************************************
  * @file    bsp_gpio.h
  * @brief   GPIO 底层配置（寄存器级）
  *
  * 分工说明：CubeMX 生成的 gpio.c 负责"上电那一刻"的默认状态；
  *           本文件负责"运行期改脚"（例如把 PB6/PB7 从默认态改成串口复用态）。
  *           两处不重叠，CubeMX 重新生成代码不会冲掉这里。
  ******************************************************************************
  */
#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include <stdint.h>
#include "stm32f1xx.h"

/** 输出速度（STM32F1 的 GPIO 有 2/10/50MHz 三档）*/
typedef enum {
    BSP_GPIO_SPEED_2MHZ  = 2u,
    BSP_GPIO_SPEED_10MHZ = 1u,
    BSP_GPIO_SPEED_50MHZ = 3u
} bsp_gpio_speed_t;

/**
  * @brief  把某个脚配成「复用推挽输出」（串口 TX、PWM 这类）
  * @param  port  GPIOA/GPIOB/... 
  * @param  pin   0~15
  * @param  speed 输出速度
  * @note   F1 的 GPIO 是 CRL(0~7) / CRH(8~15) 每 4 位管一个脚：MODE[1:0]+CNF[1:0]。
  *         复用推挽 = CNF=10(0xA 里体现)、MODE=速度。这里的 0xB 是 CNF=10|MODE=11。
  */
void bsp_gpio_set_af_pp(GPIO_TypeDef *port, uint8_t pin, bsp_gpio_speed_t speed);

/** @brief 把某个脚配成「浮空输入」（串口 RX 用；RX 空闲是高电平，浮空也不误触发）*/
void bsp_gpio_set_input_floating(GPIO_TypeDef *port, uint8_t pin);

#endif /* __BSP_GPIO_H */
