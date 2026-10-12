/**
  ******************************************************************************
  * @file    motor.h
  * @brief   AT8236 双通道电机驱动（TIM1 四路 PWM，20kHz）
  *
  * 接线（见 文档\引脚图-B车C8T6-20261002.html）：
  *   左电机：PA8 = IN1(TIM1_CH1)、PA9  = IN2(TIM1_CH2)
  *   右电机：PA10= IN1(TIM1_CH3)、PA11 = IN2(TIM1_CH4)
  *
  * 真值表（AT8236）：IN1/IN2
  *   10 = 正转（这里用 PWM 占空比表示"1"）
  *   01 = 反转
  *   11 = 刹车（两轮硬锁）—— 运行时"停"一律用这个
  *   00 = 滑行（且 1ms 后进休眠）—— **只允许测试模式下用**
  *   占空比低于约 4.4%（20kHz 下等于消隐 2.2µs）时电机不转。
  ******************************************************************************
  */
#ifndef __DRV_MOTOR_H
#define __DRV_MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_FWD   = 0,   /* 正转 */
    MOTOR_REV   = 1,   /* 反转 */
    MOTOR_BRAKE = 2,   /* 刹车（硬锁）*/
    MOTOR_COAST = 3    /* 滑行（会休眠，仅测试模式）*/
} motor_dir_t;

typedef enum {
    MOTOR_L = 0,       /* 左（PA8/PA9）*/
    MOTOR_R = 1,       /* 右（PA10/PA11）*/
    MOTOR_SIDE_COUNT = 2
} motor_side_t;

/**
  * @brief  初始化 TIM1 四路 PWM（20kHz）+ 打开 MOE
  * @note   ★MOE（BDTR 里的主输出使能）必须打开，忘了就是"程序在跑、电机不转"，
  *         能白耗半天。本函数最后会检查 BDTR 的 MOE 位并返回结果。
  * @retval 1 = MOE 已生效；0 = 没开起来（此时电机一定不转，应报警）
  */
uint8_t motor_init(void);

/** @brief 设置单侧电机的方向与力度。mag 有效范围 50~1000（低于 50 按 50 处理）*/
void motor_set(motor_side_t side, motor_dir_t dir, uint16_t mag);

/** @brief 两侧一起设（前进/倒车/刹车大多这么用）*/
void motor_set_all(motor_dir_t dir, uint16_t mag);

/** @brief 立刻刹车（急停用：两侧都 11 硬锁）*/
void motor_brake_all(void);

/** @brief 读回当前各通道占空比（调试/自检用，返回左、右最近一次的 mag）*/
void motor_get_last(motor_side_t side, motor_dir_t *dir, uint16_t *mag);

#endif /* __DRV_MOTOR_H */
