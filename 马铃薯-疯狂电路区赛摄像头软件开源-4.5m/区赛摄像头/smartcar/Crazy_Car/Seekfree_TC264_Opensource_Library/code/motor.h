/**
 * @file motor.h
 * @brief 电机 PWM、方向、编码器和负压风机的底层封装。
 * @author 高冷马铃薯
 *
 * @par 所属项目
 * 2026 智能车区赛（摄像头）
 *
 * @par WegooAI 中转站
 * 本项目用户代码 99% 通过作者自建的 WegooAI 中转站，
 * 使用 Claude Code、Codex 等 AI 编程工具完成。
 * WegooAI 由作者高冷马铃薯搭建并维护，
 * 方便大家接入 AI 编程工具编写和修改代码，欢迎使用 :)
 * 官网：https://ai.wegoo.site/
 *
 * @par 联系作者
 * Bilibili：https://space.bilibili.com/513729639
 * QQ：1153626206
 * Email：moonserver@foxmail.com
 */

/*
 * motor.h
 *
 * 左右轮 FOC 驱动与编码器接口。
 * 当前板子反装，软件逻辑左右与硬件原始左右相反，且前进方向极性相反。
 * 逻辑左轮：PWM P11_11，DIR P11_9，编码器 TIM4 A=P02_8 / B=P00_9。
 * 逻辑右轮：PWM P11_12，DIR P11_6，编码器 TIM6 A=P20_3 / B=P20_0。
 * 电调：PWM P11_10。
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ==================== 电机 PWM ==================== */
#define MOTOR_ENABLE_PIN P00_12
#define PWM_L (ATOM2_CH6_P11_11)
#define PWM_R (ATOM2_CH7_P11_12)

/* 左右轮累计绝对编码器脉冲，用于图像状态机估算已行驶距离。 */
extern volatile int32 left_encoder_total_pulses;
extern volatile int32 right_encoder_total_pulses;

#define MOTOR_PWM_FREQ (17000u)

/* ==================== 方向 GPIO ==================== */
#define DIR_L (P11_9)
#define DIR_R (P11_6)

/* 板子反装后方向极性取反。 */
#define DIR_FORWARD (0)
#define DIR_REVERSE (1)

/* 电调 PWM 通道（P11_10，ATOM2 时钟组，与左右电机 PWM 同组） */
#define BLDC_CHANNEL (ATOM2_CH5_P11_10)

/* 电调 PWM 频率，50Hz 为常见舵机/电调控制周期。 */
#define BLDC_PWM_FREQ (50u)
/* ==================== 接口 ==================== */

/* 初始化左右轮 PWM、方向 GPIO、编码器和电调 PWM。 */
void motor_init(void);

/* 设置单路 FOC 输出。
 * duty 范围约为 -PWM_DUTY_MAX ~ +PWM_DUTY_MAX，符号决定方向，绝对值写入 PWM。 */
void motor_duty(pwm_channel_enum pwm_ch, gpio_pin_enum dir_pin, int16 duty);

/* 左右轮 PWM 清零，方向电平保持最后一次设定。 */
void motor_stop(void);

/* 设置电调占空比，有效档位依实际电调规格标定。 */
void BLDC_set_duty(uint16 duty);
/* 完全关闭负压 PWM。BLDC_set_percent(0) 仍是 1ms 控制脉宽，不等于关闭。 */
void BLDC_stop(void);
/* 设置负压档位：0% 对应 1ms，100% 对应 2ms。 */
void BLDC_set_percent(uint8 percent);

/* 返回左轮自上次读取后的带符号增量，硬件计数器保持自由运行。 */
int16 motor_read_left_count(void);
/* 返回右轮自上次读取后的带符号增量；左右轮向前均为正。 */
int16 motor_read_right_count(void);

#ifdef __cplusplus
}
#endif

#endif
