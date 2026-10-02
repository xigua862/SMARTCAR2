/**
 * @file motor.h
 * @brief 电机 PWM、方向、编码器和负压风机的底层封装。
 * @author 高冷马铃薯
 *
 * @par 所属项目
 * 2026 智能车国赛（光电管）
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

// motor.h —— 左右轮 FOC 驱动、编码器读取、负压电调(BLDC) 接口。
// 引脚、duty方向和编码器符号必须成组配置：左轮编码器取反，右轮编码器直通，
// 两轮逻辑前进时都向上层返回正增量。

#ifndef MOTOR_H_
#define MOTOR_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 电机 PWM 引脚 / 使能
#define MOTOR_ENABLE_PIN P33_10       // FOC 驱动板总使能
#define PWM_L (ATOM2_CH3_P11_6)       // 逻辑左轮速度 PWM
#define PWM_R (ATOM2_CH6_P11_11)      // 逻辑右轮速度 PWM

// 左右轮累计绝对编码器脉冲，供光电停车保护估算里程

#define MOTOR_PWM_FREQ (17000u) // 速度 PWM 频率 17kHz，越高噪音越小、越平顺

// 方向 GPIO
#define DIR_L (P11_9)  // 逻辑左轮方向
#define DIR_R (P11_12) // 逻辑右轮方向

// 方向逻辑固定为：DIR_FORWARD=0、DIR_REVERSE=1；motor_duty() 按 duty 符号选择方向。
#define DIR_FORWARD (0)
#define DIR_REVERSE (1)

#define BLDC_CHANNEL (ATOM2_CH5_P11_10) // 电调 PWM 通道，与左右电机 PWM 同组
#define BLDC_PWM_FREQ (50u)             // 电调 PWM 频率 50Hz

// 对外接口
void motor_init(void);   // 初始化 PWM、方向 GPIO、编码器、电调 PWM（使能脚建为低电平）
void motor_enable(void); // 使能 8701 驱动板（高1.5ms、低10us、再保持高）
void motor_disable(void); // 关闭 8701 驱动板总使能（拉低 MOTOR_ENABLE_PIN）

// 设置单路 FOC 输出。duty 约 -PWM_DUTY_MAX~+PWM_DUTY_MAX，符号决定方向，绝对值写入 PWM。
void motor_duty(pwm_channel_enum pwm_ch, gpio_pin_enum dir_pin, int16 duty);

void motor_stop(void); // 左右轮 PWM 清零，方向电平保持最后一次设定

void bldc_set_duty(uint16 duty); // 设置电调占空比
void bldc_stop(void);            // 完全关闭负压 PWM(不同于 bldc_set_percent(0) 的 1ms 脉宽)
void bldc_set_percent(uint8 percent); // 负压档位：0%=1ms，100%=2ms

int16 motor_read_left_count(void);  // 左轮自上次读取后的带符号增量(前进为正)，计数器自由运行
int16 motor_read_right_count(void); // 右轮自上次读取后的带符号增量(前进为正)，计数器自由运行

#ifdef __cplusplus
}
#endif

#endif
