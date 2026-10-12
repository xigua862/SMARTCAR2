/**
  ******************************************************************************
  * @file    bcar_config.h
  * @brief   B 车固件全局开关与常量（改行为先改这里）
  *
  * 说明：本文件是所有"编译期开关"的唯一入口。数值型参数（PID、速度上限等）
  *       不属于这里，它们走 app/param.c 的参数表 + 串口 SET。
  ******************************************************************************
  */
#ifndef __BCAR_CONFIG_H
#define __BCAR_CONFIG_H

#include <stdint.h>

/* ---- 版本 ---- */
#define BCAR_FW_VERSION      "bcar-0.1.0"

/* ---- 时基 ---- */
#define BCAR_TICK_MS         5u      /* 控制节拍：5ms（框架 §3.1）*/
#define BCAR_SYSTICK_MS      1u      /* HAL 时基仍是 1ms，5ms 由它整除得到 */

/* ---- 串口 ---- */
#define BCAR_UART_BAUD       115200u
#define BCAR_UART_RX_BUF     256u    /* 接收环形缓冲字节数（必须是 2 的幂）*/

/* ---- 状态灯极性 ----
 * 板上 3 只 LED：GPIO → 470Ω → LED → GND，所以「输出高 = 亮」。
 * 若实测反了（亮灭相反），把这里改成 0 即可，不用改驱动。 */
#define BCAR_LED_ACTIVE_HIGH 1

/* ---- 上电行为 ----
 * 0 = 上电只做自检 + 待机，等 PC13 发车键（安全，默认）
 * 1 = 上电直接进运行状态（调试用，比赛前必须回 0） */
#define BCAR_AUTORUN_DEFAULT 0

/* ---- 电机（AT8236 ×2，TIM1 四路 PWM）---- */
#define BCAR_PWM_FREQ_HZ     20000u  /* 20kHz：超出听觉范围，且 AT8236 消隐只吃 4.4% */
#define BCAR_MOTION_MIN_MAG  50u     /* 占空比下限：低于此 AT8236 因消隐 2.2µs 不转，
                                        所以 mag 有效范围是 50~1000 而不是 0~1000 */
#define BCAR_MAG_MAX         1000u

/* 电机方向整体翻转开关。
 * 侧框架前后翻转后（编码器到车头、电机到车尾），若测下来"FWD 反而倒着走"，
 * 把它改成 1 —— 两只电机一起翻，不要分别改线。 */
#define BCAR_MOTOR_DIR_FLIP  0

/* ---- 蜂鸣器（PB10 → 1kΩ → SS8050 低边 → 5V 有源蜂鸣器）---- */
#define BCAR_BUZZER_ON_LEVEL 1u      /* 输出高 = 三极管导通 = 响 */

/* ---- 按键消抖 ---- */
#define BCAR_KEY_DEBOUNCE_MS 15u     /* 连续稳定 15ms 才算数（主循环 5ms 一拍 → 3 拍）*/

/* ---- 里程标定（★必须实测，现在这个值是猜的）----
 * 标定方法：把车推 1 米（1000mm），看串口 `ENC` 的 total 涨了多少计数，
 *           然后 mm_per_count = 1000 / 那个计数，改这里重新编译。
 * ⚠ v1.8 更正：编码器是**齿轮驱动**（不是装轮轴），老的 πD/4096 不成立。 */
#define BCAR_MM_PER_COUNT    0.1f    /* 【待实测】占位值，标定前别拿它算距离 */
#define BCAR_TRACK_MM      110.0f    /* 轮距 110mm（实测真值）*/

#endif /* __BCAR_CONFIG_H */
