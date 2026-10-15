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
#define BCAR_UART_TX_BUF     192u    /* TX DMA 缓冲：够装一条 LAP 行（约 100~140 字节）*/

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

/* ---- 发车读秒里自动标一次陀螺零偏（2026-10-15 加）----
 * 为什么要：2026-10-15 在台上用串口实测（车静止 45 秒），陀螺 Z 的原始零偏 ≈ **0.4 dps**；
 *   不标定就直接积分 ⇒ 航向以 0.4°/s 漂，跑一圈（约 40 秒）就能飘掉约 16° ——
 *   "惯导为主"的设计会直接垮掉，而且**不报错**，只在赛道上表现为"怎么调都跑不直"。
 * 怎么用：发车读秒那 3 秒车本来就刹着不动，正好采 1 秒静止样本把零偏标掉；
 *   采集期间若车明显被碰/在转（样本极差 > BCAR_GYRO_AUTOCAL_MAX_SPREAD_DPS），
 *   本次**放弃**自动标定、沿用原零偏，并在串口说明（宁可不标，也不标一个错的）。
 * 不想要就把开关改 0：那时只认台上手敲的 GYROZERO。 */
#define BCAR_GYRO_AUTOCAL_ON_LAUNCH        1
#define BCAR_GYRO_AUTOCAL_MAX_SPREAD_DPS   1.0f

/* ---- 按键长按门限（发车流程，2026-10-15 需求）----
 * PC13（KEY2 / 正式发车键）**长按 200ms 才发车，短按不响应**；
 * 短于这个时长的按下只报一次"短按"（串口提示一句），不发车。 */
#define BCAR_KEY_LONG_MS     200u

/* ---- 发车流程（按键 → 读秒 → 循迹）----
 * 时序（哪一声、隔多久）在 `src/app/launch.c` 里；这里只定"响多久"这种物理量。
 * 读秒节奏 = 短 2 声 + 长 1 声（用户 2026-10-15 定）。 */
#define BCAR_BEEP_SHORT_MS   80u     /* "哔"：读秒前两声、测试模式的每一声 */
#define BCAR_BEEP_LONG_MS    600u    /* "哔——"：读秒第 3 声 = 松手起跑那声 */
#define BCAR_BEEP_GAP_MS     120u    /* 连鸣两声之间的间隔（测试模式"哔 哔"）*/

/* ---- 里程标定（★必须实测，现在这个值是猜的）----
 * 标定方法：把车推 1 米（1000mm），看串口 `ENC` 的 total 涨了多少计数，
 *           然后 mm_per_count = 1000 / 那个计数，改这里重新编译。
 * ⚠ v1.8 更正：编码器是**齿轮驱动**（不是装轮轴），老的 πD/4096 不成立。 */
#define BCAR_MM_PER_COUNT    0.1f    /* 【待实测】占位值，标定前别拿它算距离 */
#define BCAR_TRACK_MM      110.0f    /* 轮距 110mm（实测真值）*/

/* ---- 陀螺（IMU660RC / LSM6DSV16X）---- */
#define BCAR_GYRO_DEAD_ZONE_DPS   0.3f   /* 死区：减掉零偏后小于它当 0，抑制噪声积分漂移 */
#define BCAR_GYRO_CAL_MS          1000u  /* 上电自检 / GYROZERO 的静止采集时长 */
#define BCAR_GYRO_CAL_SAMPLES     (BCAR_GYRO_CAL_MS / BCAR_TICK_MS)  /* 采集样本数 */

/* ---- 灰度 8 路（PA0~PA7，模块供 5V、板上 1k/1.5k 分压）---- */
#define BCAR_IR_PITCH_MM     6.0f   /* ★探头间距，待实测（量相邻两探头的中心距）*/
#define BCAR_IR_LOST_SUM_P   0.4f   /* Σp 低于它判丢线 */
#define BCAR_IR_CAL_SAMPLES  20u    /* 现场"教黑白"时每路平均的样本数 */

/* ---- 循迹控制（阶段 7 起步版：只闭环转向，速度先固定）---- */
#define BCAR_CTRL_BASE_MAG     350u    /* 起步速度：先慢。能跑通一圈再往上提 */
#define BCAR_CTRL_KP           25.0f   /* 转向 P：mag / mm（误差 10mm → 转向量 250）*/
#define BCAR_CTRL_KD            3.0f   /* 转向 D：mag / mm（抑制过冲）*/
#define BCAR_CTRL_STEER_MAX   600.0f   /* 转向量限幅 */
#define BCAR_CTRL_LOST_TICKS    40u    /* 丢线容忍：40 拍 = 200ms，超了就停 */

#endif /* __BCAR_CONFIG_H */
