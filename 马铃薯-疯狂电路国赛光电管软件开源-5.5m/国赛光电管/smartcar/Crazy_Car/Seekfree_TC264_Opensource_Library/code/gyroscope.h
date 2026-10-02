/**
 * @file gyroscope.h
 * @brief CPU1 IMU 采样、VQF、上电零偏和发车姿态重锚。
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
 *
 * @note CPU1 是 VQF 状态唯一写者，CPU0 只读完整快照。
 */

// gyroscope.h — LSM6DSV16XTR 姿态角与连续航向接口。底层驱动符号沿用 IMU660RC。

#ifndef CODE_GYROSCOPE_H_
#define CODE_GYROSCOPE_H_

#include "zf_common_headfile.h"
#include "globals.h"

// CPU1光电任务读取同核纯陀螺yaw，避免与IMU ISR竞争跨核发布双槽。
float gyro_cpu1_get_pure_yaw_deg(void);

// VQF 航向相对发车零点(度)，符号与地图航向一致。
float gyro_get_heading_deg(void);

// 发车瞬间请求：CPU0发布，CPU1在一帧VQF结果上应用并确认。
uint32 gyro_request_launch_rezero(void);
uint8 gyro_command_acknowledged(uint32 command_seq);
uint8 gyro_cpu1_service_ready(void);
uint8 gyro_cpu1_init(void);                    // 0=成功；失败保持未就绪，但不阻塞CPU0界面
void gyro_cpu1_external_interrupt_update(void);  // CPU1 IMU数据就绪采样入口
float gyro_get_roll_delta(void);
float gyro_get_pitch_delta(void);
uint8 gyro_zero_calibration_done(void);
uint8 gyro_zero_calibration_running(void);

#endif /* CODE_GYROSCOPE_H_ */
