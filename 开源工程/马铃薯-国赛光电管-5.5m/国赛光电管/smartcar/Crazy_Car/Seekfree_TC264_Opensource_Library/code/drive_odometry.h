/**
 * @file drive_odometry.h
 * @brief 编码器距离与 VQF 航向驱动的原始航位推算。
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

// drive_odometry.h —— 车体平面位姿积分的纯几何边界。
//
// 导航使用左右轮平均位移和原始 VQF 中点航向积分。

#ifndef CODE_DRIVE_ODOMETRY_H_
#define CODE_DRIVE_ODOMETRY_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    float x_mm; // 发车坐标系 x，车头方向为正，单位 mm
    float y_mm; // 发车坐标系 y，车右侧为正，单位 mm
    float yaw_rad; // 连续航向：正常模式来自 VQF，编码器测试模式来自差速拟合
    float linear_speed_mmps; // 本周期车体中心线速度，单位 mm/s
    float yaw_rate_radps; // 本周期航向变化率，单位 rad/s，仅供里程计诊断
    float gyro_heading_rad; // 上一帧归一化航向，用于跨±180°解包
    uint8 gyro_heading_initialized;
} DriveOdometryPose;

// 清零平面位置、连续航向和本周期速度状态。
void drive_odometry_reset(volatile DriveOdometryPose *pose);

// 使用左右轮本周期位移计算中心前进距离，并使用原始 VQF yaw 的中点航向积分。
// delta_s_left_mm / delta_s_right_mm 均已转换为“前进为正”的逻辑位移；
// gyro_heading_rad 为当前原始 VQF yaw(弧度)，sample_period_s 为本周期时长。
void drive_odometry_integrate_wheel_delta(volatile DriveOdometryPose *pose,
                                          float delta_s_left_mm,
                                          float delta_s_right_mm,
                                          float gyro_heading_rad,
                                          float sample_period_s);

// 测试模式专用：用左右轮差速拟合航向，不读取 IMU/VQF heading。
// yaw_per_diff_mm 使用车端现有编码器 yaw 符号：左轮比右轮多走 1 mm
// 对应正 yaw（与 control.c 的 slip guard 保持一致）。
// 正常比赛路径仍使用上面的 VQF 版本，避免改变现有控制标定。
void drive_odometry_integrate_wheel_delta_encoder_heading(
    volatile DriveOdometryPose *pose,
    float delta_s_left_mm,
    float delta_s_right_mm,
    float yaw_per_diff_mm,
    float sample_period_s);

#ifdef __cplusplus
}
#endif

#endif /* CODE_DRIVE_ODOMETRY_H_ */
