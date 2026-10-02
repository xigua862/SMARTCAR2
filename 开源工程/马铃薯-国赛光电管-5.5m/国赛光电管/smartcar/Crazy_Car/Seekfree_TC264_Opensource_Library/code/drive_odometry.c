/**
 * @file drive_odometry.c
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

// drive_odometry.c —— 编码器中心距离与原始 VQF 中点航向位姿积分。

#include "drive_odometry.h"

#include <math.h>

static float drive_odometry_wrap_delta_rad(float delta_rad)
{
    const float pi = 3.14159265358979323846f;
    const float two_pi = 6.28318530717958647692f;

    while (delta_rad > pi)
    {
        delta_rad -= two_pi;
    }
    while (delta_rad <= -pi)
    {
        delta_rad += two_pi;
    }
    return delta_rad;
}

void drive_odometry_reset(volatile DriveOdometryPose *pose)
{
    if (NULL == pose)
    {
        return;
    }

    pose->x_mm = 0.0f;
    pose->y_mm = 0.0f;
    pose->yaw_rad = 0.0f;
    pose->linear_speed_mmps = 0.0f;
    pose->yaw_rate_radps = 0.0f;
    pose->gyro_heading_rad = 0.0f;
    pose->gyro_heading_initialized = 0u;
}

void drive_odometry_integrate_wheel_delta(volatile DriveOdometryPose *pose,
                                          float delta_s_left_mm,
                                          float delta_s_right_mm,
                                          float gyro_heading_rad,
                                          float sample_period_s)
{
    float center_delta_mm;
    float yaw_delta_rad;
    float midpoint_yaw_rad;

    if ((NULL == pose) || (sample_period_s <= 0.0f))
    {
        return;
    }

    if (!pose->gyro_heading_initialized)
    {
        pose->gyro_heading_rad = gyro_heading_rad;
        pose->yaw_rad = gyro_heading_rad;
        pose->gyro_heading_initialized = 1u;
    }

    center_delta_mm = 0.5f * (delta_s_left_mm + delta_s_right_mm);
    yaw_delta_rad = drive_odometry_wrap_delta_rad(
        gyro_heading_rad - pose->gyro_heading_rad);
    midpoint_yaw_rad = pose->yaw_rad + 0.5f * yaw_delta_rad;

    // 中点航向积分：编码器平均值只提供距离，方向来自相邻原始 VQF 航向。
    pose->x_mm += center_delta_mm * cosf(midpoint_yaw_rad);
    pose->y_mm += center_delta_mm * sinf(midpoint_yaw_rad);
    pose->yaw_rad += yaw_delta_rad;
    pose->gyro_heading_rad = gyro_heading_rad;
    pose->linear_speed_mmps = center_delta_mm / sample_period_s;
    pose->yaw_rate_radps = yaw_delta_rad / sample_period_s;
}

void drive_odometry_integrate_wheel_delta_encoder_heading(
    volatile DriveOdometryPose *pose,
    float delta_s_left_mm,
    float delta_s_right_mm,
    float yaw_per_diff_mm,
    float sample_period_s)
{
    float center_delta_mm;
    float yaw_delta_rad;
    float midpoint_yaw_rad;
    const float pi = 3.14159265358979323846f;
    const float two_pi = 6.28318530717958647692f;

    if ((NULL == pose) || (sample_period_s <= 0.0f))
    {
        return;
    }

    center_delta_mm = 0.5f * (delta_s_left_mm + delta_s_right_mm);
    // 保持车端现有 yaw 符号：与 control.c slip guard 的 (left-right)
    // 编码器角速度完全同号；yaw_per_diff_mm 已吸收编码器/轮距标定。
    yaw_delta_rad = (delta_s_left_mm - delta_s_right_mm) * yaw_per_diff_mm;
    midpoint_yaw_rad = pose->yaw_rad + 0.5f * yaw_delta_rad;

    pose->x_mm += center_delta_mm * cosf(midpoint_yaw_rad);
    pose->y_mm += center_delta_mm * sinf(midpoint_yaw_rad);
    pose->yaw_rad += yaw_delta_rad;
    pose->linear_speed_mmps = center_delta_mm / sample_period_s;
    pose->yaw_rate_radps = yaw_delta_rad / sample_period_s;

    // 该字段保留给地图变换/诊断接口：测试模式下它代表编码器拟合的连续航向，
    // 而不是 VQF heading。归一化只为兼容旧的跨 ±pi 语义，yaw_rad 本身保持连续。
    pose->gyro_heading_rad = pose->yaw_rad;
    while (pose->gyro_heading_rad > pi)
    {
        pose->gyro_heading_rad -= two_pi;
    }
    while (pose->gyro_heading_rad <= -pi)
    {
        pose->gyro_heading_rad += two_pi;
    }
    pose->gyro_heading_initialized = 1u;
}
