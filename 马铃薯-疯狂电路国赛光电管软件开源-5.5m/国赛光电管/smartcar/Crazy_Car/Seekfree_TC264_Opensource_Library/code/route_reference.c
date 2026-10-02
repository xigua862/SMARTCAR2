/**
 * @file route_reference.c
 * @brief 离线路线航向、曲率和参考量的插值读取。
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

// route_reference.c —— 离散路径参考点与曲率查询。

#include "route_reference.h"

static float route_reference_wrap_deg(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle <= -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

float route_reference_heading_deg_at_progress(const signed short *path_yaw_0p01deg,
                                              uint16 point_count,
                                              float progress_points)
{
    uint16 index;
    float fraction;
    float heading_deg;
    float delta_deg;

    if ((NULL == path_yaw_0p01deg) || (0u == point_count))
    {
        return 0.0f;
    }
    if (progress_points <= 0.0f)
    {
        return (float)path_yaw_0p01deg[0] * 0.01f;
    }
    if (progress_points >= (float)(point_count - 1u))
    {
        return (float)path_yaw_0p01deg[point_count - 1u] * 0.01f;
    }

    index = (uint16)progress_points;
    fraction = progress_points - (float)index;
    heading_deg = (float)path_yaw_0p01deg[index] * 0.01f;
    delta_deg = route_reference_wrap_deg(
        ((float)path_yaw_0p01deg[index + 1u] -
         (float)path_yaw_0p01deg[index]) * 0.01f);
    return heading_deg + fraction * delta_deg;
}

float route_reference_curvature_at_progress(const signed short *path_curvature,
                                            uint16 point_count,
                                            float progress_points,
                                            float scale_per_lsb)
{
    uint16 index;
    float fraction;
    float value0;
    float value1;

    if ((NULL == path_curvature) || (0u == point_count) ||
        (scale_per_lsb <= 0.0f))
    {
        return 0.0f;
    }
    if (progress_points <= 0.0f)
    {
        return (float)path_curvature[0] * scale_per_lsb;
    }
    if (progress_points >= (float)(point_count - 1u))
    {
        return (float)path_curvature[point_count - 1u] * scale_per_lsb;
    }

    index = (uint16)progress_points;
    fraction = progress_points - (float)index;
    value0 = (float)path_curvature[index];
    value1 = (float)path_curvature[index + 1u];
    return (value0 + fraction * (value1 - value0)) * scale_per_lsb;
}
