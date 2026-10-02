/**
 * @file route_projection.c
 * @brief 车辆地图位姿到离线路径的连续前向投影。
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

// route_projection.c —— 当前导航路径投影与最近点查询。

#include "route_projection.h"

#include <math.h>

uint8 route_projection_project_forward(const signed short *path_x,
                                        const signed short *path_y,
                                        uint16 point_count,
                                        uint16 start_index,
                                        uint16 search_points,
                                        float position_x_mm,
                                        float position_y_mm,
                                        float sample_mm,
                                        RouteProjectionResult *out)
{
    uint32 end_segment;
    uint32 segment;
    uint16 best_index = 0u;
    float best_fraction = 0.0f;
    float best_distance_sq = 1.0e30f;
    float best_dx = 0.0f;
    float best_dy = 0.0f;
    float best_vx = 0.0f;
    float best_vy = 0.0f;
    float best_len_sq = 0.0f;

    if ((NULL == path_x) || (NULL == path_y) || (NULL == out) ||
        (point_count < 2u) || (sample_mm <= 0.0f))
    {
        return 0u;
    }
    if (start_index >= point_count)
    {
        start_index = (uint16)(point_count - 1u);
    }
    if (start_index >= (uint16)(point_count - 1u))
    {
        start_index = (uint16)(point_count - 2u);
    }

    end_segment = (uint32)start_index + (uint32)search_points;
    if (end_segment >= (uint32)(point_count - 1u))
    {
        end_segment = (uint32)(point_count - 2u);
    }

    for (segment = (uint32)start_index; segment <= end_segment; segment++)
    {
        uint16 i = (uint16)segment;
        float ax = (float)path_x[i];
        float ay = (float)path_y[i];
        float bx = (float)path_x[i + 1u];
        float by = (float)path_y[i + 1u];
        float vx = bx - ax;
        float vy = by - ay;
        float len_sq = vx * vx + vy * vy;
        float t;
        float px;
        float py;
        float dx;
        float dy;
        float distance_sq;

        if (len_sq <= 1.0e-6f)
        {
            continue;
        }
        t = ((position_x_mm - ax) * vx + (position_y_mm - ay) * vy) / len_sq;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        px = ax + t * vx;
        py = ay + t * vy;
        dx = position_x_mm - px;
        dy = position_y_mm - py;
        distance_sq = dx * dx + dy * dy;

        if (distance_sq < best_distance_sq)
        {
            best_index = i;
            best_fraction = t;
            best_distance_sq = distance_sq;
            best_dx = dx;
            best_dy = dy;
            best_vx = vx;
            best_vy = vy;
            best_len_sq = len_sq;
        }
    }

    if (best_distance_sq >= 1.0e29f)
    {
        return 0u;
    }
    out->index = best_index;
    out->fraction = best_fraction;
    out->progress_points = (float)best_index + best_fraction;
    out->distance_sq_mm2 = best_distance_sq;
    {
        float best_length = sqrtf(best_len_sq);

        // 路径右法向=(-sin(theta), cos(theta))，车辆-参考点投影为右侧正。
        out->signed_cross_track_mm =
            best_dx * (-best_vy / best_length) +
            best_dy * (best_vx / best_length);
    }
    return 1u;
}
