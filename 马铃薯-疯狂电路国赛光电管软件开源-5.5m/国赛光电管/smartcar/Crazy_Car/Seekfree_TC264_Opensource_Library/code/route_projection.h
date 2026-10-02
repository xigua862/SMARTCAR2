/**
 * @file route_projection.h
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

// route_projection.h —— 路径离散点投影/最近点查询边界。
//
// 从上次进度向前开窗，先在离散点表覆盖的线段上做连续投影，再由导航层
// 把索引游标取整给路口/速度表等离散接口。

#ifndef CODE_ROUTE_PROJECTION_H_
#define CODE_ROUTE_PROJECTION_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    uint16 index;          // 最近路径点索引
    float fraction;        // 从 index 到 index+1 的线段内投影比例[0,1]
    float progress_points; // 连续路径进度=index+fraction
    float distance_sq_mm2; // 车辆到该点的距离平方，单位 mm²
    float signed_cross_track_mm; // 按路径右法向计的横向误差，右侧为正
} RouteProjectionResult;

// 在前向搜索窗内把车辆投影到路径线段，而不是只比较离散路径点。
// 搜索范围为 [start_index, start_index + search_points] 覆盖的线段，
// 适用于路径点按固定里程采样的 nav_path.h。
uint8 route_projection_project_forward(const signed short *path_x,
                                        const signed short *path_y,
                                        uint16 point_count,
                                        uint16 start_index,
                                        uint16 search_points,
                                        float position_x_mm,
                                        float position_y_mm,
                                        float sample_mm,
                                        RouteProjectionResult *out);

#ifdef __cplusplus
}
#endif

#endif /* CODE_ROUTE_PROJECTION_H_ */
