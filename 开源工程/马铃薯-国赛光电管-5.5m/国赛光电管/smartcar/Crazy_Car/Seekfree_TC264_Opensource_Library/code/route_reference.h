/**
 * @file route_reference.h
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

// route_reference.h —— 离散路径参考点与曲率查询边界。

#ifndef CODE_ROUTE_REFERENCE_H_
#define CODE_ROUTE_REFERENCE_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 在连续路径进度(index+fraction)处线性插值参考航向，处理 ±180° 绕回。
float route_reference_heading_deg_at_progress(const signed short *path_yaw_0p01deg,
                                              uint16 point_count,
                                              float progress_points);

// 在连续路径进度处线性插值离线曲率表，返回 rad/mm。
float route_reference_curvature_at_progress(const signed short *path_curvature,
                                            uint16 point_count,
                                            float progress_points,
                                            float scale_per_lsb);

#ifdef __cplusplus
}
#endif

#endif /* CODE_ROUTE_REFERENCE_H_ */
