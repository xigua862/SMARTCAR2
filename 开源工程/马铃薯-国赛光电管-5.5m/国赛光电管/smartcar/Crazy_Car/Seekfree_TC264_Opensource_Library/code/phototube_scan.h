/**
 * @file phototube_scan.h
 * @brief 光电命中点的坐标变换、在线点云缓存和统计。
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

// phototube_scan.h —— 把单排光电观测换算到原始 odom 扫图坐标系。
//
// 光电阵列装在车轴前方，车在直道上横向摆动或有小偏航时，直接把
// front_offset_mm 连线会把一条物理直线画成蛇形。本模块使用当前运行的
// 原始里程位姿 (x, y) 与连续陀螺航向 yaw，将每个命中点变换到 odom 坐标系，再做
// 方向无关的加权TLS/PCA局部直线诊断。它只负责记录/评估扫图，不直接接管转向。

#ifndef CODE_PHOTOTUBE_SCAN_H_
#define CODE_PHOTOTUBE_SCAN_H_

#include "zf_common_headfile.h"
#include "drive_odometry.h"
#include "globals.h"
#include "phototube_array.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Online matching keeps a 1280-point logical ring in CPU0 DSRAM.  The
 * independent EMEM archive is streamed by telemetry and does not inherit
 * this ring's overwrite boundary. */
// 相邻真实观测按阵列安装点实际运动距离去重；不会在两帧之间插值伪造扫描点。
// 采集时刻必须能在 raw odom 位姿历史中以该最大单侧年龄完成插值。
#define PHOTOTUBE_SCAN_FORWARD_MM (PHOTOTUBE_ARRAY_FORWARD_MM)
/* 实物阵列与车体横向轴平行；这里不是车辆实时 yaw，而是机械安装标定角。 */
/* 正常地图运行时连续记录扫图点并供定位器匹配，但不直接生成转向量。
 * 扫图坐标以 nav_start() 清零后的路线坐标为基准；台架/直线速度测试不自动记录。 */

typedef enum
{
    PHOTOTUBE_SCAN_POSE_INVALID = 0u,
    PHOTOTUBE_SCAN_POSE_INTERPOLATED = 1u
} PhototubeScanPoseSource;

typedef struct
{
    uint32 frame_seq;
    float x_mm;             // 原始 odom 坐标系，前进方向为正
    float y_mm;             // 原始 odom 坐标系，车辆右侧为正
    uint16 travel_distance_mod_mm; // 采集里程模65536；完整值由在线环上下文恢复
    uint8 confidence;       // 同一帧归一化后的匹配权重
    uint8 point_flags;      // VisionScanHitFlag
} PhototubeScanPoint;

typedef struct
{
    uint8 active;
    uint8 valid;
    uint16 point_count;
    uint32 first_frame_seq;
    uint32 last_frame_seq;
    float origin_x_mm;
    float origin_y_mm;
    float origin_yaw_rad;
    float line_slope;       // y = line_slope*x + line_intercept
    float line_intercept;
    float line_center_x_mm; // TLS/PCA局部直线中心
    float line_center_y_mm;
    float line_direction_x; // 单位方向向量，可表达竖直线
    float line_direction_y;
    float x_min_mm;
    float x_max_mm;
    float rms_error_mm;
    float max_abs_error_mm;
    /* Counts cover consumed observations, including frames later rejected by
     * the line-quality gate.  They are diagnostic counters, not point_count. */
    uint16 pose_interpolated_count;
    uint16 pose_rejected_count;
    uint16 mailbox_skipped_count;
    uint16 source_frame_skipped_count;
    uint16 source_schedule_drop_count;
    uint16 source_schedule_skipped_count;
    uint16 max_processing_latency_ms;
    uint16 max_capture_to_cloud_latency_ms;
    uint16 received_frame_count;
    uint16 accepted_frame_count;
    uint16 accepted_hit_count;
    uint16 spatial_dedup_frame_count;
    uint16 travel_reversal_frame_count;
    uint16 online_ring_overwrite_point_count;
    uint16 empty_hit_frame_count;
    uint16 staging_backpressure_count;
    uint16 staging_high_watermark;
    uint16 pose_wait_count;
    uint16 epoch_filtered_frame_count;
    uint16 epoch_ack_timeout_count;
} PhototubeScanSnapshot;

void phototube_scan_init(void);

// 开始一段新扫图。调用方应已完成 nav_start()；扫图原点固定为发车路线坐标原点。
void phototube_scan_start(void);
void phototube_scan_stop(void);
void phototube_scan_clear(void);
uint8 phototube_scan_is_active(void);

// CPU0读取到一帧独立扫描命中包和同一时刻位姿后调用。该帧只做一次运动门控，
// 通过后把单峰/多峰/全覆盖扫描段的全部真实命中写入点云。
uint16 phototube_scan_push_hits(const VisionScanHitPacket *packet,
                                const DriveOdometryPose *pose,
                                float travel_distance_mm,
                                uint32 consume_ms);
void phototube_scan_note_pose_source(PhototubeScanPoseSource source);
void phototube_scan_note_pose_rejected(void);
void phototube_scan_note_pose_wait(void);
void phototube_scan_note_mailbox_gap(uint32 skipped_count);
void phototube_scan_note_hit_packet(const VisionScanHitPacket *packet);
void phototube_scan_note_staging_backpressure(void);
void phototube_scan_note_staging_high_watermark(uint16 count);
void phototube_scan_note_epoch_filtered(void);
void phototube_scan_note_epoch_ack_timeout(void);
void phototube_scan_finish_batch(void);

uint8 phototube_scan_get_snapshot(PhototubeScanSnapshot *snapshot);
uint8 phototube_scan_get_point(uint16 index, PhototubeScanPoint *point);
// 返回最近实际行驶窗口在逻辑时间序列中的首索引和点数。只读指针在下一次
// 扫图写入前有效；匹配服务与写点服务均在CPU0后台串行执行。
uint8 phototube_scan_get_travel_window(float travel_window_mm,
                                       uint16 *first_index,
                                       uint16 *point_count);
const PhototubeScanPoint *phototube_scan_get_point_view(uint16 logical_index);
uint8 phototube_scan_get_point_travel_distance_mm(uint16 logical_index,
                                                  float *travel_distance_mm);

#ifdef __cplusplus
}
#endif

#endif /* CODE_PHOTOTUBE_SCAN_H_ */
