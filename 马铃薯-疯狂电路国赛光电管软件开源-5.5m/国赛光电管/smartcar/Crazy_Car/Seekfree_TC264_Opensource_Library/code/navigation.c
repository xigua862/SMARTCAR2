/**
 * @file navigation.c
 * @brief 点云地图定位、路径投影和 v17 横向控制。
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
 * @note 点云匹配在后台计算，定位修正在 1 ms 导航中断原子提交。
 */

/*
 * navigation.c —— 惯性导航、连续投影与地图参考循迹实现。见 navigation.h。
 * nav_update 每个控制拍完成:编码器平均距离+（正常模式VQF/测试模式编码器差速）中点航向 → 原始odom位姿 →
 * T_map_from_odom校正位姿 → 连续路径投影。取得本拍规划速度和实际平均车速后
 * 从延迟补偿后的离线曲率生成目标偏航率，并叠加Frenet误差回正和陀螺闭环。
 * 光电只生成定位点云和保护观测，不直接生成控制目标。
 */

#include "navigation.h"
#include "encoder_test.h"
#include "control.h"   /* DIST_PER_PULSE_MM:每个编码器脉冲对应的行进距离(mm) */
#include "gyroscope.h" /* VQF yaw:航向反馈；bias校正gyro_z:偏航率阻尼 */
#include "globals.h"
#include "navigation_hash.h"
#include "nav_path.h"
#include "phototube_array.h"
#include "telemetry.h"
#include "drive_odometry.h"
#include "route_projection.h"
#include "route_reference.h"
#include "phototube_scan.h"
#include <math.h>
#include <string.h>

// 速度段表是当前地图协议的一部分。
#ifndef NAV_SPEED_TABLES
#error "nav_path.h 过旧：请用地图编辑器重新导出（需含速度决策段表 NAV_SPEED_TABLES）"
#endif

// 地图曲率用于进度预测、目标偏航率前馈和遥测，只认生成器离线导出的曲率表。
#ifndef NAV_REFERENCE_CURVATURE_TABLES
#error "nav_path.h 过旧：请用地图编辑器重新导出（需含曲率参考表 NAV_REFERENCE_CURVATURE_TABLES）"
#endif
#ifndef NAV_REFERENCE_CURVATURE_DERIVATIVE_TABLES
#error "nav_path.h 过旧：请用地图编辑器重新导出（需含曲率变化率表）"
#endif
#ifndef NAV_SPEED_PROFILE_MAX_SPEED_CMPS
#error "nav_path.h 过旧：请用地图编辑器重新导出（需含全局速度上限）"
#endif
#ifndef NAV_SPEED_PROFILE_MAX_LATERAL_ACCEL_CMPS2
#error "nav_path.h 过旧：请用地图编辑器重新导出（需含横向加速度上限）"
#endif
#ifndef NAV_REFERENCE_SPEED_TABLES
#error "nav_path.h 过旧：请用地图编辑器重新导出（需含参考速度段表）"
#endif
#ifndef NAV_REFERENCE_ACCELERATION_TABLES
#error "nav_path.h 过旧：请用地图编辑器重新导出（需含参考加速度段表）"
#endif
#ifndef NAV_SPEED_PROFILE_PRECOMPUTED
#error "nav_path.h 过旧：速度必须由快速地图编辑器离线规划到每个位置"
#endif
#if NAV_SPEED_PROFILE_PRECOMPUTED != 1
#error "nav_path.h 速度表不是最终离线位置速度轨迹"
#endif
#ifndef NAV_SPEED_PROFILE_END_CMPS
#error "nav_path.h 缺少离线终点停车速度"
#endif
#ifndef NAV_ROUTE_BLIND_BOX
/* Older exported maps have no blind-box marker; treat them as ordinary
 * routes so their endpoint uses the post-end active-braking policy. */
#define NAV_ROUTE_BLIND_BOX (0u)
#endif

#ifndef NAV_MAP_TAKEOVER_TABLES
#error "nav_path.h 过旧：请用地图编辑器重新导出（需含地图接管区表 NAV_MAP_TAKEOVER_TABLES）"
#endif
#ifndef NAV_MAP_TAKEOVER_ENTRY_LEAD_MM
#error "nav_path.h 过旧：请用地图编辑器重新导出（需含地图接管区前瞻 NAV_MAP_TAKEOVER_ENTRY_LEAD_MM）"
#endif
#ifndef NAV_SCAN_FEATURE_TABLES
#error "nav_path.h 过旧：请用地图编辑器重新导出（需含原始扫描特征表 NAV_SCAN_FEATURE_TABLES）"
#endif
#ifndef NAV_SCAN_VERTEX_TYPE_STRAIGHT
#error "nav_path.h 过旧：缺少扫描特征类型定义"
#endif
#ifndef NAV_SCAN_VERTEX_TYPE_CORNER
#error "nav_path.h 过旧：缺少扫描转角类型定义"
#endif
#ifndef NAV_SCAN_VERTEX_TYPE_JUNCTION
#error "nav_path.h 过旧：缺少扫描路口类型定义"
#endif
#ifndef NAV_SCAN_JUNCTION_SUPPORT_TABLES
#error "nav_path.h 过旧：缺少路口非共线支路表"
#endif
#ifndef NAV_SCAN_SEGMENT_VALID_TABLES
#error "nav_path.h 过旧：缺少物理扫描分段有效表"
#endif
#ifndef NAV_SCAN_CORNER_CALIBRATION_MASK_TABLES
#error "nav_path.h 过旧：缺少禁止角点校准掩码表"
#endif
#ifndef NAV_SCAN_SEGMENT_CALIBRATION_MASK_TABLES
#error "nav_path.h 过旧：缺少禁止线段校准掩码表"
#endif
#ifndef NAV_SINGLE_ROUTE_PROTOCOL
#error "nav_path.h 过旧：请只导出当前选中的单条路线"
#endif
#ifndef NAV_SOURCE_ROUTE_INDEX
#error "nav_path.h 缺少当前编辑器路线来源编号"
#endif
#if NAV_PATH_COUNT != 1
#error "车端只接受单路线 nav_path.h；请在地图编辑器选择路线和方向后重新导出"
#endif
#if NAV_SCAN_VERTEX_COUNT_0 > 255
#error "当前定位事务的feature occurrence仅支持0..255"
#endif
#if NAV_SCAN_VERTEX_COUNT_0 > TELEMETRY_LOCALIZATION_FEATURE_CAPACITY
#error "nav_path物理特征数超过逐特征定位诊断槽容量"
#endif
#if NAV_MAP_TAKEOVER_ENTRY_LEAD_MM != NAV_PP_MIN_LOOKAHEAD_MM_INT
#error "地图接管区入口提前量必须与动态前瞻下限相同；请同步地图编辑器并重新导出 nav_path.h"
#endif

// 毫米换算成路径点数(路径按 NAV_PATH_SAMPLE_MM 间隔采样)。
#define NAV_PP_PREDICT_SEARCH_POINTS ((uint16)(NAV_PP_PREDICT_SEARCH_MM / NAV_PATH_SAMPLE_MM))

// 扫图定位使用原始物理线段/特征点，绝不拿平滑 nav_path 反推直角，也不在
// 整张自交地图上盲搜。普通直线按当前路线进度选段；CORNER/JUNCTION
// 都以物理顶点为圆心进入路线候选门，几何点云证据严格限制在顶点150mm内求解。空间补充候选仍受当前滚动里程窗约束，
// 不能因物理坐标重合误选路线另一端的同名角点。转弯路口使用路线入/出边，直穿路口
// 再使用地图导出的非共线支路，因此三岔和十字都可进行完整位姿校准。
// 点云始终保留在原始odom系；匹配时把固定物理地图反变换到odom系。
// 接受修正只更新T_map_from_odom，不改原始里程、位姿历史或历史点云。
#define NAV_SCAN_MATCH_ALGORITHM_VERSION (62u)
#define NAV_SCAN_CORRECTION_TRANSACTION_VERSION (2u)
#define NAV_SCAN_MATCH_SPATIAL_PROGRESS_LIMIT_MM \
    (NAV_SCAN_MATCH_TRAVEL_WINDOW_MM + NAV_SCAN_MATCH_FEATURE_RADIUS_MM)
#define NAV_SCAN_MATCH_FEATURE_WORD_COUNT \
    ((NAV_SCAN_VERTEX_COUNT_0 + 31u) / 32u)

typedef char nav_telemetry_progress_range_check[
    (((sizeof(nav_path_data_0) / sizeof(nav_path_data_0[0])) * 4u) <=
     65535u) ? 1 : -1];

// 角度/弧度互换常数。
static const float nav_deg_per_rad = 57.29577951f;
static const float nav_rad_per_deg = 0.017453293f;

static float nav_wrap_deg(float angle);
static float nav_wrap_rad(float angle);
static void nav_scan_match_reset(void);
static uint8 nav_scan_project_corrected_progress(float current_progress_points,
                                                  float x_mm,
                                                  float y_mm,
                                                  float *progress_points);
static uint8 nav_scan_project_corrected_pose(float current_progress_points,
                                             float x_mm,
                                             float y_mm,
                                             RouteProjectionResult *projection);
static uint8 nav_scan_project_corrected_pose_near_hint(
    float hint_progress_points,
    float x_mm,
    float y_mm,
    RouteProjectionResult *projection);
static uint8 nav_scan_correction_path_error_is_valid(
    float progress_before,
    const DriveOdometryPose *map_pose_before,
    const RouteProjectionResult *candidate_projection,
    uint8 near_hint_only,
    uint8 complete_feature);
static uint8 nav_scan_correction_service_result(void);
static uint8 nav_scan_apply_pending_correction_from_isr(
    DriveOdometryPose *map_pose,
    RouteProjectionResult *projection);
static void nav_scan_delta_translation(float anchor_x_mm,
                                       float anchor_y_mm,
                                       float shift_x_mm,
                                       float shift_y_mm,
                                       float yaw_rad,
                                       float *tx_mm,
                                       float *ty_mm);
static uint8 nav_scan_feature_is_localized(uint16 vertex_index);
static void nav_scan_mark_feature_localized(uint16 vertex_index);
static uint8 nav_scan_feature_is_before_watermark(uint16 vertex_index);
static void nav_scan_track_feature_evidence_boundary(uint16 vertex_index);
static void nav_scan_track_next_route_feature(float sensor_progress_points);
static void nav_scan_advance_feature_evidence_boundary(
    float sensor_progress_points, float boundary_travel_mm,
    uint32 latest_frame_seq);
static uint8 nav_scan_feature_progress_is_consistent(
    uint16 vertex_index, float corrected_progress_points);
static uint16 nav_scan_trim_window_to_cutoff(
    uint16 first_index, uint16 point_count, uint8 have_cutoff,
    float cutoff_travel_mm, uint32 cutoff_frame_seq,
    uint16 *trimmed_first_index, uint16 *trimmed_point_count);

// 导航运行状态以下变量必须落在 CPU0 DSRAM。
// 以下变量必须落在 CPU0 DSRAM(导航跑在控制核 CPU0),用 #pragma section 括到文件末尾
// restore。千万别改动这对括号的范围。
#pragma section all "cpu0_dsram"

typedef struct
{
    uint32 frame_seq;
    float travel_mm;
    float anchor_x_mm;
    float anchor_y_mm;
    float shift_x_mm;
    float shift_y_mm;
    float yaw_rad;
    uint16 key_index;
    uint8 match_type;
    uint8 valid;
    uint8 requires_strong_confirmation;
} NavScanConfirmationCandidate;

typedef struct
{
    float tx_mm;
    float ty_mm;
    float yaw_rad;
} NavMapFromOdom;

typedef struct
{
    NavMapFromOdom transform;
    float cos_yaw;
    float sin_yaw;
} NavScanMapToOdomCache;

// 导航系统全部运行时状态。
typedef struct
{
    NavSystemState state;
    const signed short *data; // 唯一路线航向数据
    const signed short *xs;   // 唯一路线 x 坐标(mm,发车系)
    const signed short *ys;   // 唯一路线 y 坐标(mm,发车系)
    const signed short *curvature; // 地图生成器离线曲率表
    const signed short *curvature_derivative; // κ'(s)离线参考表
    const signed short *scan_vertex_x; // 原始物理特征顶点，禁止用平滑路径替代
    const signed short *scan_vertex_y;
    const unsigned short *scan_vertex_progress;
    const unsigned char *scan_vertex_type;
    /* 1=允许完整 CORNER/JUNCTION SE(2), 0=只允许普通直线窗口校准。 */
    const unsigned char *scan_vertex_corner_calibration_allowed;
    const signed short *scan_vertex_support_x; // 直穿路口的非共线支路端点
    const signed short *scan_vertex_support_y;
    const unsigned char *scan_segment_valid; // 1=实体物理线, 0=虚拟边,禁止点云匹配
    /* 1=允许普通直线校准, 0=该物理线段明确禁止普通直线校准。 */
    const unsigned char *scan_segment_calibration_allowed;
    float curvature_lsb_per_mm;    // 曲率表每 LSB 对应 rad/mm
    float curvature_derivative_lsb_per_mm2;
    uint16 point_count;       // 当前路线点数
    uint16 scan_vertex_count;  // 当前路线原始扫描特征顶点数
    uint16 scan_segment_count; // 扫描顶点之间的分段数
    uint32 route_hash;        // 当前路线 yaw/x/y/曲率/速度表内容标识
    uint16 replay_index;      // 进度:当前最近路径点序号
    float progress_points;    // 连续路径进度(点序号,用于前视插值)
    DriveOdometryPose raw_odometry; // 仅由编码器和原始VQF推进，定位不得改写
    uint32 raw_pose_generation; // 每次原始里程积分递增，供定位事务复核
    NavMapFromOdom map_from_odom;   // 地图位姿 = T_map_from_odom * raw_odometry
    DriveOdometryPose map_pose;     // 当前派生地图位姿，与变换/进度原子提交
    uint16 localization_update_seq;
    float correction;         // 地图目标 yaw 与地图系实际 yaw 的误差(度)

    // 速度决策段表(编辑器导出,RLE):end=段末样本下标(含),cmps=该段目标速度(cm/s)。
    // 速度值在编辑器里设、随地图烘进 nav_path.h——改速度=改图重导出,不走 Flash。
    const uint16 *speed_end_p;
    const int16 *speed_cmps_p;
    const uint16 *acceleration_end_p;
    const int16 *acceleration_cmps2_p;
    uint16 speed_seg_cnt;
    uint16 speed_seg_idx; // 当前段游标:随 replay_index 单调前进,nav_start 清零
    uint16 acceleration_seg_cnt;
    uint16 acceleration_seg_idx;

    // 地图物理特征接管区；当前只用于点云可信时短暂放宽全黑保护。
    const uint16 *map_takeover_lo_p;
    const uint16 *map_takeover_hi_p;
    uint8 map_takeover_cnt;

    float kappa_per_mm;        // 当前点曲率(rad/mm):读取离线曲率表,右转为正
    float reference_curvature_per_mm; // 投影点处曲率，供 Frenet 进度预测使用
    float ey_mm;               // 有符号横向误差，路径右侧为正，供遥测诊断
    float epsi_rad;            // VQF 航向误差，车辆航向−路径航向

} NavigationSystem;

// 单写(1ms 速度环)多读(菜单/后台任务),字段按 32 位对齐读取。
static volatile NavigationSystem nav;
static uint32 nav_telemetry_last_ms = 0u;
static float nav_signed_cross_track_mm = 0.0f;
static uint8 nav_scan_vertex_progress_valid = 0u;
static NavScanMapToOdomCache nav_scan_map_to_odom_cache;

typedef struct
{
    uint32 last_scan_frame_seq;
    uint32 current_evidence_frame_seq;
    float current_evidence_travel_mm;
    uint32 last_match_ms;
    float last_match_progress_points;
    float last_rms_before_mm;
    float last_rms_after_mm;
    float last_yaw_correction_rad;
    uint16 last_vertex_index;
    uint16 last_inlier_count;
    uint8 last_match_type;
    uint8 last_reject_reason;
    uint8 valid;
    uint8 feature_localized;
    float last_consumed_travel_mm;
    uint32 last_consumed_frame_seq;
    uint8 have_consumed_evidence;
    NavScanConfirmationCandidate straight_pending;
    NavScanConfirmationCandidate feature_pending;
    float last_feature_consumed_travel_mm;
    uint32 last_feature_consumed_frame_seq;
    uint16 last_localized_feature_progress;
    uint8 have_feature_consumed_evidence;
    uint8 have_localized_feature_progress;
    uint16 pending_feature_boundary_index;
    float pending_feature_boundary_progress;
    uint8 have_pending_feature_boundary;
    float feature_evidence_boundary_travel_mm;
    uint32 feature_evidence_boundary_frame_seq;
    uint16 last_feature_evidence_progress;
    uint8 have_feature_evidence_boundary;
    float last_straight_diagnostic_travel_mm;
    uint8 last_straight_diagnostic_reason;
    uint8 have_straight_diagnostic;
    uint32 localized_feature_bits[NAV_SCAN_MATCH_FEATURE_WORD_COUNT];
} NavScanMapMatchState;

static NavScanMapMatchState nav_scan_match;
static uint16 nav_scan_match_first_index;
static float nav_scan_straight_residual[PHOTOTUBE_SCAN_MAX_POINTS];
static float nav_scan_straight_mode_histogram[
    NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT];

static const PhototubeScanPoint *nav_scan_match_point_at(uint16 index)
{
    return phototube_scan_get_point_view(
        (uint16)(nav_scan_match_first_index + index));
}

static uint8 nav_scan_match_point_travel_distance_mm(
    uint16 index,
    float *travel_distance_mm)
{
    return phototube_scan_get_point_travel_distance_mm(
        (uint16)(nav_scan_match_first_index + index), travel_distance_mm);
}

/* Apply one caller-selected evidence cutoff to a logical point window.  The
 * feature and straight matchers maintain independent cutoffs so an accepted
 * straight correction cannot hide incoming-edge evidence from a later
 * complete feature solve. */
static uint16 nav_scan_trim_window_to_cutoff(
    uint16 first_index, uint16 point_count, uint8 have_cutoff,
    float cutoff_travel_mm, uint32 cutoff_frame_seq,
    uint16 *trimmed_first_index, uint16 *trimmed_point_count)
{
    uint16 offset = 0u;

    if ((NULL == trimmed_first_index) || (NULL == trimmed_point_count))
    {
        return 0u;
    }
    if (have_cutoff)
    {
        while (offset < point_count)
        {
            const PhototubeScanPoint *point =
                phototube_scan_get_point_view(
                    (uint16)(first_index + offset));
            float point_travel_mm = 0.0f;

            if ((NULL != point) &&
                phototube_scan_get_point_travel_distance_mm(
                    (uint16)(first_index + offset), &point_travel_mm) &&
                (point->frame_seq > cutoff_frame_seq) &&
                (point_travel_mm > cutoff_travel_mm))
            {
                break;
            }
            offset++;
        }
    }
    *trimmed_first_index = (uint16)(first_index + offset);
    *trimmed_point_count = (uint16)(point_count - offset);
    return *trimmed_point_count;
}

static float nav_scan_point_weight(const PhototubeScanPoint *point)
{
    if ((NULL == point) || (0u == point->confidence))
    {
        return 0.0f;
    }
    return (float)point->confidence / 255.0f;
}

/* A wide continuous run publishes its centre plus both measured boundaries.
 * Boundaries describe width, not an independent road edge.  Complete feature
 * matching requires a unique, non-edge (non-multiple) centre; ordinary
 * straight matching may separately use a clipped narrow single centre below.
 * A junction may opt into multi/full-coverage evidence separately below.
 * Legacy points without the new semantic bits remain usable for old recordings. */
static uint8 nav_scan_point_is_centerline(
    const PhototubeScanPoint *point)
{
    if ((NULL == point) || (0u == point->confidence) ||
        (0u != (point->point_flags & VISION_SCAN_HIT_BOUNDARY)))
    {
        return 0u;
    }
    return (uint8)((0u != (point->point_flags & VISION_SCAN_HIT_CENTER)) ||
                   ((0u == (point->point_flags &
                            (VISION_SCAN_HIT_CENTER |
                             VISION_SCAN_HIT_BOUNDARY))) &&
                    (0u == (point->point_flags &
                            VISION_SCAN_HIT_FULL_COVERAGE))));
}

static uint8 nav_scan_point_is_junction_evidence(
    const PhototubeScanPoint *point)
{
    if ((NULL == point) || (0u == point->confidence) ||
        (0u != (point->point_flags & VISION_SCAN_HIT_BOUNDARY)))
    {
        return 0u;
    }
    return (uint8)(nav_scan_point_is_centerline(point) ||
                   (0u != (point->point_flags &
                           VISION_SCAN_HIT_FULL_COVERAGE)));
}

static uint8 nav_scan_point_is_primary_centerline(
    const PhototubeScanPoint *point)
{
    return (uint8)(nav_scan_point_is_centerline(point) &&
                   (0u == (point->point_flags &
                           (VISION_SCAN_HIT_MULTIPLE |
                            VISION_SCAN_HIT_FULL_COVERAGE |
                            VISION_SCAN_HIT_EDGE))));
}

static uint8 nav_scan_point_is_straight_centerline(
    const PhototubeScanPoint *point)
{
    uint8 flags;

    if (!nav_scan_point_is_centerline(point))
    {
        return 0u;
    }
    flags = point->point_flags;
    if (0u != (flags & (VISION_SCAN_HIT_MULTIPLE |
                        VISION_SCAN_HIT_FULL_COVERAGE)))
    {
        return 0u;
    }
    if (0u == (flags & VISION_SCAN_HIT_EDGE))
    {
        return 1u;
    }
    /* Only a narrow, unique centre clipped by one array end is usable.  A
     * wide run touching the edge has an unknown centre and remains diagnostic
     * evidence only. */
    return (uint8)((0u != (flags & VISION_SCAN_HIT_SINGLE)) &&
                   (0u != (flags & VISION_SCAN_HIT_CENTER)) &&
                   (0u == (flags & VISION_SCAN_HIT_RUN_SAMPLE)));
}

static float nav_scan_straight_point_weight(
    const PhototubeScanPoint *point)
{
    float weight = nav_scan_point_weight(point);

    if ((NULL != point) &&
        (0u != (point->point_flags & VISION_SCAN_HIT_EDGE)))
    {
        weight *= NAV_SCAN_MATCH_STRAIGHT_EDGE_WEIGHT_RATIO;
    }
    return weight;
}

/* Preserve the old short-edge guard: only an unqualified single centre hit is
 * strong enough to rescue a very short edge from component noise. */
static uint8 nav_scan_point_is_clean_centerline(
    const PhototubeScanPoint *point)
{
    if (!nav_scan_point_is_primary_centerline(point))
    {
        return 0u;
    }
    return (uint8)((0u != (point->point_flags & VISION_SCAN_HIT_SINGLE)) &&
                   (0u == (point->point_flags &
                           (VISION_SCAN_HIT_EDGE |
                            VISION_SCAN_HIT_RUN_SAMPLE |
                            VISION_SCAN_HIT_MULTIPLE |
                             VISION_SCAN_HIT_FULL_COVERAGE))));
}

static uint8 nav_scan_point_is_feature_evidence(
    const PhototubeScanPoint *point, uint8 feature_type)
{
    if (NAV_SCAN_VERTEX_TYPE_JUNCTION == feature_type)
    {
        return nav_scan_point_is_junction_evidence(point);
    }
    return nav_scan_point_is_primary_centerline(point);
}

static void nav_scan_merge_evidence_cutoff(uint8 *have_cutoff,
                                           float *cutoff_travel_mm,
                                           uint32 *cutoff_frame_seq,
                                           float travel_mm,
                                           uint32 frame_seq)
{
    if ((NULL == have_cutoff) || (NULL == cutoff_travel_mm) ||
        (NULL == cutoff_frame_seq))
    {
        return;
    }
    if (!*have_cutoff || (travel_mm > *cutoff_travel_mm) ||
        ((travel_mm == *cutoff_travel_mm) &&
         (frame_seq > *cutoff_frame_seq)))
    {
        *have_cutoff = 1u;
        *cutoff_travel_mm = travel_mm;
        *cutoff_frame_seq = frame_seq;
    }
}

static void nav_scan_merge_pending_candidate_cutoff(
    const NavScanConfirmationCandidate *candidate,
    uint8 *have_cutoff,
    float *cutoff_travel_mm,
    uint32 *cutoff_frame_seq)
{
    float pending_cutoff_travel_mm;

    if ((NULL == candidate) || !candidate->valid)
    {
        return;
    }
    pending_cutoff_travel_mm = candidate->travel_mm -
        NAV_SCAN_MATCH_CONFIRM_OVERLAP_MM;
    if (pending_cutoff_travel_mm < 0.0f)
    {
        pending_cutoff_travel_mm = 0.0f;
    }
    nav_scan_merge_evidence_cutoff(
        have_cutoff, cutoff_travel_mm, cutoff_frame_seq,
        pending_cutoff_travel_mm, 0u);
}

/* In-place quickselect for the small bounded straight-match scratch buffer. */
static float nav_scan_select_float(float *values, uint16 count, uint16 target)
{
    int32 left;
    int32 right;

    if ((NULL == values) || (0u == count) || (target >= count))
    {
        return 0.0f;
    }
    left = 0;
    right = (int32)count - 1;
    while (left < right)
    {
        int32 i = left;
        int32 j = right;
        float pivot = values[(uint16)((left + right) / 2)];

        while (i <= j)
        {
            while (values[(uint16)i] < pivot) i++;
            while (values[(uint16)j] > pivot) j--;
            if (i <= j)
            {
                float temporary = values[(uint16)i];
                values[(uint16)i] = values[(uint16)j];
                values[(uint16)j] = temporary;
                i++;
                j--;
            }
        }
        if ((int32)target <= j)
        {
            right = j;
        }
        else if ((int32)target >= i)
        {
            left = i;
        }
        else
        {
            break;
        }
    }
    return values[target];
}

/* Select a confidence-weighted residual mode.  A second non-overlapping mode
 * with comparable support means the straight observation is ambiguous: do
 * not guess which stripe is the route line. */
static uint8 nav_scan_select_straight_residual_mode(float *center_mm)
{
    uint16 center;
    uint16 best_center = 0u;
    float best_support = 0.0f;
    float second_support = 0.0f;
    float selected_center_sum = 0.0f;

    if (NULL == center_mm)
    {
        return 0u;
    }
    for (center = 0u;
         center < NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT;
         center++)
    {
        uint16 first = (center >
                        NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS) ?
            (uint16)(center -
                     NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS) : 0u;
        uint16 last = (uint16)(center +
                              NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS);
        uint16 bin;
        float support = 0.0f;
        if (last >= NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT)
        {
            last = (uint16)(NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT - 1u);
        }
        for (bin = first; bin <= last; bin++)
        {
            support += nav_scan_straight_mode_histogram[bin];
        }
        if (support > best_support)
        {
            best_support = support;
            best_center = center;
        }
    }
    if (best_support < NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT)
    {
        return 0u;
    }

    for (center = 0u;
         center < NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT;
         center++)
    {
        uint16 separation = (center > best_center) ?
            (uint16)(center - best_center) :
            (uint16)(best_center - center);
        uint16 first;
        uint16 last;
        uint16 bin;
        float support = 0.0f;
        if (separation <
            NAV_SCAN_MATCH_STRAIGHT_MODE_MIN_SEPARATION_BINS)
        {
            continue;
        }
        first = (center > NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS) ?
            (uint16)(center -
                     NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS) : 0u;
        last = (uint16)(center +
                        NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS);
        if (last >= NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT)
        {
            last = (uint16)(NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT - 1u);
        }
        for (bin = first; bin <= last; bin++)
        {
            support += nav_scan_straight_mode_histogram[bin];
        }
        if (support > second_support)
        {
            second_support = support;
        }
    }

    {
        uint16 first = (best_center >
                        NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS) ?
            (uint16)(best_center -
                     NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS) : 0u;
        uint16 last = (uint16)(best_center +
                              NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS);
        uint16 bin;
        if (last >= NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT)
        {
            last = (uint16)(NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT - 1u);
        }
        for (bin = first; bin <= last; bin++)
        {
            float bin_center =
                -NAV_SCAN_MATCH_MAX_STRAIGHT_POINT_RESIDUAL_MM +
                ((float)bin + 0.5f) *
                    NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_MM;
            selected_center_sum +=
                nav_scan_straight_mode_histogram[bin] * bin_center;
        }
    }
    *center_mm = selected_center_sum / best_support;
    if ((second_support >= NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT) &&
        (second_support >= best_support *
                           NAV_SCAN_MATCH_STRAIGHT_AMBIGUITY_WEIGHT_RATIO))
    {
        return 2u;
    }
    return 1u;
}

enum
{
    NAV_SCAN_MATCH_TYPE_NONE = 0u,
    NAV_SCAN_MATCH_TYPE_STRAIGHT = 1u,
    NAV_SCAN_MATCH_TYPE_CORNER = 2u,
    NAV_SCAN_MATCH_TYPE_JUNCTION = 3u
};

enum
{
    NAV_SCAN_MATCH_REJECT_NONE = 0u,
    NAV_SCAN_MATCH_REJECT_NOT_ENOUGH_POINTS = 1u,
    NAV_SCAN_MATCH_REJECT_NEAR_JUNCTION = 2u,
    NAV_SCAN_MATCH_REJECT_PARTIAL_CORNER = 3u,
    NAV_SCAN_MATCH_REJECT_GEOMETRY = 4u,
    NAV_SCAN_MATCH_REJECT_RMS = 5u,
    NAV_SCAN_MATCH_REJECT_CORRECTION = 6u,
    NAV_SCAN_MATCH_REJECT_AMBIGUOUS_FEATURE = 7u,
    NAV_SCAN_MATCH_REJECT_PARTIAL_JUNCTION = 8u,
    NAV_SCAN_MATCH_REJECT_FEATURE_CONSUMED = 9u,
    NAV_SCAN_MATCH_REJECT_WAIT_CONFIRMATION = 10u,
    NAV_SCAN_MATCH_REJECT_NO_FRESH_EVIDENCE = 11u,
    NAV_SCAN_MATCH_REJECT_NO_IMPROVEMENT = 12u,
    NAV_SCAN_MATCH_REJECT_PROJECTION = 13u,
    NAV_SCAN_MATCH_REJECT_COMMIT_BUSY = 14u,
    NAV_SCAN_MATCH_REJECT_FEATURE_ORDER = 15u,
    NAV_SCAN_MATCH_REJECT_FEATURE_PROGRESS = 16u,
    NAV_SCAN_MATCH_REJECT_STOPPED = 17u,
    NAV_SCAN_MATCH_REJECT_STRAIGHT_NOT_SIGNIFICANT = 18u,
    NAV_SCAN_MATCH_REJECT_AMBIGUOUS_STRAIGHT = 19u,
    NAV_SCAN_MATCH_REJECT_PROGRESS_REGRESSION = 20u,
    NAV_SCAN_MATCH_REJECT_STRONG_CONFIRMATION = 21u,
    NAV_SCAN_MATCH_REJECT_PATH_ERROR = 22u,
    /* Straight-line matcher diagnostics.  These are separate transaction
     * reasons so a structural miss is not collapsed into generic geometry. */
    NAV_SCAN_MATCH_REJECT_STRAIGHT_NO_MAP_LINE = 23u,
    NAV_SCAN_MATCH_REJECT_STRAIGHT_CANDIDATE_POINTS = 24u,
    NAV_SCAN_MATCH_REJECT_STRAIGHT_CANDIDATE_WEIGHT = 25u,
    NAV_SCAN_MATCH_REJECT_STRAIGHT_RESIDUAL_MODE = 26u,
    NAV_SCAN_MATCH_REJECT_STRAIGHT_MODE_POINTS = 27u,
    NAV_SCAN_MATCH_REJECT_STRAIGHT_INLIER_POINTS = 28u,
    NAV_SCAN_MATCH_REJECT_STRAIGHT_INLIER_WEIGHT = 29u,
    NAV_SCAN_MATCH_REJECT_STRAIGHT_INLIER_RATIO = 30u,
    /* Span, independent-frame count, and tangent variance all describe
     * whether yaw is observable from the retained straight window. */
    NAV_SCAN_MATCH_REJECT_STRAIGHT_SPAN = 31u,
    NAV_SCAN_MATCH_REJECT_STRAIGHT_CALIBRATION_BLOCKED = 32u
};

enum
{
    NAV_SCAN_CORRECTION_IDLE = 0u,
    NAV_SCAN_CORRECTION_PENDING = 1u,
    NAV_SCAN_CORRECTION_COMPLETE = 2u
};

static NavScanConfirmationCandidate *nav_scan_pending_candidate(
    uint8 match_type)
{
    if (NAV_SCAN_MATCH_TYPE_STRAIGHT == match_type)
    {
        return &nav_scan_match.straight_pending;
    }
    if ((NAV_SCAN_MATCH_TYPE_CORNER == match_type) ||
        (NAV_SCAN_MATCH_TYPE_JUNCTION == match_type))
    {
        return &nav_scan_match.feature_pending;
    }
    return NULL;
}

static void nav_scan_clear_pending_candidate(uint8 match_type)
{
    NavScanConfirmationCandidate *candidate =
        nav_scan_pending_candidate(match_type);

    if (NULL != candidate)
    {
        candidate->valid = 0u;
        candidate->requires_strong_confirmation = 0u;
    }
}

static void nav_scan_clear_all_pending_candidates(void)
{
    nav_scan_match.straight_pending.valid = 0u;
    nav_scan_match.straight_pending.requires_strong_confirmation = 0u;
    nav_scan_match.feature_pending.valid = 0u;
    nav_scan_match.feature_pending.requires_strong_confirmation = 0u;
}

static const NavScanConfirmationCandidate *
nav_scan_latest_pending_candidate(void)
{
    const NavScanConfirmationCandidate *straight =
        &nav_scan_match.straight_pending;
    const NavScanConfirmationCandidate *feature =
        &nav_scan_match.feature_pending;

    if (!straight->valid)
    {
        return feature->valid ? feature : NULL;
    }
    if (!feature->valid)
    {
        return straight;
    }
    return (feature->travel_mm >= straight->travel_mm) ? feature : straight;
}

/* The point-cloud solver and the wide projection run in the CPU0 main loop.
 * The next 1 ms navigation update recomputes the corrected pose from its
 * authoritative raw odometry and performs only a short exact projection near
 * this hint before atomically publishing map pose, progress and controller
 * state. */
typedef struct
{
    volatile uint8 state;
    uint8 status;
    uint8 reject_reason;
    uint8 match_type;
    uint8 complete_feature;
    uint16 vertex_index;
    uint16 inlier_count;
    uint16 localization_update_seq;
    uint32 commit_timestamp_ms;
    uint32 evidence_frame_seq;
    float evidence_travel_mm;
    NavMapFromOdom transform_before;
    NavMapFromOdom transform_after;
    RouteProjectionResult projection_hint;
    float projection_hint_source_progress;
    DriveOdometryPose raw_pose;
    float delta_x_mm;
    float delta_y_mm;
    float delta_yaw_rad;
    uint8 partial_feature;
    float progress_before;
    float progress_after;
    float rms_before_mm;
    float rms_after_mm;
    uint32 raw_pose_generation_hint;
} NavScanCorrectionTransaction;

static NavScanCorrectionTransaction nav_scan_correction;

typedef struct
{
    float ax;
    float ay;
    float bx;
    float by;
    float tx;
    float ty;
    float nx;
    float ny;
    float length;
} NavScanLine;

static void nav_scan_line_residual_unbounded(const NavScanLine *line,
                                             float x_mm, float y_mm,
                                             float *residual_mm,
                                             float *tangent_mm)
{
    float dx;
    float dy;

    dx = x_mm - line->ax;
    dy = y_mm - line->ay;
    *residual_mm = line->nx * dx + line->ny * dy;
    *tangent_mm = line->tx * dx + line->ty * dy;
}

typedef struct
{
    uint32 detail_flags;
    uint16 line_point_count[3];
    uint16 line_frame_count[3];
    float failure_value;
} NavScanFeatureAttemptDiagnostics;

static NavScanFeatureAttemptDiagnostics nav_scan_feature_attempt;

/* The scan-to-map seed chooses correspondences automatically.  Once the best
 * local pose is found, the final robust solve fixes that correspondence set;
 * 0xff means that the point is not an inlier. */
static uint8 nav_scan_feature_assignment[PHOTOTUBE_SCAN_MAX_POINTS];
static uint16 nav_scan_feature_search_indices[PHOTOTUBE_SCAN_MAX_POINTS];

static void nav_scan_feature_attempt_reset(void)
{
    memset(&nav_scan_feature_attempt, 0,
           sizeof(nav_scan_feature_attempt));
}

static uint8 nav_scan_u16_to_u8_saturated(uint16 value)
{
    return (value > 255u) ? 255u : (uint8)value;
}

static void nav_scan_feature_capture_lines(const uint16 *counts,
                                           const uint16 *frame_counts,
                                           uint8 line_count)
{
    uint8 index;

    for (index = 0u; index < 3u; index++)
    {
        nav_scan_feature_attempt.line_point_count[index] =
            (index < line_count) ? counts[index] : 0u;
        nav_scan_feature_attempt.line_frame_count[index] =
            (index < line_count) ? frame_counts[index] : 0u;
    }
}

static void nav_scan_feature_mark_line_quality(
    const uint16 *counts,
    const float *weights,
    const float *tangent_min,
    const float *tangent_max,
    const uint16 *frame_counts,
    const uint16 *run_frame_counts,
    uint8 allow_run_frame_pair,
    uint8 line_count,
    float min_side_effective_weight)
{
    uint8 index;

    nav_scan_feature_capture_lines(counts, frame_counts, line_count);
    for (index = 0u; index < line_count; index++)
    {
        if (counts[index] < NAV_SCAN_MATCH_MIN_CORNER_SIDE_COUNT)
        {
            nav_scan_feature_attempt.detail_flags |=
                (TELEMETRY_LOCALIZATION_DETAIL_LINE0_POINTS << index);
        }
        if (weights[index] < min_side_effective_weight)
        {
            nav_scan_feature_attempt.detail_flags |=
                (TELEMETRY_LOCALIZATION_DETAIL_LINE0_WEIGHT << index);
        }
        if ((tangent_max[index] - tangent_min[index]) <
            NAV_SCAN_MATCH_MIN_FEATURE_SIDE_SPAN_MM)
        {
            nav_scan_feature_attempt.detail_flags |=
                (TELEMETRY_LOCALIZATION_DETAIL_LINE0_SPAN << index);
        }
        if ((frame_counts[index] < NAV_SCAN_MATCH_MIN_SIDE_FRAME_COUNT) &&
            (!allow_run_frame_pair ||
             (NULL == run_frame_counts) ||
             (frame_counts[index] <
              NAV_SCAN_MATCH_MIN_RUN_SIDE_FRAME_COUNT) ||
             (run_frame_counts[index] <
              NAV_SCAN_MATCH_MIN_RUN_SIDE_FRAME_COUNT)))
        {
            nav_scan_feature_attempt.detail_flags |=
                (TELEMETRY_LOCALIZATION_DETAIL_LINE0_FRAMES << index);
        }
    }
}

typedef struct
{
    float target_rate_radps;
    float rate_error_radps; // omega_cmd - omega_measured
    uint8 valid;
} NavYawRateTrackerState;

static NavYawRateTrackerState nav_yaw_rate_tracker;
/* One coherent IMU sample is read at the start of each control update.  The
 * turn calculation below reuses this yaw-rate instead of reading a second,
 * potentially newer VQF publication. */
static float nav_control_yaw_rate_radps = 0.0f;
static uint8 nav_control_imu_valid = 0u;

static ImuSnapshot nav_last_imu_snapshot;
static uint8 nav_have_last_imu_snapshot = 0u;
static uint32 nav_last_imu_local_us = 0u;
static uint8 nav_have_imu_local_clock = 0u;
static uint32 nav_imu_missing_start_us = 0u;
static uint8 nav_have_imu_missing_clock = 0u;
static volatile uint8 nav_imu_stale_fault = 0u;

static void nav_reset_imu_snapshot_history(void)
{
    memset((void *)&nav_last_imu_snapshot, 0,
           sizeof(nav_last_imu_snapshot));
    nav_have_last_imu_snapshot = 0u;
    nav_last_imu_local_us = 0u;
    nav_have_imu_local_clock = 0u;
    nav_imu_missing_start_us = 0u;
    nav_have_imu_missing_clock = 0u;
    nav_imu_stale_fault = 0u;
    imu_snapshot_read_failure_count = 0u;
    imu_snapshot_consecutive_failure_count = 0u;
    imu_snapshot_max_consecutive_failure_count = 0u;
    imu_snapshot_last_age_us = 0u;
    imu_snapshot_max_age_us = 0u;
}

static float nav_imu_snapshot_heading_at_now(const ImuSnapshot *snapshot,
                                             uint32 age_us)
{
    uint32 extrapolation_us = age_us;

    if (extrapolation_us > NAV_IMU_MAX_EXTRAPOLATION_US)
    {
        extrapolation_us = NAV_IMU_MAX_EXTRAPOLATION_US;
    }
    return snapshot->heading_deg +
           snapshot->yaw_rate_radps * ((float)extrapolation_us * 1.0e-6f) *
           nav_deg_per_rad;
}

/* Read one coherent CPU1 sample for the control tick.  The encoder deltas
 * have already been consumed by control.c before this helper is called, so a
 * failed seqlock read must fall back to the last sample instead of returning
 * without integrating the tick. */
static uint8 nav_read_imu_for_control(float *heading_deg,
                                      float *yaw_rate_radps)
{
    ImuSnapshot snapshot;
    uint32 now_us;
    uint32 age_us;
    uint32 publish_gap_us;
    uint8 yaw_rate_valid;
    uint8 new_snapshot;

    if ((NULL == heading_deg) || (NULL == yaw_rate_radps))
    {
        return 0u;
    }

    now_us = system_getval_us();
    if (imu_snapshot_read(&snapshot))
    {
        /* sample_us belongs to CPU1's local STM epoch.  Do not subtract it
         * from CPU0's STM value; instead anchor the age when CPU0 first sees
         * each published sequence. */
        new_snapshot = (uint8)(
            !nav_have_imu_local_clock ||
            !nav_have_last_imu_snapshot ||
            (snapshot.publish_seq != nav_last_imu_snapshot.publish_seq));
        if (new_snapshot)
        {
            /* Check the interval that just elapsed before re-anchoring the
             * local arrival clock.  Re-anchoring first would turn a long CPU1
             * publication gap into an apparent age of zero as soon as the
             * next frame arrived, defeating the sticky stale-IMU stop gate. */
            if (nav_have_imu_local_clock && nav_have_last_imu_snapshot)
            {
                publish_gap_us = now_us - nav_last_imu_local_us;
                if (publish_gap_us > imu_snapshot_max_age_us)
                {
                    imu_snapshot_max_age_us = publish_gap_us;
                }
                if (publish_gap_us >= NAV_IMU_HARD_STALE_US)
                {
                    nav_imu_stale_fault = 1u;
                }
            }
            nav_last_imu_local_us = now_us;
            nav_have_imu_local_clock = 1u;
        }
        age_us = nav_have_imu_local_clock ?
            (now_us - nav_last_imu_local_us) : 0u;
        nav_last_imu_snapshot = snapshot;
        nav_have_last_imu_snapshot = 1u;
        nav_have_imu_missing_clock = 0u;
        imu_snapshot_consecutive_failure_count = 0u;
        imu_snapshot_last_age_us = age_us;
        if (age_us > imu_snapshot_max_age_us)
        {
            imu_snapshot_max_age_us = age_us;
        }
        if (age_us >= NAV_IMU_HARD_STALE_US)
        {
            nav_imu_stale_fault = 1u;
        }
        *heading_deg = nav_imu_snapshot_heading_at_now(&snapshot, age_us);
        yaw_rate_valid = (uint8)(age_us <= NAV_IMU_MAX_EXTRAPOLATION_US);
        *yaw_rate_radps = yaw_rate_valid ? snapshot.yaw_rate_radps : 0.0f;
        return yaw_rate_valid;
    }

    if (imu_snapshot_read_failure_count < 0xFFFFFFFFu)
    {
        imu_snapshot_read_failure_count++;
    }
    if (imu_snapshot_consecutive_failure_count < 0xFFFFu)
    {
        imu_snapshot_consecutive_failure_count++;
    }
    if (imu_snapshot_consecutive_failure_count >
        imu_snapshot_max_consecutive_failure_count)
    {
        imu_snapshot_max_consecutive_failure_count =
            imu_snapshot_consecutive_failure_count;
    }

    if (!nav_have_last_imu_snapshot)
    {
        /* No coherent sample exists yet.  Keep the current raw heading, if
         * the odometry has already been initialized, and still integrate the
         * encoder distance for this tick. */
        *heading_deg = nav.raw_odometry.gyro_heading_initialized ?
            nav.raw_odometry.gyro_heading_rad * nav_deg_per_rad : 0.0f;
        *yaw_rate_radps = 0.0f;
        imu_snapshot_last_age_us = 0u;
        if (!nav_have_imu_missing_clock)
        {
            nav_imu_missing_start_us = now_us;
            nav_have_imu_missing_clock = 1u;
        }
        else if ((uint32)(now_us - nav_imu_missing_start_us) >=
                 NAV_IMU_HARD_STALE_US)
        {
            nav_imu_stale_fault = 1u;
        }
        return 0u;
    }

    age_us = nav_have_imu_local_clock ?
        (now_us - nav_last_imu_local_us) : 0u;
    imu_snapshot_last_age_us = age_us;
    if (age_us > imu_snapshot_max_age_us)
    {
        imu_snapshot_max_age_us = age_us;
    }
    if (age_us >= NAV_IMU_HARD_STALE_US)
    {
        nav_imu_stale_fault = 1u;
    }
    *heading_deg = nav_imu_snapshot_heading_at_now(
        &nav_last_imu_snapshot, age_us);
    yaw_rate_valid = (uint8)(age_us <= NAV_IMU_MAX_EXTRAPOLATION_US);
    *yaw_rate_radps = yaw_rate_valid ?
        nav_last_imu_snapshot.yaw_rate_radps : 0.0f;
    return yaw_rate_valid;
}

uint8 nav_imu_stale_faulted(void)
{
    return nav_imu_stale_fault;
}

static void nav_yaw_rate_tracker_reset(void)
{
    nav_yaw_rate_tracker.target_rate_radps = 0.0f;
    nav_yaw_rate_tracker.rate_error_radps = 0.0f;
    nav_yaw_rate_tracker.valid = 0u;
}

/* 最新线路观测只用于点云之外的遥测诊断。扫图任务使用独立命中队列，
 * 控制器不读取 line_position_mm 生成目标 yaw。 */
static uint32 nav_line_last_publish_seq = 0u;

static void nav_line_session_reset(void)
{
    nav_line_last_publish_seq = vision_pose_publish_seq;
}

static void nav_line_update_observation(void)
{
    VisionPoseObservation observation;

    if (vision_observation_read_new(&nav_line_last_publish_seq, &observation))
    {
        if (observation.capture_epoch != vision_scan_run_epoch)
        {
            return;
        }
        telemetry_capture_vision_isr(observation.capture_ms,
                                     observation.frame_seq,
                                     observation.line_position_mm,
                                     observation.active_mask,
                                     observation.flags,
                                     observation.peak_count,
                                     observation.white_percent,
                                     observation.threshold_gray);
    }
}

void nav_update_line_diagnostics(void)
{
    nav_line_update_observation();
}

// 地图接管区边界以5mm样本下标导出；直接比较连续进度，避免整数回放索引
// 在边界附近引入一个采样点的切换量化误差。
static uint8 nav_scan_match_is_fresh(void)
{
    if ((NAV_STATE_NAVIGATING != nav.state) ||
        !nav_scan_match.valid ||
        ((uint32)(vision_clock_ms - nav_scan_match.last_match_ms) >
         NAV_SCAN_MATCH_BLACKOUT_HOLD_MS))
    {
        return 0u;
    }
    return 1u;
}

static uint8 nav_progress_in_map_takeover_zone(float progress_points)
{
    uint8 i;

    if (0u == nav.map_takeover_cnt)
    {
        return 0u;
    }
    if ((NULL == nav.map_takeover_lo_p) || (NULL == nav.map_takeover_hi_p))
    {
        return 1u;
    }
    for (i = 0u; i < nav.map_takeover_cnt; i++)
    {
        if ((progress_points >= (float)nav.map_takeover_lo_p[i]) &&
            (progress_points <= (float)nav.map_takeover_hi_p[i]))
        {
            return 1u;
        }
    }
    return 0u;
}

static void nav_capture_state_transition(NavSystemState previous_state,
                                         NavSystemState current_state)
{
    /* nav_start() also resets the boot-time push-debug session with the same enum value. */
    telemetry_capture_event_isr(TELEMETRY_EVENT_NAV_STATE,
                                TELEMETRY_EVENT_SOURCE_NAVIGATION,
                                (uint16)current_state,
                                (int32)previous_state,
                                (int32)current_state);
}

static uint32 nav_calculate_localization_config_hash(void)
{
    uint32 hash = 2166136261u;

    hash = nav_hash_u32(hash, NAV_SCAN_MATCH_ALGORITHM_VERSION);
    hash = nav_hash_u8(hash, (uint8)PHOTOTUBE_ARRAY_ALGORITHM_VERSION);
    hash = nav_hash_u8(hash, (uint8)PHOTOTUBE_ARRAY_COUNT);
    hash = nav_hash_u8(hash, (uint8)PHOTOTUBE_ARRAY_Y0_IS_LEFT);
    hash = nav_hash_u32(hash, (uint32)PHOTOTUBE_ARRAY_SCAN_HZ);
    hash = nav_hash_u16(hash, (uint16)PHOTOTUBE_ARRAY_LINE_THRESHOLD_ADC);
    hash = nav_hash_float(hash, PHOTOTUBE_ARRAY_SPACING_MM);
    hash = nav_hash_float(hash, PHOTOTUBE_ARRAY_FORWARD_MM);
    hash = nav_hash_u8(hash, (uint8)PHOTOTUBE_SCAN_POSE_MAX_AGE_MS);
    hash = nav_hash_float(hash, PHOTOTUBE_SCAN_MIN_STEP_MM);
    hash = nav_hash_u16(hash, PHOTOTUBE_SCAN_MAX_POINTS);
    hash = nav_hash_u8(hash, (uint8)VISION_OBSERVATION_QUEUE_CAPACITY);
    hash = nav_hash_u8(hash, (uint8)VISION_SCAN_HIT_QUEUE_CAPACITY);
    hash = nav_hash_u8(hash, (uint8)VISION_SCAN_HIT_MAX_COUNT);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_TRAVEL_WINDOW_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_STRAIGHT_TRAVEL_WINDOW_MM);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_MIN_POINT_COUNT);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_MIN_CORNER_SIDE_COUNT);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_MIN_SIDE_FRAME_COUNT);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_MIN_RUN_SIDE_FRAME_COUNT);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_MIN_SHORT_SIDE_CLEAN_POINTS);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_RUN_FRAME_MAX_GAP);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MIN_STRAIGHT_SPAN_MM);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_MIN_STRAIGHT_FRAME_COUNT);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_CONFIRM_TRAVEL_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_CONFIRM_POSITION_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_CONFIRM_YAW_DEG);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_MIN_TRANSLATION_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_MIN_YAW_DEG);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_SIGNIFICANCE_SIGMA);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MIN_FEATURE_SIDE_SPAN_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_RADIUS_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_TRANSLATION_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_SEARCH_YAW_DEG);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_COARSE_STEP_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_COARSE_STEP_DEG);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_TRANSLATION_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_YAW_DEG);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_DEG);
    hash = nav_hash_u16(hash,
                        NAV_SCAN_MATCH_FEATURE_SEARCH_POINT_LIMIT);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_IDENTITY_MAX_SCORE_MM2);
    hash = nav_hash_u16(hash,
                        NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_POINT_COUNT);
    hash = nav_hash_u16(hash,
                        NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_FRAME_COUNT);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_BOUNDARY_FALLBACK_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_SPATIAL_PROGRESS_LIMIT_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_MARGIN_POINTS);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_SEGMENT_MARGIN_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_MAX_STRAIGHT_POINT_RESIDUAL_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_ROBUST_MIN_GATE_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_ROBUST_MAX_GATE_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_ROBUST_MAD_SCALE);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_MIN_INLIER_RATIO);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_MM);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT);
    hash = nav_hash_u16(hash,
                        NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS);
    hash = nav_hash_u16(hash,
                        NAV_SCAN_MATCH_STRAIGHT_MODE_MIN_SEPARATION_BINS);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_MODE_SEED_GATE_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_AMBIGUITY_WEIGHT_RATIO);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_MAX_FEATURE_POINT_RESIDUAL_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MAX_RMS_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_PROGRESS_AMBIGUITY_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_ASSOCIATION_MARGIN_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_AMBIGUOUS_WEIGHT_RATIO);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_MIN_INLIER_WEIGHT_RATIO);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MAX_CORRECTION_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MAX_CORRECTION_YAW_DEG);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_COMPLETE_FEATURE_MAX_YAW_DEG);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MIN_SIDE_EFFECTIVE_WEIGHT);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_JUNCTION_MIN_SIDE_EFFECTIVE_WEIGHT);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_COUNT);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_EFFECTIVE_WEIGHT);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_SPAN_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MIN_RMS_IMPROVEMENT_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_CONFIRM_TRAVEL_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_CONFIRM_POSITION_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_CONFIRM_YAW_DEG);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_CONFIRM_OVERLAP_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MAX_PROGRESS_BACKTRACK_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_STRONG_CORRECTION_YAW_DEG);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_STRONG_CORRECTION_POSE_JUMP_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_STRONG_CONFIRM_TRAVEL_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_STRONG_CONFIRM_MAX_RMS_MM);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_STRONG_CONFIRM_MIN_INLIER_COUNT);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_PATH_CROSS_TRACK_TOLERANCE_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_COMPLETE_FEATURE_PATH_CROSS_TRACK_TOLERANCE_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_MAX_PATH_CROSS_TRACK_AFTER_MM);
    hash = nav_hash_u8(hash, NAV_SCAN_CORRECTION_TRANSACTION_VERSION);
    hash = nav_hash_u16(hash, NAV_SCAN_COMMIT_HINT_BEHIND_POINTS);
    hash = nav_hash_u16(hash, NAV_SCAN_COMMIT_HINT_SEARCH_POINTS);
    hash = nav_hash_float(hash,
                          PHOTOTUBE_ARRAY_BLACK_STOP_DISTANCE_MM);
    return hash;
}

static uint32 nav_calculate_route_hash(void)
{
    uint32 hash = 2166136261u;
    const signed short *data = nav_path_data[0];
    const signed short *xs = nav_path_x[0];
    const signed short *ys = nav_path_y[0];
    const uint16 *speed_end = nav_speed_end[0];
    const signed short *speed_cmps = nav_speed_cmps[0];
    const signed short *curvature_derivative = nav_path_curvature_derivative[0];
    const signed short *reference_acceleration = nav_acceleration_cmps2[0];
    uint16 point_count = nav_path_point_count[0];
    uint16 speed_count = nav_speed_count[0];
    uint16 index;

    hash = nav_hash_u16(hash, (uint16)NAV_SOURCE_ROUTE_INDEX);
    hash = nav_hash_u16(hash, point_count);
    for (index = 0u; index < point_count; index++)
    {
        hash = nav_hash_u16(hash, (uint16)data[index]);
        hash = nav_hash_u16(hash, (uint16)xs[index]);
        hash = nav_hash_u16(hash, (uint16)ys[index]);
    }
    // 曲率表参与进度预测和遥测，必须纳入路线指纹，防止
    // 只替换曲率数组而遥测/调试端仍误认为是同一份路线。
    hash = nav_hash_float(hash, NAV_REFERENCE_CURVATURE_LSB_PER_MM);
    for (index = 0u; index < point_count; index++)
    {
        hash = nav_hash_u16(hash, (uint16)nav_path_curvature[0][index]);
    }
    hash = nav_hash_float(hash, NAV_REFERENCE_CURVATURE_DERIVATIVE_LSB_PER_MM2);
    hash = nav_hash_float(hash, NAV_REFERENCE_SPEED_LSB_CMPS);
    hash = nav_hash_float(hash, NAV_REFERENCE_ACCELERATION_LSB_CMPS2);
    hash = nav_hash_float(hash, NAV_SPEED_PROFILE_MAX_SPEED_CMPS);
    hash = nav_hash_float(hash,
                          NAV_SPEED_PROFILE_MAX_LATERAL_ACCEL_CMPS2);
    hash = nav_hash_float(hash, NAV_SPEED_PROFILE_MAX_ACCEL_CMPS2);
    hash = nav_hash_float(hash, NAV_SPEED_PROFILE_MAX_DECEL_CMPS2);
    hash = nav_hash_float(hash, NAV_SPEED_PROFILE_SAFETY_MARGIN_MM);
    hash = nav_hash_float(hash, NAV_SPEED_PROFILE_END_CMPS);
    hash = nav_hash_u8(hash, (uint8)NAV_ROUTE_BLIND_BOX);
    for (index = 0u; index < point_count; index++)
    {
        hash = nav_hash_u16(hash, (uint16)curvature_derivative[index]);
    }
    hash = nav_hash_u16(hash, nav_acceleration_count[0]);
    for (index = 0u; index < nav_acceleration_count[0]; index++)
    {
        hash = nav_hash_u16(hash, nav_acceleration_end[0][index]);
        hash = nav_hash_u16(hash, (uint16)reference_acceleration[index]);
    }
    hash = nav_hash_u16(hash, speed_count);
    for (index = 0u; index < speed_count; index++)
    {
        hash = nav_hash_u16(hash, speed_end[index]);
        hash = nav_hash_u16(hash, (uint16)speed_cmps[index]);
    }

    hash = nav_hash_u8(hash, nav_map_takeover_count[0]);
    for (index = 0u; index < nav_map_takeover_count[0]; index++)
    {
        hash = nav_hash_u16(hash, nav_map_takeover_lo[0][index]);
        hash = nav_hash_u16(hash, nav_map_takeover_hi[0][index]);
    }

    hash = nav_hash_u16(hash, nav_scan_vertex_count[0]);
    for (index = 0u; index < nav_scan_vertex_count[0]; index++)
    {
        hash = nav_hash_u16(hash, (uint16)nav_scan_vertex_x[0][index]);
        hash = nav_hash_u16(hash, (uint16)nav_scan_vertex_y[0][index]);
        hash = nav_hash_u16(hash, nav_scan_vertex_progress[0][index]);
        hash = nav_hash_u8(hash, nav_scan_vertex_type[0][index]);
        hash = nav_hash_u8(hash,
                           nav_scan_vertex_corner_calibration_allowed[0][index]);
        hash = nav_hash_u16(hash, (uint16)nav_scan_vertex_support_x[0][index]);
        hash = nav_hash_u16(hash, (uint16)nav_scan_vertex_support_y[0][index]);
    }
    hash = nav_hash_u16(hash, nav_scan_segment_valid_count[0]);
    for (index = 0u; index < nav_scan_segment_valid_count[0]; index++)
    {
        hash = nav_hash_u8(hash, nav_scan_segment_valid[0][index]);
        hash = nav_hash_u8(hash,
                          nav_scan_segment_calibration_allowed[0][index]);
    }
    hash = nav_hash_u8(hash, NAV_SCAN_MATCH_ALGORITHM_VERSION);
    hash = nav_hash_u8(hash, (uint8)VISION_OBSERVATION_QUEUE_CAPACITY);
    hash = nav_hash_u8(hash, (uint8)VISION_SCAN_HIT_QUEUE_CAPACITY);
    hash = nav_hash_u8(hash, (uint8)VISION_SCAN_HIT_MAX_COUNT);
    hash = nav_hash_u8(hash, (uint8)PHOTOTUBE_SCAN_POSE_MAX_AGE_MS);
    hash = nav_hash_float(hash, PHOTOTUBE_SCAN_MIN_STEP_MM);
    hash = nav_hash_u16(hash, PHOTOTUBE_SCAN_MAX_POINTS);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_TRAVEL_WINDOW_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_STRAIGHT_TRAVEL_WINDOW_MM);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_MIN_CORNER_SIDE_COUNT);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_MIN_SIDE_FRAME_COUNT);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_MIN_RUN_SIDE_FRAME_COUNT);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_MIN_SHORT_SIDE_CLEAN_POINTS);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_RUN_FRAME_MAX_GAP);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MIN_STRAIGHT_SPAN_MM);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_MIN_STRAIGHT_FRAME_COUNT);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_CONFIRM_TRAVEL_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_CONFIRM_POSITION_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_CONFIRM_YAW_DEG);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_MIN_TRANSLATION_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_MIN_YAW_DEG);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_SIGNIFICANCE_SIGMA);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MIN_FEATURE_SIDE_SPAN_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MAX_RMS_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_MAX_STRAIGHT_POINT_RESIDUAL_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_ROBUST_MIN_GATE_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_ROBUST_MAX_GATE_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_ROBUST_MAD_SCALE);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_MIN_INLIER_RATIO);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_MM);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT);
    hash = nav_hash_u16(hash,
                        NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS);
    hash = nav_hash_u16(hash,
                        NAV_SCAN_MATCH_STRAIGHT_MODE_MIN_SEPARATION_BINS);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_MODE_SEED_GATE_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_STRAIGHT_AMBIGUITY_WEIGHT_RATIO);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_MAX_FEATURE_POINT_RESIDUAL_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_RADIUS_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_TRANSLATION_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_SEARCH_YAW_DEG);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_COARSE_STEP_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_COARSE_STEP_DEG);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_TRANSLATION_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_YAW_DEG);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_DEG);
    hash = nav_hash_u16(hash,
                        NAV_SCAN_MATCH_FEATURE_SEARCH_POINT_LIMIT);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_SEARCH_IDENTITY_MAX_SCORE_MM2);
    hash = nav_hash_u16(hash,
                        NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_POINT_COUNT);
    hash = nav_hash_u16(hash,
                        NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_FRAME_COUNT);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_PROGRESS_AMBIGUITY_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_ASSOCIATION_MARGIN_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_AMBIGUOUS_WEIGHT_RATIO);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_MIN_INLIER_WEIGHT_RATIO);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_FEATURE_BOUNDARY_FALLBACK_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_SPATIAL_PROGRESS_LIMIT_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_MARGIN_POINTS);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MIN_SIDE_EFFECTIVE_WEIGHT);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_JUNCTION_MIN_SIDE_EFFECTIVE_WEIGHT);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_COUNT);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_EFFECTIVE_WEIGHT);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_SPAN_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MAX_CORRECTION_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MAX_CORRECTION_YAW_DEG);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_COMPLETE_FEATURE_MAX_YAW_DEG);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MIN_RMS_IMPROVEMENT_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_CONFIRM_TRAVEL_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_CONFIRM_POSITION_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_FEATURE_CONFIRM_YAW_DEG);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_CONFIRM_OVERLAP_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_MAX_PROGRESS_BACKTRACK_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_STRONG_CORRECTION_YAW_DEG);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_STRONG_CORRECTION_POSE_JUMP_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_STRONG_CONFIRM_TRAVEL_MM);
    hash = nav_hash_float(hash, NAV_SCAN_MATCH_STRONG_CONFIRM_MAX_RMS_MM);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_STRONG_CONFIRM_MIN_INLIER_COUNT);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_PATH_CROSS_TRACK_TOLERANCE_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_COMPLETE_FEATURE_PATH_CROSS_TRACK_TOLERANCE_MM);
    hash = nav_hash_float(hash,
                          NAV_SCAN_MATCH_MAX_PATH_CROSS_TRACK_AFTER_MM);
    hash = nav_hash_u8(hash, NAV_SCAN_CORRECTION_TRANSACTION_VERSION);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_PROGRESS_BEHIND_POINTS);
    hash = nav_hash_u16(hash, NAV_SCAN_MATCH_PROGRESS_SEARCH_POINTS);
    hash = nav_hash_u16(hash, NAV_SCAN_COMMIT_HINT_BEHIND_POINTS);
    hash = nav_hash_u16(hash, NAV_SCAN_COMMIT_HINT_SEARCH_POINTS);

    hash = nav_hash_u32(hash, nav_calculate_localization_config_hash());
    hash = nav_hash_float(hash, DRIVE_WHEEL_TRACK_MM);
    hash = nav_hash_float(hash, NAV_PP_PREVIEW_TIME_S);
    hash = nav_hash_float(hash, NAV_PP_MIN_LOOKAHEAD_MM);
    hash = nav_hash_float(hash, NAV_PP_MAX_LOOKAHEAD_MM);
    hash = nav_hash_float(hash, NAV_PP_CURVE_MIN_LOOKAHEAD_MM);
#if NAV_TRACKING_REFERENCE_ALGORITHM_VERSION >= 17u
    hash = nav_hash_float(hash, NAV_TRACK_CURVATURE_PREVIEW_TIME_S);
    hash = nav_hash_float(hash,
                          NAV_TRACK_HEADING_CORRECTION_RADPS_PER_RAD);
    hash = nav_hash_float(hash,
                          NAV_TRACK_CROSS_TRACK_CORRECTION_MMPS_PER_MM);
    hash = nav_hash_float(hash, NAV_TRACK_MIN_CORRECTION_SPEED_MMPS);
#endif
    hash = nav_hash_float(hash, NAV_TRACK_KD_CMPS_PER_DEGPS_PER_SPEED_SCALE);
    hash = nav_hash_float(
        hash,
        (NAV_TRACKING_REFERENCE_ALGORITHM_VERSION >= 16u) ?
            NAV_TRACK_YAW_RATE_FB_CMPS_PER_RADPS :
            NAV_TRACK_GKD_CMPS_PER_RADPS);
    hash = nav_hash_float(hash, NAV_TURN_LIMIT_RATIO);
    hash = nav_hash_u16(hash, NAV_TRACK_TARGET_RATE_WINDOW_MS);
    hash = nav_hash_u16(hash, NAV_TRACK_TARGET_RATE_WINDOW_SAMPLES);
    hash = nav_hash_float(hash, SLIP_GUARD_MIN_SPEED_CMPS);
    hash = nav_hash_float(hash, SLIP_GUARD_MIN_YAW_RATE_DEGPS);
    hash = nav_hash_float(hash, SLIP_GUARD_RATE_FILTER_TAU_S);
    hash = nav_hash_float(hash, SLIP_GUARD_MISMATCH_DEADBAND_DEGPS);
    hash = nav_hash_float(hash, SLIP_GUARD_MISMATCH_RELEASE_DEGPS);
    hash = nav_hash_float(hash, SLIP_GUARD_GAIN_CMPS_PER_RADPS);
    hash = nav_hash_float(hash, SLIP_GUARD_MAX_CORRECTION_CMPS);
    hash = nav_hash_u8(hash, NAV_TRACKING_REFERENCE_ALGORITHM_VERSION);
    return hash;
}

static uint8 nav_scan_vertex_progress_is_valid(void)
{
    uint16 index;

    if ((nav.scan_vertex_count < 2u) || (nav.point_count < 2u) ||
        (NULL == nav.scan_vertex_progress))
    {
        return 0u;
    }
    for (index = 0u; index < nav.scan_vertex_count; index++)
    {
        uint16 progress = nav.scan_vertex_progress[index];

        if (progress >= nav.point_count)
        {
            return 0u;
        }
        if ((index > 0u) &&
            (progress <= nav.scan_vertex_progress[index - 1u]))
        {
            return 0u;
        }
    }
    return 1u;
}

static uint8 nav_scan_segment_valid_is_valid(void)
{
    uint16 index;

    if ((nav.scan_vertex_count < 2u) ||
        (nav.scan_segment_count != (uint16)(nav.scan_vertex_count - 1u)) ||
        (NULL == nav.scan_segment_valid))
    {
        return 0u;
    }
    for (index = 0u; index < nav.scan_segment_count; index++)
    {
        if (nav.scan_segment_valid[index] > 1u)
        {
            return 0u;
        }
    }
    return 1u;
}

static uint8 nav_scan_segment_calibration_mask_is_valid(void)
{
    uint16 index;

    if ((nav.scan_vertex_count < 2u) ||
        (nav.scan_segment_count != (uint16)(nav.scan_vertex_count - 1u)) ||
        (NULL == nav.scan_segment_calibration_allowed))
    {
        return 0u;
    }
    for (index = 0u; index < nav.scan_segment_count; index++)
    {
        if (nav.scan_segment_calibration_allowed[index] > 1u)
        {
            return 0u;
        }
    }
    return 1u;
}

// 装载地图生成器导出的唯一一条路线。
static void nav_load_path(void)
{
    nav.data = nav_path_data[0];
    nav.xs = nav_path_x[0];
    nav.ys = nav_path_y[0];
    nav.point_count = nav_path_point_count[0];

    nav.curvature = nav_path_curvature[0];
    nav.curvature_lsb_per_mm = NAV_REFERENCE_CURVATURE_LSB_PER_MM;
    nav.curvature_derivative = nav_path_curvature_derivative[0];
    nav.curvature_derivative_lsb_per_mm2 = NAV_REFERENCE_CURVATURE_DERIVATIVE_LSB_PER_MM2;
    nav.scan_vertex_x = nav_scan_vertex_x[0];
    nav.scan_vertex_y = nav_scan_vertex_y[0];
    nav.scan_vertex_progress = nav_scan_vertex_progress[0];
    nav.scan_vertex_type = nav_scan_vertex_type[0];
    nav.scan_vertex_corner_calibration_allowed =
        nav_scan_vertex_corner_calibration_allowed[0];
    nav.scan_vertex_support_x = nav_scan_vertex_support_x[0];
    nav.scan_vertex_support_y = nav_scan_vertex_support_y[0];
    nav.scan_vertex_count = nav_scan_vertex_count[0];
    nav.scan_segment_valid = nav_scan_segment_valid[0];
    nav.scan_segment_calibration_allowed =
        nav_scan_segment_calibration_allowed[0];
    nav.scan_segment_count = nav_scan_segment_valid_count[0];
    nav_scan_vertex_progress_valid = nav_scan_vertex_progress_is_valid();
    nav_scan_vertex_progress_valid = (uint8)(
        nav_scan_vertex_progress_valid && nav_scan_segment_valid_is_valid() &&
        nav_scan_segment_calibration_mask_is_valid());

    nav.speed_end_p = nav_speed_end[0];
    nav.speed_cmps_p = nav_speed_cmps[0];
    nav.acceleration_end_p = nav_acceleration_end[0];
    nav.acceleration_cmps2_p = nav_acceleration_cmps2[0];
    nav.speed_seg_cnt = nav_speed_count[0];
    nav.speed_seg_idx = 0u;
    nav.acceleration_seg_cnt = nav_acceleration_count[0];
    nav.acceleration_seg_idx = 0u;
    nav.map_takeover_cnt = nav_map_takeover_count[0];
    nav.map_takeover_lo_p = nav_map_takeover_lo[0];
    nav.map_takeover_hi_p = nav_map_takeover_hi[0];
    nav.route_hash = nav_calculate_route_hash();
}

static void nav_map_from_odom_reset(void)
{
    nav.map_from_odom.tx_mm = 0.0f;
    nav.map_from_odom.ty_mm = 0.0f;
    nav.map_from_odom.yaw_rad = 0.0f;
    nav.map_pose = nav.raw_odometry;
    nav.localization_update_seq = 0u;
}

static void nav_odom_point_to_map_with(const volatile NavMapFromOdom *transform,
                                        float odom_x_mm,
                                        float odom_y_mm,
                                        float *map_x_mm,
                                        float *map_y_mm)
{
    float cos_yaw = cosf(transform->yaw_rad);
    float sin_yaw = sinf(transform->yaw_rad);

    *map_x_mm = transform->tx_mm +
                odom_x_mm * cos_yaw - odom_y_mm * sin_yaw;
    *map_y_mm = transform->ty_mm +
                odom_x_mm * sin_yaw + odom_y_mm * cos_yaw;
}

static void nav_scan_prepare_map_to_odom_cache(float *progress_points,
                                               float *travel_distance_mm)
{
    NavMapFromOdom transform;
    uint32 interrupt_state = interrupt_global_disable();

    transform = nav.map_from_odom;
    if (NULL != progress_points)
    {
        *progress_points = nav.progress_points;
    }
    if (NULL != travel_distance_mm)
    {
        /* total_dist is advanced by the encoder consumer in the control
         * interrupt.  Capture it under the same mask as progress/T so the
         * matcher cannot associate a pre-correction route cursor with a
         * post-correction travel watermark. */
        *travel_distance_mm = total_dist * 1000.0f;
    }
    interrupt_global_enable(interrupt_state);
    nav_scan_map_to_odom_cache.transform = transform;
    nav_scan_map_to_odom_cache.cos_yaw = cosf(transform.yaw_rad);
    nav_scan_map_to_odom_cache.sin_yaw = sinf(transform.yaw_rad);
}

static void nav_map_point_to_odom(float map_x_mm,
                                   float map_y_mm,
                                   float *odom_x_mm,
                                   float *odom_y_mm)
{
    float dx = map_x_mm - nav_scan_map_to_odom_cache.transform.tx_mm;
    float dy = map_y_mm - nav_scan_map_to_odom_cache.transform.ty_mm;
    float cos_yaw = nav_scan_map_to_odom_cache.cos_yaw;
    float sin_yaw = nav_scan_map_to_odom_cache.sin_yaw;

    *odom_x_mm = dx * cos_yaw + dy * sin_yaw;
    *odom_y_mm = -dx * sin_yaw + dy * cos_yaw;
}

static void nav_raw_pose_to_map_with(const volatile NavMapFromOdom *transform,
                                      const volatile DriveOdometryPose *raw_pose,
                                      DriveOdometryPose *map_pose)
{
    *map_pose = *raw_pose;
    nav_odom_point_to_map_with(transform, raw_pose->x_mm, raw_pose->y_mm,
                               &map_pose->x_mm, &map_pose->y_mm);
    map_pose->yaw_rad = nav_wrap_rad(raw_pose->yaw_rad + transform->yaw_rad);
    if (map_pose->gyro_heading_initialized)
    {
        map_pose->gyro_heading_rad = nav_wrap_rad(
            raw_pose->gyro_heading_rad + transform->yaw_rad);
    }
}

static void nav_compose_map_from_odom(const NavMapFromOdom *current,
                                      float anchor_x_mm,
                                      float anchor_y_mm,
                                      float shift_x_mm,
                                      float shift_y_mm,
                                      float yaw_correction_rad,
                                      NavMapFromOdom *updated)
{
    float delta_cos = cosf(yaw_correction_rad);
    float delta_sin = sinf(yaw_correction_rad);
    float delta_tx = anchor_x_mm + shift_x_mm -
                     (delta_cos * anchor_x_mm - delta_sin * anchor_y_mm);
    float delta_ty = anchor_y_mm + shift_y_mm -
                     (delta_sin * anchor_x_mm + delta_cos * anchor_y_mm);
    float current_cos = cosf(current->yaw_rad);
    float current_sin = sinf(current->yaw_rad);

    updated->tx_mm = current->tx_mm +
                     current_cos * delta_tx - current_sin * delta_ty;
    updated->ty_mm = current->ty_mm +
                     current_sin * delta_tx + current_cos * delta_ty;
    updated->yaw_rad = nav_wrap_rad(current->yaw_rad + yaw_correction_rad);
}

// 上电初始化:清状态、装载唯一导出路线、复位线路诊断会话。
void nav_init(void)
{
    nav.state = NAV_STATE_IDLE;
    nav_control_yaw_rate_radps = 0.0f;
    nav_control_imu_valid = 0u;
    nav_reset_imu_snapshot_history();
    nav_yaw_rate_tracker_reset();
    nav.replay_index = 0u;
    nav.progress_points = 0.0f;
    nav.correction = 0.0f;
    nav.raw_pose_generation = 0u;
    drive_odometry_reset(&nav.raw_odometry);
    nav_map_from_odom_reset();
    nav_scan_match_reset();
    nav_load_path();
    nav_line_session_reset();
}

uint8 nav_ready_for_start(void)
{
    return (uint8)((nav.point_count > 0u) &&
                   (NULL != nav.data) &&
                   (NULL != nav.xs) &&
                   (NULL != nav.ys) &&
                   (NULL != nav.scan_vertex_x) &&
                   (NULL != nav.scan_vertex_y) &&
                   (NULL != nav.scan_vertex_type) &&
                   (NULL != nav.scan_vertex_support_x) &&
                   (NULL != nav.scan_vertex_support_y) &&
                   (NULL != nav.scan_segment_valid) &&
                   (NULL != nav.scan_segment_calibration_allowed) &&
                   nav_scan_vertex_progress_valid);
}

// 发车:清空位姿与进度并进入导航态;无有效路径进 ERROR。
void nav_start(void)
{
    NavSystemState previous_state = nav.state;

    if (!nav_ready_for_start())
    {
        nav.state = NAV_STATE_ERROR;
        nav.correction = 0.0f;
        nav_yaw_rate_tracker_reset();
        nav_capture_state_transition(previous_state, nav.state);
        return;
    }

    nav.replay_index = 0u;
    nav_control_yaw_rate_radps = 0.0f;
    nav_control_imu_valid = 0u;
    nav_reset_imu_snapshot_history();
    nav.progress_points = 0.0f;
    nav.raw_pose_generation = 0u;
    drive_odometry_reset(&nav.raw_odometry);
    nav_map_from_odom_reset();
    nav_signed_cross_track_mm = 0.0f;
    nav.correction = 0.0f;
    nav_telemetry_last_ms = vision_clock_ms;
    nav.speed_seg_idx = 0u;
    nav.kappa_per_mm = 0.0f;
    nav.reference_curvature_per_mm = 0.0f;
    nav.ey_mm = 0.0f;
    nav.epsi_rad = 0.0f;
    nav_scan_match_reset();
    nav_yaw_rate_tracker_reset();
    nav_line_session_reset();
    // 航向基准:prestart 已锁存 VQF yaw 发车零点，发车朝向即 0 度。
    nav.state = NAV_STATE_NAVIGATING;
    nav_capture_state_transition(previous_state, nav.state);
}

// 停止回放:回空闲、清修正输出。可被任何停车路径重复调用。
void nav_stop(void)
{
    nav.state = NAV_STATE_IDLE;
    nav_control_yaw_rate_radps = 0.0f;
    nav_control_imu_valid = 0u;
    nav.correction = 0.0f;
    nav.raw_pose_generation = 0u;
    drive_odometry_reset(&nav.raw_odometry);
    nav_map_from_odom_reset();
    nav_scan_match_reset();
    nav_yaw_rate_tracker_reset();
    nav_line_session_reset();
}

// 把角度规整到 (-180, 180],便于比较角差不受 ±360 缠绕影响。
// (路径投影、点云匹配和遥测共用,故放在条件编译之外。)
static float nav_wrap_deg(float angle)
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

static float nav_wrap_rad(float angle)
{
    const float pi = 3.141592654f;
    const float two_pi = 6.283185307f;

    while (angle > pi)
    {
        angle -= two_pi;
    }
    while (angle <= -pi)
    {
        angle += two_pi;
    }
    return angle;
}

static float nav_scan_match_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static uint8 nav_scan_match_solve_3x3(const float m[3][3],
                                       const float b[3],
                                       float x[3])
{
    float determinant;

    if ((NULL == m) || (NULL == b) || (NULL == x))
    {
        return 0u;
    }
    determinant =
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
        m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
        m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if (fabsf(determinant) < 1.0e-3f)
    {
        return 0u;
    }

    x[0] =
        (b[0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
         m[0][1] * (b[1] * m[2][2] - m[1][2] * b[2]) +
         m[0][2] * (b[1] * m[2][1] - m[1][1] * b[2])) / determinant;
    x[1] =
        (m[0][0] * (b[1] * m[2][2] - m[1][2] * b[2]) -
         b[0] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
         m[0][2] * (m[1][0] * b[2] - b[1] * m[2][0])) / determinant;
    x[2] =
        (m[0][0] * (m[1][1] * b[2] - b[1] * m[2][1]) -
         m[0][1] * (m[1][0] * b[2] - b[1] * m[2][0]) +
         b[0] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / determinant;
    return 1u;
}

static uint8 nav_scan_make_line(uint16 segment, NavScanLine *line)
{
    float map_ax;
    float map_ay;
    float map_bx;
    float map_by;
    float vx;
    float vy;
    float length;

    if ((NULL == line) || (segment >= nav.scan_segment_count) ||
        (NULL == nav.scan_segment_valid) ||
        (0u == nav.scan_segment_valid[segment]) ||
        (segment + 1u >= nav.scan_vertex_count) ||
        (NULL == nav.scan_vertex_x) || (NULL == nav.scan_vertex_y))
    {
        return 0u;
    }
    map_ax = (float)nav.scan_vertex_x[segment];
    map_ay = (float)nav.scan_vertex_y[segment];
    map_bx = (float)nav.scan_vertex_x[segment + 1u];
    map_by = (float)nav.scan_vertex_y[segment + 1u];
    nav_map_point_to_odom(map_ax, map_ay, &line->ax, &line->ay);
    nav_map_point_to_odom(map_bx, map_by, &line->bx, &line->by);
    vx = line->bx - line->ax;
    vy = line->by - line->ay;
    length = sqrtf(vx * vx + vy * vy);
    if (length < 1.0e-3f)
    {
        return 0u;
    }
    line->length = length;
    line->tx = vx / length;
    line->ty = vy / length;
    line->nx = -line->ty;
    line->ny = line->tx;
    return 1u;
}

static uint8 nav_scan_project_line(const NavScanLine *line,
                                    float position_x_mm,
                                    float position_y_mm,
                                    float margin_mm,
                                    float *project_x_mm,
                                    float *project_y_mm,
                                    float *normal_residual_mm,
                                    float *tangent_position_mm)
{
    float tangent;
    float clamped_tangent;
    float qx;
    float qy;

    if ((NULL == line) || (NULL == project_x_mm) ||
        (NULL == project_y_mm) || (NULL == normal_residual_mm))
    {
        return 0u;
    }
    tangent = (position_x_mm - line->ax) * line->tx +
              (position_y_mm - line->ay) * line->ty;
    if ((tangent < -margin_mm) || (tangent > line->length + margin_mm))
    {
        return 0u;
    }
    clamped_tangent = nav_scan_match_clamp(tangent, 0.0f, line->length);
    qx = line->ax + clamped_tangent * line->tx;
    qy = line->ay + clamped_tangent * line->ty;
    *project_x_mm = qx;
    *project_y_mm = qy;
    *normal_residual_mm =
        (position_x_mm - qx) * line->nx +
        (position_y_mm - qy) * line->ny;
    if (NULL != tangent_position_mm)
    {
        *tangent_position_mm = tangent;
    }
    return 1u;
}

/* Each point carries capture-time wheel travel.  The resulting progress span
 * follows the distance actually traveled through corners rather than the
 * Euclidean chord between points.  Rigid map corrections do not change it. */
static uint8 nav_scan_measure_progress_window(float sensor_progress_points,
                                               uint16 point_count,
                                               float *progress_low_points,
                                               float *progress_high_points)
{
    float span_mm;
    float first_travel_mm;
    float last_travel_mm;

    if ((point_count < 2u) || (NULL == progress_low_points) ||
        (NULL == progress_high_points))
    {
        return 0u;
    }
    if (!nav_scan_match_point_travel_distance_mm(0u, &first_travel_mm) ||
        !nav_scan_match_point_travel_distance_mm(
            (uint16)(point_count - 1u), &last_travel_mm))
    {
        return 0u;
    }
    span_mm = last_travel_mm - first_travel_mm;
    if (span_mm < 0.0f)
    {
        return 0u;
    }
    *progress_low_points = sensor_progress_points -
                           span_mm / NAV_PATH_SAMPLE_MM -
                           NAV_SCAN_MATCH_FEATURE_MARGIN_POINTS;
    if (*progress_low_points < 0.0f)
    {
        *progress_low_points = 0.0f;
    }
    *progress_high_points = sensor_progress_points +
                            NAV_SCAN_MATCH_FEATURE_MARGIN_POINTS;
    return 1u;
}

/* CORNER/JUNCTION 候选先经原始物理顶点为圆心的空间门。
 * 车辆内切时,采集到的两条物理边可能落在拐点前后不同的进度区间。
 * 物理顶点先由当前T_map_from_odom逆变换到原始odom系，再与点云比较。 */
static float nav_scan_feature_nearest_distance_sq_mm2(uint16 vertex_index,
                                                       uint16 point_count,
                                                       uint16 *evidence_count,
                                                       uint16 *evidence_frame_count)
{
    uint16 index;
    float vertex_x;
    float vertex_y;
    float nearest_squared = 1.0e30f;
    uint16 count = 0u;
    uint16 frame_count = 0u;
    uint32 last_frame_seq = 0u;
    uint8 have_frame = 0u;

    if (NULL != evidence_count)
    {
        *evidence_count = 0u;
    }
    if (NULL != evidence_frame_count)
    {
        *evidence_frame_count = 0u;
    }

    if ((vertex_index >= nav.scan_vertex_count) ||
        (NULL == nav.scan_vertex_x) || (NULL == nav.scan_vertex_y) ||
        (NULL == nav.scan_vertex_type) ||
        (NULL == nav_scan_match_point_at(0u)))
    {
        return 1.0e30f;
    }
    nav_map_point_to_odom((float)nav.scan_vertex_x[vertex_index],
                          (float)nav.scan_vertex_y[vertex_index],
                          &vertex_x, &vertex_y);
    for (index = 0u; index < point_count; index++)
    {
        const PhototubeScanPoint *point = nav_scan_match_point_at(index);
        float dx;
        float dy;
        float squared;

        if (((NAV_SCAN_VERTEX_TYPE_JUNCTION ==
              nav.scan_vertex_type[vertex_index]) ?
             !nav_scan_point_is_junction_evidence(point) :
             !nav_scan_point_is_primary_centerline(point)) ||
            (nav_scan_point_weight(point) <= 0.0f))
        {
            continue;
        }
        dx = point->x_mm - vertex_x;
        dy = point->y_mm - vertex_y;
        squared = dx * dx + dy * dy;
        if (squared > NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM *
                     NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM)
        {
            continue;
        }
        count++;
        if (!have_frame || (point->frame_seq != last_frame_seq))
        {
            frame_count++;
            last_frame_seq = point->frame_seq;
            have_frame = 1u;
        }
        if (squared < nearest_squared)
        {
            nearest_squared = squared;
        }
    }
    if (NULL != evidence_count)
    {
        *evidence_count = count;
    }
    if (NULL != evidence_frame_count)
    {
        *evidence_frame_count = frame_count;
    }
    return nearest_squared;
}

static uint16 nav_scan_feature_progress_lower_bound(float progress_points)
{
    uint16 first = 0u;
    uint16 last = nav.scan_vertex_count;

    while (first < last)
    {
        uint16 middle = (uint16)(first + (last - first) / 2u);

        if ((float)nav.scan_vertex_progress[middle] < progress_points)
        {
            first = (uint16)(middle + 1u);
        }
        else
        {
            last = middle;
        }
    }
    return first;
}

static uint16 nav_scan_feature_progress_upper_bound(float progress_points)
{
    uint16 first = 0u;
    uint16 last = nav.scan_vertex_count;

    while (first < last)
    {
        uint16 middle = (uint16)(first + (last - first) / 2u);

        if ((float)nav.scan_vertex_progress[middle] <= progress_points)
        {
            first = (uint16)(middle + 1u);
        }
        else
        {
            last = middle;
        }
    }
    return first;
}

static uint8 nav_scan_find_window_feature(float sensor_progress_points,
                                          float progress_low_points,
                                          float progress_high_points,
                                          uint16 point_count,
                                          uint16 *vertex_index,
                                          uint8 *vertex_type,
                                          uint8 *ambiguous)
{
    uint16 index;
    uint16 first_candidate;
    uint16 candidate_end;
    uint16 nearest_progress_index = 0u;
    uint16 nearest_spatial_corner_index = 0u;
    uint8 nearest_progress_type = NAV_SCAN_VERTEX_TYPE_STRAIGHT;
    uint8 nearest_progress_found = 0u;
    uint8 second_progress_found = 0u;
    uint8 nearest_spatial_corner_found = 0u;
    uint8 spatial_corner_count = 0u;
    float nearest_progress_delta = 1.0e30f;
    float second_progress_delta = 1.0e30f;
    float nearest_progress_distance_sq = 1.0e30f;
    float nearest_spatial_corner_distance_sq = 1.0e30f;
    float spatial_limit_points =
        NAV_SCAN_MATCH_SPATIAL_PROGRESS_LIMIT_MM / NAV_PATH_SAMPLE_MM;
    float candidate_progress_low = sensor_progress_points -
                                   spatial_limit_points;
    float candidate_progress_high = sensor_progress_points +
                                    spatial_limit_points;

    if ((NULL == vertex_index) || (NULL == vertex_type) ||
        (NULL == ambiguous) || (NULL == nav.scan_vertex_progress) ||
        (NULL == nav.scan_vertex_type))
    {
        return 0u;
    }
    *ambiguous = 0u;
    if (progress_low_points < candidate_progress_low)
    {
        candidate_progress_low = progress_low_points;
    }
    if (progress_high_points > candidate_progress_high)
    {
        candidate_progress_high = progress_high_points;
    }
    first_candidate = nav_scan_feature_progress_lower_bound(
        candidate_progress_low);
    candidate_end = nav_scan_feature_progress_upper_bound(
        candidate_progress_high);
    for (index = first_candidate; index < candidate_end; index++)
    {
        uint8 type = nav.scan_vertex_type[index];
        float feature_progress = (float)nav.scan_vertex_progress[index];
        float feature_progress_distance_mm;
        float feature_distance_sq;
        uint16 feature_evidence_count;
        uint16 feature_evidence_frame_count;
        uint8 in_progress_window;

        if ((type != NAV_SCAN_VERTEX_TYPE_CORNER) &&
            (type != NAV_SCAN_VERTEX_TYPE_JUNCTION))
        {
            continue;
        }
        if (nav_scan_feature_is_localized(index))
        {
            continue;
        }
        if (nav_scan_feature_is_before_watermark(index))
        {
            continue;
        }

        feature_progress_distance_mm =
            fabsf(feature_progress - sensor_progress_points) *
            NAV_PATH_SAMPLE_MM;
        in_progress_window = (uint8)(
            (feature_progress >= progress_low_points) &&
            (feature_progress <= progress_high_points));

        /* Route progress is the cheap semantic gate.  Only corners may use
         * the spatial supplement outside the measured progress window, and
         * even those must remain inside the rolling-cloud progress range.
         * This avoids scanning every point for every feature occurrence. */
        if (!in_progress_window &&
            ((type != NAV_SCAN_VERTEX_TYPE_CORNER) ||
             (feature_progress_distance_mm >
              NAV_SCAN_MATCH_SPATIAL_PROGRESS_LIMIT_MM)))
        {
            continue;
        }

        feature_distance_sq = nav_scan_feature_nearest_distance_sq_mm2(
            index, point_count, &feature_evidence_count,
            &feature_evidence_frame_count);
        if ((feature_distance_sq >
             NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM *
                 NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM) ||
            (feature_evidence_count <
             NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_POINT_COUNT) ||
            (feature_evidence_frame_count <
             NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_FRAME_COUNT))
        {
            continue;
        }

        if (in_progress_window)
        {
            float progress_delta = fabsf(feature_progress -
                                         sensor_progress_points);
            if ((!nearest_progress_found) ||
                 (progress_delta < nearest_progress_delta) ||
                 ((fabsf(progress_delta - nearest_progress_delta) < 0.5f) &&
                  (feature_distance_sq < nearest_progress_distance_sq)))
            {
                if (nearest_progress_found)
                {
                    second_progress_found = 1u;
                    second_progress_delta = nearest_progress_delta;
                }
                nearest_progress_found = 1u;
                nearest_progress_index = index;
                nearest_progress_type = type;
                nearest_progress_delta = progress_delta;
                nearest_progress_distance_sq = feature_distance_sq;
            }
            else if (!second_progress_found ||
                     (progress_delta < second_progress_delta))
            {
                second_progress_found = 1u;
                second_progress_delta = progress_delta;
            }
        }
        else
        {
            /* 车辆内切时两条边可能没有同时落入进度窗。只有纯直角
             * 允许使用空间候选补充；该角点仍必须属于当前1m滚动证据
             * 附近的路线进度，路口则始终要求当前路线进度窗一致。 */
            spatial_corner_count++;
            if ((!nearest_spatial_corner_found) ||
                (feature_distance_sq < nearest_spatial_corner_distance_sq))
            {
                nearest_spatial_corner_found = 1u;
                nearest_spatial_corner_index = index;
                nearest_spatial_corner_distance_sq = feature_distance_sq;
            }
        }
    }

    /* 进度窗是路线语义的主消歧条件。同一扫图窗可能覆盖多个相邻
     * 路口，但它们不再因“数量大于1”被整体拒绝，而是选当前进度最近的
     * 路线顶点，再由双线几何、RMS 和修正幅度门决定是否写回。 */
    if (nearest_progress_found)
    {
        *vertex_index = nearest_progress_index;
        *vertex_type = nearest_progress_type;
        *ambiguous = (uint8)(second_progress_found &&
            ((second_progress_delta - nearest_progress_delta) *
             NAV_PATH_SAMPLE_MM <= NAV_SCAN_MATCH_FEATURE_PROGRESS_AMBIGUITY_MM));
        return 1u;
    }

    if (nearest_spatial_corner_found)
    {
        *vertex_index = nearest_spatial_corner_index;
        *vertex_type = NAV_SCAN_VERTEX_TYPE_CORNER;
        *ambiguous = (uint8)(spatial_corner_count > 1u);
        return 1u;
    }
    return 0u;
}

static uint8 nav_scan_find_seed_segment(float progress_points,
                                        uint16 *segment_index)
{
    uint16 lower_bound;

    if ((NULL == segment_index) || (NULL == nav.scan_vertex_progress) ||
        (NULL == nav.scan_vertex_type) || (nav.scan_vertex_count < 2u))
    {
        return 0u;
    }

    lower_bound = nav_scan_feature_progress_lower_bound(progress_points);
    if (0u == lower_bound)
    {
        *segment_index = 0u;
    }
    else if (lower_bound >= nav.scan_vertex_count)
    {
        *segment_index = (uint16)(nav.scan_vertex_count - 2u);
    }
    else
    {
        /* An exact vertex hit selects the preceding segment, matching the
         * former first-minimum linear scan. */
        *segment_index = (uint16)(lower_bound - 1u);
    }
    return 1u;
}

/* Merge adjacent collinear raw segments so a point window can cover one
 * physical straight across editor vertices and straight-through junctions.
 * A real corner still stops the group through the tangent dot-product gate. */
static uint8 nav_scan_make_straight_line(uint16 seed_segment,
                                         NavScanLine *line,
                                         uint16 *group_first_segment)
{
    uint16 first = seed_segment;
    uint16 last = seed_segment;
    NavScanLine seed;
    float cos_tolerance = 0.996f;

    if ((seed_segment >= nav.scan_segment_count) ||
        (NULL == nav.scan_segment_calibration_allowed) ||
        (0u == nav.scan_segment_calibration_allowed[seed_segment]) ||
        !nav_scan_make_line(seed_segment, &seed))
    {
        return 0u;
    }
    while (first > 0u)
    {
        NavScanLine previous;
        float dot;
        if ((NULL == nav.scan_segment_calibration_allowed) ||
            (0u == nav.scan_segment_calibration_allowed[first - 1u]) ||
            !nav_scan_make_line(first - 1u, &previous))
        {
            break;
        }
        dot = previous.tx * seed.tx + previous.ty * seed.ty;
        if (dot < cos_tolerance)
        {
            break;
        }
        first--;
    }
    while (last + 1u < (uint16)(nav.scan_vertex_count - 1u))
    {
        NavScanLine next;
        float dot;
        if ((NULL == nav.scan_segment_calibration_allowed) ||
            (0u == nav.scan_segment_calibration_allowed[last + 1u]) ||
            !nav_scan_make_line(last + 1u, &next))
        {
            break;
        }
        dot = next.tx * seed.tx + next.ty * seed.ty;
        if (dot < cos_tolerance)
        {
            break;
        }
        last++;
    }
    nav_map_point_to_odom((float)nav.scan_vertex_x[first],
                          (float)nav.scan_vertex_y[first],
                          &line->ax, &line->ay);
    nav_map_point_to_odom((float)nav.scan_vertex_x[last + 1u],
                          (float)nav.scan_vertex_y[last + 1u],
                          &line->bx, &line->by);
    line->tx = seed.tx;
    line->ty = seed.ty;
    line->nx = seed.nx;
    line->ny = seed.ny;
    line->length = (line->bx - line->ax) * line->tx +
                   (line->by - line->ay) * line->ty;
    if (NULL != group_first_segment)
    {
        *group_first_segment = first;
    }
    return (uint8)(line->length > 1.0f);
}

static uint8 nav_scan_match_straight(float progress_points,
                                      uint16 point_count,
                                      uint16 *segment_index,
                                      uint16 *line_group_index,
                                      float *anchor_x_mm,
                                      float *anchor_y_mm,
                                      float *shift_x_mm,
                                      float *shift_y_mm,
                                      float *yaw_correction_rad,
                                      float *rms_before_mm,
                                      float *rms_after_mm,
                                      uint16 *inlier_count,
                                      uint16 *frame_count,
                                      float *span_mm,
                                      float *tangent_information_mm2,
                                      uint8 *reject_reason)
{
    NavScanLine line;
    uint16 seed_segment;
    uint16 group_first_segment;
    uint16 index;
    uint16 used = 0u;
    uint16 used_frames = 0u;
    uint16 candidate_count = 0u;
    uint16 mode_candidate_count = 0u;
    uint32 last_used_frame_seq = 0u;
    uint8 have_used_frame = 0u;
    float candidate_weight = 0.0f;
    float candidate_residual_square = 0.0f;
    float used_weight = 0.0f;
    float tangent_sum = 0.0f;
    float residual_sum = 0.0f;
    float tangent_square_sum = 0.0f;
    float tangent_residual_sum = 0.0f;
    float residual_square_sum = 0.0f;
    float tangent_min = 1.0e30f;
    float tangent_max = -1.0e30f;
    float mean_tangent;
    float mean_residual;
    float tangent_variance;
    float residual_variance;
    float tangent_residual_covariance;
    float residual_slope;
    float correction_sin;
    float correction_cos;
    float corrected_mean_residual;
    float corrected_residual_variance;
    float residual_median;
    float residual_mad;
    float residual_mode_center;
    float robust_gate;
    float span;
    uint8 mode_selection;

    if (NULL != reject_reason)
    {
        *reject_reason = NAV_SCAN_MATCH_REJECT_GEOMETRY;
    }
    if (NULL != tangent_information_mm2)
    {
        *tangent_information_mm2 = 0.0f;
    }

    if (!nav_scan_find_seed_segment(progress_points, &seed_segment))
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_STRAIGHT_NO_MAP_LINE;
        }
        return 0u;
    }
    if ((NULL == nav.scan_segment_calibration_allowed) ||
        (seed_segment >= nav.scan_segment_count) ||
        (0u == nav.scan_segment_calibration_allowed[seed_segment]))
    {
        if (NULL != reject_reason)
        {
            *reject_reason =
                NAV_SCAN_MATCH_REJECT_STRAIGHT_CALIBRATION_BLOCKED;
        }
        return 0u;
    }
    if (!nav_scan_make_straight_line(seed_segment, &line,
                                     &group_first_segment))
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_STRAIGHT_NO_MAP_LINE;
        }
        return 0u;
    }
    *segment_index = seed_segment;
    *line_group_index = group_first_segment;
    *anchor_x_mm = line.ax;
    *anchor_y_mm = line.ay;
    memset(nav_scan_straight_mode_histogram, 0,
           sizeof(nav_scan_straight_mode_histogram));
    /* First collect only points that are geometrically compatible with the
     * current map line.  This is deliberately a coarse gate: the current
     * transform may still be wrong by a substantial lateral amount. */
    for (index = 0u; index < point_count; index++)
    {
        const PhototubeScanPoint *point = nav_scan_match_point_at(index);
        float qx;
        float qy;
        float residual;
        float tangent;
        float segment_dx;
        float segment_dy;
        float weight = nav_scan_straight_point_weight(point);
        if (!nav_scan_point_is_straight_centerline(point) ||
            weight <= 0.0f ||
            !nav_scan_project_line(&line, point->x_mm, point->y_mm,
                                   NAV_SCAN_MATCH_SEGMENT_MARGIN_MM,
                                   &qx, &qy, &residual, &tangent) ||
            (fabsf(residual) >
             NAV_SCAN_MATCH_MAX_STRAIGHT_POINT_RESIDUAL_MM))
        {
            continue;
        }
        segment_dx = point->x_mm - qx;
        segment_dy = point->y_mm - qy;
        if ((segment_dx * segment_dx + segment_dy * segment_dy) >
            NAV_SCAN_MATCH_MAX_STRAIGHT_POINT_RESIDUAL_MM *
                NAV_SCAN_MATCH_MAX_STRAIGHT_POINT_RESIDUAL_MM)
        {
            continue;
        }
        if (candidate_count < PHOTOTUBE_SCAN_MAX_POINTS)
        {
            uint16 bin = (uint16)(
                (residual +
                 NAV_SCAN_MATCH_MAX_STRAIGHT_POINT_RESIDUAL_MM) /
                NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_MM);
            if (bin >= NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT)
            {
                bin = (uint16)(NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT - 1u);
            }
            nav_scan_straight_residual[candidate_count] =
                residual;
            nav_scan_straight_mode_histogram[bin] += weight;
            candidate_count++;
        }
        candidate_weight += weight;
        candidate_residual_square += weight * residual * residual;
    }
    if (candidate_count < NAV_SCAN_MATCH_MIN_POINT_COUNT)
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_STRAIGHT_CANDIDATE_POINTS;
        }
        return 0u;
    }
    if (candidate_weight < NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT)
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_STRAIGHT_CANDIDATE_WEIGHT;
        }
        return 0u;
    }

    *rms_before_mm = sqrtf(candidate_residual_square / candidate_weight);
    mode_selection = nav_scan_select_straight_residual_mode(
        &residual_mode_center);
    if (2u == mode_selection)
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_AMBIGUOUS_STRAIGHT;
        }
        return 0u;
    }
    if (0u == mode_selection)
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_STRAIGHT_RESIDUAL_MODE;
        }
        return 0u;
    }

    /* Seed the robust estimator only from the selected weighted mode.  This
     * prevents a multi-hit component cluster with many low-confidence points
     * from winning an unweighted median by point count alone. */
    for (index = 0u; index < candidate_count; index++)
    {
        float residual =
            nav_scan_straight_residual[index];
        if (fabsf(residual - residual_mode_center) <=
            NAV_SCAN_MATCH_STRAIGHT_MODE_SEED_GATE_MM)
        {
            nav_scan_straight_residual[mode_candidate_count] =
                residual;
            mode_candidate_count++;
        }
    }
    if (mode_candidate_count < NAV_SCAN_MATCH_MIN_POINT_COUNT)
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_STRAIGHT_MODE_POINTS;
        }
        return 0u;
    }

    /* Components on the straight can form a second parallel or diagonal
     * cluster.  Refine the unique confidence-weighted mode with median/MAD
     * without letting those points pull the regression toward themselves. */
    residual_median = nav_scan_select_float(
        nav_scan_straight_residual, mode_candidate_count,
        (uint16)(mode_candidate_count / 2u));
    for (index = 0u; index < mode_candidate_count; index++)
    {
        nav_scan_straight_residual[index] = fabsf(
            nav_scan_straight_residual[index] -
            residual_median);
    }
    residual_mad = nav_scan_select_float(
        nav_scan_straight_residual, mode_candidate_count,
        (uint16)(mode_candidate_count / 2u));
    robust_gate = NAV_SCAN_MATCH_STRAIGHT_ROBUST_MIN_GATE_MM;
    {
        float mad_gate = NAV_SCAN_MATCH_STRAIGHT_ROBUST_MAD_SCALE *
                         1.4826f * residual_mad;
        if (mad_gate > robust_gate) robust_gate = mad_gate;
    }
    if (robust_gate > NAV_SCAN_MATCH_STRAIGHT_ROBUST_MAX_GATE_MM)
    {
        robust_gate = NAV_SCAN_MATCH_STRAIGHT_ROBUST_MAX_GATE_MM;
    }

    /* Recompute the regression using only the dominant residual cluster. */
    for (index = 0u; index < point_count; index++)
    {
        const PhototubeScanPoint *point = nav_scan_match_point_at(index);
        float qx;
        float qy;
        float residual;
        float tangent;
        float segment_dx;
        float segment_dy;
        float weight = nav_scan_straight_point_weight(point);
        if (!nav_scan_point_is_straight_centerline(point) ||
            weight <= 0.0f ||
            !nav_scan_project_line(&line, point->x_mm, point->y_mm,
                                   NAV_SCAN_MATCH_SEGMENT_MARGIN_MM,
                                   &qx, &qy, &residual, &tangent) ||
            (fabsf(residual) >
             NAV_SCAN_MATCH_MAX_STRAIGHT_POINT_RESIDUAL_MM) ||
            (fabsf(residual - residual_mode_center) >
             NAV_SCAN_MATCH_STRAIGHT_MODE_SEED_GATE_MM) ||
            (fabsf(residual - residual_median) > robust_gate))
        {
            continue;
        }
        segment_dx = point->x_mm - qx;
        segment_dy = point->y_mm - qy;
        if ((segment_dx * segment_dx + segment_dy * segment_dy) >
            NAV_SCAN_MATCH_MAX_STRAIGHT_POINT_RESIDUAL_MM *
                NAV_SCAN_MATCH_MAX_STRAIGHT_POINT_RESIDUAL_MM)
        {
            continue;
        }
        tangent_sum += weight * tangent;
        residual_sum += weight * residual;
        tangent_square_sum += weight * tangent * tangent;
        tangent_residual_sum += weight * tangent * residual;
        residual_square_sum += weight * residual * residual;
        used_weight += weight;
        if (tangent < tangent_min) tangent_min = tangent;
        if (tangent > tangent_max) tangent_max = tangent;
        if (!have_used_frame || (point->frame_seq != last_used_frame_seq))
        {
            used_frames++;
            last_used_frame_seq = point->frame_seq;
            have_used_frame = 1u;
        }
        used++;
    }
    span = tangent_max - tangent_min;
    if (used < NAV_SCAN_MATCH_MIN_POINT_COUNT)
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_STRAIGHT_INLIER_POINTS;
        }
        return 0u;
    }
    if (used_weight < NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT)
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_STRAIGHT_INLIER_WEIGHT;
        }
        return 0u;
    }
    if (used_weight < candidate_weight *
                       NAV_SCAN_MATCH_STRAIGHT_MIN_INLIER_RATIO)
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_STRAIGHT_INLIER_RATIO;
        }
        return 0u;
    }
    if (span < NAV_SCAN_MATCH_MIN_STRAIGHT_SPAN_MM)
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_STRAIGHT_SPAN;
        }
        return 0u;
    }
    if (used_frames < NAV_SCAN_MATCH_MIN_STRAIGHT_FRAME_COUNT)
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_STRAIGHT_SPAN;
        }
        return 0u;
    }
    mean_tangent = tangent_sum / used_weight;
    mean_residual = residual_sum / used_weight;
    if (NULL != tangent_information_mm2)
    {
        /* This is the actual weighted regression information in the
         * tangential direction.  Using span*sqrt(frame_count) here treats
         * the end points as uniformly and independently distributed even
         * when the retained points are clustered. */
        *tangent_information_mm2 = tangent_square_sum -
            (tangent_sum * tangent_sum) / used_weight;
        if (*tangent_information_mm2 < 0.0f)
        {
            *tangent_information_mm2 = 0.0f;
        }
    }
    tangent_variance = tangent_square_sum / used_weight -
                       mean_tangent * mean_tangent;
    residual_variance = residual_square_sum / used_weight -
                        mean_residual * mean_residual;
    tangent_residual_covariance = tangent_residual_sum / used_weight -
                                  mean_tangent * mean_residual;
    if (tangent_variance <= 1.0e-3f)
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_STRAIGHT_SPAN;
        }
        return 0u;
    }

    /* In the map-line frame the captured points satisfy r=a+m*t.  Rotating
     * odom points by -atan(m) removes the observable line-direction error;
     * the remaining mean normal residual gives the only observable
     * translation.  Translation along the line remains untouched. */
    residual_slope = tangent_residual_covariance / tangent_variance;
    *yaw_correction_rad = -atanf(residual_slope);
    correction_sin = sinf(*yaw_correction_rad);
    correction_cos = cosf(*yaw_correction_rad);
    corrected_mean_residual = mean_tangent * correction_sin +
                              mean_residual * correction_cos;
    corrected_residual_variance =
        correction_sin * correction_sin * tangent_variance +
        correction_cos * correction_cos * residual_variance +
        2.0f * correction_sin * correction_cos *
            tangent_residual_covariance;
    *anchor_x_mm = line.ax;
    *anchor_y_mm = line.ay;
    *shift_x_mm = -corrected_mean_residual * line.nx;
    *shift_y_mm = -corrected_mean_residual * line.ny;
    /* Compare before/after RMS on the same robust inlier set.  The coarse
     * candidate RMS above is useful for diagnostics, but must not make an
     * outlier-heavy window appear to improve merely because outliers were
     * removed by the robust gate. */
    *rms_before_mm = sqrtf(nav_scan_match_clamp(
        residual_square_sum / used_weight, 0.0f, 1.0e12f));
    *rms_after_mm = sqrtf(nav_scan_match_clamp(
        corrected_residual_variance, 0.0f, 1.0e12f));
    *inlier_count = used;
    *frame_count = used_frames;
    *span_mm = span;
    *segment_index = seed_segment;
    *line_group_index = group_first_segment;
    if (sqrtf((*shift_x_mm * *shift_x_mm) +
              (*shift_y_mm * *shift_y_mm)) > NAV_SCAN_MATCH_MAX_CORRECTION_MM)
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_CORRECTION;
        }
        return 0u;
    }
    if (*rms_after_mm > NAV_SCAN_MATCH_MAX_RMS_MM)
    {
        if (NULL != reject_reason)
        {
            *reject_reason = NAV_SCAN_MATCH_REJECT_RMS;
        }
        return 0u;
    }
    return 1u;
}

static void nav_scan_transform_point(float x_mm,
                                     float y_mm,
                                     float anchor_x_mm,
                                     float anchor_y_mm,
                                     float shift_x_mm,
                                     float shift_y_mm,
                                     float cos_yaw,
                                     float sin_yaw,
                                     float *out_x_mm,
                                     float *out_y_mm)
{
    float relative_x = x_mm - anchor_x_mm;
    float relative_y = y_mm - anchor_y_mm;

    *out_x_mm = anchor_x_mm + shift_x_mm +
                relative_x * cos_yaw - relative_y * sin_yaw;
    *out_y_mm = anchor_y_mm + shift_y_mm +
                relative_x * sin_yaw + relative_y * cos_yaw;
}

static uint8 nav_scan_make_junction_support_line(uint16 vertex_index,
                                                  NavScanLine *line)
{
    float vx;
    float vy;
    float length;

    if ((NULL == line) || (vertex_index >= nav.scan_vertex_count) ||
        (NULL == nav.scan_vertex_x) || (NULL == nav.scan_vertex_y) ||
        (NULL == nav.scan_vertex_support_x) ||
        (NULL == nav.scan_vertex_support_y))
    {
        return 0u;
    }
    nav_map_point_to_odom((float)nav.scan_vertex_x[vertex_index],
                          (float)nav.scan_vertex_y[vertex_index],
                          &line->ax, &line->ay);
    nav_map_point_to_odom((float)nav.scan_vertex_support_x[vertex_index],
                          (float)nav.scan_vertex_support_y[vertex_index],
                          &line->bx, &line->by);
    vx = line->bx - line->ax;
    vy = line->by - line->ay;
    length = sqrtf(vx * vx + vy * vy);
    if (length < 1.0f)
    {
        return 0u;
    }
    line->length = length;
    line->tx = vx / length;
    line->ty = vy / length;
    line->nx = -line->ty;
    line->ny = line->tx;
    return 1u;
}

/* A junction table predates the four-arm crossing case and stores one
 * non-route support endpoint.  At a true cross, the scan can see the same
 * physical branch on both sides of the vertex.  Do not invent that arm for a
 * T: promote the one-sided segment only when the current cloud contains a
 * separate, multi-frame cluster on the opposite ray. */
static uint8 nav_scan_promote_bidirectional_junction_support(
    uint16 vertex_index, uint16 point_count, NavScanLine *line)
{
    NavScanLine one_sided;
    NavScanLine opposite_ray;
    float vertex_x;
    float vertex_y;
    float ux;
    float uy;
    float opposite_length;
    uint16 point_hits = 0u;
    uint16 frame_hits = 0u;
    uint32 last_frame = 0u;
    uint8 have_frame = 0u;
    uint16 index;

    if ((NULL == line) ||
        !nav_scan_make_junction_support_line(vertex_index, &one_sided) ||
        (point_count < NAV_SCAN_MATCH_MIN_POINT_COUNT))
    {
        return 0u;
    }
    nav_map_point_to_odom((float)nav.scan_vertex_x[vertex_index],
                          (float)nav.scan_vertex_y[vertex_index],
                          &vertex_x, &vertex_y);
    ux = one_sided.tx;
    uy = one_sided.ty;
    opposite_length = one_sided.length;
    if (opposite_length < NAV_SCAN_MATCH_MIN_FEATURE_SIDE_SPAN_MM)
    {
        return 0u;
    }

    /* The opposite-ray test is intentionally wider than the final 25 mm
     * residual gate.  It is only a topology probe performed before the pose
     * search, so it must tolerate the current lateral/yaw error. */
    opposite_ray.ax = vertex_x;
    opposite_ray.ay = vertex_y;
    opposite_ray.bx = vertex_x - ux * opposite_length;
    opposite_ray.by = vertex_y - uy * opposite_length;
    opposite_ray.tx = -ux;
    opposite_ray.ty = -uy;
    opposite_ray.nx = -opposite_ray.ty;
    opposite_ray.ny = opposite_ray.tx;
    opposite_ray.length = opposite_length;

    for (index = 0u; index < point_count; index++)
    {
        const PhototubeScanPoint *point = nav_scan_match_point_at(index);
        float qx;
        float qy;
        float residual;
        float tangent;
        float dx;
        float dy;
        float distance_mm;

        if ((NULL == point) ||
            !nav_scan_point_is_junction_evidence(point) ||
            (nav_scan_point_weight(point) <= 0.0f) ||
            !nav_scan_project_line(&opposite_ray, point->x_mm, point->y_mm,
                                   NAV_SCAN_MATCH_SEGMENT_MARGIN_MM,
                                   &qx, &qy, &residual, &tangent) ||
            (tangent < 20.0f))
        {
            continue;
        }
        dx = point->x_mm - qx;
        dy = point->y_mm - qy;
        distance_mm = sqrtf(dx * dx + dy * dy);
        if (distance_mm > 100.0f)
        {
            continue;
        }
        point_hits++;
        if (!have_frame || (point->frame_seq != last_frame))
        {
            frame_hits++;
            last_frame = point->frame_seq;
            have_frame = 1u;
        }
    }
    if ((point_hits < NAV_SCAN_MATCH_MIN_CORNER_SIDE_COUNT) ||
        (frame_hits < NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_FRAME_COUNT))
    {
        return 0u;
    }

    /* Keep the line count/telemetry ABI unchanged: one finite segment now
     * covers both arms, with the vertex in its interior. */
    line->ax = vertex_x - ux * opposite_length;
    line->ay = vertex_y - uy * opposite_length;
    line->bx = one_sided.bx;
    line->by = one_sided.by;
    line->tx = ux;
    line->ty = uy;
    line->nx = -uy;
    line->ny = ux;
    line->length = opposite_length * 2.0f;
    return 1u;
}

static uint8 nav_scan_junction_ray_evidence(
    const NavScanLine *ray, uint16 point_count,
    uint16 *point_hits, uint16 *frame_hits)
{
    uint16 points = 0u;
    uint16 frames = 0u;
    uint32 last_frame = 0u;
    uint8 have_frame = 0u;
    uint16 index;

    if ((NULL == ray) || (NULL == point_hits) || (NULL == frame_hits))
    {
        return 0u;
    }
    for (index = 0u; index < point_count; index++)
    {
        const PhototubeScanPoint *point = nav_scan_match_point_at(index);
        float qx;
        float qy;
        float residual;
        float tangent;
        float dx;
        float dy;

        if ((NULL == point) ||
            !nav_scan_point_is_junction_evidence(point) ||
            (nav_scan_point_weight(point) <= 0.0f) ||
            !nav_scan_project_line(ray, point->x_mm, point->y_mm,
                                   NAV_SCAN_MATCH_SEGMENT_MARGIN_MM,
                                   &qx, &qy, &residual, &tangent) ||
            (tangent < 20.0f))
        {
            continue;
        }
        dx = point->x_mm - qx;
        dy = point->y_mm - qy;
        if ((dx * dx + dy * dy) > (100.0f * 100.0f))
        {
            continue;
        }
        points++;
        if (!have_frame || (point->frame_seq != last_frame))
        {
            frames++;
            last_frame = point->frame_seq;
            have_frame = 1u;
        }
    }
    *point_hits = points;
    *frame_hits = frames;
    return (uint8)((points >= NAV_SCAN_MATCH_MIN_CORNER_SIDE_COUNT) &&
                   (frames >= NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_FRAME_COUNT));
}

/* For a turning T/cross the exporter has no support endpoint because the
 * route is not straight through the vertex.  The two route edges still give
 * enough geometry to propose the two possible continuation rays.  Select one
 * only when the cloud itself confirms it; otherwise leave the map topology
 * unchanged. */
static uint8 nav_scan_make_observed_junction_continuation(
    const NavScanLine *route_in, const NavScanLine *route_out,
    uint16 point_count, NavScanLine *line)
{
    NavScanLine candidates[2];
    uint16 best_points = 0u;
    uint16 best_frames = 0u;
    uint8 best = 0xffu;
    uint8 index;

    if ((NULL == route_in) || (NULL == route_out) || (NULL == line))
    {
        return 0u;
    }
    /* route_in points toward the vertex; continue it beyond the vertex. */
    candidates[0] = *route_in;
    candidates[0].ax = route_in->bx;
    candidates[0].ay = route_in->by;
    candidates[0].bx = route_in->bx + route_in->tx * route_in->length;
    candidates[0].by = route_in->by + route_in->ty * route_in->length;
    /* route_out points away from the vertex; extend it behind the vertex. */
    candidates[1] = *route_out;
    candidates[1].ax = route_out->ax - route_out->tx * route_out->length;
    candidates[1].ay = route_out->ay - route_out->ty * route_out->length;
    candidates[1].bx = route_out->ax;
    candidates[1].by = route_out->ay;

    for (index = 0u; index < 2u; index++)
    {
        uint16 points;
        uint16 frames;
        if (!nav_scan_junction_ray_evidence(
                &candidates[index], point_count, &points, &frames))
        {
            continue;
        }
        if ((best == 0xffu) || (points > best_points) ||
            ((points == best_points) && (frames > best_frames)))
        {
            best = index;
            best_points = points;
            best_frames = frames;
        }
    }
    if (best == 0xffu)
    {
        return 0u;
    }
    *line = candidates[best];
    return 1u;
}

static uint8 nav_scan_feature_nearest_line(const NavScanLine *lines,
                                            uint8 line_count,
                                            float x_mm,
                                            float y_mm,
                                            uint8 *line_index,
                                            float *residual_mm,
                                            float *tangent_mm)
{
    uint8 index;
    uint8 found = 0u;
    uint8 best_index = 0u;
    float best_distance_mm = 1.0e30f;
    float second_noncollinear_distance_mm = 1.0e30f;

    if ((NULL == lines) || (0u == line_count) || (NULL == line_index) ||
        (NULL == residual_mm) || (NULL == tangent_mm))
    {
        return 0u;
    }
    for (index = 0u; index < line_count; index++)
    {
        float qx;
        float qy;
        float residual;
        float tangent;
        float dx;
        float dy;
        float distance_mm;

        if (!nav_scan_project_line(&lines[index], x_mm, y_mm,
                                   NAV_SCAN_MATCH_SEGMENT_MARGIN_MM,
                                   &qx, &qy, &residual, &tangent))
        {
            continue;
        }
        dx = x_mm - qx;
        dy = y_mm - qy;
        distance_mm = sqrtf(dx * dx + dy * dy);
        if ((distance_mm >
             NAV_SCAN_MATCH_MAX_FEATURE_POINT_RESIDUAL_MM) ||
            (found && (distance_mm >= best_distance_mm)))
        {
            continue;
        }
        found = 1u;
        best_distance_mm = distance_mm;
        best_index = index;
        *line_index = index;
        *residual_mm = residual;
        *tangent_mm = tangent;
    }
    if (!found)
    {
        return 0u;
    }

    /* Near a feature vertex two non-collinear finite segments can have
     * almost identical distances.  Such a point has no unique line
     * correspondence and must not be allowed to steer the SE(2) solve. */
    for (index = 0u; index < line_count; index++)
    {
        float qx;
        float qy;
        float residual;
        float tangent;
        float dx;
        float dy;
        float distance_mm;
        float cross;

        if (index == best_index ||
            !nav_scan_project_line(&lines[index], x_mm, y_mm,
                                   NAV_SCAN_MATCH_SEGMENT_MARGIN_MM,
                                   &qx, &qy, &residual, &tangent))
        {
            continue;
        }
        cross = fabsf(lines[best_index].tx * lines[index].ty -
                      lines[best_index].ty * lines[index].tx);
        if (cross < 0.2f)
        {
            continue;
        }
        dx = x_mm - qx;
        dy = y_mm - qy;
        distance_mm = sqrtf(dx * dx + dy * dy);
        if ((distance_mm <= NAV_SCAN_MATCH_MAX_FEATURE_POINT_RESIDUAL_MM) &&
            (distance_mm < second_noncollinear_distance_mm))
        {
            second_noncollinear_distance_mm = distance_mm;
        }
    }
    if ((second_noncollinear_distance_mm < 1.0e29f) &&
        ((second_noncollinear_distance_mm - best_distance_mm) <
         NAV_SCAN_MATCH_FEATURE_ASSOCIATION_MARGIN_MM))
    {
        return 2u;
    }
    return 1u;
}

static float nav_scan_feature_robust_weight(float base_weight,
                                            float residual_mm)
{
    float absolute_residual = fabsf(residual_mm);

    if (absolute_residual <= NAV_SCAN_MATCH_MAX_RMS_MM)
    {
        return base_weight;
    }
    return base_weight * NAV_SCAN_MATCH_MAX_RMS_MM / absolute_residual;
}

static uint8 nav_scan_feature_geometry_valid(
    const NavScanLine *lines, uint8 line_count, const uint16 *counts,
    const float *weights, const float *tangent_min, const float *tangent_max,
    const uint16 *frame_counts, const uint16 *run_frame_counts,
    const uint16 *clean_points, uint8 feature_type)
{
    uint8 first;
    uint8 second;
    uint8 line_valid[3] = {0u, 0u, 0u};
    float min_side_effective_weight =
        (NAV_SCAN_VERTEX_TYPE_JUNCTION == feature_type) ?
        NAV_SCAN_MATCH_JUNCTION_MIN_SIDE_EFFECTIVE_WEIGHT :
        NAV_SCAN_MATCH_MIN_SIDE_EFFECTIVE_WEIGHT;

    nav_scan_feature_capture_lines(counts, frame_counts, line_count);
    /* Every observed edge uses the same evidence gates.  The map may provide
     * two or three non-collinear edges, but no route occurrence gets a special
     * threshold; only the actual point count, span and frame evidence matter. */
    for (first = 0u; first < line_count; first++)
    {
        uint8 frame_ok = (uint8)(
            (frame_counts[first] >= NAV_SCAN_MATCH_MIN_SIDE_FRAME_COUNT) ||
            ((frame_counts[first] >= NAV_SCAN_MATCH_MIN_RUN_SIDE_FRAME_COUNT) &&
             (run_frame_counts[first] >= NAV_SCAN_MATCH_MIN_RUN_SIDE_FRAME_COUNT)));
        line_valid[first] = (uint8)(
            (counts[first] >= NAV_SCAN_MATCH_MIN_CORNER_SIDE_COUNT) &&
            (weights[first] >= min_side_effective_weight) &&
            ((tangent_max[first] - tangent_min[first]) >=
             NAV_SCAN_MATCH_MIN_FEATURE_SIDE_SPAN_MM) &&
            frame_ok &&
            !(((frame_counts[first] <= NAV_SCAN_MATCH_MIN_SIDE_FRAME_COUNT) &&
               (counts[first] <= 5u)) &&
              (clean_points[first] <
               NAV_SCAN_MATCH_MIN_SHORT_SIDE_CLEAN_POINTS)));
    }
    for (first = 0u; first < line_count; first++)
    {
        if (!line_valid[first])
        {
            continue;
        }
        for (second = (uint8)(first + 1u); second < line_count; second++)
        {
            float cross;
            if (!line_valid[second])
            {
                continue;
            }
            cross = fabsf(lines[first].tx * lines[second].ty -
                          lines[first].ty * lines[second].tx);
            if (cross >= 0.2f)
            {
                return 1u;
            }
        }
    }
    nav_scan_feature_mark_line_quality(
        counts, weights, tangent_min, tangent_max, frame_counts,
        run_frame_counts, 1u, line_count, min_side_effective_weight);
    nav_scan_feature_attempt.detail_flags |=
        TELEMETRY_LOCALIZATION_DETAIL_NONCOLLINEAR;
    return 0u;
}

/* Score a candidate local pose by assigning each transformed point to its
 * nearest finite map segment.  This is the coarse scan-to-map stage: it is
 * deliberately independent of the final fixed-correspondence least-squares
 * solve below, so an early wrong branch assignment cannot seed the optimizer. */
static uint8 nav_scan_feature_score_pose(
    const NavScanLine *lines,
    uint8 line_count,
    uint8 feature_type,
    uint16 search_count,
    float anchor_x_mm,
    float anchor_y_mm,
    float shift_x_mm,
    float shift_y_mm,
    float yaw_rad,
    float *score,
    uint16 *matched_count,
    uint16 *matched_frame_count)
{
    uint16 search_index;
    uint16 count = 0u;
    uint16 frame_count = 0u;
    uint32 last_frame_seq = 0u;
    uint8 have_frame = 0u;
    float weight_sum = 0.0f;
    float cost_sum = 0.0f;
    float cos_yaw;
    float sin_yaw;
    float local_radius_sq = NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM *
                             NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM;

    if ((NULL == lines) || (0u == line_count) ||
        (NULL == score) || (NULL == matched_count) ||
        (NULL == matched_frame_count))
    {
        return 0u;
    }
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    for (search_index = 0u; search_index < search_count; search_index++)
    {
        const PhototubeScanPoint *point = nav_scan_match_point_at(
            nav_scan_feature_search_indices[search_index]);
        float transformed_x;
        float transformed_y;
        float dx;
        float dy;
        float residual;
        float tangent;
        float absolute_residual;
        float weight;
        uint8 selected_line;
        uint8 association;

        if (NULL == point)
        {
            continue;
        }
        weight = nav_scan_point_weight(point);
        if ((weight <= 0.0f) ||
            !nav_scan_point_is_feature_evidence(point, feature_type))
        {
            continue;
        }
        nav_scan_transform_point(point->x_mm, point->y_mm,
                                 anchor_x_mm, anchor_y_mm,
                                 shift_x_mm, shift_y_mm,
                                 cos_yaw, sin_yaw,
                                 &transformed_x, &transformed_y);
        dx = transformed_x - anchor_x_mm;
        dy = transformed_y - anchor_y_mm;
        if ((dx * dx + dy * dy) > local_radius_sq)
        {
            continue;
        }
        association = nav_scan_feature_nearest_line(
            lines, line_count, transformed_x, transformed_y,
            &selected_line, &residual, &tangent);
        if (1u != association)
        {
            continue;
        }
        absolute_residual = fabsf(residual);
        if (absolute_residual <= NAV_SCAN_MATCH_MAX_RMS_MM)
        {
            cost_sum += weight * residual * residual;
        }
        else
        {
            /* Huber continuation prevents a slipping/outlying point from
             * winning the grid search while retaining its weak evidence. */
            cost_sum += weight *
                (2.0f * NAV_SCAN_MATCH_MAX_RMS_MM * absolute_residual -
                 NAV_SCAN_MATCH_MAX_RMS_MM * NAV_SCAN_MATCH_MAX_RMS_MM);
        }
        weight_sum += weight;
        count++;
        if (!have_frame || (point->frame_seq != last_frame_seq))
        {
            frame_count++;
            last_frame_seq = point->frame_seq;
            have_frame = 1u;
        }
    }
    if ((count < NAV_SCAN_MATCH_MIN_POINT_COUNT) ||
        (weight_sum < NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT) ||
        (0u == frame_count))
    {
        return 0u;
    }
    *score = cost_sum / weight_sum;
    *matched_count = count;
    *matched_frame_count = frame_count;
    return 1u;
}

/* Search a small pose neighborhood around the current odometry alignment.
 * The broad point list is collected once, then every candidate is evaluated
 * by nearest finite-segment distance.  A fine pass around the best coarse
 * pose gives the robust solver a correspondence-free, physically local seed. */
static uint8 nav_scan_feature_search_pose(
    const NavScanLine *lines,
    uint8 line_count,
    uint8 feature_type,
    uint16 point_count,
    float anchor_x_mm,
    float anchor_y_mm,
    float *shift_x_mm,
    float *shift_y_mm,
    float *yaw_rad)
{
    uint16 index;
    uint16 search_count = 0u;
    uint16 local_count = 0u;
    uint16 local_stride = 1u;
    uint16 local_seen = 0u;
    int16 coarse_tx;
    int16 coarse_ty;
    int16 coarse_yaw_deg;
    int16 fine_tx;
    int16 fine_ty;
    int16 fine_yaw_deg;
    int16 fine_translation_steps;
    int16 fine_yaw_steps;
    float broad_radius_mm = NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM;
    float broad_radius_sq = broad_radius_mm * broad_radius_mm;
    float best_score = 1.0e30f;
    float best_shift_x = 0.0f;
    float best_shift_y = 0.0f;
    float best_yaw = 0.0f;
    uint8 have_best = 0u;

    if ((NULL == lines) || (0u == line_count) ||
        (NULL == shift_x_mm) || (NULL == shift_y_mm) ||
        (NULL == yaw_rad))
    {
        return 0u;
    }
    /* The grid search is only a correspondence-free seed.  Do not replay all
     * 1280 retained points for every pose candidate: the final fixed-
     * correspondence solve below still evaluates the complete cloud.  Keep
     * a uniformly spaced subset so both sides of a feature remain represented
     * while bounding the expensive sin/cos and finite-segment projections. */
    for (index = 0u; index < point_count; index++)
    {
        const PhototubeScanPoint *point = nav_scan_match_point_at(index);
        float dx;
        float dy;

        if ((NULL == point) ||
            (nav_scan_point_weight(point) <= 0.0f) ||
            !nav_scan_point_is_feature_evidence(point, feature_type))
        {
            continue;
        }
        dx = point->x_mm - anchor_x_mm;
        dy = point->y_mm - anchor_y_mm;
        if ((dx * dx + dy * dy) > broad_radius_sq)
        {
            continue;
        }
        local_count++;
    }
    if (local_count > NAV_SCAN_MATCH_FEATURE_SEARCH_POINT_LIMIT)
    {
        local_stride = (uint16)((local_count +
                                 NAV_SCAN_MATCH_FEATURE_SEARCH_POINT_LIMIT -
                                 1u) /
                                NAV_SCAN_MATCH_FEATURE_SEARCH_POINT_LIMIT);
    }
    for (index = 0u; index < point_count; index++)
    {
        const PhototubeScanPoint *point = nav_scan_match_point_at(index);
        float dx;
        float dy;

        if ((NULL == point) ||
            (nav_scan_point_weight(point) <= 0.0f) ||
            !nav_scan_point_is_feature_evidence(point, feature_type))
        {
            continue;
        }
        dx = point->x_mm - anchor_x_mm;
        dy = point->y_mm - anchor_y_mm;
        if ((dx * dx + dy * dy) > broad_radius_sq)
        {
            continue;
        }
        if ((0u != local_seen) &&
            ((local_seen % local_stride) != 0u) &&
            (local_count > NAV_SCAN_MATCH_FEATURE_SEARCH_POINT_LIMIT))
        {
            local_seen++;
            continue;
        }
        if (search_count < PHOTOTUBE_SCAN_MAX_POINTS)
        {
            nav_scan_feature_search_indices[search_count] = index;
            search_count++;
        }
        local_seen++;
    }
    if (search_count < NAV_SCAN_MATCH_MIN_POINT_COUNT)
    {
        return 0u;
    }

    /* Score the current alignment before entering the expensive broad grid.
     * Most feature attempts happen after a recent straight update, so a
     * nearby identity seed plus the fine pass is both faster and less likely
     * to select a distant symmetric branch. */
    {
        float score;
        uint16 matched_count;
        uint16 matched_frames;

        if (nav_scan_feature_score_pose(
                lines, line_count, feature_type, search_count,
                anchor_x_mm, anchor_y_mm, 0.0f, 0.0f, 0.0f,
                &score, &matched_count, &matched_frames) &&
            (matched_count >= NAV_SCAN_MATCH_MIN_POINT_COUNT) &&
            (matched_frames >= NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_FRAME_COUNT))
        {
            best_score = score;
            best_shift_x = 0.0f;
            best_shift_y = 0.0f;
            best_yaw = 0.0f;
            have_best = 1u;
        }
    }

    if (!have_best ||
        (best_score > NAV_SCAN_MATCH_FEATURE_SEARCH_IDENTITY_MAX_SCORE_MM2))
    {
        for (coarse_tx = (int16)-NAV_SCAN_MATCH_FEATURE_SEARCH_TRANSLATION_MM;
             coarse_tx <= (int16)NAV_SCAN_MATCH_FEATURE_SEARCH_TRANSLATION_MM;
             coarse_tx = (int16)(coarse_tx +
                 (int16)NAV_SCAN_MATCH_FEATURE_SEARCH_COARSE_STEP_MM))
        {
            for (coarse_ty = (int16)-NAV_SCAN_MATCH_FEATURE_SEARCH_TRANSLATION_MM;
                 coarse_ty <= (int16)NAV_SCAN_MATCH_FEATURE_SEARCH_TRANSLATION_MM;
                 coarse_ty = (int16)(coarse_ty +
                     (int16)NAV_SCAN_MATCH_FEATURE_SEARCH_COARSE_STEP_MM))
            {
                for (coarse_yaw_deg = (int16)-
                         (int16)NAV_SCAN_MATCH_FEATURE_SEARCH_YAW_DEG;
                     coarse_yaw_deg <=
                     (int16)NAV_SCAN_MATCH_FEATURE_SEARCH_YAW_DEG;
                     coarse_yaw_deg = (int16)(coarse_yaw_deg +
                         (int16)NAV_SCAN_MATCH_FEATURE_SEARCH_COARSE_STEP_DEG))
                {
                    float score;
                    uint16 matched_count;
                    uint16 matched_frames;
                    float candidate_x = (float)coarse_tx;
                    float candidate_y = (float)coarse_ty;
                    float candidate_yaw = (float)coarse_yaw_deg *
                                          nav_rad_per_deg;

                    if (nav_scan_feature_score_pose(
                            lines, line_count, feature_type, search_count,
                            anchor_x_mm, anchor_y_mm,
                            candidate_x, candidate_y, candidate_yaw,
                            &score, &matched_count, &matched_frames) &&
                        (matched_count >= NAV_SCAN_MATCH_MIN_POINT_COUNT) &&
                        (matched_frames >=
                         NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_FRAME_COUNT) &&
                        (score < best_score))
                    {
                        best_score = score;
                        best_shift_x = candidate_x;
                        best_shift_y = candidate_y;
                        best_yaw = candidate_yaw;
                        have_best = 1u;
                    }
                }
            }
        }
    }
    if (!have_best)
    {
        return 0u;
    }

    /* Refine only around the winning coarse cell. */
    fine_translation_steps = (int16)(
        NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_TRANSLATION_MM /
        NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_MM);
    fine_yaw_steps = (int16)(
        NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_YAW_DEG /
        NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_DEG);
    for (fine_tx = (int16)-fine_translation_steps;
         fine_tx <= fine_translation_steps; fine_tx++)
    {
        for (fine_ty = (int16)-fine_translation_steps;
             fine_ty <= fine_translation_steps; fine_ty++)
        {
            for (fine_yaw_deg = (int16)-fine_yaw_steps;
                 fine_yaw_deg <= fine_yaw_steps; fine_yaw_deg++)
            {
                float score;
                uint16 matched_count;
                uint16 matched_frames;
                float candidate_x = best_shift_x +
                    (float)fine_tx * NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_MM;
                float candidate_y = best_shift_y +
                    (float)fine_ty * NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_MM;
                float candidate_yaw = best_yaw +
                    (float)fine_yaw_deg * NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_DEG *
                    nav_rad_per_deg;

                if ((fabsf(candidate_x) >
                     NAV_SCAN_MATCH_FEATURE_SEARCH_TRANSLATION_MM) ||
                    (fabsf(candidate_y) >
                     NAV_SCAN_MATCH_FEATURE_SEARCH_TRANSLATION_MM) ||
                    (fabsf(candidate_yaw) >
                     NAV_SCAN_MATCH_FEATURE_SEARCH_YAW_DEG * nav_rad_per_deg))
                {
                    continue;
                }
                if (nav_scan_feature_score_pose(
                        lines, line_count, feature_type, search_count,
                        anchor_x_mm, anchor_y_mm,
                        candidate_x, candidate_y, candidate_yaw,
                        &score, &matched_count, &matched_frames) &&
                    (matched_count >= NAV_SCAN_MATCH_MIN_POINT_COUNT) &&
                    (matched_frames >=
                     NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_FRAME_COUNT) &&
                    (score < best_score))
                {
                    best_score = score;
                    best_shift_x = candidate_x;
                    best_shift_y = candidate_y;
                    best_yaw = candidate_yaw;
                }
            }
        }
    }
    *shift_x_mm = best_shift_x;
    *shift_y_mm = best_shift_y;
    *yaw_rad = best_yaw;
    return 1u;
}

/* Corner and junction observations use a local scan-to-map seed followed by
 * a fixed-correspondence robust SE(2) solve.  The seed searches pose directly
 * instead of trusting the first point-to-edge assignment. */
static uint8 nav_scan_match_feature(uint16 vertex_index,
                                    uint8 feature_type,
                                    uint16 point_count,
                                    float *anchor_x_mm,
                                    float *anchor_y_mm,
                                    float *shift_x_mm,
                                    float *shift_y_mm,
                                    float *yaw_correction_rad,
                                    float *rms_before_mm,
                                    float *rms_after_mm,
                                    uint16 *inlier_count)
{
    NavScanLine lines[3];
    uint8 line_count = 2u;
    uint8 iteration;
    uint16 index;
    float total_shift_x = 0.0f;
    float total_shift_y = 0.0f;
    float total_yaw = 0.0f;
    float anchor_x;
    float anchor_y;
    float before_sum = 0.0f;
    float before_weight = 0.0f;
    float after_sum = 0.0f;
    float after_weight = 0.0f;
    uint16 after_count = 0u;
    uint16 final_counts[3] = {0u, 0u, 0u};
    float final_weights[3] = {0.0f, 0.0f, 0.0f};
    float final_min[3] = {1.0e30f, 1.0e30f, 1.0e30f};
    float final_max[3] = {-1.0e30f, -1.0e30f, -1.0e30f};
    uint32 final_last_frame[3] = {0u, 0u, 0u};
    uint16 final_frame_counts[3] = {0u, 0u, 0u};
    uint8 final_have_frame[3] = {0u, 0u, 0u};
    uint32 final_last_run_frame[3] = {0u, 0u, 0u};
    uint16 final_run_frame_counts[3] = {0u, 0u, 0u};
    uint16 final_run_frame_streak[3] = {0u, 0u, 0u};
    uint8 final_have_run_frame[3] = {0u, 0u, 0u};
    uint16 final_clean_points[3] = {0u, 0u, 0u};
    float assignment_weight = 0.0f;
    float ambiguous_weight = 0.0f;
    float unique_assignment_weight;
    uint16 assignment_count = 0u;

    if (((NAV_SCAN_VERTEX_TYPE_CORNER != feature_type) &&
         (NAV_SCAN_VERTEX_TYPE_JUNCTION != feature_type)) ||
        (vertex_index == 0u) ||
        (vertex_index + 1u >= nav.scan_vertex_count) ||
        !nav_scan_make_line((uint16)(vertex_index - 1u), &lines[0]) ||
        !nav_scan_make_line(vertex_index, &lines[1]))
    {
        nav_scan_feature_attempt.detail_flags |=
            TELEMETRY_LOCALIZATION_DETAIL_MAP_GEOMETRY;
        return 0u;
    }
    if ((NAV_SCAN_VERTEX_TYPE_JUNCTION == feature_type) &&
        nav_scan_make_junction_support_line(vertex_index, &lines[2]))
    {
        (void)nav_scan_promote_bidirectional_junction_support(
            vertex_index, point_count, &lines[2]);
        line_count = 3u;
    }
    else if ((NAV_SCAN_VERTEX_TYPE_JUNCTION == feature_type) &&
             nav_scan_make_observed_junction_continuation(
                 &lines[0], &lines[1], point_count, &lines[2]))
    {
        line_count = 3u;
    }
    nav_map_point_to_odom((float)nav.scan_vertex_x[vertex_index],
                          (float)nav.scan_vertex_y[vertex_index],
                          &anchor_x, &anchor_y);

    /* First find a locally best pose without trusting a preselected edge.
     * The later correspondence set is fixed only after this scan-to-map seed. */
    if (!nav_scan_feature_search_pose(
            lines, line_count, feature_type, point_count,
            anchor_x, anchor_y, &total_shift_x, &total_shift_y,
            &total_yaw))
    {
        nav_scan_feature_attempt.detail_flags |=
            TELEMETRY_LOCALIZATION_DETAIL_TOTAL_POINTS;
        return 0u;
    }

    /* Establish one local, unambiguous correspondence set after the search.
     * It is deliberately never re-assigned by the subsequent least-squares
     * iterations; this prevents RMS improvement being manufactured by branch
     * switching while the solver moves. */
    for (index = 0u; index < point_count; index++)
    {
        const PhototubeScanPoint *point = nav_scan_match_point_at(index);
        float transformed_x;
        float transformed_y;
        float local_dx;
        float local_dy;
        float total_cos_yaw = cosf(total_yaw);
        float total_sin_yaw = sinf(total_yaw);
        uint8 selected_line = 0u;
        float residual = 0.0f;
        float tangent = 0.0f;
        uint8 association;
        float weight = nav_scan_point_weight(point);

        nav_scan_feature_assignment[index] = 0xffu;
        if (!nav_scan_point_is_feature_evidence(point, feature_type) ||
            (weight <= 0.0f))
        {
            continue;
        }
        nav_scan_transform_point(point->x_mm, point->y_mm,
                                 anchor_x, anchor_y,
                                 total_shift_x, total_shift_y,
                                 total_cos_yaw, total_sin_yaw,
                                 &transformed_x, &transformed_y);
        local_dx = transformed_x - anchor_x;
        local_dy = transformed_y - anchor_y;
        if ((local_dx * local_dx + local_dy * local_dy) >
            NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM *
                NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM)
        {
            continue;
        }
        association = nav_scan_feature_nearest_line(
            lines, line_count, transformed_x, transformed_y,
            &selected_line, &residual, &tangent);
        if (2u == association)
        {
            /* Only points that reached a finite, non-collinear map line are
             * evidence for the assignment ambiguity ratio.  A hit inside the
             * local feature radius but outside every segment is an outlier;
             * counting it in the denominator would dilute the ambiguous
             * weight and could let an under-constrained feature pass. */
            assignment_weight += weight;
            ambiguous_weight += weight;
            continue;
        }
        if (1u != association)
        {
            continue;
        }
        assignment_weight += weight;
        nav_scan_feature_assignment[index] = selected_line;
        assignment_count++;
    }
    unique_assignment_weight = assignment_weight - ambiguous_weight;
    if (assignment_count < NAV_SCAN_MATCH_MIN_POINT_COUNT)
    {
        nav_scan_feature_attempt.detail_flags |=
            TELEMETRY_LOCALIZATION_DETAIL_TOTAL_POINTS;
        nav_scan_feature_attempt.failure_value = (float)assignment_count;
        return 0u;
    }
    if (unique_assignment_weight < NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT)
    {
        nav_scan_feature_attempt.detail_flags |=
            TELEMETRY_LOCALIZATION_DETAIL_TOTAL_WEIGHT;
        nav_scan_feature_attempt.failure_value = unique_assignment_weight;
        return 0u;
    }
    if ((assignment_weight > 0.0f) &&
        (ambiguous_weight >= assignment_weight *
         NAV_SCAN_MATCH_FEATURE_AMBIGUOUS_WEIGHT_RATIO))
    {
        nav_scan_feature_attempt.detail_flags |=
            TELEMETRY_LOCALIZATION_DETAIL_AMBIGUOUS;
        nav_scan_feature_attempt.failure_value = ambiguous_weight;
        return 0u;
    }

    for (iteration = 0u; iteration < 2u; iteration++)
    {
        float normal_matrix[3][3] = {
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f}};
        float normal_rhs[3] = {0.0f, 0.0f, 0.0f};
        float solution[3] = {0.0f, 0.0f, 0.0f};
        uint16 counts[3] = {0u, 0u, 0u};
        float line_weights[3] = {0.0f, 0.0f, 0.0f};
        float tangent_min[3] = {1.0e30f, 1.0e30f, 1.0e30f};
        float tangent_max[3] = {-1.0e30f, -1.0e30f, -1.0e30f};
        uint32 last_frame[3] = {0u, 0u, 0u};
        uint16 frame_counts[3] = {0u, 0u, 0u};
        uint8 have_frame[3] = {0u, 0u, 0u};
        uint32 last_run_frame[3] = {0u, 0u, 0u};
        uint16 run_frame_counts[3] = {0u, 0u, 0u};
        uint16 run_frame_streak[3] = {0u, 0u, 0u};
        uint8 have_run_frame[3] = {0u, 0u, 0u};
        uint16 clean_points[3] = {0u, 0u, 0u};
        float correspondence_weight = 0.0f;
        uint16 correspondence_count = 0u;
        float cos_total_yaw = cosf(total_yaw);
        float sin_total_yaw = sinf(total_yaw);

        for (index = 0u; index < point_count; index++)
        {
            const PhototubeScanPoint *point = nav_scan_match_point_at(index);
            float transformed_x;
            float transformed_y;
            float residual;
            float tangent;
            float jacobian[3];
            float weight = nav_scan_point_weight(point);
            float solve_weight;
            uint8 selected_line = nav_scan_feature_assignment[index];
            float weighted_jacobian[3];

            if (!nav_scan_point_is_feature_evidence(point, feature_type) ||
                (weight <= 0.0f) || (selected_line >= line_count))
            {
                continue;
            }
            nav_scan_transform_point(point->x_mm, point->y_mm,
                                     anchor_x, anchor_y,
                                     total_shift_x, total_shift_y,
                                     cos_total_yaw, sin_total_yaw,
                                     &transformed_x, &transformed_y);
            nav_scan_line_residual_unbounded(&lines[selected_line],
                                             transformed_x, transformed_y,
                                             &residual, &tangent);
            solve_weight = nav_scan_feature_robust_weight(weight, residual);
            jacobian[0] = lines[selected_line].nx;
            jacobian[1] = lines[selected_line].ny;
            jacobian[2] =
                lines[selected_line].nx *
                    (-(transformed_y - anchor_y - total_shift_y)) +
                lines[selected_line].ny *
                    (transformed_x - anchor_x - total_shift_x);
            weighted_jacobian[0] = solve_weight * jacobian[0];
            weighted_jacobian[1] = solve_weight * jacobian[1];
            weighted_jacobian[2] = solve_weight * jacobian[2];
            normal_rhs[0] -= weighted_jacobian[0] * residual;
            normal_rhs[1] -= weighted_jacobian[1] * residual;
            normal_rhs[2] -= weighted_jacobian[2] * residual;
            normal_matrix[0][0] += weighted_jacobian[0] * jacobian[0];
            normal_matrix[0][1] += weighted_jacobian[0] * jacobian[1];
            normal_matrix[0][2] += weighted_jacobian[0] * jacobian[2];
            normal_matrix[1][1] += weighted_jacobian[1] * jacobian[1];
            normal_matrix[1][2] += weighted_jacobian[1] * jacobian[2];
            normal_matrix[2][2] += weighted_jacobian[2] * jacobian[2];
            correspondence_weight += weight;
            counts[selected_line]++;
            line_weights[selected_line] += weight;
            if (nav_scan_point_is_clean_centerline(point))
            {
                clean_points[selected_line]++;
            }
            if (tangent < tangent_min[selected_line])
                tangent_min[selected_line] = tangent;
            if (tangent > tangent_max[selected_line])
                tangent_max[selected_line] = tangent;
            if (!have_frame[selected_line] ||
                (point->frame_seq != last_frame[selected_line]))
            {
                last_frame[selected_line] = point->frame_seq;
                frame_counts[selected_line]++;
                have_frame[selected_line] = 1u;
            }
            if ((0u != (point->point_flags &
                        VISION_SCAN_HIT_RUN_SAMPLE)) &&
                (!have_run_frame[selected_line] ||
                 (point->frame_seq != last_run_frame[selected_line])))
            {
                if (have_run_frame[selected_line] &&
                    ((uint32)(point->frame_seq -
                              last_run_frame[selected_line]) <=
                     NAV_SCAN_MATCH_RUN_FRAME_MAX_GAP))
                {
                    run_frame_streak[selected_line]++;
                }
                else
                {
                    run_frame_streak[selected_line] = 1u;
                }
                last_run_frame[selected_line] = point->frame_seq;
                if (run_frame_streak[selected_line] >
                    run_frame_counts[selected_line])
                {
                    run_frame_counts[selected_line] =
                        run_frame_streak[selected_line];
                }
                have_run_frame[selected_line] = 1u;
            }
            correspondence_count++;
        }
        normal_matrix[1][0] = normal_matrix[0][1];
        normal_matrix[2][0] = normal_matrix[0][2];
        normal_matrix[2][1] = normal_matrix[1][2];
        if (correspondence_count < NAV_SCAN_MATCH_MIN_POINT_COUNT)
        {
            nav_scan_feature_attempt.detail_flags |=
                TELEMETRY_LOCALIZATION_DETAIL_TOTAL_POINTS;
            nav_scan_feature_attempt.failure_value =
                (float)correspondence_count;
            nav_scan_feature_capture_lines(counts, frame_counts, line_count);
            return 0u;
        }
        if (correspondence_weight < NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT)
        {
            nav_scan_feature_attempt.detail_flags |=
                TELEMETRY_LOCALIZATION_DETAIL_TOTAL_WEIGHT;
            nav_scan_feature_attempt.failure_value = correspondence_weight;
            nav_scan_feature_capture_lines(counts, frame_counts, line_count);
            return 0u;
        }
        if (!nav_scan_feature_geometry_valid(
                lines, line_count, counts, line_weights,
                tangent_min, tangent_max, frame_counts,
                run_frame_counts, clean_points, feature_type))
        {
            return 0u;
        }
        if (!nav_scan_match_solve_3x3(normal_matrix, normal_rhs, solution))
        {
            nav_scan_feature_attempt.detail_flags |=
                TELEMETRY_LOCALIZATION_DETAIL_SOLVER;
            return 0u;
        }
        total_shift_x += solution[0];
        total_shift_y += solution[1];
        total_yaw += solution[2];
    }

    {
        float cos_total_yaw = cosf(total_yaw);
        float sin_total_yaw = sinf(total_yaw);

        for (index = 0u; index < point_count; index++)
        {
        const PhototubeScanPoint *point = nav_scan_match_point_at(index);
        float transformed_x;
        float transformed_y;
        float residual;
        float tangent;
        float project_x;
        float project_y;
        float finite_dx;
        float finite_dy;
        float before_residual;
        float weight = nav_scan_point_weight(point);
        uint8 selected_line = nav_scan_feature_assignment[index];

        if (!nav_scan_point_is_feature_evidence(point, feature_type) ||
            (weight <= 0.0f) || (selected_line >= line_count))
        {
            continue;
        }
        nav_scan_transform_point(point->x_mm, point->y_mm,
                                 anchor_x, anchor_y,
                                 total_shift_x, total_shift_y,
                                 cos_total_yaw, sin_total_yaw,
                                 &transformed_x, &transformed_y);
        if (!nav_scan_project_line(&lines[selected_line],
                                   transformed_x, transformed_y,
                                   NAV_SCAN_MATCH_SEGMENT_MARGIN_MM,
                                   &project_x, &project_y,
                                   &residual, &tangent))
        {
            continue;
        }
        finite_dx = transformed_x - project_x;
        finite_dy = transformed_y - project_y;
        if ((finite_dx * finite_dx + finite_dy * finite_dy) >
            NAV_SCAN_MATCH_MAX_RMS_MM * NAV_SCAN_MATCH_MAX_RMS_MM)
        {
            continue;
        }
        before_residual =
            lines[selected_line].nx *
                (point->x_mm - lines[selected_line].ax) +
            lines[selected_line].ny *
                (point->y_mm - lines[selected_line].ay);
        before_sum += weight * before_residual * before_residual;
        before_weight += weight;
        after_sum += weight * residual * residual;
        after_weight += weight;
        final_counts[selected_line]++;
        final_weights[selected_line] += weight;
        if (nav_scan_point_is_clean_centerline(point))
        {
            final_clean_points[selected_line]++;
        }
        if (tangent < final_min[selected_line])
            final_min[selected_line] = tangent;
        if (tangent > final_max[selected_line])
            final_max[selected_line] = tangent;
        if (!final_have_frame[selected_line] ||
            (point->frame_seq != final_last_frame[selected_line]))
        {
            final_last_frame[selected_line] = point->frame_seq;
            final_frame_counts[selected_line]++;
            final_have_frame[selected_line] = 1u;
        }
        if ((0u != (point->point_flags & VISION_SCAN_HIT_RUN_SAMPLE)) &&
            (!final_have_run_frame[selected_line] ||
             (point->frame_seq != final_last_run_frame[selected_line])))
        {
            if (final_have_run_frame[selected_line] &&
                ((uint32)(point->frame_seq -
                          final_last_run_frame[selected_line]) <=
                 NAV_SCAN_MATCH_RUN_FRAME_MAX_GAP))
            {
                final_run_frame_streak[selected_line]++;
            }
            else
            {
                final_run_frame_streak[selected_line] = 1u;
            }
            final_last_run_frame[selected_line] = point->frame_seq;
            if (final_run_frame_streak[selected_line] >
                final_run_frame_counts[selected_line])
            {
                final_run_frame_counts[selected_line] =
                    final_run_frame_streak[selected_line];
            }
            final_have_run_frame[selected_line] = 1u;
        }
        after_count++;
        }
    }
    if (after_count < NAV_SCAN_MATCH_MIN_POINT_COUNT)
    {
        nav_scan_feature_attempt.detail_flags |=
            TELEMETRY_LOCALIZATION_DETAIL_TOTAL_POINTS;
        nav_scan_feature_attempt.failure_value = (float)after_count;
        nav_scan_feature_capture_lines(
            final_counts, final_frame_counts, line_count);
        return 0u;
    }
    if ((before_weight < NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT) ||
        (after_weight < NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT) ||
        (after_weight < unique_assignment_weight *
                            NAV_SCAN_MATCH_FEATURE_MIN_INLIER_WEIGHT_RATIO))
    {
        nav_scan_feature_attempt.detail_flags |=
            TELEMETRY_LOCALIZATION_DETAIL_TOTAL_WEIGHT;
        nav_scan_feature_attempt.failure_value = after_weight;
        nav_scan_feature_capture_lines(
            final_counts, final_frame_counts, line_count);
        return 0u;
    }
    if (!nav_scan_feature_geometry_valid(
            lines, line_count, final_counts, final_weights,
            final_min, final_max, final_frame_counts,
            final_run_frame_counts, final_clean_points, feature_type))
    {
        return 0u;
    }
    if (sqrtf(after_sum / after_weight) > NAV_SCAN_MATCH_MAX_RMS_MM)
    {
        nav_scan_feature_attempt.detail_flags |=
            TELEMETRY_LOCALIZATION_DETAIL_RMS;
        nav_scan_feature_attempt.failure_value =
            sqrtf(after_sum / after_weight);
        return 0u;
    }
    if (sqrtf(total_shift_x * total_shift_x +
              total_shift_y * total_shift_y) > NAV_SCAN_MATCH_MAX_CORRECTION_MM)
    {
        nav_scan_feature_attempt.detail_flags |=
            TELEMETRY_LOCALIZATION_DETAIL_TRANSLATION;
        nav_scan_feature_attempt.failure_value =
            sqrtf(total_shift_x * total_shift_x +
                  total_shift_y * total_shift_y);
        return 0u;
    }
    if (fabsf(total_yaw) > NAV_SCAN_MATCH_MAX_CORRECTION_YAW_DEG *
                            nav_rad_per_deg)
    {
        nav_scan_feature_attempt.detail_flags |=
            TELEMETRY_LOCALIZATION_DETAIL_YAW;
        nav_scan_feature_attempt.failure_value =
            fabsf(total_yaw) * nav_deg_per_rad;
        return 0u;
    }
    *anchor_x_mm = anchor_x;
    *anchor_y_mm = anchor_y;
    *shift_x_mm = total_shift_x;
    *shift_y_mm = total_shift_y;
    *yaw_correction_rad = total_yaw;
    *rms_before_mm = sqrtf(before_sum / before_weight);
    *rms_after_mm = sqrtf(after_sum / after_weight);
    *inlier_count = after_count;
    return 1u;
}

static void nav_scan_match_segment_indices_for(
    uint16 replay_index,
    uint16 *speed_seg_idx,
    uint16 *acceleration_seg_idx)
{
    uint16 speed_index = nav.speed_seg_idx;
    uint16 acceleration_index = nav.acceleration_seg_idx;

    if ((0u == nav.speed_seg_cnt) || (NULL == nav.speed_end_p))
    {
        speed_index = 0u;
    }
    else
    {
        if (speed_index >= nav.speed_seg_cnt)
        {
            speed_index = (uint16)(nav.speed_seg_cnt - 1u);
        }
        while ((speed_index > 0u) &&
               (replay_index <= nav.speed_end_p[speed_index - 1u]))
        {
            speed_index--;
        }
        while (((uint16)(speed_index + 1u) < nav.speed_seg_cnt) &&
               (replay_index > nav.speed_end_p[speed_index]))
        {
            speed_index++;
        }
    }
    if ((0u == nav.acceleration_seg_cnt) ||
        (NULL == nav.acceleration_end_p))
    {
        acceleration_index = 0u;
    }
    else
    {
        if (acceleration_index >= nav.acceleration_seg_cnt)
        {
            acceleration_index = (uint16)(nav.acceleration_seg_cnt - 1u);
        }
        while ((acceleration_index > 0u) &&
               (replay_index <=
                nav.acceleration_end_p[acceleration_index - 1u]))
        {
            acceleration_index--;
        }
        while (((uint16)(acceleration_index + 1u) <
                nav.acceleration_seg_cnt) &&
               (replay_index >
                nav.acceleration_end_p[acceleration_index]))
        {
            acceleration_index++;
        }
    }
    if (NULL != speed_seg_idx)
    {
        *speed_seg_idx = speed_index;
    }
    if (NULL != acceleration_seg_idx)
    {
        *acceleration_seg_idx = acceleration_index;
    }
}

static uint8 nav_scan_transform_equal(const NavMapFromOdom *left,
                                      const volatile NavMapFromOdom *right)
{
    return (uint8)((left->tx_mm == right->tx_mm) &&
                   (left->ty_mm == right->ty_mm) &&
                   (left->yaw_rad == right->yaw_rad));
}

/* A raw-pose generation normally changes together with every odometry
 * integration.  Compare the payload as well: a start/stop reset or a future
 * producer change must never let a projection calculated from one pose be
 * committed alongside another pose merely because the generation counter was
 * reused.  The caller performs this comparison while interrupts are masked,
 * so the volatile right-hand side is coherent. */
static uint8 nav_scan_raw_pose_equal(const DriveOdometryPose *left,
                                     const volatile DriveOdometryPose *right)
{
    return (uint8)((left->x_mm == right->x_mm) &&
                   (left->y_mm == right->y_mm) &&
                   (left->yaw_rad == right->yaw_rad) &&
                   (left->linear_speed_mmps == right->linear_speed_mmps) &&
                   (left->yaw_rate_radps == right->yaw_rate_radps) &&
                   (left->gyro_heading_rad == right->gyro_heading_rad) &&
                   (left->gyro_heading_initialized ==
                    right->gyro_heading_initialized));
}

static void nav_scan_record_localization(
    uint8 status,
    uint8 reject_reason,
    uint8 match_type,
    uint16 vertex_index,
    uint16 localization_update_seq,
    uint32 transaction_timestamp_ms,
    uint32 capture_frame_seq,
    float evidence_travel_mm,
    const NavMapFromOdom *transform_before,
    float delta_x_mm,
    float delta_y_mm,
    float delta_yaw_rad,
    const DriveOdometryPose *raw_pose,
    float progress_before,
    float progress_after,
    float rms_before_mm,
    float rms_after_mm,
    uint16 inlier_count)
{
    TelemetryLocalizationRecord record;

    memset(&record, 0, sizeof(record));
    record.timestamp_ms = transaction_timestamp_ms;
    record.capture_frame_seq = capture_frame_seq;
    record.travel_distance_mm = telemetry_float_to_u16(
        evidence_travel_mm, 1.0f);
    record.localization_update_seq = localization_update_seq;
    record.route_feature_occurrence = (uint8)vertex_index;
    record.match_type = match_type;
    record.t_before_tx_mm = telemetry_float_to_i16(
        transform_before->tx_mm, 1.0f);
    record.t_before_ty_mm = telemetry_float_to_i16(
        transform_before->ty_mm, 1.0f);
    record.t_before_yaw_cdeg = telemetry_yaw_to_cdeg(
        transform_before->yaw_rad * 57.29577951f);
    record.delta_odom_dx_mm = telemetry_float_to_i16(delta_x_mm, 1.0f);
    record.delta_odom_dy_mm = telemetry_float_to_i16(delta_y_mm, 1.0f);
    record.delta_odom_dyaw_cdeg = telemetry_yaw_to_cdeg(
        delta_yaw_rad * 57.29577951f);
    record.raw_pose_x_mm = telemetry_float_to_i16(raw_pose->x_mm, 1.0f);
    record.raw_pose_y_mm = telemetry_float_to_i16(raw_pose->y_mm, 1.0f);
    record.raw_pose_yaw_cdeg = telemetry_yaw_to_cdeg(
        nav_wrap_rad(raw_pose->yaw_rad) * 57.29577951f);
    record.progress_before_quarter_points = telemetry_float_to_u16(
        progress_before, 4.0f);
    record.progress_after_quarter_points = (progress_after < 0.0f) ?
        0xFFFFu : telemetry_float_to_u16(progress_after, 4.0f);
    record.rms_before_mm_x10 = telemetry_float_to_i16(rms_before_mm, 10.0f);
    record.rms_after_mm_x10 = telemetry_float_to_i16(rms_after_mm, 10.0f);
    record.inlier_count = inlier_count;
    record.status = status;
    record.reject_reason = reject_reason;
    telemetry_record_localization(&record);
}

static void nav_scan_record_feature_diagnostics_ex(
    uint8 accepted,
    uint8 partial_feature,
    uint8 reason,
    uint8 match_type,
    uint16 vertex_index,
    uint32 transaction_timestamp_ms,
    float observed_progress_points)
{
    TelemetryLocalizationFeatureRecord record;
    uint8 index;

    if (((NAV_SCAN_MATCH_TYPE_CORNER != match_type) &&
         (NAV_SCAN_MATCH_TYPE_JUNCTION != match_type)) ||
        (vertex_index >= nav.scan_vertex_count) ||
        (NULL == nav.scan_vertex_type) ||
        (NULL == nav.scan_vertex_progress))
    {
        return;
    }

    memset(&record, 0, sizeof(record));
    record.route_feature_occurrence = (uint8)vertex_index;
    record.status_flags = accepted ?
        TELEMETRY_LOCALIZATION_FEATURE_ACCEPTED : 0u;
    if (accepted && partial_feature)
    {
        record.status_flags |= TELEMETRY_LOCALIZATION_FEATURE_PARTIAL;
    }
    record.last_reject_reason = reason;
    if ((reason > 0u) && (reason < 32u))
    {
        record.reject_reason_mask = (uint32)1u << reason;
    }
    record.detail_flags = nav_scan_feature_attempt.detail_flags;
    switch (reason)
    {
        case NAV_SCAN_MATCH_REJECT_AMBIGUOUS_FEATURE:
            record.detail_flags |= TELEMETRY_LOCALIZATION_DETAIL_AMBIGUOUS;
            break;
        case NAV_SCAN_MATCH_REJECT_WAIT_CONFIRMATION:
            if (0u == (record.detail_flags &
                       TELEMETRY_LOCALIZATION_DETAIL_CONFIRMATION_INCONSISTENT))
            {
                record.detail_flags |=
                    TELEMETRY_LOCALIZATION_DETAIL_CONFIRMATION_WAIT;
            }
            break;
        case NAV_SCAN_MATCH_REJECT_NO_FRESH_EVIDENCE:
            record.detail_flags |=
                TELEMETRY_LOCALIZATION_DETAIL_NO_FRESH_EVIDENCE;
            break;
        case NAV_SCAN_MATCH_REJECT_NO_IMPROVEMENT:
            record.detail_flags |=
                TELEMETRY_LOCALIZATION_DETAIL_NO_IMPROVEMENT;
            break;
        case NAV_SCAN_MATCH_REJECT_PROJECTION:
            record.detail_flags |= TELEMETRY_LOCALIZATION_DETAIL_PROJECTION;
            break;
        case NAV_SCAN_MATCH_REJECT_COMMIT_BUSY:
            record.detail_flags |= TELEMETRY_LOCALIZATION_DETAIL_COMMIT_BUSY;
            break;
        case NAV_SCAN_MATCH_REJECT_FEATURE_ORDER:
            record.detail_flags |=
                TELEMETRY_LOCALIZATION_DETAIL_FEATURE_ORDER;
            break;
        case NAV_SCAN_MATCH_REJECT_FEATURE_PROGRESS:
            record.detail_flags |=
                TELEMETRY_LOCALIZATION_DETAIL_FEATURE_PROGRESS;
            break;
        case NAV_SCAN_MATCH_REJECT_PROGRESS_REGRESSION:
            record.detail_flags |=
                TELEMETRY_LOCALIZATION_DETAIL_PROGRESS_REGRESSION;
            break;
        case NAV_SCAN_MATCH_REJECT_FEATURE_CONSUMED:
            record.detail_flags |=
                TELEMETRY_LOCALIZATION_DETAIL_FEATURE_CONSUMED;
            break;
        default:
            break;
    }
    record.attempt_count = 1u;
    record.last_timestamp_ms = telemetry_relative_timestamp_ms(
        transaction_timestamp_ms);
    record.observed_progress_points = telemetry_float_to_u16(
        observed_progress_points, 1.0f);
    for (index = 0u; index < 3u; index++)
    {
        record.line_point_count[index] = nav_scan_u16_to_u8_saturated(
            nav_scan_feature_attempt.line_point_count[index]);
        record.line_frame_count[index] = nav_scan_u16_to_u8_saturated(
            nav_scan_feature_attempt.line_frame_count[index]);
    }
    record.failure_value_x10 = telemetry_float_to_i16(
        nav_scan_feature_attempt.failure_value, 10.0f);
    telemetry_record_localization_feature(&record);
}

static void nav_scan_record_feature_diagnostics(
    uint8 accepted,
    uint8 reason,
    uint8 match_type,
    uint16 vertex_index,
    uint32 transaction_timestamp_ms,
    float observed_progress_points)
{
    nav_scan_record_feature_diagnostics_ex(
        accepted, 0u, reason, match_type, vertex_index,
        transaction_timestamp_ms, observed_progress_points);
}

static void nav_scan_match_record_reject_detail(
    uint8 reason,
    uint8 match_type,
    uint16 vertex_index,
    float anchor_x_mm,
    float anchor_y_mm,
    float shift_x_mm,
    float shift_y_mm,
    float yaw_correction_rad,
    float rms_before_mm,
    float rms_after_mm,
    uint16 inlier_count)
{
    NavMapFromOdom transform_before;
    NavMapFromOdom transform_after;
    DriveOdometryPose raw_pose;
    DriveOdometryPose map_pose_after;
    float progress_before;
    float progress_after;
    float delta_x_mm;
    float delta_y_mm;
    uint16 localization_update_seq;
    uint32 timestamp_ms;
    uint32 capture_frame_seq;
    float evidence_travel_mm;
    uint8 have_candidate;
    uint8 record_structural_straight = 0u;
    uint32 interrupt_state;

    /* Rejected attempts need the same immutable anchors as accepted ones.  A
     * rejection does not change T_map_from_odom, but the candidate transform
     * and the hypothetical projected progress remain useful for diagnosing
     * why a solver was refused. */
    interrupt_state = interrupt_global_disable();
    transform_before = nav.map_from_odom;
    raw_pose = nav.raw_odometry;
    progress_before = nav.progress_points;
    localization_update_seq = nav.localization_update_seq;
    timestamp_ms = vision_clock_ms;
    capture_frame_seq = nav_scan_match.current_evidence_frame_seq;
    evidence_travel_mm = nav_scan_match.current_evidence_travel_mm;
    interrupt_global_enable(interrupt_state);

    nav_scan_delta_translation(anchor_x_mm, anchor_y_mm,
                               shift_x_mm, shift_y_mm,
                               yaw_correction_rad,
                               &delta_x_mm, &delta_y_mm);
    have_candidate = (uint8)((NAV_SCAN_MATCH_TYPE_NONE != match_type) &&
                             ((inlier_count > 0u) ||
                              (fabsf(delta_x_mm) > 1.0e-3f) ||
                              (fabsf(delta_y_mm) > 1.0e-3f) ||
                              (fabsf(yaw_correction_rad) > 1.0e-6f) ||
                              (fabsf(rms_before_mm) > 1.0e-3f) ||
                              (fabsf(rms_after_mm) > 1.0e-3f)));
    if ((NAV_SCAN_MATCH_TYPE_STRAIGHT == match_type) &&
        (reason >= NAV_SCAN_MATCH_REJECT_STRAIGHT_NO_MAP_LINE) &&
        (reason <= NAV_SCAN_MATCH_REJECT_STRAIGHT_CALIBRATION_BLOCKED) &&
        (!nav_scan_match.have_straight_diagnostic ||
         (reason != nav_scan_match.last_straight_diagnostic_reason) ||
         ((evidence_travel_mm -
           nav_scan_match.last_straight_diagnostic_travel_mm) >=
          NAV_SCAN_MATCH_STRAIGHT_DIAGNOSTIC_INTERVAL_MM)))
    {
        record_structural_straight = 1u;
        nav_scan_match.last_straight_diagnostic_reason = reason;
        nav_scan_match.last_straight_diagnostic_travel_mm =
            evidence_travel_mm;
        nav_scan_match.have_straight_diagnostic = 1u;
    }
    progress_after = progress_before;
    if (have_candidate || record_structural_straight)
    {
        nav_compose_map_from_odom(&transform_before,
                                  anchor_x_mm, anchor_y_mm,
                                  shift_x_mm, shift_y_mm,
                                  yaw_correction_rad,
                                  &transform_after);
        nav_raw_pose_to_map_with(&transform_after, &raw_pose,
                                 &map_pose_after);
        if (!nav_scan_project_corrected_progress(
                progress_before, map_pose_after.x_mm, map_pose_after.y_mm,
                &progress_after))
        {
            progress_after = -1.0f;
        }
    }

    nav_scan_match.last_reject_reason = reason;
    /* The localization ledger is a transaction ledger, not a 100 Hz solver
     * heartbeat.  Keep every attempt that produced a candidate.  Structural
     * straight failures have no candidate transform, but their precise stage
     * is needed for tuning, so retain the first/change and then at most one
     * sample per 100 mm.  This preserves coverage without allowing a long
     * no-line interval to overwrite the 512-entry ledger. */
    if (have_candidate)
    {
        nav_scan_record_localization(
            TELEMETRY_LOCALIZATION_STATUS_REJECTED, reason, match_type,
            vertex_index, localization_update_seq, timestamp_ms,
            capture_frame_seq, evidence_travel_mm,
            &transform_before, delta_x_mm, delta_y_mm, yaw_correction_rad,
            &raw_pose, progress_before, progress_after,
            rms_before_mm, rms_after_mm, inlier_count);
    }
    nav_scan_record_feature_diagnostics(
        0u, reason, match_type, vertex_index,
        timestamp_ms, progress_before);
}

static void nav_scan_match_record_reject(uint8 reason, uint16 vertex_index)
{
    nav_scan_match_record_reject_detail(
        reason, NAV_SCAN_MATCH_TYPE_NONE, vertex_index,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0u);
}

static uint8 nav_scan_project_corrected_progress(float current_progress_points,
                                                   float x_mm,
                                                   float y_mm,
                                                   float *progress_points)
{
    RouteProjectionResult projection;

    if ((NULL == progress_points) ||
        !nav_scan_project_corrected_pose(current_progress_points,
                                         x_mm, y_mm, &projection))
    {
        return 0u;
    }
    *progress_points = projection.progress_points;
    return 1u;
}

static uint8 nav_scan_project_corrected_pose(float current_progress_points,
                                             float x_mm,
                                             float y_mm,
                                             RouteProjectionResult *projection)
{
    uint16 start_index;

    if ((NULL == projection) || (nav.point_count < 2u))
    {
        return 0u;
    }
    start_index = (current_progress_points >
                   (float)NAV_SCAN_MATCH_PROGRESS_BEHIND_POINTS) ?
                  (uint16)(current_progress_points -
                           (float)NAV_SCAN_MATCH_PROGRESS_BEHIND_POINTS) : 0u;
    if (!route_projection_project_forward(
            nav.xs, nav.ys, nav.point_count, start_index,
            NAV_SCAN_MATCH_PROGRESS_SEARCH_POINTS,
            x_mm, y_mm, NAV_PATH_SAMPLE_MM, projection))
    {
        return 0u;
    }
    return 1u;
}

static uint8 nav_scan_project_corrected_pose_near_hint(
    float hint_progress_points,
    float x_mm,
    float y_mm,
    RouteProjectionResult *projection)
{
    uint16 start_index;

    if ((NULL == projection) || (nav.point_count < 2u))
    {
        return 0u;
    }
    start_index = (hint_progress_points >
                   (float)NAV_SCAN_COMMIT_HINT_BEHIND_POINTS) ?
                  (uint16)(hint_progress_points -
                           (float)NAV_SCAN_COMMIT_HINT_BEHIND_POINTS) : 0u;
    return route_projection_project_forward(
        nav.xs, nav.ys, nav.point_count, start_index,
        (uint16)(NAV_SCAN_COMMIT_HINT_SEARCH_POINTS - 1u),
        x_mm, y_mm, NAV_PATH_SAMPLE_MM, projection);
}

/* Check the candidate against the same smooth route that supplies the
 * steering target.  The physical feature solver can legitimately reduce its
 * point-to-line RMS while selecting a branch that is incompatible with the
 * route.  In that case accepting the transform would immediately inject a
 * large lateral jump into the active curvature/Frenet tracker.  The ordinary
 * straight observer keeps a tight tolerance, while a complete corner/junction
 * gets a bounded allowance for physical-vs-smoothed corner offset; the hard
 * 120 mm ceiling still catches a clearly false association. */
static uint8 nav_scan_correction_path_error_is_valid(
    float progress_before,
    const DriveOdometryPose *map_pose_before,
    const RouteProjectionResult *candidate_projection,
    uint8 near_hint_only,
    uint8 complete_feature)
{
    RouteProjectionResult before_projection;
    uint8 projected;
    float before_abs;
    float after_abs;
    float cross_track_tolerance = complete_feature ?
        NAV_SCAN_MATCH_COMPLETE_FEATURE_PATH_CROSS_TRACK_TOLERANCE_MM :
        NAV_SCAN_MATCH_PATH_CROSS_TRACK_TOLERANCE_MM;

    if ((NULL == map_pose_before) || (NULL == candidate_projection))
    {
        return 0u;
    }
    if (near_hint_only)
    {
        projected = nav_scan_project_corrected_pose_near_hint(
            progress_before, map_pose_before->x_mm, map_pose_before->y_mm,
            &before_projection);
    }
    else
    {
        projected = nav_scan_project_corrected_pose(
            progress_before, map_pose_before->x_mm, map_pose_before->y_mm,
            &before_projection);
    }
    /* A missing before-projection means the route-consistency transaction is
     * incomplete.  Fail closed instead of silently accepting a correction
     * whose effect on the steering path cannot be checked. */
    if (!projected)
    {
        return 0u;
    }
    before_abs = fabsf(before_projection.signed_cross_track_mm);
    after_abs = fabsf(candidate_projection->signed_cross_track_mm);
    return (uint8)((after_abs <= NAV_SCAN_MATCH_MAX_PATH_CROSS_TRACK_AFTER_MM) &&
                   (after_abs <=
                    before_abs + cross_track_tolerance));
}

static uint8 nav_scan_feature_is_localized(uint16 vertex_index)
{
    uint16 word = (uint16)(vertex_index / 32u);
    uint8 bit = (uint8)(vertex_index % 32u);

    if (word >= NAV_SCAN_MATCH_FEATURE_WORD_COUNT)
    {
        return 1u;
    }
    return (uint8)((nav_scan_match.localized_feature_bits[word] >> bit) & 1u);
}

static void nav_scan_mark_feature_localized(uint16 vertex_index)
{
    uint16 word = (uint16)(vertex_index / 32u);
    uint8 bit = (uint8)(vertex_index % 32u);

    if (word < NAV_SCAN_MATCH_FEATURE_WORD_COUNT)
    {
        nav_scan_match.localized_feature_bits[word] |= ((uint32)1u << bit);
    }
}

static float nav_scan_feature_boundary_progress_points(uint16 vertex_index)
{
    float feature_progress;
    float boundary_progress;
    uint16 index;

    if ((vertex_index >= nav.scan_vertex_count) ||
        (NULL == nav.scan_vertex_progress) ||
        (NULL == nav.scan_vertex_type))
    {
        return 1.0e30f;
    }
    feature_progress = (float)nav.scan_vertex_progress[vertex_index];
    boundary_progress = feature_progress +
        NAV_SCAN_MATCH_FEATURE_BOUNDARY_FALLBACK_MM / NAV_PATH_SAMPLE_MM;
    for (index = (uint16)(vertex_index + 1u);
         index < nav.scan_vertex_count; index++)
    {
        uint8 type = nav.scan_vertex_type[index];
        float next_progress = (float)nav.scan_vertex_progress[index];

        if (((NAV_SCAN_VERTEX_TYPE_CORNER == type) ||
             (NAV_SCAN_VERTEX_TYPE_JUNCTION == type)) &&
            (next_progress > feature_progress))
        {
            boundary_progress = 0.5f * (feature_progress + next_progress);
            break;
        }
    }
    if ((nav.point_count > 0u) &&
        (boundary_progress > (float)(nav.point_count - 1u)))
    {
        boundary_progress = (float)(nav.point_count - 1u);
    }
    return boundary_progress;
}

static void nav_scan_track_feature_evidence_boundary(uint16 vertex_index)
{
    if ((vertex_index >= nav.scan_vertex_count) ||
        (NULL == nav.scan_vertex_progress) ||
        (NULL == nav.scan_vertex_type) ||
        nav_scan_feature_is_before_watermark(vertex_index))
    {
        return;
    }
    if (nav_scan_match.have_pending_feature_boundary)
    {
        return;
    }
    nav_scan_match.pending_feature_boundary_index = vertex_index;
    nav_scan_match.pending_feature_boundary_progress =
        nav_scan_feature_boundary_progress_points(vertex_index);
    nav_scan_match.have_pending_feature_boundary = 1u;
}

static void nav_scan_track_next_route_feature(float sensor_progress_points)
{
    uint16 index;
    float approach_limit_points =
        NAV_SCAN_MATCH_FEATURE_RADIUS_MM / NAV_PATH_SAMPLE_MM;

    if (nav_scan_match.have_pending_feature_boundary ||
        (NULL == nav.scan_vertex_progress) ||
        (NULL == nav.scan_vertex_type))
    {
        return;
    }
    for (index = 0u; index < nav.scan_vertex_count; index++)
    {
        uint8 type = nav.scan_vertex_type[index];
        float feature_progress = (float)nav.scan_vertex_progress[index];

        if (((NAV_SCAN_VERTEX_TYPE_CORNER != type) &&
             (NAV_SCAN_VERTEX_TYPE_JUNCTION != type)) ||
            nav_scan_feature_is_localized(index) ||
            nav_scan_feature_is_before_watermark(index))
        {
            continue;
        }
        if (feature_progress <= sensor_progress_points + approach_limit_points)
        {
            nav_scan_track_feature_evidence_boundary(index);
        }
        break;
    }
}

static void nav_scan_advance_feature_evidence_boundary(
    float sensor_progress_points, float boundary_travel_mm,
    uint32 latest_frame_seq)
{
    uint16 vertex_index;
    uint16 feature_progress;

    if (!nav_scan_match.have_pending_feature_boundary ||
        (sensor_progress_points <
         nav_scan_match.pending_feature_boundary_progress))
    {
        return;
    }
    vertex_index = nav_scan_match.pending_feature_boundary_index;
    if ((vertex_index >= nav.scan_vertex_count) ||
        (NULL == nav.scan_vertex_progress))
    {
        nav_scan_match.have_pending_feature_boundary = 0u;
        return;
    }
    feature_progress = nav.scan_vertex_progress[vertex_index];
    if (!nav_scan_match.have_feature_evidence_boundary ||
        (feature_progress > nav_scan_match.last_feature_evidence_progress))
    {
        nav_scan_match.feature_evidence_boundary_travel_mm = boundary_travel_mm;
        nav_scan_match.feature_evidence_boundary_frame_seq = latest_frame_seq;
        nav_scan_match.last_feature_evidence_progress = feature_progress;
        nav_scan_match.have_feature_evidence_boundary = 1u;
    }
    nav_scan_match.have_pending_feature_boundary = 0u;

    /* A corner/junction candidate cannot be confirmed using evidence from a
     * later occurrence.  The independent straight candidate remains valid;
     * its physical line-group key decides whether a later window belongs to
     * the same line. */
    if (nav_scan_match.feature_pending.valid &&
        (nav_scan_match.feature_pending.key_index < nav.scan_vertex_count) &&
        (nav.scan_vertex_progress[
             nav_scan_match.feature_pending.key_index] <= feature_progress))
    {
        nav_scan_match.feature_pending.valid = 0u;
        nav_scan_match.feature_pending.requires_strong_confirmation = 0u;
    }
}

static uint8 nav_scan_feature_is_before_watermark(uint16 vertex_index)
{
    uint16 feature_progress;

    if ((vertex_index >= nav.scan_vertex_count) ||
        (NULL == nav.scan_vertex_progress))
    {
        return 0u;
    }
    feature_progress = nav.scan_vertex_progress[vertex_index];
    if (nav_scan_match.have_localized_feature_progress &&
        (feature_progress <= nav_scan_match.last_localized_feature_progress))
    {
        return 1u;
    }
    return (uint8)(nav_scan_match.have_feature_evidence_boundary &&
                   (feature_progress <=
                    nav_scan_match.last_feature_evidence_progress));
}

static uint8 nav_scan_feature_progress_is_consistent(
    uint16 vertex_index, float corrected_progress_points)
{
    float feature_progress_points;

    if ((vertex_index >= nav.scan_vertex_count) ||
        (NULL == nav.scan_vertex_progress))
    {
        return 0u;
    }
    feature_progress_points = (float)nav.scan_vertex_progress[vertex_index];
    return (uint8)(fabsf(corrected_progress_points -
                         feature_progress_points) * NAV_PATH_SAMPLE_MM <=
                   NAV_SCAN_MATCH_FEATURE_RADIUS_MM);
}

static void nav_scan_delta_translation(float anchor_x_mm,
                                       float anchor_y_mm,
                                       float shift_x_mm,
                                       float shift_y_mm,
                                       float yaw_rad,
                                       float *tx_mm,
                                       float *ty_mm)
{
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    *tx_mm = anchor_x_mm + shift_x_mm -
             (cos_yaw * anchor_x_mm - sin_yaw * anchor_y_mm);
    *ty_mm = anchor_y_mm + shift_y_mm -
             (sin_yaw * anchor_x_mm + cos_yaw * anchor_y_mm);
}

static uint8 nav_scan_straight_correction_is_significant(
    float shift_x_mm,
    float shift_y_mm,
    float yaw_correction_rad,
    float rms_after_mm,
    uint16 frame_count,
    float span_mm,
    float tangent_information_mm2)
{
    float translation_mm;
    float translation_threshold_mm =
        NAV_SCAN_MATCH_STRAIGHT_MIN_TRANSLATION_MM;
    float yaw_threshold_rad =
        NAV_SCAN_MATCH_STRAIGHT_MIN_YAW_DEG * nav_rad_per_deg;

    /* span_mm remains part of the diagnostic call contract; the acceptance
     * gate uses the exact weighted information below instead of a span-based
     * uniform-distribution approximation. */
    (void)span_mm;

    if (frame_count > 0u)
    {
        float statistical_threshold_mm =
            NAV_SCAN_MATCH_STRAIGHT_SIGNIFICANCE_SIGMA * rms_after_mm /
            sqrtf((float)frame_count);
        if (statistical_threshold_mm > translation_threshold_mm)
        {
            translation_threshold_mm = statistical_threshold_mm;
        }
        if (tangent_information_mm2 > 1.0e-3f)
        {
            float statistical_yaw_threshold_rad = atanf(
                NAV_SCAN_MATCH_STRAIGHT_SIGNIFICANCE_SIGMA * rms_after_mm /
                sqrtf(tangent_information_mm2));
            if (statistical_yaw_threshold_rad > yaw_threshold_rad)
            {
                yaw_threshold_rad = statistical_yaw_threshold_rad;
            }
        }
        else
        {
            /* No tangential spread means yaw is unobservable from this
             * straight window; translation can still be significant. */
            yaw_threshold_rad = 0.5f * 3.14159265359f;
        }
    }
    translation_mm = sqrtf(shift_x_mm * shift_x_mm +
                            shift_y_mm * shift_y_mm);
    return (uint8)((translation_mm >= translation_threshold_mm) ||
                   (fabsf(yaw_correction_rad) >= yaw_threshold_rad));
}

static void nav_scan_apply_correction(uint8 match_type,
                                      uint16 vertex_index,
                                      float anchor_x_mm,
                                      float anchor_y_mm,
                                      float shift_x_mm,
                                      float shift_y_mm,
                                      float yaw_correction_rad,
                                      float rms_before_mm,
                                      float rms_after_mm,
                                      uint16 inlier_count,
                                      uint32 evidence_frame_seq,
                                      float evidence_travel_mm)
{
    NavScanCorrectionTransaction pending;
    NavMapFromOdom transform_before;
    NavMapFromOdom transform_after;
    DriveOdometryPose hint_raw_pose;
    DriveOdometryPose hint_map_pose_before;
    DriveOdometryPose hint_map_pose;
    RouteProjectionResult projection_hint;
    float hint_progress_before = 0.0f;
    float delta_x_mm;
    float delta_y_mm;
    float map_pose_jump_mm;
    float correction_scale = 1.0f;
    float applied_shift_x_mm = shift_x_mm;
    float applied_shift_y_mm = shift_y_mm;
    float applied_yaw_correction_rad = yaw_correction_rad;
    uint16 localization_update_seq = 0u;
    uint32 hint_raw_pose_generation;
    uint8 publish_reject_reason = NAV_SCAN_MATCH_REJECT_NONE;
    uint8 complete_feature =
        (uint8)(((NAV_SCAN_MATCH_TYPE_CORNER == match_type) ||
                 (NAV_SCAN_MATCH_TYPE_JUNCTION == match_type)));
    uint32 interrupt_state;

    if (complete_feature && nav_scan_feature_is_before_watermark(vertex_index))
    {
        nav_scan_clear_pending_candidate(match_type);
        nav_scan_match_record_reject_detail(
            NAV_SCAN_MATCH_REJECT_FEATURE_ORDER, match_type, vertex_index,
            anchor_x_mm, anchor_y_mm, shift_x_mm, shift_y_mm,
            yaw_correction_rad, rms_before_mm, rms_after_mm, inlier_count);
        return;
    }

    if (NAV_SCAN_MATCH_TYPE_STRAIGHT == match_type)
    {
        correction_scale = NAV_SCAN_MATCH_STRAIGHT_CORRECTION_BLEND;
        applied_shift_x_mm *= correction_scale;
        applied_shift_y_mm *= correction_scale;
        applied_yaw_correction_rad *= correction_scale;
    }

    nav_scan_delta_translation(anchor_x_mm, anchor_y_mm,
                               applied_shift_x_mm, applied_shift_y_mm,
                               applied_yaw_correction_rad,
                               &delta_x_mm, &delta_y_mm);

    interrupt_state = interrupt_global_disable();
    if (NAV_STATE_NAVIGATING != nav.state)
    {
        publish_reject_reason = NAV_SCAN_MATCH_REJECT_STOPPED;
    }
    else if (NAV_SCAN_CORRECTION_IDLE != nav_scan_correction.state)
    {
        publish_reject_reason = NAV_SCAN_MATCH_REJECT_COMMIT_BUSY;
    }
    else
    {
        transform_before = nav.map_from_odom;
        localization_update_seq = nav.localization_update_seq;
        hint_raw_pose = nav.raw_odometry;
        hint_progress_before = nav.progress_points;
        hint_raw_pose_generation = nav.raw_pose_generation;
    }
    interrupt_global_enable(interrupt_state);
    if (NAV_SCAN_MATCH_REJECT_NONE != publish_reject_reason)
    {
        nav_scan_clear_pending_candidate(match_type);
        nav_scan_match_record_reject_detail(
            publish_reject_reason, match_type, vertex_index,
            anchor_x_mm, anchor_y_mm,
            applied_shift_x_mm, applied_shift_y_mm,
            applied_yaw_correction_rad,
            rms_before_mm, rms_after_mm, inlier_count);
        return;
    }

    nav_compose_map_from_odom(&transform_before,
                              anchor_x_mm, anchor_y_mm,
                              applied_shift_x_mm, applied_shift_y_mm,
                              applied_yaw_correction_rad,
                              &transform_after);
    nav_raw_pose_to_map_with(&transform_before, &hint_raw_pose,
                             &hint_map_pose_before);
    nav_raw_pose_to_map_with(&transform_after, &hint_raw_pose,
                             &hint_map_pose);
    {
        float pose_dx = hint_map_pose.x_mm - hint_map_pose_before.x_mm;
        float pose_dy = hint_map_pose.y_mm - hint_map_pose_before.y_mm;
        map_pose_jump_mm = sqrtf(pose_dx * pose_dx + pose_dy * pose_dy);
    }
    /* The transform-origin translation can be large solely because a small
     * yaw correction is applied far from odom origin.  Reject only an
     * physically excessive vehicle-pose innovation here; no magnitude-based
     * confirmation delay is applied before the transaction is published. */
    if ((NAV_SCAN_MATCH_TYPE_STRAIGHT == match_type) &&
        (map_pose_jump_mm > NAV_SCAN_MATCH_STRAIGHT_MAX_POSE_STEP_MM))
    {
        /* Include the lever-arm effect of yaw at a distant anchor when
         * limiting the vehicle-pose innovation. */
        correction_scale *= NAV_SCAN_MATCH_STRAIGHT_MAX_POSE_STEP_MM /
                            map_pose_jump_mm;
        applied_shift_x_mm = shift_x_mm * correction_scale;
        applied_shift_y_mm = shift_y_mm * correction_scale;
        applied_yaw_correction_rad = yaw_correction_rad * correction_scale;
        nav_scan_delta_translation(anchor_x_mm, anchor_y_mm,
                                   applied_shift_x_mm, applied_shift_y_mm,
                                   applied_yaw_correction_rad,
                                   &delta_x_mm, &delta_y_mm);
        nav_compose_map_from_odom(&transform_before,
                                  anchor_x_mm, anchor_y_mm,
                                  applied_shift_x_mm, applied_shift_y_mm,
                                  applied_yaw_correction_rad,
                                  &transform_after);
        nav_raw_pose_to_map_with(&transform_after, &hint_raw_pose,
                                 &hint_map_pose);
        {
            float pose_dx = hint_map_pose.x_mm - hint_map_pose_before.x_mm;
            float pose_dy = hint_map_pose.y_mm - hint_map_pose_before.y_mm;
            map_pose_jump_mm = sqrtf(pose_dx * pose_dx + pose_dy * pose_dy);
        }
    }
    if (((NAV_SCAN_MATCH_TYPE_STRAIGHT == match_type) &&
         (map_pose_jump_mm > NAV_SCAN_MATCH_STRAIGHT_MAX_POSE_STEP_MM)) ||
        (map_pose_jump_mm > NAV_SCAN_MATCH_MAX_CORRECTION_MM) ||
        (fabsf(applied_yaw_correction_rad) >
         NAV_SCAN_MATCH_MAX_CORRECTION_YAW_DEG * nav_rad_per_deg) ||
        (complete_feature &&
         (fabsf(applied_yaw_correction_rad) >
          NAV_SCAN_MATCH_COMPLETE_FEATURE_MAX_YAW_DEG * nav_rad_per_deg)))
    {
        nav_scan_clear_pending_candidate(match_type);
        nav_scan_feature_attempt.detail_flags |=
            TELEMETRY_LOCALIZATION_DETAIL_TRANSLATION;
        nav_scan_match_record_reject_detail(
            NAV_SCAN_MATCH_REJECT_CORRECTION, match_type, vertex_index,
            anchor_x_mm, anchor_y_mm,
            applied_shift_x_mm, applied_shift_y_mm,
            applied_yaw_correction_rad, rms_before_mm, rms_after_mm,
            inlier_count);
        return;
    }
    if (!nav_scan_project_corrected_pose(
            hint_progress_before, hint_map_pose.x_mm, hint_map_pose.y_mm,
            &projection_hint))
    {
        nav_scan_clear_pending_candidate(match_type);
        nav_scan_match_record_reject_detail(
            NAV_SCAN_MATCH_REJECT_PROJECTION, match_type, vertex_index,
            anchor_x_mm, anchor_y_mm,
            applied_shift_x_mm, applied_shift_y_mm,
            applied_yaw_correction_rad, rms_before_mm, rms_after_mm,
            inlier_count);
        return;
    }
    projection_hint.progress_points = nav_scan_match_clamp(
        projection_hint.progress_points, 0.0f,
        (float)(nav.point_count - 1u));
    if (!nav_scan_correction_path_error_is_valid(
            hint_progress_before, &hint_map_pose_before, &projection_hint,
            0u, complete_feature))
    {
        nav_scan_clear_pending_candidate(match_type);
        nav_scan_feature_attempt.detail_flags |=
            TELEMETRY_LOCALIZATION_DETAIL_PROJECTION;
        nav_scan_match_record_reject_detail(
            NAV_SCAN_MATCH_REJECT_PATH_ERROR, match_type, vertex_index,
            anchor_x_mm, anchor_y_mm,
            applied_shift_x_mm, applied_shift_y_mm,
            applied_yaw_correction_rad,
            rms_before_mm, rms_after_mm, inlier_count);
        return;
    }
    /* A candidate that has passed the point-cloud geometry, RMS and route
     * projection gates is actionable immediately.  Do not defer a large
     * translation/yaw correction to a second travel window: that delay was
     * allowing a real offset to persist until the vehicle had already passed
     * the correction point.  The correction transaction below remains
     * atomic, and its final pose/path bounds still reject impossible jumps. */
    nav_scan_clear_pending_candidate(match_type);
    memset(&pending, 0, sizeof(pending));
    pending.match_type = match_type;
    pending.complete_feature = complete_feature;
    pending.partial_feature = 0u;
    pending.vertex_index = vertex_index;
    pending.inlier_count = inlier_count;
    pending.localization_update_seq = localization_update_seq;
    pending.evidence_frame_seq = evidence_frame_seq;
    pending.evidence_travel_mm = evidence_travel_mm;
    pending.transform_before = transform_before;
    pending.transform_after = transform_after;
    pending.projection_hint = projection_hint;
    pending.projection_hint_source_progress = hint_progress_before;
    pending.raw_pose_generation_hint = hint_raw_pose_generation;
    pending.delta_x_mm = delta_x_mm;
    pending.delta_y_mm = delta_y_mm;
    pending.delta_yaw_rad = applied_yaw_correction_rad;
    pending.rms_before_mm = rms_before_mm;
    pending.rms_after_mm = rms_after_mm;

    /* Publish the fully-built request with state last.  The wide projection
     * above is only a search hint: the next control tick rereads its current
     * raw pose/progress and recomputes the exact projection near that hint. */
    interrupt_state = interrupt_global_disable();
    if ((NAV_STATE_NAVIGATING != nav.state) ||
        (NAV_SCAN_CORRECTION_IDLE != nav_scan_correction.state) ||
        !nav_scan_transform_equal(&transform_before, &nav.map_from_odom) ||
        (localization_update_seq != nav.localization_update_seq))
    {
        interrupt_global_enable(interrupt_state);
        nav_scan_clear_pending_candidate(match_type);
        nav_scan_match_record_reject_detail(
            NAV_SCAN_MATCH_REJECT_COMMIT_BUSY, match_type, vertex_index,
            anchor_x_mm, anchor_y_mm,
            applied_shift_x_mm, applied_shift_y_mm,
            applied_yaw_correction_rad,
            rms_before_mm, rms_after_mm, inlier_count);
        return;
    }
    nav_scan_correction = pending;
    __dsync();
    nav_scan_correction.state = NAV_SCAN_CORRECTION_PENDING;
    interrupt_global_enable(interrupt_state);
}

static uint8 nav_scan_correction_service_result(void)
{
    NavScanCorrectionTransaction completed;
    uint32 interrupt_state;

    interrupt_state = interrupt_global_disable();
    if (NAV_SCAN_CORRECTION_PENDING == nav_scan_correction.state)
    {
        interrupt_global_enable(interrupt_state);
        return 1u;
    }
    if (NAV_SCAN_CORRECTION_COMPLETE != nav_scan_correction.state)
    {
        interrupt_global_enable(interrupt_state);
        return 0u;
    }
    completed = nav_scan_correction;
    nav_scan_correction.state = NAV_SCAN_CORRECTION_IDLE;
    interrupt_global_enable(interrupt_state);

    if (TELEMETRY_LOCALIZATION_STATUS_ACCEPTED == completed.status)
    {
        nav_scan_match.last_match_ms = completed.commit_timestamp_ms;
        nav_scan_match.last_match_progress_points = completed.progress_after;
        nav_scan_match.last_rms_before_mm = completed.rms_before_mm;
        nav_scan_match.last_rms_after_mm = completed.rms_after_mm;
        nav_scan_match.last_yaw_correction_rad = completed.delta_yaw_rad;
        nav_scan_match.last_vertex_index = completed.vertex_index;
        nav_scan_match.last_inlier_count = completed.inlier_count;
        nav_scan_match.last_match_type = completed.match_type;
        nav_scan_match.last_reject_reason = NAV_SCAN_MATCH_REJECT_NONE;
        nav_scan_match.valid = 1u;
        nav_scan_match.feature_localized = completed.complete_feature;
        nav_scan_match.last_consumed_frame_seq = completed.evidence_frame_seq;
        nav_scan_match.last_consumed_travel_mm = completed.evidence_travel_mm;
        nav_scan_match.have_consumed_evidence = 1u;
        nav_scan_match.have_straight_diagnostic = 0u;
        /* Any accepted transform changes the coordinate frame underlying
         * both candidate estimates.  Neither chain may reuse its old hint. */
        nav_scan_clear_all_pending_candidates();
        if (completed.complete_feature)
        {
            nav_scan_mark_feature_localized(completed.vertex_index);
            nav_scan_match.last_feature_consumed_frame_seq =
                completed.evidence_frame_seq;
            nav_scan_match.last_feature_consumed_travel_mm =
                completed.evidence_travel_mm;
            nav_scan_match.have_feature_consumed_evidence = 1u;
            nav_scan_match.last_localized_feature_progress =
                nav.scan_vertex_progress[completed.vertex_index];
            nav_scan_match.have_localized_feature_progress = 1u;
            if (nav_scan_match.have_pending_feature_boundary &&
                (nav_scan_match.pending_feature_boundary_index ==
                 completed.vertex_index))
            {
                nav_scan_match.have_pending_feature_boundary = 0u;
            }
        }
    }
    else
    {
        nav_scan_match.last_reject_reason = completed.reject_reason;
        /* A pending candidate is only a legacy telemetry hint, never a
         * durable correction request.  Once the control ISR has closed the
         * transaction (including commit-busy, projection, correction-bound,
         * or stopped rejects), clear it so stale evidence cannot be retried
         * over later frames. */
        nav_scan_clear_pending_candidate(completed.match_type);
    }

    nav_scan_record_localization(
        completed.status, completed.reject_reason,
        completed.match_type, completed.vertex_index,
        completed.localization_update_seq, completed.commit_timestamp_ms,
        completed.evidence_frame_seq, completed.evidence_travel_mm,
        &completed.transform_before,
        completed.delta_x_mm, completed.delta_y_mm, completed.delta_yaw_rad,
        &completed.raw_pose,
        completed.progress_before, completed.progress_after,
        completed.rms_before_mm, completed.rms_after_mm,
        completed.inlier_count);
    nav_scan_record_feature_diagnostics_ex(
        (uint8)(TELEMETRY_LOCALIZATION_STATUS_ACCEPTED == completed.status),
        completed.partial_feature, completed.reject_reason,
        completed.match_type, completed.vertex_index,
        completed.commit_timestamp_ms,
        (completed.progress_after >= 0.0f) ? completed.progress_after :
                                            completed.progress_before);
    return 1u;
}

static uint8 nav_scan_compute_pending_projection(
    const DriveOdometryPose *raw_pose,
    float progress_before,
    DriveOdometryPose *candidate_map_pose,
    RouteProjectionResult *candidate_projection,
    uint8 *reject_reason)
{
    float hint_progress_points;

    if ((NULL == raw_pose) || (NULL == candidate_map_pose) ||
        (NULL == candidate_projection) || (NULL == reject_reason))
    {
        return 0u;
    }
    nav_raw_pose_to_map_with(&nav_scan_correction.transform_after,
                             raw_pose, candidate_map_pose);
    hint_progress_points = nav_scan_correction.projection_hint.progress_points;
    if (progress_before > nav_scan_correction.projection_hint_source_progress)
    {
        hint_progress_points += progress_before -
            nav_scan_correction.projection_hint_source_progress;
    }
    if (!nav_scan_project_corrected_pose_near_hint(
            hint_progress_points, candidate_map_pose->x_mm,
            candidate_map_pose->y_mm, candidate_projection))
    {
        *reject_reason = NAV_SCAN_MATCH_REJECT_PROJECTION;
        return 0u;
    }
    candidate_projection->progress_points = nav_scan_match_clamp(
        candidate_projection->progress_points, 0.0f,
        (float)(nav.point_count - 1u));
    if ((candidate_projection->progress_points < progress_before) &&
        ((progress_before - candidate_projection->progress_points) >
         (NAV_SCAN_MATCH_MAX_PROGRESS_BACKTRACK_MM / NAV_PATH_SAMPLE_MM)))
    {
        *reject_reason = NAV_SCAN_MATCH_REJECT_PROGRESS_REGRESSION;
        return 0u;
    }
    if (nav_scan_correction.complete_feature &&
        !nav_scan_feature_progress_is_consistent(
            nav_scan_correction.vertex_index,
            candidate_projection->progress_points))
    {
        *reject_reason = NAV_SCAN_MATCH_REJECT_FEATURE_PROGRESS;
        return 0u;
    }
    *reject_reason = NAV_SCAN_MATCH_REJECT_NONE;
    return 1u;
}

void nav_scan_finalize_pending_transaction(void)
{
    uint32 interrupt_state;

    interrupt_state = interrupt_global_disable();
    if (NAV_SCAN_CORRECTION_PENDING == nav_scan_correction.state)
    {
        nav_scan_correction.status = TELEMETRY_LOCALIZATION_STATUS_REJECTED;
        nav_scan_correction.reject_reason = NAV_SCAN_MATCH_REJECT_STOPPED;
        nav_scan_correction.raw_pose = nav.raw_odometry;
        nav_scan_correction.progress_before = nav.progress_points;
        nav_scan_correction.progress_after = -1.0f;
        nav_scan_correction.commit_timestamp_ms = vision_clock_ms;
        __dsync();
        nav_scan_correction.state = NAV_SCAN_CORRECTION_COMPLETE;
    }
    interrupt_global_enable(interrupt_state);
    (void)nav_scan_correction_service_result();
}

static uint8 nav_scan_apply_pending_correction_from_isr(
    DriveOdometryPose *map_pose,
    RouteProjectionResult *projection)
{
    DriveOdometryPose candidate_map_pose;
    DriveOdometryPose current_map_pose;
    DriveOdometryPose raw_pose;
    DriveOdometryPose current_raw_pose;
    RouteProjectionResult candidate_projection;
    uint16 replay_index;
    uint16 speed_seg_idx;
    uint16 acceleration_seg_idx;
    uint16 localization_update_seq;
    float progress_before;
    float current_progress;
    uint8 reject_reason = NAV_SCAN_MATCH_REJECT_NONE;
    uint8 retry_count = 0u;
    uint32 raw_pose_generation_before;
    uint32 interrupt_state;

    if ((NULL == map_pose) || (NULL == projection) ||
        (NAV_SCAN_CORRECTION_PENDING != nav_scan_correction.state))
    {
        return 0u;
    }

retry_pending_correction:
    reject_reason = NAV_SCAN_MATCH_REJECT_NONE;
    memset(&candidate_projection, 0, sizeof(candidate_projection));
    /* Take the complete input snapshot together.  The wide matcher runs in
     * the background; this short masked section is only the hand-off point
     * for its exact projection hint. */
    interrupt_state = interrupt_global_disable();
    raw_pose = nav.raw_odometry;
    progress_before = nav.progress_points;
    raw_pose_generation_before = nav.raw_pose_generation;
    if (!nav_scan_transform_equal(&nav_scan_correction.transform_before,
                                  &nav.map_from_odom) ||
        (nav_scan_correction.localization_update_seq !=
         nav.localization_update_seq))
    {
        reject_reason = NAV_SCAN_MATCH_REJECT_COMMIT_BUSY;
    }
    interrupt_global_enable(interrupt_state);

    if (NAV_SCAN_MATCH_REJECT_NONE == reject_reason)
    {
        (void)nav_scan_compute_pending_projection(
            &raw_pose, progress_before, &candidate_map_pose,
            &candidate_projection, &reject_reason);
    }
    if (NAV_SCAN_MATCH_REJECT_NONE == reject_reason)
    {
        float pose_dx;
        float pose_dy;

        /* The background solver checked its hint pose, but the vehicle may
         * have moved before this control tick.  Re-evaluate the configured
         * physical innovation bound with the exact raw pose that would be
         * committed; never publish a transform whose actual map-pose jump
         * was only validated at an older hint pose. */
        nav_raw_pose_to_map_with(&nav_scan_correction.transform_before,
                                 &raw_pose, &current_map_pose);
        pose_dx = candidate_map_pose.x_mm - current_map_pose.x_mm;
        pose_dy = candidate_map_pose.y_mm - current_map_pose.y_mm;
        if (((pose_dx * pose_dx + pose_dy * pose_dy) >
             NAV_SCAN_MATCH_MAX_CORRECTION_MM *
             NAV_SCAN_MATCH_MAX_CORRECTION_MM) ||
            (fabsf(nav_scan_correction.delta_yaw_rad) >
             NAV_SCAN_MATCH_MAX_CORRECTION_YAW_DEG * nav_rad_per_deg))
        {
            reject_reason = NAV_SCAN_MATCH_REJECT_CORRECTION;
        }
        else if (!nav_scan_correction_path_error_is_valid(
                     progress_before, &current_map_pose,
                     &candidate_projection, 1u,
                     nav_scan_correction.complete_feature))
        {
            reject_reason = NAV_SCAN_MATCH_REJECT_PATH_ERROR;
        }
    }

    if (NAV_SCAN_MATCH_REJECT_NONE != reject_reason)
    {
        interrupt_state = interrupt_global_disable();
        if (NAV_SCAN_CORRECTION_PENDING == nav_scan_correction.state)
        {
            uint8 snapshot_changed;

            /* The solver runs with interrupts enabled.  A projection or
             * feature-progress rejection therefore may be reached after one
             * or more control ticks have advanced raw odometry.  Do not put
             * the stale pre-solve pose into the transaction ledger (and do
             * not claim a stale projection result is still current). */
            current_raw_pose = nav.raw_odometry;
            current_progress = nav.progress_points;
            snapshot_changed = (uint8)(
                (raw_pose_generation_before != nav.raw_pose_generation) ||
                !nav_scan_raw_pose_equal(&raw_pose, &nav.raw_odometry) ||
                (current_progress != progress_before) ||
                !nav_scan_transform_equal(
                    &nav_scan_correction.transform_before,
                    &nav.map_from_odom) ||
                (nav_scan_correction.localization_update_seq !=
                 nav.localization_update_seq));
            if (snapshot_changed)
            {
                reject_reason = NAV_SCAN_MATCH_REJECT_COMMIT_BUSY;
            }
            nav_scan_correction.status =
                TELEMETRY_LOCALIZATION_STATUS_REJECTED;
            nav_scan_correction.reject_reason = reject_reason;
            nav_scan_correction.raw_pose = current_raw_pose;
            nav_scan_correction.progress_before = current_progress;
            nav_scan_correction.progress_after =
                ((NAV_SCAN_MATCH_REJECT_FEATURE_PROGRESS == reject_reason) ||
                 (NAV_SCAN_MATCH_REJECT_PROGRESS_REGRESSION == reject_reason)) ?
                     candidate_projection.progress_points : -1.0f;
            nav_scan_correction.commit_timestamp_ms = vision_clock_ms;
            __dsync();
            nav_scan_correction.state = NAV_SCAN_CORRECTION_COMPLETE;
        }
        interrupt_global_enable(interrupt_state);
        return 0u;
    }

    replay_index = (uint16)(candidate_projection.progress_points + 0.5f);
    if (replay_index >= nav.point_count)
    {
        replay_index = (uint16)(nav.point_count - 1u);
    }
    nav_scan_match_segment_indices_for(
        replay_index, &speed_seg_idx, &acceleration_seg_idx);

    /* The wide path search was completed by the background solver; this ISR
     * only verified exactly 24 bounded segments using its current raw
     * pose.  Publish every state consumer under one short critical section. */
    interrupt_state = interrupt_global_disable();
    current_raw_pose = nav.raw_odometry;
    current_progress = nav.progress_points;
    if ((NAV_SCAN_CORRECTION_PENDING != nav_scan_correction.state) ||
        (raw_pose_generation_before != nav.raw_pose_generation) ||
        !nav_scan_raw_pose_equal(&raw_pose, &nav.raw_odometry) ||
        (current_progress != progress_before) ||
        !nav_scan_transform_equal(&nav_scan_correction.transform_before,
                                  &nav.map_from_odom) ||
        (nav_scan_correction.localization_update_seq !=
         nav.localization_update_seq))
    {
        if ((NAV_SCAN_CORRECTION_PENDING == nav_scan_correction.state) &&
            ((raw_pose_generation_before != nav.raw_pose_generation) ||
             !nav_scan_raw_pose_equal(&raw_pose, &nav.raw_odometry) ||
             (current_progress != progress_before)) &&
            (0u == retry_count))
        {
            retry_count = 1u;
            interrupt_global_enable(interrupt_state);
            goto retry_pending_correction;
        }
        if (NAV_SCAN_CORRECTION_PENDING == nav_scan_correction.state)
        {
            nav_scan_correction.status =
                TELEMETRY_LOCALIZATION_STATUS_REJECTED;
            nav_scan_correction.reject_reason =
                NAV_SCAN_MATCH_REJECT_COMMIT_BUSY;
            /* The candidate was computed from an older snapshot.  A
             * commit-busy ledger entry must describe the state actually
             * observed at rejection, not that stale hint snapshot. */
            nav_scan_correction.raw_pose = current_raw_pose;
            nav_scan_correction.progress_before = current_progress;
            nav_scan_correction.progress_after = -1.0f;
            nav_scan_correction.commit_timestamp_ms = vision_clock_ms;
            __dsync();
            nav_scan_correction.state = NAV_SCAN_CORRECTION_COMPLETE;
        }
        interrupt_global_enable(interrupt_state);
        return 0u;
    }
    /* The equality checks above prove that the candidate projection and this
     * raw pose/progress snapshot describe the same instant.  Publish every
     * derived state in this one critical section; the next control consumer
     * therefore cannot observe a new transform with an old progress cursor. */
    nav_scan_correction.raw_pose = current_raw_pose;
    nav_scan_correction.progress_before = current_progress;
    nav.map_from_odom = nav_scan_correction.transform_after;
    nav.map_pose = candidate_map_pose;
    nav.progress_points = candidate_projection.progress_points;
    nav.replay_index = replay_index;
    nav.speed_seg_idx = speed_seg_idx;
    nav.acceleration_seg_idx = acceleration_seg_idx;
    localization_update_seq = (uint16)(nav.localization_update_seq + 1u);
    nav.localization_update_seq = localization_update_seq;
    nav_yaw_rate_tracker_reset();
    nav_scan_correction.status = TELEMETRY_LOCALIZATION_STATUS_ACCEPTED;
    nav_scan_correction.reject_reason = NAV_SCAN_MATCH_REJECT_NONE;
    nav_scan_correction.localization_update_seq = localization_update_seq;
    nav_scan_correction.commit_timestamp_ms = vision_clock_ms;
    nav_scan_correction.progress_after = candidate_projection.progress_points;
    __dsync();
    nav_scan_correction.state = NAV_SCAN_CORRECTION_COMPLETE;
    interrupt_global_enable(interrupt_state);

    *map_pose = candidate_map_pose;
    *projection = candidate_projection;
    return 1u;
}

static void nav_scan_match_reset(void)
{
    uint32 interrupt_state = interrupt_global_disable();

    /* nav_scan_correction is consumed by the 1 ms ISR.  Resetting it from a
     * menu/stop transaction must not expose a half-cleared state or turn a
     * pending request into an apparently idle one midway through memset(). */
    memset((void *)&nav_scan_match, 0, sizeof(nav_scan_match));
    memset((void *)&nav_scan_correction, 0, sizeof(nav_scan_correction));
    nav_scan_match_first_index = 0u;
    __dsync();
    interrupt_global_enable(interrupt_state);
}

void nav_scan_map_match_service(void)
{
    PhototubeScanSnapshot snapshot;
    const PhototubeScanPoint *latest_point;
    uint16 point_count;
    uint16 feature_first_index = 0u;
    uint16 feature_point_count = 0u;
    uint16 straight_first_index = 0u;
    uint16 straight_point_count = 0u;
    uint16 feature_index = 0u;
    uint16 seed_segment = 0u;
    uint16 straight_line_group = 0u;
    uint16 straight_frame_count = 0u;
    uint8 feature_type = NAV_SCAN_VERTEX_TYPE_STRAIGHT;
    uint8 feature_found = 0u;
    uint8 feature_ambiguous = 0u;
    uint8 feature_calibration_allowed = 1u;
    uint8 feature_match_type = NAV_SCAN_MATCH_TYPE_NONE;
    uint8 feature_reject_recorded = 0u;
    uint8 have_feature_cutoff = 0u;
    uint8 have_straight_cutoff = 0u;
    float sensor_progress;
    float matching_progress_points;
    float point_progress_low;
    float point_progress_high;
    float anchor_x_mm = 0.0f;
    float anchor_y_mm = 0.0f;
    float shift_x_mm = 0.0f;
    float shift_y_mm = 0.0f;
    float yaw_correction_rad = 0.0f;
    float rms_before_mm = 0.0f;
    float rms_after_mm = 0.0f;
    float straight_span_mm = 0.0f;
    float straight_tangent_information_mm2 = 0.0f;
    float latest_travel_mm;
    float matching_travel_mm = 0.0f;
    float feature_cutoff_travel_mm = 0.0f;
    float straight_cutoff_travel_mm = 0.0f;
    uint32 feature_cutoff_frame_seq = 0u;
    uint32 straight_cutoff_frame_seq = 0u;
    uint16 inlier_count = 0u;
    uint8 straight_reject_reason = NAV_SCAN_MATCH_REJECT_GEOMETRY;

    if (nav_scan_correction_service_result())
    {
        return;
    }
    if ((NAV_STATE_NAVIGATING != nav.state) ||
        !phototube_scan_get_snapshot(&snapshot) ||
        (snapshot.last_frame_seq == nav_scan_match.last_scan_frame_seq))
    {
        return;
    }
    nav_scan_match.last_scan_frame_seq = snapshot.last_frame_seq;
    nav_scan_match.current_evidence_frame_seq = snapshot.last_frame_seq;
    /* Keep the current evidence distance valid before the point window is
     * available.  Generic early failures are not full-feature attempts and
     * therefore do not consume a fixed feature diagnostic slot. */
    nav_scan_match.current_evidence_travel_mm = total_dist * 1000.0f;
    if (!phototube_scan_get_travel_window(
            NAV_SCAN_MATCH_TRAVEL_WINDOW_MM, &feature_first_index,
            &feature_point_count) ||
        (feature_point_count < NAV_SCAN_MATCH_MIN_POINT_COUNT))
    {
        nav_scan_match_record_reject(NAV_SCAN_MATCH_REJECT_NOT_ENOUGH_POINTS,
                                     0u);
        return;
    }
    /* Complete features use the full local history.  All helpers below read
     * through nav_scan_match_first_index, so select this window explicitly. */
    nav_scan_match_first_index = feature_first_index;
    point_count = feature_point_count;
    latest_point = nav_scan_match_point_at((uint16)(point_count - 1u));
    if (NULL == latest_point)
    {
        nav_scan_match_record_reject(NAV_SCAN_MATCH_REJECT_GEOMETRY, 0u);
        return;
    }
    if (!nav_scan_match_point_travel_distance_mm(
            (uint16)(point_count - 1u), &latest_travel_mm))
    {
        nav_scan_match_record_reject(NAV_SCAN_MATCH_REJECT_GEOMETRY, 0u);
        return;
    }
    nav_scan_match.current_evidence_frame_seq = snapshot.last_frame_seq;
    nav_scan_match.current_evidence_travel_mm = latest_travel_mm;
    /* Select route features and transform the physical feature map with one
     * coherent navigation snapshot.  Reading progress first and T later can
     * otherwise pair a pre-correction route cursor with a post-correction
     * map transform while the 1 ms control ISR commits localization. */
    nav_scan_prepare_map_to_odom_cache(&matching_progress_points,
                                       &matching_travel_mm);
    sensor_progress = matching_progress_points +
                      PHOTOTUBE_ARRAY_FORWARD_MM / NAV_PATH_SAMPLE_MM;
    nav_scan_track_next_route_feature(sensor_progress);
    nav_scan_advance_feature_evidence_boundary(
        sensor_progress, matching_travel_mm, snapshot.last_frame_seq);

    /* Keep every raw point in the archive/ring.  Straight corrections advance
     * only the straight cutoff, preserving incoming-edge evidence for the
     * complete feature solver.  A complete feature correction or passed
     * feature boundary advances both cutoffs. */
    if (nav_scan_match.have_consumed_evidence)
    {
        nav_scan_merge_evidence_cutoff(
            &have_straight_cutoff, &straight_cutoff_travel_mm,
            &straight_cutoff_frame_seq,
            nav_scan_match.last_consumed_travel_mm,
            nav_scan_match.last_consumed_frame_seq);
    }
    if (nav_scan_match.have_feature_consumed_evidence)
    {
        nav_scan_merge_evidence_cutoff(
            &have_feature_cutoff, &feature_cutoff_travel_mm,
            &feature_cutoff_frame_seq,
            nav_scan_match.last_feature_consumed_travel_mm,
            nav_scan_match.last_feature_consumed_frame_seq);
        nav_scan_merge_evidence_cutoff(
            &have_straight_cutoff, &straight_cutoff_travel_mm,
            &straight_cutoff_frame_seq,
            nav_scan_match.last_feature_consumed_travel_mm,
            nav_scan_match.last_feature_consumed_frame_seq);
    }
    if (nav_scan_match.have_feature_evidence_boundary)
    {
        nav_scan_merge_evidence_cutoff(
            &have_feature_cutoff, &feature_cutoff_travel_mm,
            &feature_cutoff_frame_seq,
            nav_scan_match.feature_evidence_boundary_travel_mm,
            nav_scan_match.feature_evidence_boundary_frame_seq);
        nav_scan_merge_evidence_cutoff(
            &have_straight_cutoff, &straight_cutoff_travel_mm,
            &straight_cutoff_frame_seq,
            nav_scan_match.feature_evidence_boundary_travel_mm,
            nav_scan_match.feature_evidence_boundary_frame_seq);
    }
    /* Retain cutoff handling for any legacy pending record; newly matched
     * corrections are submitted immediately below. */
    nav_scan_merge_pending_candidate_cutoff(
        &nav_scan_match.straight_pending, &have_straight_cutoff,
        &straight_cutoff_travel_mm, &straight_cutoff_frame_seq);
    nav_scan_merge_pending_candidate_cutoff(
        &nav_scan_match.feature_pending, &have_feature_cutoff,
        &feature_cutoff_travel_mm, &feature_cutoff_frame_seq);

    /* Match a complete CORNER/JUNCTION first, using the full cutoff-trimmed
     * feature window.  Ordinary straight fitting is only the fallback after
     * the structural feature hypothesis has been evaluated. */
    (void)nav_scan_trim_window_to_cutoff(
        feature_first_index, feature_point_count, have_feature_cutoff,
        feature_cutoff_travel_mm, feature_cutoff_frame_seq,
        &nav_scan_match_first_index, &point_count);
    if (point_count < NAV_SCAN_MATCH_MIN_POINT_COUNT)
    {
        const NavScanConfirmationCandidate *latest_pending =
            nav_scan_latest_pending_candidate();
        nav_scan_match_record_reject_detail(
            NAV_SCAN_MATCH_REJECT_NO_FRESH_EVIDENCE,
            (NULL != latest_pending) ? latest_pending->match_type :
                                       NAV_SCAN_MATCH_TYPE_NONE,
            (NULL != latest_pending) ? latest_pending->key_index : 0u,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0u);
        return;
    }

    if (!nav_scan_measure_progress_window(sensor_progress, point_count,
                                          &point_progress_low,
                                          &point_progress_high))
    {
        nav_scan_match_record_reject(NAV_SCAN_MATCH_REJECT_GEOMETRY, 0u);
        return;
    }
    feature_found = nav_scan_find_window_feature(
        sensor_progress, point_progress_low, point_progress_high, point_count,
        &feature_index, &feature_type, &feature_ambiguous);
    if (feature_found)
    {
        nav_scan_track_feature_evidence_boundary(feature_index);
        nav_scan_feature_attempt_reset();
        if (feature_type == NAV_SCAN_VERTEX_TYPE_CORNER)
        {
            feature_match_type = NAV_SCAN_MATCH_TYPE_CORNER;
        }
        else if (feature_type == NAV_SCAN_VERTEX_TYPE_JUNCTION)
        {
            feature_match_type = NAV_SCAN_MATCH_TYPE_JUNCTION;
        }
        feature_calibration_allowed = (uint8)(
            (NULL != nav.scan_vertex_corner_calibration_allowed) &&
            (feature_index < nav.scan_vertex_count) &&
            (0u != nav.scan_vertex_corner_calibration_allowed[feature_index]));
        if (!feature_calibration_allowed &&
            (NAV_SCAN_MATCH_TYPE_NONE != feature_match_type))
        {
            /* This physical feature is intentionally excluded from full
             * CORNER/JUNCTION SE(2) correction.  Keep its evidence boundary
             * and fall through to the ordinary straight fit below. */
            nav_scan_clear_pending_candidate(feature_match_type);
        }
    }
    /* Match only an unambiguous structural feature, as in the 02:57 backup.
     * Ambiguous progress windows fall through to the ordinary straight fit. */
    if (feature_found && !feature_ambiguous &&
        feature_calibration_allowed)
    {
        uint8 feature_matched = 0u;

        if (nav_scan_feature_is_localized(feature_index))
        {
            nav_scan_clear_pending_candidate(feature_match_type);
            nav_scan_match_record_reject_detail(
                NAV_SCAN_MATCH_REJECT_FEATURE_CONSUMED,
                feature_match_type, feature_index,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0u);
            return;
        }

        feature_matched = nav_scan_match_feature(
            feature_index, feature_type, point_count,
            &anchor_x_mm, &anchor_y_mm, &shift_x_mm, &shift_y_mm,
            &yaw_correction_rad, &rms_before_mm, &rms_after_mm,
            &inlier_count);
        if (feature_matched)
        {
            if ((rms_before_mm - rms_after_mm) <
                NAV_SCAN_MATCH_MIN_RMS_IMPROVEMENT_MM)
            {
                nav_scan_feature_attempt.detail_flags |=
                    TELEMETRY_LOCALIZATION_DETAIL_NO_IMPROVEMENT;
                nav_scan_feature_attempt.failure_value =
                    rms_before_mm - rms_after_mm;
                nav_scan_match_record_reject_detail(
                    NAV_SCAN_MATCH_REJECT_NO_IMPROVEMENT,
                    feature_match_type, feature_index,
                    anchor_x_mm, anchor_y_mm, shift_x_mm, shift_y_mm,
                    yaw_correction_rad, rms_before_mm, rms_after_mm,
                    inlier_count);
                feature_reject_recorded = 1u;
            }
            else
            {
                /* A complete corner/junction solve observes both sides and is
                 * actionable as soon as the current point cloud passes the
                 * geometric gates. */
                nav_scan_apply_correction(
                    feature_match_type, feature_index, anchor_x_mm,
                    anchor_y_mm, shift_x_mm, shift_y_mm, yaw_correction_rad,
                    rms_before_mm, rms_after_mm, inlier_count,
                    snapshot.last_frame_seq, latest_travel_mm);
                return;
            }
        }
    }

    if (feature_found && feature_calibration_allowed)
    {
        uint8 rejected_match_type =
            (feature_type == NAV_SCAN_VERTEX_TYPE_JUNCTION) ?
            NAV_SCAN_MATCH_TYPE_JUNCTION : NAV_SCAN_MATCH_TYPE_CORNER;
        uint8 reject_reason = feature_ambiguous ?
            NAV_SCAN_MATCH_REJECT_AMBIGUOUS_FEATURE :
            ((feature_type == NAV_SCAN_VERTEX_TYPE_JUNCTION) ?
             NAV_SCAN_MATCH_REJECT_PARTIAL_JUNCTION :
             NAV_SCAN_MATCH_REJECT_PARTIAL_CORNER);
        if (!feature_reject_recorded && feature_ambiguous)
        {
            nav_scan_feature_attempt.detail_flags |=
                TELEMETRY_LOCALIZATION_DETAIL_AMBIGUOUS;
        }
        if (!feature_reject_recorded)
        {
            nav_scan_match_record_reject_detail(
                reject_reason, rejected_match_type, feature_index,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0u);
        }
        /* Only a failed geometric fit reaches this block.  Clear any stale
         * hint for this feature so it cannot affect a later immediate solve. */
        nav_scan_clear_pending_candidate(rejected_match_type);
        /* A partial/ambiguous feature only disproves the full SE(2) solve;
         * the same fresh centerline points may still constrain a straight. */
    }

    /* Backup order: only after the complete structural feature hypothesis has
     * been evaluated do we fit the newest 120 mm ordinary straight window. */
    if (!phototube_scan_get_travel_window(
            NAV_SCAN_MATCH_STRAIGHT_TRAVEL_WINDOW_MM,
            &straight_first_index, &straight_point_count))
    {
        nav_scan_match_record_reject(
            NAV_SCAN_MATCH_REJECT_NOT_ENOUGH_POINTS, 0u);
        return;
    }
    (void)nav_scan_trim_window_to_cutoff(
        straight_first_index, straight_point_count, have_straight_cutoff,
        straight_cutoff_travel_mm, straight_cutoff_frame_seq,
        &nav_scan_match_first_index, &point_count);
    if (point_count < NAV_SCAN_MATCH_MIN_POINT_COUNT)
    {
        nav_scan_match_record_reject_detail(
            NAV_SCAN_MATCH_REJECT_NO_FRESH_EVIDENCE,
            NAV_SCAN_MATCH_TYPE_STRAIGHT, seed_segment,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0u);
        return;
    }
    if (!nav_scan_match_straight(sensor_progress, point_count, &seed_segment,
                                 &straight_line_group, &anchor_x_mm,
                                 &anchor_y_mm, &shift_x_mm, &shift_y_mm,
                                 &yaw_correction_rad, &rms_before_mm,
                                 &rms_after_mm, &inlier_count,
                                 &straight_frame_count, &straight_span_mm,
                                 &straight_tangent_information_mm2,
                                 &straight_reject_reason))
    {
        nav_scan_match_record_reject_detail(
            straight_reject_reason, NAV_SCAN_MATCH_TYPE_STRAIGHT, seed_segment,
            anchor_x_mm, anchor_y_mm, shift_x_mm, shift_y_mm,
            yaw_correction_rad, rms_before_mm, rms_after_mm, inlier_count);
        return;
    }
    (void)straight_line_group;
    if (!nav_scan_straight_correction_is_significant(
            shift_x_mm, shift_y_mm, yaw_correction_rad, rms_after_mm,
            straight_frame_count, straight_span_mm,
            straight_tangent_information_mm2))
    {
        nav_scan_match_record_reject_detail(
            NAV_SCAN_MATCH_REJECT_STRAIGHT_NOT_SIGNIFICANT,
            NAV_SCAN_MATCH_TYPE_STRAIGHT, seed_segment,
            anchor_x_mm, anchor_y_mm, shift_x_mm, shift_y_mm,
            yaw_correction_rad, rms_before_mm, rms_after_mm, inlier_count);
        return;
    }
    if ((rms_before_mm - rms_after_mm) <
        NAV_SCAN_MATCH_MIN_RMS_IMPROVEMENT_MM)
    {
        nav_scan_match_record_reject_detail(
            NAV_SCAN_MATCH_REJECT_NO_IMPROVEMENT,
            NAV_SCAN_MATCH_TYPE_STRAIGHT, seed_segment,
            anchor_x_mm, anchor_y_mm, shift_x_mm, shift_y_mm,
            yaw_correction_rad, rms_before_mm, rms_after_mm, inlier_count);
        return;
    }

    nav_scan_clear_pending_candidate(NAV_SCAN_MATCH_TYPE_STRAIGHT);
    nav_scan_apply_correction(NAV_SCAN_MATCH_TYPE_STRAIGHT, seed_segment,
                              anchor_x_mm, anchor_y_mm, shift_x_mm,
                              shift_y_mm, yaw_correction_rad, rms_before_mm,
                              rms_after_mm, inlier_count,
                              snapshot.last_frame_seq, latest_travel_mm);
}

uint8 nav_scan_map_match_allows_blackout(void)
{
    if (!nav_scan_match_is_fresh())
    {
        return 0u;
    }
    return nav_progress_in_map_takeover_zone(nav.progress_points);
}

float nav_get_line_loss_stop_distance_mm(void)
{
    return PHOTOTUBE_ARRAY_BLACK_STOP_DISTANCE_MM;
}


// 当前点曲率直接读取地图生成器离线表，供直道门控和遥测使用。
static void nav_update_curvature(void)
{
    nav.kappa_per_mm = route_reference_curvature_at_progress(
        nav.curvature, nav.point_count, nav.progress_points,
        nav.curvature_lsb_per_mm);
}

static void nav_capture_telemetry_if_due(uint8 force)
{
    DriveOdometryPose map_pose;
    uint32 now_ms;
    TelemetryNavigationSample sample;

    if (!telemetry_is_capturing())
    {
        return;
    }
    now_ms = vision_clock_ms;
    if (!force && ((uint32)(now_ms - nav_telemetry_last_ms) < 10u))
    {
        return;
    }
    nav_telemetry_last_ms = now_ms;
    if ((0u == nav.point_count) || (NULL == nav.data) ||
        (NULL == nav.xs) || (NULL == nav.ys))
    {
        return;
    }
    nav_raw_pose_to_map_with(&nav.map_from_odom, &nav.raw_odometry,
                             &map_pose);

    sample.timestamp_ms = now_ms;
    sample.progress_points = nav.progress_points;
    sample.actual_x_mm = map_pose.x_mm;
    sample.actual_y_mm = map_pose.y_mm;
    sample.actual_yaw_deg = map_pose.yaw_rad * nav_deg_per_rad;
    sample.raw_x_mm = nav.raw_odometry.x_mm;
    sample.raw_y_mm = nav.raw_odometry.y_mm;
    sample.raw_yaw_deg = nav.raw_odometry.yaw_rad * nav_deg_per_rad;
    sample.localization_update_seq = nav.localization_update_seq;
    telemetry_capture_navigation_isr(&sample);
}

// 推进回放。传入本周期左右轮编码器脉冲增量(前进为正)。见文件头四步总览。
void nav_update(int16 pulses_l, int16 pulses_r, float sample_period_s)
{
    DriveOdometryPose map_pose;
    RouteProjectionResult projection;
    float raw_yaw_deg;
    float corrected_yaw_deg;
    uint8 projection_ready;

    if (NAV_STATE_NAVIGATING != nav.state)
    {
        return;
    }

    if (sample_period_s <= 0.0f)
    {
        sample_period_s = PID_SAMPLE_S;
    }
    nav_control_imu_valid = nav_read_imu_for_control(
        &raw_yaw_deg, &nav_control_yaw_rate_radps);
#if ENCODER_TEST_MODE_ENABLE
    // 地图编码器诊断模式仍读取 IMU 以保留 stale 保护，但位姿航向和角速度
    // 完全由左右轮差速拟合；正常比赛编译宏为0，继续使用 VQF yaw。
    (void)raw_yaw_deg;
    drive_odometry_integrate_wheel_delta_encoder_heading(
        &nav.raw_odometry,
        (float)pulses_l * DIST_PER_PULSE_MM,
        (float)pulses_r * DIST_PER_PULSE_MM,
        (ENCODER_YAW_DEG_PER_DIFF_PULSE * nav_rad_per_deg) /
            DIST_PER_PULSE_MM,
        sample_period_s);
    nav_control_yaw_rate_radps = nav.raw_odometry.yaw_rate_radps;
    nav_control_imu_valid = 1u;
#else
    // 第1步 原始航位推算只使用编码器中心距离和未经定位修正的 VQF yaw。
    drive_odometry_integrate_wheel_delta(&nav.raw_odometry,
                                         (float)pulses_l * DIST_PER_PULSE_MM,
                                         (float)pulses_r * DIST_PER_PULSE_MM,
                                         raw_yaw_deg * nav_rad_per_deg,
                                         sample_period_s);
#endif
    nav.raw_pose_generation++;
    projection_ready = nav_scan_apply_pending_correction_from_isr(
        &map_pose, &projection);
    if (!projection_ready)
    {
        nav_raw_pose_to_map_with(&nav.map_from_odom, &nav.raw_odometry,
                                 &map_pose);
    }
    nav.map_pose = map_pose;
    corrected_yaw_deg = map_pose.yaw_rad * nav_deg_per_rad;

        // 第2步 最近路径点:从上次进度向前开窗搜索。索引单调不回退、窗口有限,自交/折返路线不会跳错圈次。
        {
            float progress_prediction = nav.progress_points;
            uint16 projection_start;
            uint16 projection_search_points;
            float denominator = 1.0f -
                               nav.reference_curvature_per_mm * nav.ey_mm;

            // Frenet运动学预测：s_dot=v*cos(eψ)/(1-κey)。分母接近0时放弃
            // 预测，避免异常姿态把投影窗跳到错误位置。
            if ((denominator > 0.25f) || (denominator < -0.25f))
            {
                float s_dot_mmps = nav.raw_odometry.linear_speed_mmps *
                                   cosf(nav.epsi_rad) / denominator;
                float predicted_delta_points = s_dot_mmps *
                                               sample_period_s /
                                               NAV_PATH_SAMPLE_MM;
                if (predicted_delta_points > 0.0f)
                {
                    progress_prediction += predicted_delta_points;
                }
            }
            if (progress_prediction < nav.progress_points)
            {
                progress_prediction = nav.progress_points;
            }
            if (progress_prediction > (float)(nav.point_count - 1u))
            {
                progress_prediction = (float)(nav.point_count - 1u);
            }
            projection_start = (uint16)progress_prediction;
            projection_search_points = NAV_PP_PREDICT_SEARCH_POINTS;

            if (projection_ready ||
                route_projection_project_forward(nav.xs,
                                                 nav.ys,
                                                 nav.point_count,
                                                 projection_start,
                                                 projection_search_points,
                                                 map_pose.x_mm,
                                                 map_pose.y_mm,
                                                 NAV_PATH_SAMPLE_MM,
                                                 &projection))
            {
                /* Projection-derived Frenet state is one coherent tuple.
                 * If a self-intersection/noise candidate falls behind the
                 * committed route cursor, retain progress, ey, epsi and
                 * curvature together; mixing an old progress with the
                 * rejected segment's lateral error creates a false steering
                 * command. */
                if (projection.progress_points >= nav.progress_points)
                {
                    nav.progress_points = projection.progress_points;
                    nav.replay_index = (uint16)(projection.index +
                        ((projection.fraction >= 0.5f) ? 1u : 0u));
                    if (nav.replay_index >= nav.point_count)
                    {
                        nav.replay_index = (uint16)(nav.point_count - 1u);
                    }
                    nav.ey_mm = projection.signed_cross_track_mm;
                    nav_signed_cross_track_mm = nav.ey_mm;
                    // Frenet 状态使用地图系位姿，原始里程计保持不变。
                    nav.epsi_rad = nav_wrap_deg(
                        corrected_yaw_deg -
                        route_reference_heading_deg_at_progress(
                            nav.data, nav.point_count,
                            nav.progress_points)) * nav_rad_per_deg;
                    nav.reference_curvature_per_mm =
                        route_reference_curvature_at_progress(
                            nav.curvature, nav.point_count,
                            nav.progress_points,
                            nav.curvature_lsb_per_mm);
                }
            }
            else
            {
                // 投影失败即暴露为导航错误，不改用另一搜索策略继续运行。
                nav.state = NAV_STATE_ERROR;
                nav.correction = 0.0f;
                nav_yaw_rate_tracker_reset();
                nav_capture_state_transition(NAV_STATE_NAVIGATING, nav.state);
                return;
            }
        }

    // 速度/加速度段游标推进:进度单调向前,游标线性跟进(每周期至多跨一两段,O(1))
    while (((uint16)(nav.speed_seg_idx + 1u) < nav.speed_seg_cnt) &&
           (nav.replay_index > nav.speed_end_p[nav.speed_seg_idx]))
    {
        nav.speed_seg_idx++;
    }
    while (((uint16)(nav.acceleration_seg_idx + 1u) < nav.acceleration_seg_cnt) &&
           (nav.replay_index > nav.acceleration_end_p[nav.acceleration_seg_idx]))
    {
        nav.acceleration_seg_idx++;
    }

    // 播完:最近点推进到路径末点,由 Duty_10ms 消费 nav_is_complete() 停车。
    if (nav.replay_index >= (uint16)(nav.point_count - 1u))
    {
        nav.correction = 0.0f;
        nav_yaw_rate_tracker_reset();
        nav.state = NAV_STATE_FINISHED;
        nav_capture_state_transition(NAV_STATE_NAVIGATING, nav.state);
        nav_update_curvature();
        nav_capture_telemetry_if_due(1u);
        return;
    }

    // 曲率用于下一拍进度预测、延迟补偿目标偏航率前馈和遥测。
    nav_update_curvature();

    nav_capture_telemetry_if_due(0u);
}

// 非导航测试态只积分位姿，不做路径投影、速度表和转向计算。
// 这样直线速度阶跃测试也能给扫图提供连续的 x/y/yaw 基准。
void nav_update_odometry_only(int16 pulses_l, int16 pulses_r,
                              float sample_period_s)
{
    float raw_heading_deg;

    if (NAV_STATE_NAVIGATING == nav.state)
    {
        return;
    }
    if (sample_period_s <= 0.0f)
    {
        sample_period_s = PID_SAMPLE_S;
    }
    nav_control_imu_valid = nav_read_imu_for_control(
        &raw_heading_deg, &nav_control_yaw_rate_radps);
    drive_odometry_integrate_wheel_delta(&nav.raw_odometry,
                                         (float)pulses_l * DIST_PER_PULSE_MM,
                                         (float)pulses_r * DIST_PER_PULSE_MM,
                                         raw_heading_deg * nav_rad_per_deg,
                                         sample_period_s);
    nav.raw_pose_generation++;
}

// 当前导航状态。
NavSystemState nav_get_state(void)
{
    return nav.state;
}

// CPU0 内部航位推算快照。拷贝时短暂关中断，避免1ms速度中断恰好更新一半字段。
uint8 nav_get_raw_odometry_pose(DriveOdometryPose *pose)
{
    uint32 interrupt_state;

    if (NULL == pose)
    {
        return 0u;
    }

    interrupt_state = interrupt_global_disable();
    *pose = nav.raw_odometry;
    interrupt_global_enable(interrupt_state);
    return 1u;
}

void nav_get_map_from_odom(float *tx_mm, float *ty_mm, float *yaw_deg)
{
    uint32 interrupt_state = interrupt_global_disable();

    if (NULL != tx_mm) *tx_mm = nav.map_from_odom.tx_mm;
    if (NULL != ty_mm) *ty_mm = nav.map_from_odom.ty_mm;
    if (NULL != yaw_deg)
    {
        *yaw_deg = nav.map_from_odom.yaw_rad * nav_deg_per_rad;
    }
    interrupt_global_enable(interrupt_state);
}

uint16 nav_get_localization_update_sequence(void)
{
    return nav.localization_update_seq;
}

void nav_get_localization_config_snapshot(
    NavLocalizationConfigSnapshot *snapshot)
{
    if (NULL == snapshot)
    {
        return;
    }
    snapshot->algorithm_version = NAV_SCAN_MATCH_ALGORITHM_VERSION;
    snapshot->sensor_count = (uint8)PHOTOTUBE_ARRAY_COUNT;
    snapshot->adc_group = (uint8)PHOTOTUBE_ARRAY_ADC_GROUP;
    snapshot->adc_channel = (uint8)PHOTOTUBE_ARRAY_ADC_GROUP_CHANNEL;
    snapshot->mux_selector_count = (uint8)PHOTOTUBE_ARRAY_MUX_SELECTOR_COUNT;
    snapshot->scan_hz = (uint32)PHOTOTUBE_ARRAY_SCAN_HZ;
    snapshot->sensor_spacing_mm = PHOTOTUBE_ARRAY_SPACING_MM;
    snapshot->sensor_forward_mm = PHOTOTUBE_ARRAY_FORWARD_MM;
    snapshot->scan_min_step_mm = PHOTOTUBE_SCAN_MIN_STEP_MM;
    /* The exposed matching window is the active fast straight window.  The
     * complete-feature history remains 1 m and is protected by the config
     * hash, but it is not the window used for ordinary line corrections. */
    snapshot->match_travel_window_mm =
        NAV_SCAN_MATCH_STRAIGHT_TRAVEL_WINDOW_MM;
    snapshot->feature_radius_mm = NAV_SCAN_MATCH_FEATURE_RADIUS_MM;
    /* The EMEM field records the wider complete-feature evidence gate.
     * Ordinary straight matching remains fixed at the stricter 100 mm gate. */
    snapshot->max_feature_point_residual_mm =
        NAV_SCAN_MATCH_MAX_FEATURE_POINT_RESIDUAL_MM;
    snapshot->max_rms_mm = NAV_SCAN_MATCH_MAX_RMS_MM;
    snapshot->max_correction_mm = NAV_SCAN_MATCH_MAX_CORRECTION_MM;
    snapshot->max_correction_yaw_deg =
        NAV_SCAN_MATCH_MAX_CORRECTION_YAW_DEG;
    snapshot->black_stop_distance_mm =
        PHOTOTUBE_ARRAY_BLACK_STOP_DISTANCE_MM;
}

uint32 nav_get_localization_config_hash(void)
{
    return nav_calculate_localization_config_hash();
}

uint16 nav_get_scan_feature_count(void)
{
    return nav.scan_vertex_count;
}

uint8 nav_get_scan_feature_info(uint16 occurrence,
                                uint8 *feature_type,
                                uint16 *feature_progress_points)
{
    if ((occurrence >= nav.scan_vertex_count) ||
        (NULL == feature_type) || (NULL == feature_progress_points) ||
        (NULL == nav.scan_vertex_type) || (NULL == nav.scan_vertex_progress))
    {
        return 0u;
    }
    *feature_type = nav.scan_vertex_type[occurrence];
    *feature_progress_points = nav.scan_vertex_progress[occurrence];
    return 1u;
}

float nav_compute_tracking_turn_cmps(float plan_v_cmps,
                                     float actual_v_cmps)
{
    float actual_v_mmps;
    float curvature_progress_points;
    float curvature_per_mm;
    float omega_feedforward_radps;
    float heading_correction_radps;
    float cross_track_correction_radps;
    float correction_speed_mmps;
    float target_rate_radps;
    float feedforward_turn_cmps;
    float feedback_turn_cmps = 0.0f;

    if ((NAV_STATE_NAVIGATING != nav.state) ||
        (plan_v_cmps <= 0.0f))
    {
        nav_yaw_rate_tracker_reset();
        return 0.0f;
    }

    if (actual_v_cmps < 0.0f)
    {
        actual_v_cmps = 0.0f;
    }

    actual_v_mmps = actual_v_cmps * 10.0f;
    curvature_progress_points = nav.progress_points +
        actual_v_cmps * 10.0f * NAV_TRACK_CURVATURE_PREVIEW_TIME_S /
        NAV_PATH_SAMPLE_MM;
    if (curvature_progress_points > (float)(nav.point_count - 1u))
    {
        curvature_progress_points = (float)(nav.point_count - 1u);
    }
    curvature_per_mm = route_reference_curvature_at_progress(
        nav.curvature, nav.point_count, curvature_progress_points,
        nav.curvature_lsb_per_mm);
    omega_feedforward_radps = actual_v_mmps * curvature_per_mm;

    /* The map pose/projection is the only recovery path.  This retains the
     * offline curvature command while correcting a real lateral or heading
     * departure instead of replaying a blind open-loop yaw-rate table. */
    correction_speed_mmps = actual_v_mmps;
    if (correction_speed_mmps < NAV_TRACK_MIN_CORRECTION_SPEED_MMPS)
    {
        correction_speed_mmps = NAV_TRACK_MIN_CORRECTION_SPEED_MMPS;
    }
    heading_correction_radps =
        -NAV_TRACK_HEADING_CORRECTION_RADPS_PER_RAD * nav.epsi_rad;
    cross_track_correction_radps =
        -NAV_TRACK_CROSS_TRACK_CORRECTION_MMPS_PER_MM * nav.ey_mm /
        correction_speed_mmps;
    target_rate_radps = omega_feedforward_radps +
        heading_correction_radps + cross_track_correction_radps;
    nav.correction = -nav.epsi_rad * nav_deg_per_rad;

    /* v_actual[cm/s]*10 -> mm/s。右正的目标偏航率对应负 turn；
     * -b/2*omega 使用 b[cm]，故 DRIVE_WHEEL_TRACK_MM/20 得到 cm。 */
    feedforward_turn_cmps = -(DRIVE_WHEEL_TRACK_MM / 20.0f) *
                            target_rate_radps;

    nav_yaw_rate_tracker.target_rate_radps = target_rate_radps;
    nav_yaw_rate_tracker.rate_error_radps = 0.0f;
    if (nav_control_imu_valid)
    {
        nav_yaw_rate_tracker.rate_error_radps =
            nav_yaw_rate_tracker.target_rate_radps -
            nav_control_yaw_rate_radps;
        feedback_turn_cmps =
            -NAV_TRACK_YAW_RATE_FB_CMPS_PER_RADPS *
            nav_yaw_rate_tracker.rate_error_radps;
    }
    nav_yaw_rate_tracker.valid = 1u;

    return feedforward_turn_cmps + feedback_turn_cmps;
}

float nav_compute_straight_hold_turn_cmps(void)
{
    float target_rate_radps = 0.0f;
    float measured_rate_radps = 0.0f;
    float rate_error_radps;
    float feedforward_turn_cmps;
    float feedback_turn_cmps = 0.0f;

    /* nav_update_odometry_only() has already consumed the coherent IMU
     * snapshot for this control tick.  The raw odometry yaw starts at the
     * launch heading, so it is the signed heading error for this test. */
    if (nav.raw_odometry.gyro_heading_initialized)
    {
        target_rate_radps = -NAV_STRAIGHT_HOLD_HEADING_RADPS_PER_RAD *
                            nav_wrap_rad(nav.raw_odometry.yaw_rad);
    }
    if (nav_control_imu_valid)
    {
        measured_rate_radps = nav_control_yaw_rate_radps;
    }
    rate_error_radps = target_rate_radps - measured_rate_radps;

    feedforward_turn_cmps = -(DRIVE_WHEEL_TRACK_MM / 20.0f) *
                            target_rate_radps;
    if (nav_control_imu_valid)
    {
        feedback_turn_cmps =
            -NAV_TRACK_YAW_RATE_FB_CMPS_PER_RADPS * rate_error_radps;
    }

    nav_yaw_rate_tracker.target_rate_radps = target_rate_radps;
    nav_yaw_rate_tracker.rate_error_radps = rate_error_radps;
    nav_yaw_rate_tracker.valid = nav_control_imu_valid;
    return feedforward_turn_cmps + feedback_turn_cmps;
}

float nav_get_tracking_yaw_rate_error_degps(void)
{
    return nav_yaw_rate_tracker.valid ?
        (nav_yaw_rate_tracker.rate_error_radps * nav_deg_per_rad) : 0.0f;
}

float nav_get_control_yaw_rate_degps(void)
{
    return nav_control_imu_valid ?
        (nav_control_yaw_rate_radps * nav_deg_per_rad) : 0.0f;
}

uint8 nav_control_imu_is_valid(void)
{
    return nav_control_imu_valid;
}

// 路径是否播完(FINISHED 态返回 1)。
uint8 nav_is_complete(void)
{
    return (uint8)(NAV_STATE_FINISHED == nav.state);
}

// 速度决策:当前速度段的目标速度(cm/s),值来自编辑器导出的段表。语义见 navigation.h 声明处。
float nav_get_target_speed_cmps(void)
{
    float v;

    if ((NAV_STATE_NAVIGATING != nav.state) && (NAV_STATE_FINISHED != nav.state))
    {
        return -1.0f;
    }
    if ((0u == nav.speed_seg_cnt) || (NULL == nav.speed_end_p) || (NULL == nav.speed_cmps_p))
    {
        return -1.0f;
    }
    v = (float)nav.speed_cmps_p[nav.speed_seg_idx] *
        NAV_REFERENCE_SPEED_LSB_CMPS;
    /* 最后一点由nav_update先切入FINISHED，0cm/s只作为离线后向制动的终端
     * 边界；正常NAVIGATING状态读取到其它低于50cm/s的值仍判为坏地图。 */
    if ((NAV_STATE_FINISHED == nav.state) &&
        (nav.replay_index >= (uint16)(nav.point_count - 1u)) &&
        (v == NAV_SPEED_PROFILE_END_CMPS))
    {
        return v;
    }
    // 非法地图速度直接暴露为无效参考；调用方不得替换成另一速度继续运行。
    if (v < 50.0f)
    {
        return -1.0f;
    }
    return v;
}

float nav_get_speed_profile_max_accel_cmps2(void)
{
    return NAV_SPEED_PROFILE_MAX_ACCEL_CMPS2;
}

float nav_get_speed_profile_max_decel_cmps2(void)
{
    return NAV_SPEED_PROFILE_MAX_DECEL_CMPS2;
}

float nav_get_speed_profile_safety_margin_mm(void)
{
    return NAV_SPEED_PROFILE_SAFETY_MARGIN_MM;
}

void nav_fail_invalid_reference(void)
{
    if (NAV_STATE_NAVIGATING == nav.state)
    {
        nav.state = NAV_STATE_ERROR;
        nav.correction = 0.0f;
        nav_yaw_rate_tracker_reset();
        nav_capture_state_transition(NAV_STATE_NAVIGATING, nav.state);
    }
}

// 当前路线总点数。
uint16 nav_get_total_points(void)
{
    return nav.point_count;
}

uint8 nav_get_source_route_id(void)
{
    return (uint8)NAV_SOURCE_ROUTE_INDEX;
}

uint32 nav_get_route_hash(void)
{
    return nav.route_hash;
}

float nav_get_path_sample_mm(void)
{
    return NAV_PATH_SAMPLE_MM;
}

// CPU0 DSRAM 区结束,恢复默认段放置。与文件前面的 #pragma section 成对。
#pragma section all restore
