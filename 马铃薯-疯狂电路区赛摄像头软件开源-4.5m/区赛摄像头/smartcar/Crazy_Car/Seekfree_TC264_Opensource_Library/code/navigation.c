/**
 * @file navigation.c
 * @brief 地图投影、视觉定位修正和 Pure Pursuit 导航。
 * @author 高冷马铃薯
 *
 * @par 所属项目
 * 2026 智能车区赛（摄像头）
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

/*
 * navigation.c
 *
 * 惯性导航实现(纯追踪),详细说明见 navigation.h。
 * 每 1ms:编码器脉冲 + yaw_total 航位推算 (x,y) → 开窗搜索最近路径点 →
 * 前瞻点变换到车体系 → 追踪角 α 作为转向修正输出。
 */

#include "navigation.h"
#include "control.h"   /* DIST_PER_PULSE_MM */
#include "gyroscope.h" /* yaw_total */
#include "globals.h"
#include "nav_path.h"
#include "vision_fusion_math.h"
#include "calibration.h"
#include <math.h>

#ifndef NAV_HYBRID_TABLES
#error "nav_path.h 过旧：请用地图编辑器重新导出（需含摄像头区区表）"
#endif

/* 前瞻/搜索窗口折算成路径点数(采样间隔见 nav_path.h)。 */
#define NAV_PP_LOOKAHEAD_POINTS ((uint16)(NAV_PP_LOOKAHEAD_MM / NAV_PATH_SAMPLE_MM))
#define NAV_PP_SEARCH_POINTS ((uint16)(NAV_PP_SEARCH_MM / NAV_PATH_SAMPLE_MM))

/* "位于直行道"判定(nav_straight_ahead,供 CPU1 三叉补线状态机):
 * 当前回放点起前瞻窗口内路径航向偏离超过门限即置 0。
 * 窗口盖过摄像头视距(35cm)+5cm定位余量,上限0.4m(用户定):再长会把
 * "三叉后不远接弯"的路段误禁用,正好废掉需要补线的地方。 */
#define NAV_STRAIGHT_LOOKAHEAD_POINTS ((uint16)(400.0f / NAV_PATH_SAMPLE_MM))
#define NAV_STRAIGHT_YAW_TOL_0P01DEG (300) /* 3.00 度 */

enum
{
    NAV_VISION_HISTORY_CAPACITY = 32
};

typedef struct
{
    float camera_mm_per_px;
    float lateral_sign;
    float lateral_gate_mm;
    float yaw_gate_deg;
    float zone_entry_margin_mm;
    float zone_end_margin_mm;
    float lateral_gain;
    float lateral_rate_mmps;
    float heading_gain;
    float heading_rate_dps;
    uint32 maximum_age_ms;
    uint8 valid_confirm_frames;
} NavVisionConfig;

/* 这些是运行门限，不再散落成十几个无法解释的宏。
 * camera_mm_per_px=2.17:台架实测(2026-07-12)横移5cm读-23px,50/23=2.17;
 * 原写死2.5偏大15%,每帧过修15%是"过修-回拉"抖动来源之一。
 * 区头余量(zone_entry_margin_mm)=100:出弯进视觉区后 10cm 才开始纠偏
 * (用户定,2026-07-12)——出弯瞬间姿态/航向还没稳,过早吃观测容易把
 * 弯尾抖动当横向偏差修进坐标。
 * 航向门(yaw_gate_deg,±20°)加回(2026-07-12):车头相对路线航向偏超过
 * 20° 就拒绝吃这帧观测——防止车已明显偏斜时纠偏还往里灌,把位置越拉越
 * 偏、最终纯追踪转向打满甩一个电机满转(曾删除,现因该失控风险恢复)。 */
/* lateral_rate_mmps 注入限速演化:300(完赛)→1000(2026-07-12,5.7mm/帧,
 * 实车直道纠偏太陡:1m/s横向拉动≈16°切回角)→500(2.8mm/帧,切回角≈8°)。
 * 陡峭度只由本值决定(增益在大误差时不起作用,都顶在限速上);
 * 还陡降回300,嫌慢升700,一次只动一档。 */
static const NavVisionConfig nav_vision_config = {
    2.17f, 1.0f, 250.0f, 20.0f, 100.0f, 350.0f,
    1.0f, 500.0f, 0.08f, 30.0f, 20u, 2u};

static const float nav_deg_per_rad = 57.29577951f;
static const float nav_rad_per_deg = 0.017453293f;

#pragma section all "cpu0_dsram"

typedef struct
{
    NavSystemState state;
    const signed short *data; /* 当前路线航向数据 */
    const signed short *xs;   /* 当前路线 x 坐标(mm,发车系) */
    const signed short *ys;   /* 当前路线 y 坐标(mm,发车系) */
    uint16 point_count;       /* 当前路线点数 */
    uint16 replay_index;      /* 进度:当前最近路径点序号 */
    float pos_x_mm;           /* 航位推算 x(发车系,mm) */
    float pos_y_mm;           /* 航位推算 y(发车系,mm,右侧为正) */
    float correction;         /* 输出修正(度) */
    float max_abs_correction; /* 本圈 |修正| 峰值(度)，停车后看跟踪最差点 */

    /* 摄像头区横向纠偏。区表指针随 nav_load_path 装载。 */
    const uint16 *zone_lo_p; /* 摄像头区起点(样本下标) */
    const uint16 *zone_hi_p; /* 摄像头区终点(=弯起点) */
    uint8 zone_cnt;
    /* 禁校区:地图 nocal 节点(赛道元件等污染摄像头图像处)入前一段的
     * 样本区间,区内横向纠偏一律拒绝,坐标只信赖惯导。提前量由编辑器
     * 烘进表里(默认 350mm=摄像头地面视距,元件进视野即停止吃观测)。 */
    const uint16 *nocal_lo_p;
    const uint16 *nocal_hi_p;
    uint8 nocal_cnt;
    float lat_corr_mm; /* 本圈累计横向纠偏(带符号,右手方向为正),调试显示 */
    float heading_bias_deg;
    uint32 crossing_count; /* 本圈摄像头检测到直行路口的次数(上升沿),验证/调试 */
    uint8 crossing_last;   /* 上一帧观测的路口标志,用于取上升沿 */
    uint32 vision_last_seq;
    uint32 vision_last_frame_seq;
    uint32 vision_session_capture_seq;
    uint32 vision_last_capture_ms;
    uint32 vision_last_age_ms;
    uint32 vision_accept_count;
    uint32 vision_drop_count;
    float vision_last_fit_mse_px2;
    uint16 vision_last_valid_rows;
    uint16 vision_last_flags;
    uint8 vision_last_drop_reason;
    uint8 vision_valid_count;
} NavigationSystem;

/* 单写(1ms 速度环)多读(菜单/后台任务),字段按 32 位对齐读取。 */
static volatile NavigationSystem nav;

#if NAV_VISION_FUSION_ENABLE
typedef struct
{
    uint32 tick_ms;
    float pos_x_mm;
    float pos_y_mm;
    float yaw_deg;
    uint16 replay_index;
} NavPoseHistory;

static NavPoseHistory nav_pose_history[NAV_VISION_HISTORY_CAPACITY];
static uint8 nav_pose_history_head = 0u;
static uint8 nav_pose_history_count = 0u;
#endif

/* 装载指定路线的数据指针和点数。 */
static void nav_load_path(uint8 path_id)
{
    nav.data = nav_path_data[path_id];
    nav.xs = nav_path_x[path_id];
    nav.ys = nav_path_y[path_id];
    nav.point_count = nav_path_point_count[path_id];

    nav.zone_cnt = nav_zone_count[path_id];
    nav.zone_lo_p = nav_zone_lo[path_id];
    nav.zone_hi_p = nav_zone_hi[path_id];
#ifdef NAV_NOCAL_TABLES
    nav.nocal_cnt = nav_nocal_count[path_id];
    nav.nocal_lo_p = nav_nocal_lo[path_id];
    nav.nocal_hi_p = nav_nocal_hi[path_id];
#else
    /* 旧版 nav_path.h 无禁校表:0 段,门恒放行 */
    nav.nocal_cnt = 0u;
    nav.nocal_lo_p = NULL;
    nav.nocal_hi_p = NULL;
#endif
}

static void nav_vision_session_reset(void)
{
    uint32 publish_seq = vision_pose_publish_seq;
    uint32 capture_seq = vision_capture_seq;

    nav.lat_corr_mm = 0.0f;
    nav.heading_bias_deg = 0.0f;
    nav.crossing_count = 0u;
    nav.crossing_last = 0u;
    nav.vision_last_seq = publish_seq;
    nav.vision_last_frame_seq = capture_seq;
    nav.vision_session_capture_seq = capture_seq;
    nav.vision_last_capture_ms = 0u;
    nav.vision_last_age_ms = 0u;
    nav.vision_accept_count = 0u;
    nav.vision_drop_count = 0u;
    nav.vision_last_fit_mse_px2 = 0.0f;
    nav.vision_last_valid_rows = 0u;
    nav.vision_last_flags = 0u;
    nav.vision_last_drop_reason = NAV_VISION_DROP_NONE;
    nav.vision_valid_count = 0u;
#if NAV_VISION_FUSION_ENABLE
    nav_pose_history_head = 0u;
    nav_pose_history_count = 0u;
#endif
}

static void nav_vision_stop_reset(void)
{
    uint32 publish_seq = vision_pose_publish_seq;
    uint32 capture_seq = vision_capture_seq;

    nav.heading_bias_deg = 0.0f;
    nav.vision_last_seq = publish_seq;
    nav.vision_last_frame_seq = capture_seq;
    nav.vision_session_capture_seq = capture_seq;
    nav.vision_last_capture_ms = 0u;
    nav.vision_valid_count = 0u;
#if NAV_VISION_FUSION_ENABLE
    nav_pose_history_head = 0u;
    nav_pose_history_count = 0u;
#endif
}

static float nav_fused_yaw_deg(void)
{
    return yaw_total + nav.heading_bias_deg;
}

void nav_init(void)
{
    nav.state = NAV_STATE_IDLE;
    nav.replay_index = 0u;
    nav.correction = 0.0f;
    nav_load_path(0u);
    nav_vision_session_reset();
}

uint8 nav_select_path(uint8 path_id)
{
    if (path_id >= (uint8)NAV_PATH_COUNT)
    {
        return 0u;
    }
    if (NAV_STATE_NAVIGATING == nav.state)
    {
        return 0u;
    }

    nav_load_path(path_id);
    return 1u;
}

void nav_start(void)
{
    if ((0u == nav.point_count) || (NULL == nav.data))
    {
        nav.state = NAV_STATE_ERROR;
        return;
    }

    nav.replay_index = 0u;
    nav.pos_x_mm = 0.0f;
    nav.pos_y_mm = 0.0f;
    nav.correction = 0.0f;
    nav.max_abs_correction = 0.0f;
    nav_vision_session_reset();
    /* 航向基准:control_enable() 已发出 yaw_total 归零请求,
     * IMU 中断随即清零,发车朝向即 0 度,这里不再重复请求。 */
    nav.state = NAV_STATE_NAVIGATING;
}

void nav_stop(void)
{
    nav.state = NAV_STATE_IDLE;
    nav.correction = 0.0f;
    nav_vision_stop_reset();
}

static float nav_wrap_deg(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

#if NAV_VISION_FUSION_ENABLE
static uint8 nav_read_vision_observation(VisionPoseObservation *out)
{
    uint32 previous_sequence = nav.vision_last_seq;

    if (!vision_observation_read_new(&nav.vision_last_seq, out))
    {
        return 0u;
    }
    if ((uint32)(out->publish_seq - previous_sequence) != 1u)
    {
        nav.vision_valid_count = 0u;
    }
    return 1u;
}

static void nav_store_pose_history(uint32 tick_ms)
{
    NavPoseHistory *sample = &nav_pose_history[nav_pose_history_head];

    sample->tick_ms = tick_ms;
    sample->pos_x_mm = nav.pos_x_mm;
    sample->pos_y_mm = nav.pos_y_mm;
    sample->yaw_deg = nav_fused_yaw_deg();
    sample->replay_index = nav.replay_index;
    nav_pose_history_head = (uint8)((nav_pose_history_head + 1u) % NAV_VISION_HISTORY_CAPACITY);
    if (nav_pose_history_count < NAV_VISION_HISTORY_CAPACITY)
    {
        nav_pose_history_count++;
    }
}

static uint8 nav_find_capture_pose(uint32 capture_ms, NavPoseHistory *out)
{
    uint8 i;
    uint8 found = 0u;
    uint32 best_age = 0xFFFFFFFFu;

    for (i = 0u; i < nav_pose_history_count; i++)
    {
        uint8 index = (uint8)((nav_pose_history_head + NAV_VISION_HISTORY_CAPACITY - 1u - i) %
                              NAV_VISION_HISTORY_CAPACITY);
        uint32 sample_after_ms =
            (uint32)(nav_pose_history[index].tick_ms - capture_ms);
        uint32 sample_before_ms =
            (uint32)(capture_ms - nav_pose_history[index].tick_ms);
        uint32 age = (sample_after_ms < sample_before_ms)
                         ? sample_after_ms
                         : sample_before_ms;

        if ((age <= nav_vision_config.maximum_age_ms) && (age < best_age))
        {
            *out = nav_pose_history[index];
            best_age = age;
            found = 1u;
        }
    }
    return found;
}

static void nav_shift_pose_history_after_capture(uint32 capture_ms, float dx_mm, float dy_mm)
{
    uint8 i;

    for (i = 0u; i < nav_pose_history_count; i++)
    {
        uint8 index = (uint8)((nav_pose_history_head + NAV_VISION_HISTORY_CAPACITY - 1u - i) %
                              NAV_VISION_HISTORY_CAPACITY);
        uint32 age_from_capture =
            vision_age_ms(nav_pose_history[index].tick_ms, capture_ms);

        if (age_from_capture <= nav_vision_config.maximum_age_ms)
        {
            nav_pose_history[index].pos_x_mm += dx_mm;
            nav_pose_history[index].pos_y_mm += dy_mm;
        }
    }
}

static uint8 nav_index_in_vision_zone(uint16 replay_index)
{
    uint16 entry = (uint16)(nav_vision_config.zone_entry_margin_mm /
                            NAV_PATH_SAMPLE_MM);
    uint16 end_margin = (uint16)(nav_vision_config.zone_end_margin_mm /
                                 NAV_PATH_SAMPLE_MM);
    uint8 i;

    for (i = 0u; i < nav.zone_cnt; i++)
    {
        if ((replay_index >= (uint16)(nav.zone_lo_p[i] + entry)) &&
            ((uint16)(replay_index + end_margin) <= nav.zone_hi_p[i]))
        {
            return 1u;
        }
    }
    return 0u;
}

/* 禁校区判定:区间已含提前量,命中即拒绝本帧观测。与视觉区门相反,
 * 这是减法门——落在摄像头区内也不行,元件污染的线宁可不修。 */
static uint8 nav_index_in_nocal_zone(uint16 replay_index)
{
    uint8 i;

    for (i = 0u; i < nav.nocal_cnt; i++)
    {
        if ((replay_index >= nav.nocal_lo_p[i]) &&
            (replay_index <= nav.nocal_hi_p[i]))
        {
            return 1u;
        }
    }
    return 0u;
}

static void nav_reject_vision_observation(VisionRejectReason reason)
{
    nav.vision_valid_count = 0u;
    nav.vision_last_drop_reason = reason;
    nav.vision_drop_count++;
}

static void nav_hybrid_update(void)
{
    VisionPoseObservation obs;
    NavPoseHistory captured;
    uint32 age_ms;
    uint32 frame_dt_ms;
    float path_yaw_deg;
    float yaw_error_deg;
    float path_yaw_rad;
    float normal_x;
    float normal_y;
    float line_cross;
    float estimated_cross;
    float measured_cross;
    float innovation;

    if (!nav_read_vision_observation(&obs))
    {
        return;
    }

    if ((uint32)(obs.frame_seq - nav.vision_last_frame_seq) != 1u)
    {
        nav.vision_valid_count = 0u;
    }
    nav.vision_last_frame_seq = obs.frame_seq;
    nav.vision_last_fit_mse_px2 = obs.fit_mse_px2;
    nav.vision_last_valid_rows = obs.valid_rows;
    nav.vision_last_flags = obs.flags;
    /* 路口检测上升沿计数(在所有门之前,不受采纳与否影响):穿越路口时
     * 拟合常失效、观测会被质量门丢,但路口标志本身仍要独立统计,供验证
     * "摄像头在哪些地方看到了路口"以及后续地图门控弧长重置消费。 */
    {
        uint8 crossing_now = (uint8)(0u != (obs.flags & VISION_OBS_FLAG_CROSSING));
        if (crossing_now && !nav.crossing_last)
        {
            nav.crossing_count++;
        }
        nav.crossing_last = crossing_now;
    }
    age_ms = vision_age_ms(vision_clock_ms, obs.capture_ms);
    nav.vision_last_age_ms = age_ms;
    frame_dt_ms = (0u == nav.vision_last_capture_ms)
                      ? 5u
                      : vision_age_ms(obs.capture_ms, nav.vision_last_capture_ms);
    if ((0u != nav.vision_last_capture_ms) &&
        (frame_dt_ms > nav_vision_config.maximum_age_ms))
    {
        nav.vision_valid_count = 0u;
    }
    nav.vision_last_capture_ms = obs.capture_ms;
    if (frame_dt_ms < 1u)
    {
        frame_dt_ms = 1u;
    }
    if (frame_dt_ms > nav_vision_config.maximum_age_ms)
    {
        frame_dt_ms = nav_vision_config.maximum_age_ms;
    }

    if ((int32)(obs.frame_seq - nav.vision_session_capture_seq) <= 0)
    {
        nav_reject_vision_observation(NAV_VISION_DROP_PRESESSION);
        return;
    }
    if (age_ms > nav_vision_config.maximum_age_ms)
    {
        nav_reject_vision_observation(NAV_VISION_DROP_STALE);
        return;
    }
    /* SIDE_OPEN 是三岔诊断信息，不再一票否决。当前路线索引必须先落在编辑器
     * 导出的直行视觉区，且观测仍须通过完整拟合、航向和创新量质量门。 */
    if ((obs.flags & (VISION_OBS_FLAG_LATERAL_VALID |
                      VISION_OBS_FLAG_FIT_VALID |
                      VISION_OBS_FLAG_COVERAGE |
                      VISION_OBS_FLAG_EDGE_VALID)) !=
        (VISION_OBS_FLAG_LATERAL_VALID |
         VISION_OBS_FLAG_FIT_VALID |
         VISION_OBS_FLAG_COVERAGE |
         VISION_OBS_FLAG_EDGE_VALID))
    {
        nav_reject_vision_observation(NAV_VISION_DROP_QUALITY);
        return;
    }
    if (!nav_find_capture_pose(obs.capture_ms, &captured))
    {
        nav_reject_vision_observation(NAV_VISION_DROP_HISTORY);
        return;
    }
    if (!nav_index_in_vision_zone(captured.replay_index))
    {
        nav_reject_vision_observation(NAV_VISION_DROP_ZONE);
        return;
    }
    if (nav_index_in_nocal_zone(captured.replay_index))
    {
        nav_reject_vision_observation(NAV_VISION_DROP_NOCAL);
        return;
    }

    path_yaw_deg = (float)nav.data[captured.replay_index] * 0.01f;
    /* 航向门:采集时刻车头航向与路线应有航向差超过 ±yaw_gate_deg 就拒绝
     * 本帧观测——车已明显偏斜时纠偏不再往里灌,避免位置越拉越偏、纯追踪
     * 转向打满甩一个电机满转。yaw_deg 与 path_yaw_deg 同为相对发车朝向。 */
    yaw_error_deg = nav_wrap_deg(captured.yaw_deg - path_yaw_deg);
    if ((yaw_error_deg > nav_vision_config.yaw_gate_deg) ||
        (yaw_error_deg < -nav_vision_config.yaw_gate_deg))
    {
        nav_reject_vision_observation(NAV_VISION_DROP_YAW);
        return;
    }

    path_yaw_rad = path_yaw_deg * nav_rad_per_deg;
    normal_x = -sinf(path_yaw_rad);
    normal_y = cosf(path_yaw_rad);
    line_cross = (float)nav.xs[captured.replay_index] * normal_x +
                 (float)nav.ys[captured.replay_index] * normal_y;
    estimated_cross = captured.pos_x_mm * normal_x + captured.pos_y_mm * normal_y;
    measured_cross = line_cross -
                     nav_vision_config.lateral_sign * obs.line_position_px *
                         nav_vision_config.camera_mm_per_px;
    innovation = measured_cross - estimated_cross;
    if ((innovation > nav_vision_config.lateral_gate_mm) ||
        (innovation < -nav_vision_config.lateral_gate_mm))
    {
        nav_reject_vision_observation(NAV_VISION_DROP_INNOVATION);
        return;
    }

    nav.vision_last_drop_reason = NAV_VISION_DROP_NONE;
    if (nav.vision_valid_count < nav_vision_config.valid_confirm_frames)
    {
        nav.vision_valid_count++;
    }
    if (nav.vision_valid_count < nav_vision_config.valid_confirm_frames)
    {
        return;
    }

    {
        float step = vision_rate_limited_step(innovation,
                                              nav_vision_config.lateral_gain,
                                              nav_vision_config.lateral_rate_mmps,
                                              frame_dt_ms);
        float shift_x = normal_x * step;
        float shift_y = normal_y * step;

        nav.pos_x_mm += shift_x;
        nav.pos_y_mm += shift_y;
        nav_shift_pose_history_after_capture(obs.capture_ms, shift_x, shift_y);
        nav.lat_corr_mm += step;
    }

    /* 只有模型已保存且页面明确切到 ON，才让相机航向慢慢修正偏置。 */
    if (camera_heading_model_valid() && camera_heading_fusion_enabled())
    {
        float camera_relative_deg = camera_heading_estimate_deg(
            obs.line_position_px, obs.line_slope_px_per_row);
        float heading_measurement = path_yaw_deg + camera_relative_deg;
        float heading_innovation = nav_wrap_deg(heading_measurement -
                                                captured.yaw_deg);
        float heading_step = vision_rate_limited_step(heading_innovation,
                                                      nav_vision_config.heading_gain,
                                                      nav_vision_config.heading_rate_dps,
                                                      frame_dt_ms);
        nav.heading_bias_deg = nav_wrap_deg(nav.heading_bias_deg + heading_step);
    }
    nav.vision_accept_count++;
}
#else
static void nav_hybrid_update(void)
{
}
#endif /* NAV_VISION_FUSION_ENABLE */

/* 发布"位于直行道"标志:回放索引变化时重算一次,1ms 环内代价可忽略。
 * 航向差按 ±180° 环绕后与门限比较。 */
static void nav_publish_straight_ahead(uint16 index)
{
    static uint16 nav_straight_last_index = 0xFFFFu;
    uint16 end;
    uint16 i;
    int32 ref;
    uint8 straight = 1u;

    if (index == nav_straight_last_index)
    {
        return;
    }
    nav_straight_last_index = index;

    end = (uint16)(index + NAV_STRAIGHT_LOOKAHEAD_POINTS);
    if (end >= nav.point_count)
    {
        end = (uint16)(nav.point_count - 1u);
    }
    ref = (int32)nav.data[index];
    for (i = index; i <= end; i++)
    {
        int32 d = (int32)nav.data[i] - ref;

        while (d > 18000)
        {
            d -= 36000;
        }
        while (d < -18000)
        {
            d += 36000;
        }
        if ((d > NAV_STRAIGHT_YAW_TOL_0P01DEG) ||
            (d < -NAV_STRAIGHT_YAW_TOL_0P01DEG))
        {
            straight = 0u;
            break;
        }
    }
    nav_straight_ahead = straight;
}

void nav_update(int16 pulses_l, int16 pulses_r)
{
    if (NAV_STATE_NAVIGATING != nav.state)
    {
        /* 非导航态不否决三叉补线(台架观察用),见 globals.h。 */
        nav_straight_ahead = 1u;
        return;
    }

    /* 双轮平均脉冲折算里程增量(mm),前进为正。 */
    float delta_mm = (float)(pulses_l + pulses_r) * (0.5f * DIST_PER_PULSE_MM);
    float navigation_yaw_deg = nav_fused_yaw_deg();

    /* 航位推算:发车坐标系,x=发车朝向,y=其右侧,θ=yaw_total(右转为正)。
     * 编辑器导出的 nav_path_x/y 用同一坐标系定义,两边天然对齐。 */
    {
        float theta = navigation_yaw_deg * nav_rad_per_deg;
        nav.pos_x_mm += delta_mm * cosf(theta);
        nav.pos_y_mm += delta_mm * sinf(theta);
    }

    /* 最近路径点:从上次进度向前开窗搜索。索引单调不回退,
     * 窗口有限,自交/原路折返的路线不会跳到错误的圈次。 */
    {
        uint16 start = nav.replay_index;
        uint16 end = (uint16)(start + NAV_PP_SEARCH_POINTS);
        uint16 best = start;
        float best_d2 = 1e30f;
        uint16 i;

        if (end >= nav.point_count)
        {
            end = (uint16)(nav.point_count - 1u);
        }
        for (i = start; i <= end; i++)
        {
            float dx = (float)nav.xs[i] - nav.pos_x_mm;
            float dy = (float)nav.ys[i] - nav.pos_y_mm;
            float d2 = dx * dx + dy * dy;

            if (d2 < best_d2)
            {
                best_d2 = d2;
                best = i;
            }
        }
        nav.replay_index = best;
    }

    nav_publish_straight_ahead(nav.replay_index);

#if NAV_VISION_FUSION_ENABLE
    nav_store_pose_history(vision_clock_ms);
#endif

    /* 播完:最近点推进到路径末点,由 Duty_10ms 消费 nav_is_complete() 停车。 */
    if (nav.replay_index >= (uint16)(nav.point_count - 1u))
    {
        nav.correction = 0.0f;
        nav.state = NAV_STATE_FINISHED;
        return;
    }

    /* 摄像头区横向纠偏(限步微调坐标横向分量)。 */
    nav_hybrid_update();

    /* 前瞻点变换到车体系,追踪角 α = atan2(横向,纵向):
     * 点在车头右侧 α 为正 → 右转,与转向环 turn_diff 同号。
     * 横向偏差会自动体现在 α 里,被闭环拉回路径——这是相对航向回放的核心改进。 */
    {
        uint16 ti = (uint16)(nav.replay_index + NAV_PP_LOOKAHEAD_POINTS);
        float theta = navigation_yaw_deg * nav_rad_per_deg;
        float c = cosf(theta);
        float s = sinf(theta);
        float dx;
        float dy;
        float forward;
        float lateral;

        if (ti >= nav.point_count)
        {
            ti = (uint16)(nav.point_count - 1u);
        }
        dx = (float)nav.xs[ti] - nav.pos_x_mm;
        dy = (float)nav.ys[ti] - nav.pos_y_mm;
        forward = dx * c + dy * s;
        lateral = -dx * s + dy * c;

        nav.correction = atan2f(lateral, forward) * nav_deg_per_rad;
    }

    /* 峰值误差锁存：跑完后在调试页看本圈跟踪最差多少度。 */
    {
        float abs_correction = nav.correction;

        if (abs_correction < 0.0f)
        {
            abs_correction = -abs_correction;
        }
        if (abs_correction > nav.max_abs_correction)
        {
            nav.max_abs_correction = abs_correction;
        }
    }
}

NavSystemState nav_get_state(void)
{
    return nav.state;
}

float nav_get_lat_corr_mm(void)
{
    return nav.lat_corr_mm;
}

float nav_get_heading_bias_deg(void)
{
    return nav.heading_bias_deg;
}

uint32 nav_get_crossing_count(void)
{
    return nav.crossing_count;
}

uint32 nav_get_vision_age_ms(void)
{
    return nav.vision_last_age_ms;
}

uint32 nav_get_vision_accept_count(void)
{
    return nav.vision_accept_count;
}

uint32 nav_get_vision_drop_count(void)
{
    return nav.vision_drop_count;
}

VisionRejectReason nav_get_vision_last_drop_reason(void)
{
    return (VisionRejectReason)nav.vision_last_drop_reason;
}

uint16 nav_get_vision_valid_rows(void)
{
    return nav.vision_last_valid_rows;
}

uint16 nav_get_vision_flags(void)
{
    return nav.vision_last_flags;
}

float nav_get_vision_fit_mse_px2(void)
{
    return nav.vision_last_fit_mse_px2;
}

float nav_get_correction(void)
{
    return nav.correction;
}

uint8 nav_is_complete(void)
{
    return (uint8)(NAV_STATE_FINISHED == nav.state);
}

uint16 nav_get_progress(void)
{
    return nav.replay_index;
}

uint16 nav_get_total_points(void)
{
    return nav.point_count;
}

float nav_get_max_abs_correction(void)
{
    return nav.max_abs_correction;
}

#pragma section all restore
