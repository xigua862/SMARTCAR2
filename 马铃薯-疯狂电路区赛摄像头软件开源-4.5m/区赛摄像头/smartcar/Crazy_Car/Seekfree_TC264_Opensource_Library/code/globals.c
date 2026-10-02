/**
 * @file globals.c
 * @brief 模块间及双核之间共享的数据、队列和快照接口。
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

#include "globals.h"
#include "camera_pipeline.h"

#pragma section all "cpu0_dsram"

/* ==================== 控制运行变量 ==================== */
float left_TargetSpeed = 0.0f;        /* 左轮 PI 输出 */
float right_TargetSpeed = 0.0f;       /* 右轮 PI 输出 */
float max_duty_limit = 9999.0f;       /* 输出软限幅。 */
volatile float actual_l_debug = 0.0f; /* 左轮最近一次速度估计。 */
volatile float actual_r_debug = 0.0f; /* 右轮最近一次速度估计。 */
float turn_control = 0.0f;            /* 转向控制量。 */

/* ==================== PID 对象 ====================
 * 这里只做全局定义，具体参数在 control_init() 中完成初始化。
 */
PID turn_PID;   /* 转向 PID */
PID speed_PID1; /* 左轮速度 PI */
PID speed_PID2; /* 右轮速度 PI */

/* ==================== 决策与中间量 ==================== */
volatile float total_dist = 0.0f;     /* 本次发车累计行驶距离，单位 m；motor_read_*_count() 内累加，仅用于菜单显示。 */
volatile uint32 time_cost = 0u;       /* 最近一帧图像处理耗时。 */
volatile uint8 image_stop_request = 0u; /* 图像异常停车请求。 */
volatile uint8 motor_fault_request = 0u; /* 电机/编码器失控停车请求：1ms 速度环置位，Duty_10ms 消费。 */
volatile uint32 vision_clock_ms = 0u;
volatile uint32 vision_capture_seq = 0u;
volatile uint32 vision_processed_seq = 0u;
/* CPU0 VSYNC -> CPU1 图像任务的双槽采集元数据。
 * 每槽版本号奇数表示正在写，偶数表示可读；对外只经 camera_pipeline API。 */
IFX_ALIGN(4) volatile CameraCaptureMetadata vision_capture_metadata_slots[2];
volatile uint32 vision_capture_metadata_versions[2];
IFX_ALIGN(4) volatile VisionPoseObservation vision_pose_slots[2];
volatile uint32 vision_pose_publish_seq = 0u;

uint8 vision_observation_read_new(volatile uint32 *last_publish_seq,
                                  VisionPoseObservation *observation)
{
    uint8 retry;

    if ((NULL == last_publish_seq) || (NULL == observation))
    {
        return 0u;
    }
    for (retry = 0u; retry < 3u; retry++)
    {
        uint32 sequence_before = vision_pose_publish_seq;
        uint8 slot;
        uint32 sequence_after;

        if ((0u == sequence_before) || (sequence_before == *last_publish_seq))
        {
            return 0u;
        }
        slot = (uint8)(sequence_before & 1u);
        __dsync();
        observation->publish_seq = vision_pose_slots[slot].publish_seq;
        observation->frame_seq = vision_pose_slots[slot].frame_seq;
        observation->capture_ms = vision_pose_slots[slot].capture_ms;
        observation->capture_gyro_yaw_deg =
            vision_pose_slots[slot].capture_gyro_yaw_deg;
        observation->line_position_px = vision_pose_slots[slot].line_position_px;
        observation->line_slope_px_per_row =
            vision_pose_slots[slot].line_slope_px_per_row;
        observation->fit_mse_px2 = vision_pose_slots[slot].fit_mse_px2;
        observation->valid_rows = vision_pose_slots[slot].valid_rows;
        observation->candidate_rows = vision_pose_slots[slot].candidate_rows;
        observation->row_span = vision_pose_slots[slot].row_span;
        observation->flags = vision_pose_slots[slot].flags;
        __dsync();
        sequence_after = vision_pose_publish_seq;
        if ((sequence_before == sequence_after) &&
            (observation->publish_seq == sequence_before))
        {
            *last_publish_seq = sequence_before;
            return 1u;
        }
    }
    return 0u;
}

/* ==================== 摄像头区横向纠偏跨核通道(注释见 globals.h) ==================== */
volatile float image_lat_offset_px = 0.0f;
volatile uint8 image_lat_valid = 0u;
volatile uint8 nav_straight_ahead = 1u; /* 见 globals.h；导航中由 nav_update() 维护 */
volatile uint8 g_stop_reason = 0u;      /* 最近一次停车原因码，见 globals.h 的 STOP_REASON_*。 */

#pragma section all restore
