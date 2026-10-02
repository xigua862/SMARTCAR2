/**
 * @file globals.c
 * @brief 双核 IMU、光电观测和扫图队列的共享发布接口。
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

// globals.c —— 为 globals.h 的跨核共享变量分配内存并赋初值。
// CPU0控制核与CPU1光电扫描核通过这些变量互通。Flash参数结构体在flash.c。

#include "globals.h"

// 控制状态先放CPU0 DSRAM；下方CPU1发布槽会显式切到cpu1_dsram，再切回CPU0。
#pragma section all "cpu0_dsram"

// 控制运行变量（带 _debug 的仅供菜单/示波器观测）
float left_target_speed = 0.0f;        // 左轮 PI 输出
float right_target_speed = 0.0f;       // 右轮 PI 输出
float max_duty_limit = 10000.0f;       // 输出软限幅
volatile float actual_l_debug = 0.0f; // 左轮最近一次速度估计
volatile float actual_r_debug = 0.0f; // 右轮最近一次速度估计
volatile float plan_v_debug_l = 0.0f; // 速度规划当前值
volatile float plan_v_debug_r = 0.0f; // 速度规划当前值
float speed = 0.0f;                   // 当前目标速度
float turn_control = 0.0f;            // 转向控制量

// 速度 PI 对象（参数在 control_init() 初始化）
PID speed_PID1; // 左轮速度 PI
PID speed_PID2; // 右轮速度 PI

// 决策与中间量：两核间传结论/请求的核心消息位
volatile float total_dist = 0.0f;     // 本次发车累计里程(m)，motor_read_*_count() 内累加，仅菜单显示
volatile uint32 encoder_motion_seq = 0u; // 任一逻辑轮增量非零即推进，供校准/手动发车静止判定
volatile uint8 motor_fault_request = 0u; // 电机/编码器失控停车请求：1ms 速度环置位，Duty_10ms 消费
volatile uint32 imu_snapshot_read_failure_count = 0u;
volatile uint16 imu_snapshot_consecutive_failure_count = 0u;
volatile uint16 imu_snapshot_max_consecutive_failure_count = 0u;
volatile uint32 imu_snapshot_last_age_us = 0u;
volatile uint32 imu_snapshot_max_age_us = 0u;
volatile uint32 control_exec_time_us = 0u;
volatile uint32 control_exec_max_us = 0u;
volatile uint32 control_exec_overrun_count = 0u;
volatile uint32 control_period_us = 0u;
volatile uint32 control_period_max_us = 0u;
volatile uint32 control_period_overrun_count = 0u;
volatile uint32 duty_10ms_exec_time_us = 0u;
volatile uint32 duty_10ms_exec_max_us = 0u;
volatile uint32 duty_10ms_overrun_count = 0u;
volatile uint32 nav_match_exec_time_us = 0u;
volatile uint32 nav_match_exec_max_us = 0u;
volatile uint32 nav_match_overrun_count = 0u;
volatile uint32 telemetry_service_exec_time_us = 0u;
volatile uint32 telemetry_service_exec_max_us = 0u;
volatile uint32 telemetry_service_overrun_count = 0u;
volatile uint32 imu_command_publish_seq = 0u;
volatile uint16 imu_command_flags = IMU_COMMAND_NONE;
volatile uint8 imu_telemetry_tail = 0u;
volatile uint32 imu_telemetry_epoch_seq = 0u;
volatile uint32 imu_telemetry_session_start_us = 0u;
volatile uint8 imu_telemetry_capture_enabled = 0u;
volatile uint32 vision_clock_ms = 0u;
volatile uint32 vision_scan_run_epoch = 0u;

/* CPU1 is the sole writer of these publications.  Keeping payload slots in
 * CPU1 DSRAM makes both the 480 Hz IMU publisher and the 1 kHz phototube
 * publisher local-memory writers; CPU0 performs read-only snapshot copies. */
#pragma section all restore
#pragma section all "cpu1_dsram"

IFX_ALIGN(4) volatile ImuSnapshot
    imu_snapshot_slots[IMU_SNAPSHOT_SLOT_COUNT];
volatile uint32 imu_snapshot_publish_seq = 0u;
volatile uint32 imu_command_ack_seq = 0u;
volatile uint8 imu_service_ready = 0u;
volatile uint32 imu_sample_failure_count = 0u;
volatile uint32 imu_sample_gap_count = 0u;
volatile uint32 imu_isr_exec_time_us = 0u;
volatile uint32 imu_isr_exec_max_us = 0u;
volatile uint32 imu_isr_overrun_count = 0u;
IFX_ALIGN(4) volatile TelemetryImuRecord
    imu_telemetry_queue[IMU_TELEMETRY_QUEUE_CAPACITY];
volatile uint8 imu_telemetry_head = 0u;
volatile uint8 imu_telemetry_producer_active = 0u;
volatile uint16 imu_telemetry_producer_drop_count = 0u;
volatile uint32 imu_telemetry_producer_epoch_seq = 0u;
volatile uint32 time_cost = 0u;       // CPU1最近一次完整16路扫描处理耗时(us)
volatile uint32 phototube_scan_exec_max_us = 0u;
volatile uint32 phototube_scan_overrun_count = 0u;
volatile uint32 vision_capture_seq = 0u;
volatile uint32 vision_capture_ms = 0u;
volatile uint32 vision_processed_seq = 0u;
volatile uint32 vision_line_last_white_frame_seq = 0u;
volatile uint32 vision_line_last_white_epoch = 0u;
volatile uint32 vision_line_stats_seq = 0u;
volatile uint32 vision_line_stats_epoch = 0u;
volatile uint32 vision_line_stats_version = 0u;
volatile uint32 vision_capture_drop_count = 0u;
volatile uint32 vision_capture_skipped_count = 0u;
volatile uint32 vision_capture_adc_failure_count = 0u;
volatile uint32 phototube_menu_frame_seq = 0u;
volatile uint8 phototube_menu_ready_signal = 0u;
IFX_ALIGN(4) volatile VisionPoseObservation
    vision_pose_slots[VISION_OBSERVATION_QUEUE_CAPACITY];
volatile uint32 vision_pose_publish_seq = 0u;
volatile uint8 vision_pose_publish_slot =
    (uint8)(VISION_OBSERVATION_QUEUE_CAPACITY - 1u);
IFX_ALIGN(4) volatile VisionScanHitPacket
    vision_scan_hit_slots[VISION_SCAN_HIT_QUEUE_CAPACITY];
volatile uint32 vision_scan_hit_publish_seq = 0u;
volatile uint8 vision_scan_hit_publish_slot =
    (uint8)(VISION_SCAN_HIT_QUEUE_CAPACITY - 1u);
volatile uint32 vision_scan_run_epoch_ack = 0u;

#pragma section all restore
#pragma section all "cpu0_dsram"

static void vision_observation_copy_slot(uint8 slot,
                                         VisionPoseObservation *observation)
{
    observation->publish_seq = vision_pose_slots[slot].publish_seq;
    observation->frame_seq = vision_pose_slots[slot].frame_seq;
    observation->capture_ms = vision_pose_slots[slot].capture_ms;
    observation->capture_epoch = vision_pose_slots[slot].capture_epoch;
    observation->capture_gyro_yaw_deg =
        vision_pose_slots[slot].capture_gyro_yaw_deg;
    observation->line_position_px = vision_pose_slots[slot].line_position_px;
    observation->line_position_mm = vision_pose_slots[slot].line_position_mm;
    observation->line_slope_px_per_row =
        vision_pose_slots[slot].line_slope_px_per_row;
    observation->fit_mse_px2 = vision_pose_slots[slot].fit_mse_px2;
    observation->valid_rows = vision_pose_slots[slot].valid_rows;
    observation->candidate_rows = vision_pose_slots[slot].candidate_rows;
    observation->row_span = vision_pose_slots[slot].row_span;
    observation->flags = vision_pose_slots[slot].flags;
    observation->white_percent = vision_pose_slots[slot].white_percent;
    observation->reserved[0] = vision_pose_slots[slot].reserved[0];
    observation->reserved[1] = vision_pose_slots[slot].reserved[1];
    observation->reserved[2] = vision_pose_slots[slot].reserved[2];
    observation->active_mask = vision_pose_slots[slot].active_mask;
    observation->peak_count = vision_pose_slots[slot].peak_count;
    observation->threshold_gray = vision_pose_slots[slot].threshold_gray;
}

// 导航只取最新观测。读前后全局发布序号一致，说明读取期间没有切换最新槽。
uint8 vision_observation_read_new(volatile uint32 *last_publish_seq,
                                  VisionPoseObservation *observation)
{
    uint8 retry;

    if ((NULL == last_publish_seq) || (NULL == observation))
    {
        return 0u;
    }
    // 最多 3 次：读过程中恰好被 CPU1 覆盖就重来
    for (retry = 0u; retry < 3u; retry++)
    {
        uint32 sequence_before = vision_pose_publish_seq; // 读之前记下当前序号
        uint8 slot;
        uint32 sequence_after;

        // 序号 0=还没发布过；等于上次序号=没有新帧
        if ((0u == sequence_before) || (sequence_before == *last_publish_seq))
        {
            return 0u;
        }
        __dsync();
        slot = vision_pose_publish_slot;
        __dsync(); // 数据同步屏障：确保读取发生在读序号之后
        vision_observation_copy_slot(slot, observation);
        __dsync(); // 再一道屏障：确保拷贝完成后才读"读后序号"
        sequence_after = vision_pose_publish_seq; // 读之后再看一次序号
        // 前后序号一致且拷到的 publish_seq 对得上，说明快照没被写坏
        if ((sequence_before == sequence_after) &&
            (observation->publish_seq == sequence_before))
        {
            *last_publish_seq = sequence_before; // 记下"这帧已读"
            return 1u;
        }
        // 否则读中途被覆盖，循环再试
    }
    return 0u; // 3 次都被打断，放弃
}

static void vision_scan_hit_copy_slot(uint8 slot, VisionScanHitPacket *packet)
{
    uint16 index;

    packet->publish_seq = vision_scan_hit_slots[slot].publish_seq;
    packet->frame_seq = vision_scan_hit_slots[slot].frame_seq;
    packet->capture_ms = vision_scan_hit_slots[slot].capture_ms;
    packet->publish_ms = vision_scan_hit_slots[slot].publish_ms;
    packet->capture_epoch = vision_scan_hit_slots[slot].capture_epoch;
    packet->hit_count = vision_scan_hit_slots[slot].hit_count;
    packet->source_frame_gap = vision_scan_hit_slots[slot].source_frame_gap;
    packet->flags = vision_scan_hit_slots[slot].flags;
    packet->frame_quality = vision_scan_hit_slots[slot].frame_quality;
    packet->reserved = vision_scan_hit_slots[slot].reserved;
    if (packet->hit_count > VISION_SCAN_HIT_MAX_COUNT)
    {
        packet->hit_count = VISION_SCAN_HIT_MAX_COUNT;
    }
    for (index = 0u; index < packet->hit_count; index++)
    {
        packet->hits[index].lateral_mm =
            vision_scan_hit_slots[slot].hits[index].lateral_mm;
        packet->hits[index].confidence =
            vision_scan_hit_slots[slot].hits[index].confidence;
        packet->hits[index].flags = vision_scan_hit_slots[slot].hits[index].flags;
    }
}

uint8 vision_scan_hit_read_next(volatile uint32 *last_publish_seq,
                                VisionScanHitPacket *packet)
{
    uint8 retry;

    if ((NULL == last_publish_seq) || (NULL == packet))
    {
        return 0u;
    }
    for (retry = 0u; retry < 3u; retry++)
    {
        uint32 latest_sequence = vision_scan_hit_publish_seq;
        uint32 available_count;
        uint32 requested_sequence;
        uint32 requested_age;
        uint32 slot_sequence_before;
        uint32 slot_sequence_after;
        uint8 latest_slot;
        uint8 slot;

        if ((0u == latest_sequence) || (latest_sequence == *last_publish_seq))
        {
            return 0u;
        }
        __dsync();
        latest_slot = vision_scan_hit_publish_slot;
        available_count = vision_queue_sequence_distance(
            latest_sequence, *last_publish_seq);
        requested_sequence = vision_queue_sequence_next(*last_publish_seq);
        if (available_count > VISION_SCAN_HIT_QUEUE_CAPACITY)
        {
            requested_sequence = vision_queue_sequence_rewind(
                latest_sequence, VISION_SCAN_HIT_QUEUE_CAPACITY - 1u);
        }
        requested_age = vision_queue_sequence_distance(
            latest_sequence, requested_sequence);
        slot = (uint8)((latest_slot + VISION_SCAN_HIT_QUEUE_CAPACITY -
                        (requested_age &
                         (VISION_SCAN_HIT_QUEUE_CAPACITY - 1u))) &
                       (VISION_SCAN_HIT_QUEUE_CAPACITY - 1u));
        __dsync();
        slot_sequence_before = vision_scan_hit_slots[slot].publish_seq;
        if (slot_sequence_before != requested_sequence)
        {
            continue;
        }
        vision_scan_hit_copy_slot(slot, packet);
        __dsync();
        slot_sequence_after = vision_scan_hit_slots[slot].publish_seq;
        if ((slot_sequence_before == slot_sequence_after) &&
            (packet->publish_seq == requested_sequence))
        {
            *last_publish_seq = requested_sequence;
            return 1u;
        }
    }
    return 0u;
}

static void imu_snapshot_copy_slot(uint8 slot, ImuSnapshot *snapshot)
{
    uint8 index;

    snapshot->publish_seq = imu_snapshot_slots[slot].publish_seq;
    snapshot->sample_us = imu_snapshot_slots[slot].sample_us;
    snapshot->command_ack_seq = imu_snapshot_slots[slot].command_ack_seq;
    snapshot->acc_x_g = imu_snapshot_slots[slot].acc_x_g;
    snapshot->acc_y_g = imu_snapshot_slots[slot].acc_y_g;
    snapshot->acc_z_g = imu_snapshot_slots[slot].acc_z_g;
    snapshot->gyro_x_radps = imu_snapshot_slots[slot].gyro_x_radps;
    snapshot->gyro_y_radps = imu_snapshot_slots[slot].gyro_y_radps;
    snapshot->gyro_z_radps = imu_snapshot_slots[slot].gyro_z_radps;
    snapshot->roll_deg = imu_snapshot_slots[slot].roll_deg;
    snapshot->pitch_deg = imu_snapshot_slots[slot].pitch_deg;
    snapshot->yaw_deg = imu_snapshot_slots[slot].yaw_deg;
    snapshot->heading_deg = imu_snapshot_slots[slot].heading_deg;
    snapshot->yaw_rate_radps = imu_snapshot_slots[slot].yaw_rate_radps;
    snapshot->pure_yaw_deg = imu_snapshot_slots[slot].pure_yaw_deg;
    snapshot->roll_delta_deg = imu_snapshot_slots[slot].roll_delta_deg;
    snapshot->pitch_delta_deg = imu_snapshot_slots[slot].pitch_delta_deg;
    for (index = 0u; index < 4u; index++)
    {
        snapshot->quaternion[index] = imu_snapshot_slots[slot].quaternion[index];
    }
    snapshot->bias_z_dps = imu_snapshot_slots[slot].bias_z_dps;
    snapshot->bias_sigma_dps = imu_snapshot_slots[slot].bias_sigma_dps;
    snapshot->ready = imu_snapshot_slots[slot].ready;
    snapshot->zero_calibrated = imu_snapshot_slots[slot].zero_calibrated;
    snapshot->zero_calibrating = imu_snapshot_slots[slot].zero_calibrating;
    snapshot->rest_detected = imu_snapshot_slots[slot].rest_detected;
    snapshot->startup_stationary_ms =
        imu_snapshot_slots[slot].startup_stationary_ms;
}

/* CPU0 reader for the CPU1-owned IMU publication.  The sequence is checked
 * both globally and in the selected slot so a control tick never mixes two
 * VQF publications. */
uint8 imu_snapshot_read(ImuSnapshot *snapshot)
{
    uint8 retry;

    if (NULL == snapshot)
    {
        return 0u;
    }
    for (retry = 0u; retry < 3u; retry++)
    {
        uint32 sequence_before = imu_snapshot_publish_seq;
        uint32 sequence_after;
        uint32 slot_sequence;
        uint8 slot;

        if (0u == sequence_before)
        {
            return 0u;
        }
        slot = (uint8)(sequence_before & (IMU_SNAPSHOT_SLOT_COUNT - 1u));
        __dsync();
        slot_sequence = imu_snapshot_slots[slot].publish_seq;
        if (slot_sequence != sequence_before)
        {
            continue;
        }
        imu_snapshot_copy_slot(slot, snapshot);
        __dsync();
        sequence_after = imu_snapshot_publish_seq;
        if ((sequence_before == sequence_after) &&
            (snapshot->publish_seq == sequence_before))
        {
            return 1u;
        }
    }
    return 0u;
}

volatile uint8 g_stop_reason = 0u;      // 最近一次停车原因码，见 globals.h 的 STOP_REASON_*

#pragma section all restore
