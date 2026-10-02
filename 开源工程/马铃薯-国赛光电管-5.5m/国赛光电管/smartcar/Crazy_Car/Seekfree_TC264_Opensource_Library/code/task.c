/**
 * @file task.c
 * @brief CPU0 后台调度、扫图入口、地图匹配和安全裁决。
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

// task.c —— CPU0（控制核）软定时任务调度器实现
// 1ms PIT 中断只把各 cnt_Xms 计数 +1，主循环 task_main_loop() 到点派发 Duty_Xms()；
// 硬实时速度 PI 不在此，另在 CCU61_CH1。

#include "task.h"
#include "globals.h"
#include <string.h>
#include "control.h"
#include "flash.h"
#include "key.h"
#include "adc_key.h"
#include "prestart.h"
#include "gyroscope.h"
#include "navigation.h"
#include "phototube_scan.h"
#include "telemetry.h"
#include "phototube_array.h"
#include "encoder_test.h"
#include "competition_config.h"

// 以下文件作用域变量必须放进 CPU0 的 DSRAM。这对 pragma 的开合位置极其关键，严禁移动或改变包裹范围。
#pragma section all "cpu0_dsram"

// 全局调度状态实例：task.h 里 extern 声明，这里唯一定义
volatile loop_t loop;

#define ASSISTANT_SCOPE_CHANNEL_MAX (4u)        // 示波器通道数
#define ASSISTANT_PARAM_SPEED_INDEX (0u)        // 助手参数下标：目标速度
#define ASSISTANT_PARAM_SPEED_KP_INDEX (1u)     // 助手参数下标：速度环 KP
#define ASSISTANT_PARAM_SPEED_KI_INDEX (2u)     // 助手参数下标：速度环 KI
#define ASSISTANT_PARAM_ESTOP_INDEX (3u)        // 助手参数下标：急停通道
// 姿态(横滚/俯仰)超限停车，R=5/R=6。姿态源 = 加速度计重低通倾角（四元数已关闭），取相对发车零点的差值。
static seekfree_assistant_debug_param_struct assistant_params;              // 上位机下发参数缓存
static seekfree_assistant_oscilloscope_struct assistant_scope;             // 示波器发送配置
static float assistant_scope_data[ASSISTANT_SCOPE_CHANNEL_MAX];            // 示波器各通道数据
static uint32 phototube_black_scan_last_stats_seq = 0u;
static uint32 phototube_black_scan_last_white_frame_seq = 0u;
static float phototube_black_scan_start_distance_mm = 0.0f;
static float phototube_black_scan_stop_distance_mm =
    PHOTOTUBE_ARRAY_BLACK_STOP_DISTANCE_MM;
static uint8 phototube_black_scan_active = 0u;
/* CPU0 scan ingress is deliberately separate from the expensive point-cloud
 * consumer.  The source mailbox is owned by CPU1; this ring is CPU0-owned and
 * is only a bounded payload copy, so the 1 ms task never performs geometry or
 * EMEM access. */

#if ((PHOTOTUBE_SCAN_STAGING_CAPACITY & \
      (PHOTOTUBE_SCAN_STAGING_CAPACITY - 1u)) != 0u)
#error "phototube staging capacity must be a power of two"
#endif

typedef struct
{
    VisionScanHitPacket packet;
    uint8 accounted;
    uint8 wait_reported;
    uint32 wait_pose_generation;
} PhototubeScanStagingEntry;

static PhototubeScanStagingEntry phototube_scan_staging[
    PHOTOTUBE_SCAN_STAGING_CAPACITY];
static uint8 phototube_scan_staging_head = 0u;
static uint8 phototube_scan_staging_tail = 0u;
static uint8 phototube_scan_staging_count = 0u;
static uint32 phototube_scan_source_publish_seq = 0u;
static uint8 phototube_scan_ingress_active = 0u;
static uint32 phototube_scan_epoch_counter = 0u;
static uint32 phototube_scan_active_epoch = 0u;
static uint32 phototube_scan_closing_epoch = 0u;
static uint8 phototube_scan_backpressure_active = 0u;
static void phototube_black_scan_reset(void);

typedef struct
{
    uint32 tick_ms;
    DriveOdometryPose pose;
    float travel_distance_mm;
} PhototubeScanPoseSample;
static volatile PhototubeScanPoseSample phototube_scan_pose_history[
    PHOTOTUBE_SCAN_POSE_HISTORY_CAPACITY];
static volatile uint32 phototube_scan_pose_versions[
    PHOTOTUBE_SCAN_POSE_HISTORY_CAPACITY];
static volatile uint8 phototube_scan_pose_history_head = 0u;
static volatile uint8 phototube_scan_pose_history_count = 0u;
static volatile uint8 phototube_scan_pose_history_active = 0u;
static volatile uint32 phototube_scan_pose_generation = 0u;

typedef enum
{
    PHOTOTUBE_SCAN_POSE_RESULT_REJECTED = 0u,
    PHOTOTUBE_SCAN_POSE_RESULT_INTERPOLATED = 1u,
    PHOTOTUBE_SCAN_POSE_RESULT_WAITING = 2u
} PhototubeScanPoseResult;

static void phototube_scan_staging_reset(uint8 active)
{
    phototube_scan_staging_head = 0u;
    phototube_scan_staging_tail = 0u;
    phototube_scan_staging_count = 0u;
    phototube_scan_ingress_active = active;
    phototube_scan_backpressure_active = 0u;
}

void task_prepare_phototube_scan_ingress(void)
{
    uint32 source_boundary_seq = vision_scan_hit_publish_seq;

    duty_10ms_exec_time_us = 0u;
    duty_10ms_exec_max_us = 0u;
    duty_10ms_overrun_count = 0u;
    nav_match_exec_time_us = 0u;
    nav_match_exec_max_us = 0u;
    nav_match_overrun_count = 0u;
    phototube_scan_epoch_counter++;
    if (0u == phototube_scan_epoch_counter)
    {
        phototube_scan_epoch_counter = 1u;
    }
    phototube_scan_active_epoch = phototube_scan_epoch_counter;
    phototube_scan_closing_epoch = 0u;
    phototube_scan_staging_reset(1u);
    /* Snapshot the source sequence before publishing the new capture epoch.
     * An old in-flight frame may appear after this boundary, but it carries
     * the previous epoch and is consumed without entering the run staging. */
    phototube_scan_source_publish_seq = source_boundary_seq;
    __dsync();
    vision_scan_run_epoch = phototube_scan_active_epoch;
    __dsync();
    phototube_black_scan_reset();
}

void task_close_phototube_scan_ingress(void)
{
    if ((0u == phototube_scan_ingress_active) ||
        (0u == phototube_scan_active_epoch) ||
        (0u != phototube_scan_closing_epoch))
    {
        return;
    }
    phototube_scan_epoch_counter++;
    if (0u == phototube_scan_epoch_counter)
    {
        phototube_scan_epoch_counter = 1u;
    }
    phototube_scan_closing_epoch = phototube_scan_epoch_counter;
    vision_scan_run_epoch = phototube_scan_closing_epoch;
    __dsync();
}

void task_update_phototube_line_diagnostics(void)
{
    nav_update_line_diagnostics();
}

static void phototube_scan_write_pose_sample(uint8 index,
                                             uint32 tick_ms,
                                             const DriveOdometryPose *pose,
                                             float travel_distance_mm)
{
    uint32 version = phototube_scan_pose_versions[index] + 1u;

    if (0u == (version & 1u))
    {
        version++;
    }
    phototube_scan_pose_versions[index] = version;
    __dsync();
    phototube_scan_pose_history[index].tick_ms = tick_ms;
    phototube_scan_pose_history[index].pose = *pose;
    phototube_scan_pose_history[index].travel_distance_mm = travel_distance_mm;
    __dsync();
    phototube_scan_pose_versions[index] = version + 1u;
    phototube_scan_pose_generation++;
}

static uint8 phototube_scan_read_pose_sample(
    uint8 index, PhototubeScanPoseSample *sample)
{
    uint8 retry;

    for (retry = 0u; retry < 3u; retry++)
    {
        uint32 before = phototube_scan_pose_versions[index];
        uint32 after;

        if (0u != (before & 1u))
        {
            continue;
        }
        __dsync();
        *sample = phototube_scan_pose_history[index];
        __dsync();
        after = phototube_scan_pose_versions[index];
        if ((before == after) && (0u == (after & 1u)))
        {
            return 1u;
        }
    }
    return 0u;
}

void task_seed_phototube_scan_pose_history(void)
{
    DriveOdometryPose pose;

    if ((0u == phototube_scan_is_active()) ||
        !nav_get_raw_odometry_pose(&pose))
    {
        return;
    }
    phototube_scan_pose_history_head = 0u;
    phototube_scan_pose_history_count = 0u;
    phototube_scan_pose_history_active = 1u;
    memset((void *)phototube_scan_pose_versions, 0,
           sizeof(uint32) * PHOTOTUBE_SCAN_POSE_HISTORY_CAPACITY);
    phototube_scan_write_pose_sample(0u, vision_clock_ms, &pose,
                                     total_dist * 1000.0f);
    phototube_scan_pose_history_head = 1u;
    phototube_scan_pose_history_count = 1u;
}

void task_capture_phototube_scan_pose_from_isr(void)
{
    DriveOdometryPose pose;
    if (!nav_get_raw_odometry_pose(&pose))
    {
        return;
    }
    if (0u == phototube_scan_is_active())
    {
        phototube_scan_pose_history_active = 0u;
        return;
    }
    else if (0u == phototube_scan_pose_history_active)
    {
        /* nav_start() resets odometry at the launch boundary.  Discard the
         * pre-launch history before recording the first run pose, otherwise
         * a delayed phototube frame could be interpolated across the reset. */
        phototube_scan_pose_history_head = 0u;
        phototube_scan_pose_history_count = 0u;
        phototube_scan_pose_history_active = 1u;
    }
    // 原始 odom 使用连续 VQF yaw；每个1ms控制拍发布一个防撕裂槽。
    phototube_scan_write_pose_sample(phototube_scan_pose_history_head,
                                     vision_clock_ms, &pose,
                                     total_dist * 1000.0f);
    phototube_scan_pose_history_head =
        (uint8)((phototube_scan_pose_history_head + 1u) %
                PHOTOTUBE_SCAN_POSE_HISTORY_CAPACITY);
    if (phototube_scan_pose_history_count < PHOTOTUBE_SCAN_POSE_HISTORY_CAPACITY)
    {
        phototube_scan_pose_history_count++;
    }
}

static PhototubeScanPoseResult phototube_scan_find_pose(
    uint32 capture_ms,
    DriveOdometryPose *pose,
    float *travel_distance_mm)
{
    uint8 i;
    uint8 history_head;
    uint8 history_count;
    uint8 have_before = 0u;
    uint8 have_after = 0u;
    uint32 before_tick = 0u;
    uint32 after_tick = 0u;
    uint32 before_age = 0xFFFFFFFFu;
    uint32 after_age = 0xFFFFFFFFu;
    DriveOdometryPose before_pose;
    DriveOdometryPose after_pose;
    float before_distance_mm = 0.0f;
    float after_distance_mm = 0.0f;
    uint32 interrupt_state;

    if ((NULL == pose) || (NULL == travel_distance_mm))
    {
        return PHOTOTUBE_SCAN_POSE_RESULT_REJECTED;
    }
    /* Snapshot the ring ownership fields together.  Individual slots remain
     * protected by their seqlocks, so the 1 ms ISR does not need to stay
     * masked while the main loop searches the history. */
    interrupt_state = interrupt_global_disable();
    history_head = phototube_scan_pose_history_head;
    history_count = phototube_scan_pose_history_count;
    interrupt_global_enable(interrupt_state);

    for (i = 0u; i < history_count; i++)
    {
        uint8 index = (uint8)((history_head +
                               PHOTOTUBE_SCAN_POSE_HISTORY_CAPACITY - 1u - i) %
                              PHOTOTUBE_SCAN_POSE_HISTORY_CAPACITY);
        PhototubeScanPoseSample sample;
        int32 signed_delta;
        uint32 age;

        if (!phototube_scan_read_pose_sample(index, &sample))
        {
            continue;
        }
        signed_delta = (int32)(sample.tick_ms - capture_ms);
        age = (signed_delta < 0) ?
              (uint32)(-signed_delta) : (uint32)signed_delta;

        if (signed_delta <= 0)
        {
            /* Slots are visited newest to oldest.  This first sample at or
             * before capture_ms is therefore the nearest lower bracket; the
             * most recently visited positive sample is the nearest upper
             * bracket.  Older slots cannot improve either side. */
            before_age = age;
            before_tick = sample.tick_ms;
            before_pose = sample.pose;
            before_distance_mm = sample.travel_distance_mm;
            have_before = 1u;
            break;
        }
        else if (age < after_age)
        {
            after_age = age;
            after_tick = sample.tick_ms;
            after_pose = sample.pose;
            after_distance_mm = sample.travel_distance_mm;
            have_after = 1u;
        }
    }

    if (have_before && (before_tick == capture_ms))
    {
        *pose = before_pose;
        *travel_distance_mm = before_distance_mm;
        return PHOTOTUBE_SCAN_POSE_RESULT_INTERPOLATED;
    }

    if (have_before && have_after &&
        (before_age <= PHOTOTUBE_SCAN_POSE_MAX_AGE_MS) &&
        (after_age <= PHOTOTUBE_SCAN_POSE_MAX_AGE_MS) &&
        ((uint32)(after_tick - before_tick) > 0u))
    {
        float ratio = (float)(capture_ms - before_tick) /
                      (float)(after_tick - before_tick);
        float yaw_delta = after_pose.yaw_rad - before_pose.yaw_rad;

        while (yaw_delta > 3.14159265359f) yaw_delta -= 6.28318530718f;
        while (yaw_delta < -3.14159265359f) yaw_delta += 6.28318530718f;
        pose->x_mm = before_pose.x_mm +
                     ratio * (after_pose.x_mm - before_pose.x_mm);
        pose->y_mm = before_pose.y_mm +
                     ratio * (after_pose.y_mm - before_pose.y_mm);
        pose->yaw_rad = before_pose.yaw_rad + ratio * yaw_delta;
        pose->linear_speed_mmps = before_pose.linear_speed_mmps +
                                  ratio * (after_pose.linear_speed_mmps -
                                           before_pose.linear_speed_mmps);
        pose->yaw_rate_radps = before_pose.yaw_rate_radps +
                               ratio * (after_pose.yaw_rate_radps -
                                        before_pose.yaw_rate_radps);
        *travel_distance_mm = before_distance_mm +
                              ratio * (after_distance_mm - before_distance_mm);
        return PHOTOTUBE_SCAN_POSE_RESULT_INTERPOLATED;
    }

    if (have_before && !have_after)
    {
        int32 capture_age_ms = (int32)(vision_clock_ms - capture_ms);
        if ((capture_age_ms >= 0) &&
            ((uint32)capture_age_ms <= PHOTOTUBE_SCAN_POSE_MAX_AGE_MS) &&
            (before_age <= PHOTOTUBE_SCAN_POSE_MAX_AGE_MS))
        {
            /* The newest source frame normally arrives before the following
             * 1 ms odometry sample.  Keep it queued; consuming its sequence
             * here would turn a temporary condition into permanent loss. */
            return PHOTOTUBE_SCAN_POSE_RESULT_WAITING;
        }
    }

    /* Never substitute a one-sided nearest pose.  Once the bounded wait has
     * expired the frame is explicitly rejected as non-interpolable. */
    return PHOTOTUBE_SCAN_POSE_RESULT_REJECTED;
}

static void phototube_scan_ingress_limited(uint32 publish_cutoff_seq,
                                           uint8 packet_budget)
{
    uint8 copied = 0u;

    if ((0u == phototube_scan_is_active()) ||
        (0u == phototube_scan_ingress_active))
    {
        phototube_scan_source_publish_seq = vision_scan_hit_publish_seq;
        phototube_scan_staging_reset(0u);
        return;
    }

    while (copied < packet_budget)
    {
        PhototubeScanStagingEntry *entry;
        VisionScanHitPacket packet;
        uint32 previous_publish_seq;
        uint32 packet_distance;

        if (phototube_scan_staging_count >=
            PHOTOTUBE_SCAN_STAGING_CAPACITY)
        {
            /* This is backpressure, not loss: the source read sequence has
             * not advanced.  If CPU1 later overwrites the unread source ring,
             * vision_scan_hit_read_next() reports the exact mailbox gap. */
            if (!phototube_scan_backpressure_active)
            {
                phototube_scan_note_staging_backpressure();
                phototube_scan_backpressure_active = 1u;
            }
            return;
        }
        previous_publish_seq = phototube_scan_source_publish_seq;
        if (!vision_scan_hit_read_next(&phototube_scan_source_publish_seq,
                                       &packet))
        {
            return;
        }
        packet_distance = vision_queue_sequence_distance(
            packet.publish_seq, previous_publish_seq);
        if ((publish_cutoff_seq != 0u) &&
            (packet_distance > vision_queue_sequence_distance(
                publish_cutoff_seq, previous_publish_seq)))
        {
            phototube_scan_source_publish_seq = previous_publish_seq;
            return;
        }
        if (packet_distance > 1u)
        {
            phototube_scan_note_mailbox_gap(packet_distance - 1u);
        }

        copied++;
        if (packet.capture_epoch != phototube_scan_active_epoch)
        {
            phototube_scan_note_epoch_filtered();
            continue;
        }

        entry = &phototube_scan_staging[phototube_scan_staging_head];
        entry->packet = packet;
        entry->accounted = 0u;
        entry->wait_reported = 0u;
        entry->wait_pose_generation = 0u;
        phototube_scan_staging_head = (uint8)(
            (phototube_scan_staging_head + 1u) &
            (PHOTOTUBE_SCAN_STAGING_CAPACITY - 1u));
        phototube_scan_staging_count++;
        phototube_scan_note_staging_high_watermark(
            phototube_scan_staging_count);
    }
}

static void phototube_scan_process_staging(uint16 packet_budget,
                                            uint8 finalizing)
{
    uint16 processed_count = 0u;
    uint8 points_changed = 0u;

    while ((phototube_scan_staging_count > 0u) &&
           ((0u == packet_budget) || (processed_count < packet_budget)))
    {
        PhototubeScanStagingEntry *entry =
            &phototube_scan_staging[phototube_scan_staging_tail];
        DriveOdometryPose pose;
        float travel_distance_mm = 0.0f;
        PhototubeScanPoseResult pose_result;

        /* A frame waiting for the next 1 ms pose sample has no new
         * information to use until the history generation changes.  Avoid
         * rescanning all 64 history slots on every tight main-loop turn. */
        if (!finalizing && entry->wait_reported &&
            (entry->wait_pose_generation == phototube_scan_pose_generation))
        {
            break;
        }

        if (0u == entry->accounted)
        {
            phototube_scan_note_hit_packet(&entry->packet);
            entry->accounted = 1u;
        }
        pose_result = phototube_scan_find_pose(
            entry->packet.capture_ms, &pose, &travel_distance_mm);
        if (PHOTOTUBE_SCAN_POSE_RESULT_WAITING == pose_result)
        {
            if (!entry->wait_reported)
            {
                phototube_scan_note_pose_wait();
                entry->wait_reported = 1u;
                entry->wait_pose_generation = phototube_scan_pose_generation;
            }
            if (!finalizing)
            {
                break;
            }
            pose_result = PHOTOTUBE_SCAN_POSE_RESULT_REJECTED;
        }

        if (PHOTOTUBE_SCAN_POSE_RESULT_INTERPOLATED == pose_result)
        {
            phototube_scan_note_pose_source(
                PHOTOTUBE_SCAN_POSE_INTERPOLATED);
            if (phototube_scan_push_hits(&entry->packet, &pose,
                                         travel_distance_mm,
                                         vision_clock_ms) > 0u)
            {
                points_changed = 1u;
            }
        }
        else
        {
            phototube_scan_note_pose_rejected();
        }

        phototube_scan_staging_tail = (uint8)(
            (phototube_scan_staging_tail + 1u) &
            (PHOTOTUBE_SCAN_STAGING_CAPACITY - 1u));
        phototube_scan_staging_count--;
        if (phototube_scan_staging_count < PHOTOTUBE_SCAN_STAGING_CAPACITY)
        {
            phototube_scan_backpressure_active = 0u;
        }
        processed_count++;
    }
    if (points_changed)
    {
        phototube_scan_finish_batch();
    }
}

void task_finalize_phototube_scan(void)
{
    uint32 publish_cutoff_seq;
    uint32 remaining_count;
    uint32 wait_start_us;
    uint8 pass;

    if (0u == phototube_scan_is_active())
    {
        return;
    }
    /* The close epoch was published inside prestart_stop()'s interrupt-masked
     * transaction.  Wait here, outside that critical section, until CPU1 has
     * completed any in-flight active-epoch frame. */
    if (0u == phototube_scan_closing_epoch)
    {
        task_close_phototube_scan_ingress();
    }
    if (0u != phototube_scan_closing_epoch)
    {
        wait_start_us = system_getval_us();
        while ((phototube_scan_closing_epoch != vision_scan_run_epoch_ack) &&
               ((uint32)(system_getval_us() - wait_start_us) <
                PHOTOTUBE_SCAN_EPOCH_ACK_TIMEOUT_US))
        {
        }
        if (phototube_scan_closing_epoch != vision_scan_run_epoch_ack)
        {
            phototube_scan_note_epoch_ack_timeout();
        }
    }
    publish_cutoff_seq = vision_scan_hit_publish_seq;
    for (pass = 0u; pass < 4u; pass++)
    {
        phototube_scan_ingress_limited(
            publish_cutoff_seq, (uint8)VISION_SCAN_HIT_QUEUE_CAPACITY);
        phototube_scan_process_staging(0u, 1u);
        if ((phototube_scan_source_publish_seq == publish_cutoff_seq) &&
            (0u == phototube_scan_staging_count))
        {
            break;
        }
    }

    /* A bounded drain must never silently leave pre-stop packets behind.  The
     * publish sequence is monotonic; any sequence still before the cutoff is
     * an explicit CPU0 mailbox loss, while post-cutoff packets are irrelevant
     * to this run and are not charged to it. */
    if (publish_cutoff_seq != phototube_scan_source_publish_seq)
    {
        remaining_count = vision_queue_sequence_distance(
            publish_cutoff_seq, phototube_scan_source_publish_seq);
        phototube_scan_note_mailbox_gap(remaining_count);
        phototube_scan_source_publish_seq = publish_cutoff_seq;
    }
    phototube_scan_staging_reset(0u);
    phototube_scan_active_epoch = 0u;
    phototube_scan_closing_epoch = 0u;
}

typedef struct
{
    uint32 frame_seq;
    uint32 epoch;
    uint32 last_line_frame_seq;
    uint32 last_line_epoch;
} PhototubeLineStatsSnapshot;

static uint8 phototube_line_stats_read(PhototubeLineStatsSnapshot *snapshot)
{
    uint8 retry;

    if (NULL == snapshot)
    {
        return 0u;
    }
    for (retry = 0u; retry < 3u; retry++)
    {
        uint32 version_before = vision_line_stats_version;
        uint32 version_after;

        if (0u != (version_before & 1u))
        {
            continue;
        }
        __dsync();
        snapshot->frame_seq = vision_line_stats_seq;
        snapshot->epoch = vision_line_stats_epoch;
        snapshot->last_line_frame_seq = vision_line_last_white_frame_seq;
        snapshot->last_line_epoch = vision_line_last_white_epoch;
        __dsync();
        version_after = vision_line_stats_version;
        if ((version_before == version_after) &&
            (0u == (version_after & 1u)))
        {
            return 1u;
        }
    }
    return 0u;
}

static void phototube_black_scan_reset(void)
{
    PhototubeLineStatsSnapshot snapshot;

    if (phototube_line_stats_read(&snapshot))
    {
        phototube_black_scan_last_stats_seq = snapshot.frame_seq;
        phototube_black_scan_last_white_frame_seq =
            snapshot.last_line_frame_seq;
    }
    else
    {
        phototube_black_scan_last_stats_seq = 0u;
        phototube_black_scan_last_white_frame_seq = 0u;
    }
    phototube_black_scan_start_distance_mm = total_dist * 1000.0f;
    phototube_black_scan_stop_distance_mm =
        PHOTOTUBE_ARRAY_BLACK_STOP_DISTANCE_MM;
    phototube_black_scan_active = 0u;
}

/* CPU1 每帧用16路 active_mask 判断是否扫到低电平线路，并把最后有效帧号保留为
 * 单调标记。CPU0 即使以5ms周期检查，也不会漏掉两次检查之间短暂出现
 * 的线路。可信地图接管区不累计全黑距离，其余位置固定连续全黑 200 mm 停车。 */
static uint8 phototube_black_scan_need_stop(uint8 blackout_allowed)
{
    PhototubeLineStatsSnapshot snapshot;
    float distance_mm;

    if (!phototube_line_stats_read(&snapshot) || (0u == snapshot.frame_seq))
    {
        return 0u;
    }
    distance_mm = total_dist * 1000.0f;

    /* A frame that began outside this raw-odom run must not reset the 200 mm
     * no-line distance after launch.  Advance only the observed stats cursor;
     * the first frame carrying the active epoch starts the real decision. */
    if ((0u == phototube_scan_active_epoch) ||
        (snapshot.epoch != phototube_scan_active_epoch))
    {
        phototube_black_scan_last_stats_seq = snapshot.frame_seq;
        return 0u;
    }

    /* A trusted map feature may temporarily make an all-black scan expected.
     * Consume that distance here instead of carrying it to the takeover-zone
     * exit and stopping immediately with an already-expired 200 mm timer. */
    if (blackout_allowed)
    {
        phototube_black_scan_last_stats_seq = snapshot.frame_seq;
        phototube_black_scan_last_white_frame_seq =
            snapshot.last_line_frame_seq;
        phototube_black_scan_start_distance_mm = distance_mm;
        phototube_black_scan_stop_distance_mm =
            PHOTOTUBE_ARRAY_BLACK_STOP_DISTANCE_MM;
        phototube_black_scan_active = 0u;
        return 0u;
    }

    if (snapshot.frame_seq == phototube_black_scan_last_stats_seq)
    {
        return 0u;
    }
    phototube_black_scan_last_stats_seq = snapshot.frame_seq;

    if ((snapshot.last_line_epoch == phototube_scan_active_epoch) &&
        (snapshot.last_line_frame_seq !=
         phototube_black_scan_last_white_frame_seq))
    {
        phototube_black_scan_last_white_frame_seq =
            snapshot.last_line_frame_seq;
        phototube_black_scan_start_distance_mm = distance_mm;
        phototube_black_scan_stop_distance_mm =
            PHOTOTUBE_ARRAY_BLACK_STOP_DISTANCE_MM;
        phototube_black_scan_active = 0u;
        return 0u;
    }

    if (!phototube_black_scan_active ||
        (distance_mm < phototube_black_scan_start_distance_mm))
    {
        phototube_black_scan_start_distance_mm = distance_mm;
        phototube_black_scan_stop_distance_mm =
            nav_get_line_loss_stop_distance_mm();
        phototube_black_scan_active = 1u;
        return 0u;
    }

    return (uint8)((distance_mm - phototube_black_scan_start_distance_mm) >=
                   phototube_black_scan_stop_distance_mm);
}

// 判断横滚角是否触发姿态保护（相对发车零点超 ±ATTITUDE_STOP_LIMIT_DEG）。
// 只有陀螺零飘完成后才启用，避免校准前原始安装角度误停车。
static uint8 attitude_roll_need_stop(void)
{
#if ATTITUDE_STOP_ENABLE
    float roll = gyro_get_roll_delta(); // 相对发车零点的横滚角差值

    // 校准没完成时不做保护，否则会被安装时初始倾角误判成翻车
    if (!gyro_zero_calibration_done())
    {
        return 0u;
    }

    return (uint8)((roll > ATTITUDE_STOP_LIMIT_DEG) || (roll < -ATTITUDE_STOP_LIMIT_DEG));
#else
    return 0u; // 姿态横滚保护已临时关闭（ATTITUDE_STOP_ENABLE=0）
#endif
}

// 判断俯仰角是否触发姿态保护。与横滚分开写，方便单独调试是哪个姿态轴触发停车。
static uint8 attitude_pitch_need_stop(void)
{
#if ATTITUDE_STOP_ENABLE
    float pitch = gyro_get_pitch_delta(); // 相对发车零点的俯仰角差值

    if (!gyro_zero_calibration_done())
    {
        return 0u;
    }

    return (uint8)((pitch > ATTITUDE_STOP_LIMIT_DEG) || (pitch < -ATTITUDE_STOP_LIMIT_DEG));
#else
    return 0u; // 姿态俯仰保护已临时关闭（ATTITUDE_STOP_ENABLE=0）
#endif
}

#if IMU_DEBUG_PRINT_ENABLE
// 临时 IMU 调试打印（默认关）。各角度×10 转整数打印，避免浮点格式化开销。由 Duty_100ms() 调用。
static void imu_debug_print(void)
{
    ImuSnapshot snapshot;
    uint8 valid = imu_snapshot_read(&snapshot);
    int roll_x10 = (int)((valid ? snapshot.roll_deg : 0.0f) * 10.0f);
    int pitch_x10 = (int)((valid ? snapshot.pitch_deg : 0.0f) * 10.0f);
    int yaw_x10 = (int)((valid ? snapshot.heading_deg : 0.0f) * 10.0f);
    int droll_x10 = (int)((valid ? snapshot.roll_delta_deg : 0.0f) * 10.0f);
    int dpitch_x10 = (int)((valid ? snapshot.pitch_delta_deg : 0.0f) * 10.0f);
    int bias_z_x1000 = (int)((valid ? snapshot.bias_z_dps : 0.0f) * 1000.0f);
    int bias_sigma_x1000 = (int)((valid ? snapshot.bias_sigma_dps : 0.0f) * 1000.0f);

    printf("\r\nIMU run=%u cal=%u/%u vqf_rest=%u bias_z_1000=%d sigma_1000=%d roll10=%d pitch10=%d yaw10=%d droll10=%d dpitch10=%d\r\n",
           (unsigned int)is_car_running,
           (unsigned int)gyro_zero_calibration_done(),
           (unsigned int)gyro_zero_calibration_running(),
           (unsigned int)(valid ? snapshot.rest_detected : 0u),
           bias_z_x1000,
           bias_sigma_x1000,
           roll_x10,
           pitch_x10,
           yaw_x10,
           droll_x10,
           dpitch_x10);
}
#endif

// 初始化逐飞助手调参结构。示波器发送默认关闭，但参数解析仍保留用于上位机急停。
static void assistant_task_init(void)
{
    memset(&assistant_params, 0, sizeof(assistant_params));
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    seekfree_assistant_oscilloscope_config(&assistant_scope, ASSISTANT_SCOPE_CHANNEL_MAX, assistant_scope_data);
}

// 解析逐飞助手下发参数：每个参数带 update_flag，收到新值才处理并立即清标志。由 Duty_10ms() 调用。
static void assistant_apply_debug_params(void)
{
    CarParameters candidate = g_flash_parameters;
    uint8 controller_update = 0u;

    seekfree_assistant_debug_param_analysis(&assistant_params);

    /* Normal navigation owns speed through nav_path.  Consume the obsolete
     * assistant speed flag without writing the ineffective global `speed`. */
    if (assistant_params.update_flag[ASSISTANT_PARAM_SPEED_INDEX])
    {
        assistant_params.update_flag[ASSISTANT_PARAM_SPEED_INDEX] = 0u;
    }

    // 通道1/2：先组成一份候选参数，最后在同一临界区更新左右速度环。
    if (assistant_params.update_flag[ASSISTANT_PARAM_SPEED_KP_INDEX])
    {
        assistant_params.update_flag[ASSISTANT_PARAM_SPEED_KP_INDEX] = 0u;
        candidate.speed_KP =
            assistant_params.data[ASSISTANT_PARAM_SPEED_KP_INDEX] * 0.1f;
        controller_update = 1u;
    }

    if (assistant_params.update_flag[ASSISTANT_PARAM_SPEED_KI_INDEX])
    {
        assistant_params.update_flag[ASSISTANT_PARAM_SPEED_KI_INDEX] = 0u;
        candidate.speed_KI =
            assistant_params.data[ASSISTANT_PARAM_SPEED_KI_INDEX] * 0.1f;
        controller_update = 1u;
    }

    if (controller_update && prestart_is_idle() &&
        flash_parameters_valid(&candidate))
    {
        uint32 interrupt_state = interrupt_global_disable();

        g_flash_parameters = candidate;
        speed_PID1.KP = candidate.speed_KP;
        speed_PID2.KP = candidate.speed_KP;
        speed_PID1.KI = candidate.speed_KI;
        speed_PID2.KI = candidate.speed_KI;
        interrupt_global_enable(interrupt_state);
    }

    // 通道3：急停。收到大于阈值的数即立即停车
    if (assistant_params.update_flag[ASSISTANT_PARAM_ESTOP_INDEX])
    {
        assistant_params.update_flag[ASSISTANT_PARAM_ESTOP_INDEX] = 0u;
        if (assistant_params.data[ASSISTANT_PARAM_ESTOP_INDEX] > ASSISTANT_ESTOP_TRIGGER_VALUE)
        {
            speed = 0.0f;
            g_stop_reason = STOP_REASON_ASSISTANT;
            prestart_stop();
        }
    }
}

#if ASSISTANT_SCOPE_SEND_ENABLE
// 发送逐飞助手示波器数据（默认关）。四通道：左右实际速度、左右规划速度，便于台架对比调速度环。
static void assistant_send_scope(void)
{
    assistant_scope_data[0] = actual_l_debug;   // 左轮实际速度
    assistant_scope_data[1] = actual_r_debug;   // 右轮实际速度
    assistant_scope_data[2] = plan_v_debug_l;   // 左轮规划速度
    assistant_scope_data[3] = plan_v_debug_r;   // 右轮规划速度

    seekfree_assistant_oscilloscope_send(&assistant_scope);
}
#endif

// 初始化任务调度器：CCU60_CH0 每 1ms 进一次中断只累加计数，具体任务在主循环执行。由 core0_main() 调用一次。
void task_init(void)
{
    duty_10ms_exec_time_us = 0u;
    duty_10ms_exec_max_us = 0u;
    duty_10ms_overrun_count = 0u;
    nav_match_exec_time_us = 0u;
    nav_match_exec_max_us = 0u;
    nav_match_overrun_count = 0u;
    pit_ms_init(CCU60_CH0, 1);  // 启动 1ms 周期定时中断，作为整套软任务的节拍源
    assistant_task_init();      // 顺带初始化逐飞助手（急停/调参用）
    vision_scan_run_epoch = 0u;
    phototube_scan_active_epoch = 0u;
    phototube_scan_source_publish_seq = vision_scan_hit_publish_seq;
    phototube_scan_staging_reset(0u);
    phototube_black_scan_reset();
}

// 1ms 系统节拍中断回调（CCU60_CH0，优先级30）。只推进时钟和软任务计数。
void task_loop_tick(void)
{
    vision_clock_ms++;      // 视觉/计时用的毫秒时钟，供其它模块计算帧龄等

    /* UART export deliberately owns the idle CPU0 main loop.  Keep the
     * shared millisecond clock advancing, but do not accumulate deferred
     * menu/scan work or report the intentional pause as a main-loop stall. */
    if (TELEMETRY_STATE_EXPORTING == telemetry_get_state())
    {
        return;
    }

    // 各档位计数器统一 +1，到点由 task_main_loop() 判断并清零
    loop.cnt_1ms++;
    loop.cnt_2ms++;
    loop.cnt_4ms++;
    loop.cnt_5ms++;
    loop.cnt_10ms++;
    loop.cnt_20ms++;
    loop.cnt_50ms++;
    loop.cnt_100ms++;
    loop.cnt_200ms++;
    loop.cnt_500ms++;
    loop.cnt_1000ms++;

}

// 主循环任务调度器：各周期计数够阈值就清零并调用对应 Duty_Xms()。
// 后台实时性由各任务的微秒计时和overrun计数诊断；一次合法的5ms匹配不再
// 被旧的“主循环必须小于1ms”标志误报为超时。
void task_main_loop(void)
{
    /* Do not tie lossless CPU1->CPU0 scan ingress to a soft tick that may
     * collapse several elapsed milliseconds into one call.  A long matcher
     * pass can return with several source frames queued; bounded work on
     * every main-loop turn catches up without delaying the 1 ms control ISR. */
    phototube_scan_ingress_limited(0u, PHOTOTUBE_SCAN_INGRESS_BUDGET);
    phototube_scan_process_staging(PHOTOTUBE_SCAN_PROCESS_BUDGET, 0u);

    if (loop.cnt_1ms >= 1)
    {
        loop.cnt_1ms = 0;
        Duty_1ms();
    }
    if (loop.cnt_2ms >= 2)
    {
        loop.cnt_2ms = 0;
        Duty_2ms();
    }
    if (loop.cnt_4ms >= 4)
    {
        loop.cnt_4ms = 0;
        Duty_4ms();
    }
    if (loop.cnt_5ms >= 5)
    {
        loop.cnt_5ms = 0;
        Duty_5ms();
    }
    if (loop.cnt_10ms >= 10)
    {
        loop.cnt_10ms = 0;
        Duty_10ms();
    }
    if (loop.cnt_20ms >= 20)
    {
        loop.cnt_20ms = 0;
        Duty_20ms();
    }
    if (loop.cnt_50ms >= 50)
    {
        loop.cnt_50ms = 0;
        Duty_50ms();
    }
    if (loop.cnt_100ms >= 100)
    {
        loop.cnt_100ms = 0;
        Duty_100ms();
    }
    if (loop.cnt_200ms >= 200)
    {
        loop.cnt_200ms = 0;
        Duty_200ms();
    }
    if (loop.cnt_500ms >= 500)
    {
        loop.cnt_500ms = 0;
        Duty_500ms();
    }
    if (loop.cnt_1000ms >= 1000)
    {
        loop.cnt_1000ms = 0;
        Duty_1000ms();
    }
}

// 光电扫描包由每轮主循环限量推进；线路观测只供诊断，放在可抢占的
// CPU0 主循环1ms任务槽，不再占用硬实时控制 ISR。
void Duty_1ms(void)
{
    task_update_phototube_line_diagnostics();
}

void Duty_2ms(void) {}

void Duty_4ms(void) {}

/* 保持原有的互斥优先级链：一次最多只提交一个停车原因。 */
static void task_safety_service(void)
{
    // 不把发车前或上一轮的扫描带进本轮200mm全黑判断。
    if (!is_car_running || !prestart_is_running())
    {
        phototube_black_scan_reset();
    }

    if (is_car_running && prestart_is_running() &&
        !ENCODER_TEST_MODE_ENABLE &&
        phototube_black_scan_need_stop(
            nav_scan_map_match_allows_blackout()))
    {
        g_stop_reason = STOP_REASON_LINE_LOST;
        prestart_stop();
    }
    else if (is_car_running && prestart_is_running() &&
        prestart_is_straight_speed_test() &&
        (PRESTART_STRAIGHT_SPEED_TEST_MODE &&
         prestart_speed_test_should_stop()))
    {
        g_stop_reason = STOP_REASON_SPEED_TEST_DONE;
        prestart_stop();
    }
    else if (is_car_running && prestart_is_running() &&
             nav_imu_stale_faulted())
    {
        /* Keep short seqlock contention recoverable, but never continue after
         * the VQF heading and yaw-rate damping have both gone stale. */
        g_stop_reason = STOP_REASON_IMU_STALE;
        prestart_stop();
    }
    else if (is_car_running && prestart_is_running() &&
        (NAV_STATE_ERROR == nav_get_state()))
    {
        /* A failed route projection/reference is a protection condition, not
         * a normal zero-speed command.  Stop and freeze immediately so a bad
         * map/pose cannot leave the RUNNING state active with an invalid reference. */
        g_stop_reason = STOP_REASON_PLAN_MISMATCH;
        prestart_stop();
    }
    else if (is_car_running && prestart_is_running() && motor_fault_request)
    {
        // 电机/编码器失控：1ms速度环已经清PWM，这里只负责认领故障并冻结本轮。
        // 请求保持到prestart_stop()完成，避免后台繁忙期间重新放行电机输出。
        g_stop_reason = STOP_REASON_MOTOR_FAULT;
        prestart_stop();
    }
    else if (is_car_running && prestart_is_running() &&
             prestart_path_brake_ready())
    {
        // 末端主动制动已将轮速压到阈值内，再清零并禁用驱动，避免惯性滑行。
        prestart_stop();
    }
    else if (is_car_running && prestart_is_running() && nav_is_complete())
    {
        // 路径末点已经通过：先保持速度环运行并主动刹到近零，再停车。
        g_stop_reason = STOP_REASON_PATH_DONE;
        prestart_begin_path_brake();
    }
    else if (is_car_running && prestart_is_running() && attitude_roll_need_stop())
    {
        g_stop_reason = STOP_REASON_ROLL;
        prestart_stop();
    }
    else if (is_car_running && prestart_is_running() && attitude_pitch_need_stop())
    {
        g_stop_reason = STOP_REASON_PITCH;
        prestart_stop();
    }
    else if (motor_fault_request)
    {
        // 非运行状态下清掉残留请求，避免下次发车瞬间被旧标志停车
        motor_fault_request = 0u;
    }
}

// 5ms 后台任务：先执行保护裁决，再刷新菜单和扫描按键。IMU 采样/校准/yaw 积分在数据就绪外部中断完成，不走软任务。
void Duty_5ms(void)
{
    task_safety_service();
    key_scanner();          // 物理按键扫描
    adc_key_scan();         // ADC 摇杆按键扫描与消抖
    key_get();              // 读取按键事件
    menu_task();            // 先生成本拍实体键事件，再处理手动发车/停车
}

// 10ms 后台任务：参数/空闲里程，匹配前复查保护，然后执行10ms点云匹配。
void Duty_10ms(void)
{
    uint32 duty_start_us = system_getval_us();
    uint32 match_start_us;
    uint32 elapsed_us;

    assistant_apply_debug_params();

    // 地图编码器诊断模式不另读编码器；运行遥测由1ms控制/导航生产者写入EMEM。
    encoder_test_tick_10ms();

    // 停车时代读编码器累计里程，速度环运行时该函数内部直接返回
    control_odometer_idle_update();

    /* 发车或状态切换可以发生在本次 Duty_10ms() 内；进入高开销
     * 匹配器前同拍复查，保证所有停车裁决仍排在匹配之前。 */
    task_safety_service();

    /* Safety decisions must not wait behind a worst-case 1280-point match.
     * The matcher remains background work and can still be preempted by the
     * 1 ms control ISR; if a stop occurred above it sees an inactive scan and
     * returns immediately. */
    match_start_us = system_getval_us();
    nav_scan_map_match_service();
    elapsed_us = system_getval_us() - match_start_us;
    nav_match_exec_time_us = elapsed_us;
    if (elapsed_us > nav_match_exec_max_us)
    {
        nav_match_exec_max_us = elapsed_us;
    }
    if ((elapsed_us > NAV_SCAN_MATCH_SERVICE_BUDGET_US) &&
        (nav_match_overrun_count < 0xFFFFFFFFu))
    {
        nav_match_overrun_count++;
    }
#if ASSISTANT_SCOPE_SEND_ENABLE
    assistant_send_scope(); // 台架调试时才发送示波器数据，默认关闭
#endif

    elapsed_us = system_getval_us() - duty_start_us;
    duty_10ms_exec_time_us = elapsed_us;
    if (elapsed_us > duty_10ms_exec_max_us)
    {
        duty_10ms_exec_max_us = elapsed_us;
    }
    if ((elapsed_us > DUTY_10MS_BUDGET_US) &&
        (duty_10ms_overrun_count < 0xFFFFFFFFu))
    {
        duty_10ms_overrun_count++;
    }
}

void Duty_20ms(void) {}

void Duty_50ms(void) {}

// 100ms 后台任务：仅在打开 IMU_DEBUG_PRINT_ENABLE 时输出姿态调试信息
void Duty_100ms(void)
{
#if IMU_DEBUG_PRINT_ENABLE
    imu_debug_print();
#endif
}

void Duty_200ms(void) {}

void Duty_500ms(void) {}

void Duty_1000ms(void) {}

// 与文件前面的 #pragma section all "cpu0_dsram" 成对，结束 CPU0 DSRAM 段
#pragma section all restore
