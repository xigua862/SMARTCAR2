/**
 * @file phototube_scan.c
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

// phototube_scan.c —— 正常跑图期间的16路光电旁路扫图和局部直线度评估。

#include "phototube_scan.h"
#include "telemetry.h"
#include "phototube_array.h"
#include <math.h>
#include <string.h>

typedef char phototube_scan_point_size_check[
    (sizeof(PhototubeScanPoint) == 16u) ? 1 : -1];

#pragma section all "cpu0_dsram"

static PhototubeScanPoint scan_points[PHOTOTUBE_SCAN_MAX_POINTS];
static PhototubeScanSnapshot scan_snapshot;
static uint16 scan_oldest_index;
static uint16 scan_write_index;
static uint8 scan_origin_valid;
static uint8 scan_have_last_point;
static float scan_origin_cos;
static float scan_origin_sin;
static float scan_mounting_cos;
static float scan_mounting_sin;
static float scan_last_sensor_x_mm;
static float scan_last_sensor_y_mm;
static float scan_last_travel_distance_mm;
static float scan_latest_point_travel_distance_mm;
static uint32 scan_source_drop_start_count;
static uint32 scan_source_skipped_start_count;
static uint8 scan_fit_dirty;
static uint8 scan_bounds_dirty;

static uint16 phototube_scan_physical_index(uint16 logical_index)
{
    uint16 physical_index = (uint16)(scan_oldest_index + logical_index);

    /* Both operands are below PHOTOTUBE_SCAN_MAX_POINTS, so one subtraction
     * is sufficient and avoids a division in every matcher point access. */
    if (physical_index >= PHOTOTUBE_SCAN_MAX_POINTS)
    {
        physical_index = (uint16)(physical_index -
                                  PHOTOTUBE_SCAN_MAX_POINTS);
    }
    return physical_index;
}

static uint16 phototube_scan_next_physical_index(uint16 physical_index)
{
    physical_index++;
    if (physical_index >= PHOTOTUBE_SCAN_MAX_POINTS)
    {
        physical_index = 0u;
    }
    return physical_index;
}

static PhototubeScanPoint *phototube_scan_point_at(uint16 logical_index)
{
    return &scan_points[phototube_scan_physical_index(logical_index)];
}

static void phototube_scan_add_u16(uint16 *value, uint32 increment)
{
    uint32 next;

    if (NULL == value)
    {
        return;
    }
    next = (uint32)(*value) + increment;
    *value = (uint16)((next > 0xFFFFu) ? 0xFFFFu : next);
}

static void phototube_scan_refresh_source_stats(void)
{
    uint32 drop_count = vision_capture_drop_count -
                        scan_source_drop_start_count;
    uint32 skipped_count = vision_capture_skipped_count -
                           scan_source_skipped_start_count;

    scan_snapshot.source_schedule_drop_count =
        (uint16)((drop_count > 0xFFFFu) ? 0xFFFFu : drop_count);
    scan_snapshot.source_schedule_skipped_count =
        (uint16)((skipped_count > 0xFFFFu) ? 0xFFFFu : skipped_count);
}

static void phototube_scan_refresh_bounds(void)
{
    uint16 index;

    if (0u == scan_snapshot.point_count)
    {
        scan_snapshot.x_min_mm = 0.0f;
        scan_snapshot.x_max_mm = 0.0f;
        return;
    }
    scan_snapshot.x_min_mm = phototube_scan_point_at(0u)->x_mm;
    scan_snapshot.x_max_mm = phototube_scan_point_at(0u)->x_mm;
    for (index = 1u; index < scan_snapshot.point_count; index++)
    {
        const PhototubeScanPoint *point = phototube_scan_point_at(index);
        if (point->x_mm < scan_snapshot.x_min_mm)
        {
            scan_snapshot.x_min_mm = point->x_mm;
        }
        if (point->x_mm > scan_snapshot.x_max_mm)
        {
            scan_snapshot.x_max_mm = point->x_mm;
        }
    }
}

#pragma section all restore

static float phototube_scan_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void phototube_scan_fit(void)
{
    uint16 index;
    uint16 first_index;
    uint16 fit_count;
    float weight_sum = 0.0f;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float mean_x;
    float mean_y;
    float covariance_xx = 0.0f;
    float covariance_xy = 0.0f;
    float covariance_yy = 0.0f;
    float direction_x;
    float direction_y;
    float rms_sum = 0.0f;
    float max_error = 0.0f;
    float projection_min = 1.0e30f;
    float projection_max = -1.0e30f;

    scan_snapshot.valid = 0u;
    scan_snapshot.line_slope = 0.0f;
    scan_snapshot.line_intercept = 0.0f;
    scan_snapshot.line_center_x_mm = 0.0f;
    scan_snapshot.line_center_y_mm = 0.0f;
    scan_snapshot.line_direction_x = 0.0f;
    scan_snapshot.line_direction_y = 0.0f;
    scan_snapshot.rms_error_mm = 0.0f;
    scan_snapshot.max_abs_error_mm = 0.0f;

    if (scan_snapshot.point_count < 2u)
    {
        return;
    }

    /* A full route contains corners, so a single global line fit would make
     * every correct corner look like a scan failure.  Keep every point for
     * the exported map, but use only the latest short window for the optional
     * local straightness diagnostic shown in the header/UI. */
    fit_count = scan_snapshot.point_count;
    if (fit_count > PHOTOTUBE_SCAN_FIT_WINDOW_POINTS)
    {
        fit_count = PHOTOTUBE_SCAN_FIT_WINDOW_POINTS;
    }
    first_index = (uint16)(scan_snapshot.point_count - fit_count);

    for (index = first_index; index < scan_snapshot.point_count; index++)
    {
        const PhototubeScanPoint *point = phototube_scan_point_at(index);
        float weight = (float)point->confidence / 255.0f;
        weight_sum += weight;
        sum_x += weight * point->x_mm;
        sum_y += weight * point->y_mm;
    }
    if (weight_sum < 1.0e-3f)
    {
        return;
    }
    mean_x = sum_x / weight_sum;
    mean_y = sum_y / weight_sum;

    for (index = first_index; index < scan_snapshot.point_count; index++)
    {
        const PhototubeScanPoint *point = phototube_scan_point_at(index);
        float weight = (float)point->confidence / 255.0f;
        float dx = point->x_mm - mean_x;
        float dy = point->y_mm - mean_y;
        covariance_xx += weight * dx * dx;
        covariance_xy += weight * dx * dy;
        covariance_yy += weight * dy * dy;
    }
    {
        float angle = 0.5f * atan2f(2.0f * covariance_xy,
                                   covariance_xx - covariance_yy);
        direction_x = cosf(angle);
        direction_y = sinf(angle);
    }

    for (index = first_index; index < scan_snapshot.point_count; index++)
    {
        const PhototubeScanPoint *point = phototube_scan_point_at(index);
        float weight = (float)point->confidence / 255.0f;
        float dx = point->x_mm - mean_x;
        float dy = point->y_mm - mean_y;
        float projection = dx * direction_x + dy * direction_y;
        float error = -dx * direction_y + dy * direction_x;
        float abs_error = phototube_scan_abs(error);
        rms_sum += weight * error * error;
        if (projection < projection_min) projection_min = projection;
        if (projection > projection_max) projection_max = projection;
        if (abs_error > max_error)
        {
            max_error = abs_error;
        }
    }

    scan_snapshot.line_center_x_mm = mean_x;
    scan_snapshot.line_center_y_mm = mean_y;
    scan_snapshot.line_direction_x = direction_x;
    scan_snapshot.line_direction_y = direction_y;
    if (phototube_scan_abs(direction_x) > 1.0e-3f)
    {
        scan_snapshot.line_slope = direction_y / direction_x;
        scan_snapshot.line_intercept = mean_y -
                                       scan_snapshot.line_slope * mean_x;
    }
    scan_snapshot.rms_error_mm = sqrtf(rms_sum / weight_sum);
    scan_snapshot.max_abs_error_mm = max_error;
    scan_snapshot.valid = (uint8)((projection_max - projection_min >=
                                   PHOTOTUBE_SCAN_MIN_FIT_SPAN_MM) ? 1u : 0u);
}

void phototube_scan_init(void)
{
    phototube_scan_clear();
}

void phototube_scan_clear(void)
{
    /* point_count is the ownership boundary, so old records beyond it are
     * never observed.  Avoid clearing the full point archive while
     * prestart_service() holds interrupts disabled at the launch boundary. */
    memset((void *)&scan_snapshot, 0, sizeof(scan_snapshot));
    scan_oldest_index = 0u;
    scan_write_index = 0u;
    scan_origin_valid = 0u;
    scan_have_last_point = 0u;
    scan_origin_cos = 1.0f;
    scan_origin_sin = 0.0f;
    scan_mounting_cos = cosf(
        PHOTOTUBE_SCAN_MOUNTING_DEG * 0.01745329252f);
    scan_mounting_sin = sinf(
        PHOTOTUBE_SCAN_MOUNTING_DEG * 0.01745329252f);
    scan_last_sensor_x_mm = 0.0f;
    scan_last_sensor_y_mm = 0.0f;
    scan_last_travel_distance_mm = 0.0f;
    scan_latest_point_travel_distance_mm = 0.0f;
    scan_fit_dirty = 0u;
    scan_bounds_dirty = 0u;
    scan_source_drop_start_count = vision_capture_drop_count;
    scan_source_skipped_start_count = vision_capture_skipped_count;
}

void phototube_scan_start(void)
{
    phototube_scan_clear();
    /* nav_start() has just reset the route odometry and the launch gyro
     * heading.  Use that route frame directly instead of moving the origin to
     * the first valid phototube frame, otherwise a late first frame shifts the
     * whole scan map by an unknown distance. */
    scan_snapshot.origin_x_mm = 0.0f;
    scan_snapshot.origin_y_mm = 0.0f;
    scan_snapshot.origin_yaw_rad = 0.0f;
    scan_origin_cos = cosf(scan_snapshot.origin_yaw_rad);
    scan_origin_sin = sinf(scan_snapshot.origin_yaw_rad);
    scan_origin_valid = 1u;
    scan_source_drop_start_count = vision_capture_drop_count;
    scan_source_skipped_start_count = vision_capture_skipped_count;
    scan_snapshot.active = 1u;
}

void phototube_scan_stop(void)
{
    phototube_scan_refresh_source_stats();
    scan_snapshot.active = 0u;
}

uint8 phototube_scan_is_active(void)
{
    return scan_snapshot.active;
}

uint16 phototube_scan_push_hits(const VisionScanHitPacket *packet,
                                const DriveOdometryPose *pose,
                                float travel_distance_mm,
                                uint32 consume_ms)
{
    float axle_x;
    float axle_y;
    float cos_relative_yaw;
    float sin_relative_yaw;
    float cos_mounting;
    float sin_mounting;
    float sensor_x;
    float sensor_y;
    uint16 hit_index;
    uint16 accepted_hit_count = 0u;
    uint32 latency_ms;

    if ((NULL == packet) || (NULL == pose) ||
        (0u == scan_snapshot.active))
    {
        return 0u;
    }

    if (0u == scan_origin_valid)
    {
        return 0u;
    }

    if (0u == packet->hit_count)
    {
        return 0u;
    }

    /* Convert the axle and body basis to the run's raw-odom scan frame once
     * per packet.  Every hit shares this pose; per-hit sinf/cosf calls would
     * only repeat the same transform at 1 kHz. */
    {
        float dx = pose->x_mm - scan_snapshot.origin_x_mm;
        float dy = pose->y_mm - scan_snapshot.origin_y_mm;
        float relative_yaw = pose->yaw_rad - scan_snapshot.origin_yaw_rad;

        axle_x = dx * scan_origin_cos + dy * scan_origin_sin;
        axle_y = -dx * scan_origin_sin + dy * scan_origin_cos;
        cos_relative_yaw = cosf(relative_yaw);
        sin_relative_yaw = sinf(relative_yaw);
    }
    cos_mounting = scan_mounting_cos;
    sin_mounting = scan_mounting_sin;
    /* The spacing gate follows the physical sensor origin.  Unlike the old
     * axle-distance gate this includes the forward mount sweeping an arc. */
    sensor_x = axle_x + PHOTOTUBE_SCAN_FORWARD_MM * cos_relative_yaw;
    sensor_y = axle_y + PHOTOTUBE_SCAN_FORWARD_MM * sin_relative_yaw;

    if (scan_have_last_point)
    {
        float dx = sensor_x - scan_last_sensor_x_mm;
        float dy = sensor_y - scan_last_sensor_y_mm;
        float delta_travel_mm = travel_distance_mm - scan_last_travel_distance_mm;
        if (delta_travel_mm < 0.0f)
        {
            phototube_scan_add_u16(
                &scan_snapshot.travel_reversal_frame_count, 1u);
            return 0u;
        }
        if ((dx * dx + dy * dy) <
            PHOTOTUBE_SCAN_MIN_STEP_MM * PHOTOTUBE_SCAN_MIN_STEP_MM)
        {
            phototube_scan_add_u16(
                &scan_snapshot.spatial_dedup_frame_count, 1u);
            return 0u;
        }
    }

    latency_ms = consume_ms - packet->capture_ms;
    if (latency_ms > scan_snapshot.max_capture_to_cloud_latency_ms)
    {
        scan_snapshot.max_capture_to_cloud_latency_ms =
            (uint16)((latency_ms > 0xFFFFu) ? 0xFFFFu : latency_ms);
    }
    phototube_scan_add_u16(&scan_snapshot.accepted_frame_count, 1u);

    for (hit_index = 0u;
         hit_index < packet->hit_count && hit_index < VISION_SCAN_HIT_MAX_COUNT;
         hit_index++)
    {
        const VisionScanHit *hit = &packet->hits[hit_index];
        PhototubeScanPoint *point = &scan_points[scan_write_index];
        float body_x;
        float body_y;
        float scan_x;
        float scan_y;

        if (0u == hit->confidence)
        {
            continue;
        }
        body_x = PHOTOTUBE_SCAN_FORWARD_MM -
                 sin_mounting * hit->lateral_mm;
        body_y = cos_mounting * hit->lateral_mm;
        scan_x = axle_x + body_x * cos_relative_yaw -
                 body_y * sin_relative_yaw;
        scan_y = axle_y + body_x * sin_relative_yaw +
                 body_y * cos_relative_yaw;
        point->frame_seq = packet->frame_seq;
        point->x_mm = scan_x;
        point->y_mm = scan_y;
        point->travel_distance_mod_mm = (uint16)(
            (travel_distance_mm <= 0.0f) ? 0u :
            (uint32)(travel_distance_mm + 0.5f));
        point->confidence = hit->confidence;
        point->point_flags = hit->flags;

        /* Queue the immutable raw-odom hit for bounded background EMEM writes.
         * The record carries capture-time yaw so the host can reconstruct the
         * sensor/axle pose even after the online ring has advanced. */
        telemetry_record_phototube_scan_point(
            point->frame_seq,
            packet->capture_ms,
            point->x_mm,
            point->y_mm,
            hit->lateral_mm,
            pose->yaw_rad,
            travel_distance_mm,
            point->confidence,
            point->point_flags);

        if (scan_snapshot.point_count < PHOTOTUBE_SCAN_MAX_POINTS)
        {
            scan_snapshot.point_count++;
            if (1u == scan_snapshot.point_count)
            {
                scan_snapshot.x_min_mm = scan_x;
                scan_snapshot.x_max_mm = scan_x;
            }
            else
            {
                if (scan_x < scan_snapshot.x_min_mm) scan_snapshot.x_min_mm = scan_x;
                if (scan_x > scan_snapshot.x_max_mm) scan_snapshot.x_max_mm = scan_x;
            }
        }
        else
        {
            scan_oldest_index =
                phototube_scan_next_physical_index(scan_oldest_index);
            phototube_scan_add_u16(
                &scan_snapshot.online_ring_overwrite_point_count, 1u);
            scan_bounds_dirty = 1u;
        }
        scan_write_index =
            phototube_scan_next_physical_index(scan_write_index);
        phototube_scan_add_u16(&scan_snapshot.accepted_hit_count, 1u);
        accepted_hit_count++;
    }

    if ((0u == accepted_hit_count) || (0u == scan_snapshot.point_count))
    {
        /* A packet can carry hit slots whose confidence was reduced to zero
         * (for example after a producer-side quality calculation).  It did
         * not add any raw-odom point, so it must not become the next spatial
         * de-duplication anchor.  Otherwise a later valid frame within the
         * 5 mm gate would be silently discarded because of this empty frame. */
        return accepted_hit_count;
    }
    /* Advance the spatial gate only after at least one immutable point was
     * accepted.  Empty/zero-confidence frames remain visible in the counters
     * but never suppress a subsequent real observation. */
    scan_last_sensor_x_mm = sensor_x;
    scan_last_sensor_y_mm = sensor_y;
    scan_last_travel_distance_mm = travel_distance_mm;
    scan_have_last_point = 1u;
    scan_latest_point_travel_distance_mm = travel_distance_mm;
    scan_snapshot.first_frame_seq = phototube_scan_point_at(0u)->frame_seq;
    scan_snapshot.last_frame_seq = packet->frame_seq;
    scan_fit_dirty = 1u;
    return accepted_hit_count;
}

void phototube_scan_note_pose_source(PhototubeScanPoseSource source)
{
    if (0u == scan_snapshot.active)
    {
        return;
    }
    switch (source)
    {
    case PHOTOTUBE_SCAN_POSE_INTERPOLATED:
        phototube_scan_add_u16(&scan_snapshot.pose_interpolated_count, 1u);
        break;
    default:
        break;
    }
}

void phototube_scan_note_pose_rejected(void)
{
    if (0u != scan_snapshot.active)
    {
        phototube_scan_add_u16(&scan_snapshot.pose_rejected_count, 1u);
    }
}

void phototube_scan_note_pose_wait(void)
{
    if (0u != scan_snapshot.active)
    {
        phototube_scan_add_u16(&scan_snapshot.pose_wait_count, 1u);
    }
}

void phototube_scan_note_mailbox_gap(uint32 skipped_count)
{
    if (0u == scan_snapshot.active)
    {
        return;
    }
    phototube_scan_add_u16(&scan_snapshot.mailbox_skipped_count, skipped_count);
}

void phototube_scan_note_hit_packet(const VisionScanHitPacket *packet)
{
    uint16 hit_index;
    uint8 have_usable_hit = 0u;
    uint32 latency_ms;

    if ((NULL == packet) || (0u == scan_snapshot.active))
    {
        return;
    }
    phototube_scan_add_u16(&scan_snapshot.received_frame_count, 1u);
    for (hit_index = 0u;
         hit_index < packet->hit_count &&
         hit_index < VISION_SCAN_HIT_MAX_COUNT;
         hit_index++)
    {
        if (0u != packet->hits[hit_index].confidence)
        {
            have_usable_hit = 1u;
            break;
        }
    }
    if (0u == have_usable_hit)
    {
        /* Count both an explicitly empty packet and a packet whose hit slots
         * carry no usable confidence.  This is the quantity that matters to
         * the point-cloud ingress and keeps the drop reason observable without
         * growing the fixed 56-byte archive header. */
        phototube_scan_add_u16(&scan_snapshot.empty_hit_frame_count, 1u);
    }
    latency_ms = packet->publish_ms - packet->capture_ms;
    if (latency_ms > scan_snapshot.max_processing_latency_ms)
    {
        scan_snapshot.max_processing_latency_ms =
            (uint16)((latency_ms > 0xFFFFu) ? 0xFFFFu : latency_ms);
    }
    phototube_scan_add_u16(&scan_snapshot.source_frame_skipped_count,
                           packet->source_frame_gap);
}

void phototube_scan_note_staging_high_watermark(uint16 count)
{
    if ((0u != scan_snapshot.active) &&
        (count > scan_snapshot.staging_high_watermark))
    {
        scan_snapshot.staging_high_watermark = count;
    }
}

void phototube_scan_note_staging_backpressure(void)
{
    if (0u != scan_snapshot.active)
    {
        phototube_scan_add_u16(&scan_snapshot.staging_backpressure_count, 1u);
    }
}

void phototube_scan_note_epoch_filtered(void)
{
    if (0u != scan_snapshot.active)
    {
        phototube_scan_add_u16(&scan_snapshot.epoch_filtered_frame_count, 1u);
    }
}

void phototube_scan_note_epoch_ack_timeout(void)
{
    if (0u != scan_snapshot.active)
    {
        phototube_scan_add_u16(&scan_snapshot.epoch_ack_timeout_count, 1u);
    }
}

void phototube_scan_finish_batch(void)
{
    if (0u == scan_fit_dirty)
    {
        return;
    }
    if (scan_snapshot.point_count > 0u)
    {
        phototube_scan_fit();
    }
    scan_fit_dirty = 0u;
}

uint8 phototube_scan_get_snapshot(PhototubeScanSnapshot *snapshot)
{
    if (NULL == snapshot)
    {
        return 0u;
    }
    if (scan_snapshot.active)
    {
        phototube_scan_refresh_source_stats();
    }
    else if (scan_bounds_dirty)
    {
        /* x_min/x_max are only used by the stopped-run display.  Recomputing
         * them after every overwrite would scan all 1280 online points each
         * 10 ms while driving, even though localization never reads them. */
        phototube_scan_refresh_bounds();
        scan_bounds_dirty = 0u;
    }
    *snapshot = scan_snapshot;
    return 1u;
}

uint8 phototube_scan_get_point(uint16 index, PhototubeScanPoint *point)
{
    if ((NULL == point) || (index >= scan_snapshot.point_count))
    {
        return 0u;
    }
    *point = *phototube_scan_point_at(index);
    return 1u;
}

const PhototubeScanPoint *phototube_scan_get_point_view(uint16 logical_index)
{
    if (logical_index >= scan_snapshot.point_count)
    {
        return NULL;
    }
    return phototube_scan_point_at(logical_index);
}

uint8 phototube_scan_get_travel_window(float travel_window_mm,
                                       uint16 *first_index_out,
                                       uint16 *point_count)
{
    uint16 first_index;
    uint16 search_end;
    uint16 last_index;
    uint16 latest_distance_mod_mm;

    if ((NULL == first_index_out) || (NULL == point_count) ||
        (travel_window_mm <= 0.0f) ||
        (0u == scan_snapshot.point_count))
    {
        return 0u;
    }
    last_index = (uint16)(scan_snapshot.point_count - 1u);
    latest_distance_mod_mm =
        phototube_scan_point_at(last_index)->travel_distance_mod_mm;

    /* Accepted travel is monotonic and the online ring spans far less than
     * one uint16 revolution.  Point age therefore remains monotonic even
     * when the run crosses 65535 mm, allowing a lower-bound search instead
     * of walking as many as 1280 points every 10 ms. */
    first_index = 0u;
    search_end = scan_snapshot.point_count;
    while (first_index < search_end)
    {
        uint16 middle = (uint16)(first_index +
            (uint16)((search_end - first_index) / 2u));
        const PhototubeScanPoint *point = phototube_scan_point_at(middle);
        uint16 age_mm = (uint16)(latest_distance_mod_mm -
                                 point->travel_distance_mod_mm);

        if ((float)age_mm > travel_window_mm)
        {
            first_index = (uint16)(middle + 1u);
        }
        else
        {
            search_end = middle;
        }
    }
    *first_index_out = first_index;
    *point_count = (uint16)(scan_snapshot.point_count - first_index);
    return 1u;
}

uint8 phototube_scan_get_point_travel_distance_mm(uint16 logical_index,
                                                  float *travel_distance_mm)
{
    const PhototubeScanPoint *point;
    const PhototubeScanPoint *latest_point;
    uint16 age_mm;

    if ((NULL == travel_distance_mm) ||
        (logical_index >= scan_snapshot.point_count) ||
        (0u == scan_snapshot.point_count))
    {
        return 0u;
    }
    point = phototube_scan_point_at(logical_index);
    latest_point = phototube_scan_point_at(
        (uint16)(scan_snapshot.point_count - 1u));
    age_mm = (uint16)(latest_point->travel_distance_mod_mm -
                      point->travel_distance_mod_mm);
    *travel_distance_mm = scan_latest_point_travel_distance_mm -
                          (float)age_mm;
    if (*travel_distance_mm < 0.0f)
    {
        *travel_distance_mm = 0.0f;
    }
    return 1u;
}
