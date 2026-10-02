/**
 * @file phototube_array.c
 * @brief CPU1 端 16 路模拟光电扫描、候选提取和发布。
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
 * @note CPU1 以 1 kHz 扫描，发布时必须保持跨核提交顺序。
 */

// phototube_array.c - real 16-channel analog scan and observation publisher.

#include "phototube_array.h"

#include <math.h>
#include <string.h>

#include "globals.h"
#include "gyroscope.h"

#define PHOTOTUBE_ARRAY_RAD_PER_DEG (0.01745329251994329577f)
#define PHOTOTUBE_ARRAY_TIMER_TICKS_PER_SECOND (100000000u)
#define PHOTOTUBE_ARRAY_SCAN_PERIOD_TICKS \
    (PHOTOTUBE_ARRAY_TIMER_TICKS_PER_SECOND / PHOTOTUBE_ARRAY_SCAN_HZ)

#if (PHOTOTUBE_ARRAY_TIMER_TICKS_PER_SECOND % PHOTOTUBE_ARRAY_SCAN_HZ) != 0u
#error "Phototube scan frequency must divide the 10 ns timer rate"
#endif

#if ((VISION_SCAN_HIT_MAX_COUNT < PHOTOTUBE_ARRAY_MAX_CANDIDATES) || \
     (VISION_SCAN_HIT_MAX_COUNT < PHOTOTUBE_ARRAY_FULL_SCAN_HIT_COUNT))
#error "scan hit packet cannot hold all phototube candidates"
#endif

#pragma section all "cpu1_dsram"

typedef struct
{
    float center_mm;
    float width_mm;
    float variance_mm2;
    float signal_sum;
    uint8 first_index;
    uint8 last_index;
} PhototubeCandidate;

typedef struct
{
    float center_mm;
    float width_mm;
    float capture_yaw_deg;
    uint32 capture_ms;
    uint8 initialized;
} PhototubeTrackState;

static volatile uint32 phototube_snapshot_version = 0u;
static volatile PhototubeArraySnapshot phototube_snapshot;
static PhototubeTrackState phototube_track;
static uint32 phototube_frame_seq = 0u;
static uint32 phototube_last_scan_tick = 0u;
static uint32 phototube_timing_epoch_seq = 0u;
static uint8 phototube_scan_started = 0u;

#pragma section all restore

static float phototube_position_mm(uint8 index)
{
    return ((float)index - 7.5f) * PHOTOTUBE_ARRAY_SPACING_MM;
}

/* Keep physical MUX order separate from logical left-to-right order.  Every
 * consumer, including the display and raw-odom point cloud, uses the logical
 * index; no later stage is allowed to mirror it independently. */
static uint8 phototube_sensor_index(uint8 mux_channel)
{
    return PHOTOTUBE_ARRAY_Y0_IS_LEFT ? mux_channel :
        (uint8)(PHOTOTUBE_ARRAY_COUNT - 1u - mux_channel);
}

static float phototube_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float phototube_wrap_deg(float value)
{
    while (value > 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
}

static uint8 phototube_clamp_u8(float value)
{
    if (value <= 0.0f) return 0u;
    if (value >= 255.0f) return 255u;
    return (uint8)(value + 0.5f);
}

static uint8 phototube_extract_candidates(
    const float weights[PHOTOTUBE_ARRAY_COUNT],
    PhototubeCandidate candidates[PHOTOTUBE_ARRAY_MAX_CANDIDATES])
{
    uint8 candidate_count = 0u;
    uint8 index = 0u;

    while (index < PHOTOTUBE_ARRAY_COUNT)
    {
        uint8 first;
        uint8 last;
        if (weights[index] < PHOTOTUBE_ARRAY_ACTIVE_RATIO)
        {
            index++;
            continue;
        }
        first = index;
        while ((index < PHOTOTUBE_ARRAY_COUNT) &&
               (weights[index] >= PHOTOTUBE_ARRAY_ACTIVE_RATIO)) index++;
        last = (uint8)(index - 1u);
        if (candidate_count >= PHOTOTUBE_ARRAY_MAX_CANDIDATES) continue;
        candidates[candidate_count].center_mm = 0.0f;
        candidates[candidate_count].width_mm =
            (float)(last - first + 1u) * PHOTOTUBE_ARRAY_SPACING_MM;
        candidates[candidate_count].variance_mm2 = 0.0f;
        candidates[candidate_count].signal_sum = 0.0f;
        candidates[candidate_count].first_index = first;
        candidates[candidate_count].last_index = last;
        candidate_count++;
    }
    for (index = 0u; index < candidate_count; index++)
    {
        uint8 first = (0u == index) ? 0u :
            (uint8)(((uint16)candidates[index - 1u].last_index +
                      (uint16)candidates[index].first_index) / 2u + 1u);
        uint8 last = ((uint8)(index + 1u) >= candidate_count) ?
            (uint8)(PHOTOTUBE_ARRAY_COUNT - 1u) :
            (uint8)(((uint16)candidates[index].last_index +
                      (uint16)candidates[index + 1u].first_index) / 2u);
        float sum = 0.0f;
        float moment = 0.0f;
        float variance = 0.0f;
        uint8 sample;
        for (sample = first; sample <= last; sample++)
        {
            sum += weights[sample];
            moment += weights[sample] * phototube_position_mm(sample);
        }
        if (sum <= 1.0e-6f) continue;
        candidates[index].center_mm = moment / sum;
        for (sample = first; sample <= last; sample++)
        {
            float delta = phototube_position_mm(sample) -
                          candidates[index].center_mm;
            variance += weights[sample] * delta * delta;
        }
        candidates[index].variance_mm2 = variance / sum;
        candidates[index].signal_sum = sum;
    }
    return candidate_count;
}

static int16 phototube_select_candidate(
    const PhototubeCandidate candidates[PHOTOTUBE_ARRAY_MAX_CANDIDATES],
    uint8 candidate_count, uint32 capture_ms, float capture_yaw_deg,
    uint8 full_coverage)
{
    float expected_center;
    float yaw_step;
    float best_score = 1.0e30f;
    float second_score = 1.0e30f;
    int16 best = -1;
    uint8 index;

    if (full_coverage || (0u == candidate_count)) return -1;
    if (phototube_track.initialized &&
        ((uint32)(capture_ms - phototube_track.capture_ms) >
         PHOTOTUBE_ARRAY_TRACK_HOLD_MS)) phototube_track.initialized = 0u;
    if (!phototube_track.initialized)
        return (1u == candidate_count) ? 0 : -1;
    yaw_step = phototube_wrap_deg(capture_yaw_deg -
                                  phototube_track.capture_yaw_deg);
    if (phototube_abs(yaw_step) > PHOTOTUBE_ARRAY_TRACK_MAX_YAW_STEP_DEG)
    {
        phototube_track.initialized = 0u;
        return (1u == candidate_count) ? 0 : -1;
    }
    expected_center = phototube_track.center_mm -
        PHOTOTUBE_ARRAY_FORWARD_MM *
        tanf(yaw_step * PHOTOTUBE_ARRAY_RAD_PER_DEG);
    for (index = 0u; index < candidate_count; index++)
    {
        float center_step = phototube_abs(candidates[index].center_mm -
                                          expected_center);
        float width_step = phototube_abs(candidates[index].width_mm -
                                         phototube_track.width_mm);
        float score;
        if ((center_step > PHOTOTUBE_ARRAY_TRACK_MAX_CENTER_STEP_MM) ||
            (width_step > PHOTOTUBE_ARRAY_TRACK_MAX_WIDTH_STEP_MM)) continue;
        score = center_step + PHOTOTUBE_ARRAY_TRACK_WIDTH_SCORE_GAIN * width_step;
        if (score < best_score)
        {
            second_score = best_score;
            best_score = score;
            best = (int16)index;
        }
        else if (score < second_score) second_score = score;
    }
    if (best < 0) return -1;
    if ((second_score < 1.0e29f) &&
        ((second_score - best_score) < PHOTOTUBE_ARRAY_TRACK_UNIQUE_MARGIN_MM))
        return -1;
    return best;
}

static void phototube_accept_candidate(const PhototubeCandidate *candidate,
                                       uint32 capture_ms,
                                       float capture_yaw_deg)
{
    phototube_track.center_mm = candidate->center_mm;
    phototube_track.width_mm = candidate->width_mm;
    phototube_track.capture_yaw_deg = capture_yaw_deg;
    phototube_track.capture_ms = capture_ms;
    phototube_track.initialized = 1u;
}

static float phototube_hit_quality(float signal_sum, float width_mm,
                                   float variance_mm2, float contrast,
                                   uint8 edge)
{
    float quality = 0.15f + 0.30f * signal_sum * 0.28f +
                    0.30f * contrast / 60.0f +
                    0.15f / (1.0f + variance_mm2 / 900.0f) +
                    0.10f / (1.0f + width_mm / 80.0f);
    if (quality > 1.0f) quality = 1.0f;
    if (edge) quality *= 0.65f;
    if (quality < 0.05f) quality = 0.05f;
    return quality;
}

static void phototube_publish_observation(const VisionPoseObservation *source)
{
    uint32 sequence = vision_queue_sequence_next(vision_pose_publish_seq);
    uint8 slot;
    volatile VisionPoseObservation *destination;
    slot = (uint8)((vision_pose_publish_slot + 1u) &
                   (VISION_OBSERVATION_QUEUE_CAPACITY - 1u));
    destination = &vision_pose_slots[slot];
    /* Invalidate the slot before changing its payload.  A CPU0 reader may be
     * copying the previous generation at this exact moment; the final
     * sequence write below is the commit marker it must observe after the
     * payload is complete. */
    destination->publish_seq = 0u;
    __dsync();
    destination->frame_seq = source->frame_seq;
    destination->capture_ms = source->capture_ms;
    destination->capture_epoch = source->capture_epoch;
    destination->capture_gyro_yaw_deg = source->capture_gyro_yaw_deg;
    destination->line_position_px = source->line_position_px;
    destination->line_position_mm = source->line_position_mm;
    destination->line_slope_px_per_row = 0.0f;
    destination->fit_mse_px2 = source->fit_mse_px2;
    destination->valid_rows = source->valid_rows;
    destination->candidate_rows = source->candidate_rows;
    destination->row_span = source->row_span;
    destination->flags = source->flags;
    destination->white_percent = source->white_percent;
    destination->reserved[0] = 0u;
    destination->reserved[1] = 0u;
    destination->reserved[2] = 0u;
    destination->active_mask = source->active_mask;
    destination->peak_count = source->peak_count;
    destination->threshold_gray = source->threshold_gray;
    __dsync();
    destination->publish_seq = sequence;
    __dsync();
    vision_pose_publish_slot = slot;
    __dsync();
    vision_pose_publish_seq = sequence;
}

static void phototube_publish_hits(const PhototubeArraySnapshot *local,
                                   const PhototubeCandidate candidates[PHOTOTUBE_ARRAY_MAX_CANDIDATES],
                                   uint8 candidate_count,
                                   const float weights[PHOTOTUBE_ARRAY_COUNT],
                                   uint32 capture_epoch)
{
    VisionScanHitPacket packet;
    float qualities[VISION_SCAN_HIT_MAX_COUNT];
    float quality_sum = 0.0f;
    uint16 hit_count = 0u;
    uint16 index;
    uint32 sequence;
    uint32 source_sequence_delta;
    uint8 slot;
    volatile VisionScanHitPacket *destination;
    memset(&packet, 0, sizeof(packet));
    packet.frame_seq = local->frame_seq;
    packet.capture_ms = local->capture_ms;
    packet.publish_ms = vision_clock_ms;
    packet.capture_epoch = capture_epoch;
    source_sequence_delta = local->frame_seq - vision_processed_seq;
    packet.source_frame_gap = (uint16)(
        (source_sequence_delta <= 1u) ? 0u :
        ((source_sequence_delta - 1u) > 0xFFFFu) ? 0xFFFFu :
        (source_sequence_delta - 1u));
    packet.flags = local->full_coverage ? VISION_SCAN_HIT_FULL_COVERAGE : 0u;
    packet.frame_quality = phototube_clamp_u8(
        255.0f * (0.35f + local->contrast_gray / 80.0f) *
        (1.0f - (float)local->white_percent / 200.0f));
    if (packet.frame_quality < 32u) packet.frame_quality = 32u;
    if (local->full_coverage)
    {
        for (index = 0u; index < PHOTOTUBE_ARRAY_FULL_SCAN_HIT_COUNT; index++)
        {
            uint8 sample = (uint8)(((uint32)index *
                (PHOTOTUBE_ARRAY_COUNT - 1u) + 3u) / 7u);
            if ((weights[sample] > 0.02f) &&
                (hit_count < VISION_SCAN_HIT_MAX_COUNT))
            {
                packet.hits[hit_count].lateral_mm = phototube_position_mm(sample);
                packet.hits[hit_count].flags = VISION_SCAN_HIT_FULL_COVERAGE |
                    (((sample == 0u) || (sample == PHOTOTUBE_ARRAY_COUNT - 1u)) ?
                     VISION_SCAN_HIT_EDGE : 0u);
                qualities[hit_count] = 0.20f + weights[sample] +
                                       0.20f * local->contrast_gray / 60.0f;
                if (packet.hits[hit_count].flags & VISION_SCAN_HIT_EDGE)
                    qualities[hit_count] *= 0.65f;
                quality_sum += qualities[hit_count++];
            }
        }
    }
    else if (candidate_count > 1u)
    {
        for (index = 0u; index < candidate_count &&
                        hit_count < VISION_SCAN_HIT_MAX_COUNT; index++)
        {
            uint8 edge = (uint8)((candidates[index].first_index == 0u) ||
                (candidates[index].last_index == PHOTOTUBE_ARRAY_COUNT - 1u));
            packet.hits[hit_count].lateral_mm = candidates[index].center_mm;
            packet.hits[hit_count].flags = VISION_SCAN_HIT_MULTIPLE |
                                           VISION_SCAN_HIT_CENTER |
                                           (edge ? VISION_SCAN_HIT_EDGE : 0u);
            qualities[hit_count] = phototube_hit_quality(
                candidates[index].signal_sum, candidates[index].width_mm,
                candidates[index].variance_mm2, local->contrast_gray, edge);
            quality_sum += qualities[hit_count++];
        }
    }
    else if (1u == candidate_count)
    {
        const PhototubeCandidate *candidate = &candidates[0u];
        uint8 edge = (uint8)((candidate->first_index == 0u) ||
            (candidate->last_index == PHOTOTUBE_ARRAY_COUNT - 1u));
        float quality = phototube_hit_quality(candidate->signal_sum,
            candidate->width_mm, candidate->variance_mm2,
            local->contrast_gray, edge);
        packet.hits[hit_count].lateral_mm = candidate->center_mm;
        packet.hits[hit_count].flags = VISION_SCAN_HIT_SINGLE |
                                       VISION_SCAN_HIT_CENTER |
                                       (edge ? VISION_SCAN_HIT_EDGE : 0u);
        qualities[hit_count++] = quality;
        quality_sum += quality;
        if (candidate->last_index > candidate->first_index)
        {
            float first_mm = phototube_position_mm(candidate->first_index);
            float last_mm = phototube_position_mm(candidate->last_index);
            if ((fabsf(first_mm - candidate->center_mm) > 0.1f) &&
                (hit_count < VISION_SCAN_HIT_MAX_COUNT))
            {
                packet.hits[hit_count].lateral_mm = first_mm;
                packet.hits[hit_count].flags = VISION_SCAN_HIT_SINGLE |
                    VISION_SCAN_HIT_BOUNDARY |
                    ((candidate->first_index == 0u) ? VISION_SCAN_HIT_EDGE : 0u);
                qualities[hit_count] = quality *
                    PHOTOTUBE_ARRAY_BOUNDARY_QUALITY_RATIO;
                quality_sum += qualities[hit_count++];
            }
            if ((fabsf(last_mm - candidate->center_mm) > 0.1f) &&
                (fabsf(last_mm - first_mm) > 0.1f) &&
                (hit_count < VISION_SCAN_HIT_MAX_COUNT))
            {
                packet.hits[hit_count].lateral_mm = last_mm;
                packet.hits[hit_count].flags = VISION_SCAN_HIT_SINGLE |
                    VISION_SCAN_HIT_BOUNDARY |
                    ((candidate->last_index == PHOTOTUBE_ARRAY_COUNT - 1u) ?
                     VISION_SCAN_HIT_EDGE : 0u);
                qualities[hit_count] = quality *
                    PHOTOTUBE_ARRAY_BOUNDARY_QUALITY_RATIO;
                quality_sum += qualities[hit_count++];
            }
        }
        if ((candidate->last_index - candidate->first_index) *
            PHOTOTUBE_ARRAY_SPACING_MM >= PHOTOTUBE_ARRAY_RUN_GEOMETRY_MIN_SPAN_MM)
        {
            for (index = 0u; index < hit_count; index++)
                packet.hits[index].flags |= VISION_SCAN_HIT_RUN_SAMPLE;
        }
    }
    packet.hit_count = hit_count;
    for (index = 0u; index < hit_count; index++)
    {
        packet.hits[index].confidence = phototube_clamp_u8(
            (float)packet.frame_quality * qualities[index] /
            ((quality_sum > 1.0e-6f) ? quality_sum : 1.0f));
    }
    if ((hit_count > 0u) && (0u == packet.hits[0u].confidence))
        packet.hits[0u].confidence = 1u;
    sequence = vision_queue_sequence_next(vision_scan_hit_publish_seq);
    slot = (uint8)((vision_scan_hit_publish_slot + 1u) &
                   (VISION_SCAN_HIT_QUEUE_CAPACITY - 1u));
    destination = &vision_scan_hit_slots[slot];
    /* publish_seq is a two-phase slot commit marker; do not expose the new
     * generation while any hit payload field is still being written. */
    destination->publish_seq = 0u;
    __dsync();
    destination->frame_seq = packet.frame_seq;
    destination->capture_ms = packet.capture_ms;
    destination->publish_ms = packet.publish_ms;
    destination->capture_epoch = packet.capture_epoch;
    destination->hit_count = packet.hit_count;
    destination->source_frame_gap = packet.source_frame_gap;
    destination->flags = packet.flags;
    destination->frame_quality = packet.frame_quality;
    destination->reserved = 0u;
    for (index = 0u; index < hit_count; index++)
        destination->hits[index] = packet.hits[index];
    __dsync();
    destination->publish_seq = sequence;
    __dsync();
    vision_scan_hit_publish_slot = slot;
    __dsync();
    vision_scan_hit_publish_seq = sequence;
}

static void phototube_publish_snapshot(const PhototubeArraySnapshot *source)
{
    uint32 version = phototube_snapshot_version + 1u;
    if (0u == (version & 1u)) version++;
    phototube_snapshot_version = version;
    __dsync();
    phototube_snapshot = *source;
    __dsync();
    phototube_snapshot_version = version + 1u;
}

static void phototube_process(const uint16 values[PHOTOTUBE_ARRAY_COUNT],
                              uint32 frame_seq, uint32 capture_ms,
                              float capture_yaw_deg,
                              uint32 capture_epoch)
{
    PhototubeArraySnapshot local;
    VisionPoseObservation observation;
    PhototubeCandidate candidates[PHOTOTUBE_ARRAY_MAX_CANDIDATES];
    float weights[PHOTOTUBE_ARRAY_COUNT];
    uint16 minimum;
    uint16 maximum;
    uint8 candidate_count;
    int16 selected;
    uint8 index;
    int16 first_active = -1;
    int16 last_active = -1;
    uint16 flags = VISION_OBS_FLAG_VIRTUAL_LINE;
    uint32 stats_version;
    memset(&local, 0, sizeof(local));
    memset(&observation, 0, sizeof(observation));
    local.frame_seq = frame_seq;
    local.capture_ms = capture_ms;
    local.threshold_gray =
        (uint8)(PHOTOTUBE_ARRAY_LINE_THRESHOLD_ADC >> 4u);
    minimum = PHOTOTUBE_ARRAY_ADC_FULL_SCALE;
    maximum = 0u;
    for (index = 0u; index < PHOTOTUBE_ARRAY_COUNT; index++)
    {
        uint8 active;
        float weight;
        local.raw_adc[index] = values[index];
        if (values[index] < minimum) minimum = values[index];
        if (values[index] > maximum) maximum = values[index];
        active = (uint8)(values[index] <=
                         PHOTOTUBE_ARRAY_LINE_THRESHOLD_ADC);
        weight = active ?
            ((float)(PHOTOTUBE_ARRAY_ADC_FULL_SCALE - values[index]) /
             (float)PHOTOTUBE_ARRAY_ADC_FULL_SCALE) : 0.0f;
        weights[index] = weight;
        local.strength[index] = (uint8)(weight * 255.0f + 0.5f);
        local.signal_sum += weight;
        if (active)
        {
            local.active_mask |= (uint16)(1u << index);
            local.active_count++;
            if (first_active < 0) first_active = (int16)index;
            last_active = (int16)index;
        }
    }
    local.contrast_gray = (float)(maximum - minimum) / 16.0f;
    local.white_percent = (uint8)((local.active_count * 100u) /
                                  PHOTOTUBE_ARRAY_COUNT);
    local.full_coverage = (uint8)(local.white_percent >=
                                  PHOTOTUBE_ARRAY_FULL_COVERAGE_PERCENT);
    candidate_count = phototube_extract_candidates(weights, candidates);
    local.peak_count = candidate_count;
    selected = phototube_select_candidate(candidates, candidate_count,
                                           capture_ms, capture_yaw_deg,
                                           local.full_coverage);
    if (selected >= 0)
    {
        const PhototubeCandidate *candidate = &candidates[(uint8)selected];
        local.front_offset_mm = candidate->center_mm;
        local.variance_mm2 = candidate->variance_mm2;
        local.valid = 1u;
        phototube_accept_candidate(candidate, capture_ms, capture_yaw_deg);
    }
    local.edge_detected = (uint8)(local.full_coverage ||
        (0u != (local.active_mask & 0x0001u)) ||
        (0u != (local.active_mask & 0x8000u)));
    if (local.valid)
        flags |= VISION_OBS_FLAG_FIT_VALID | VISION_OBS_FLAG_COVERAGE |
                 VISION_OBS_FLAG_LATERAL_VALID;
    else if (local.full_coverage || (local.peak_count > 0u))
        flags |= VISION_OBS_FLAG_SELECTION_AMBIGUOUS;
    if (!local.edge_detected) flags |= VISION_OBS_FLAG_EDGE_VALID;
    if (local.full_coverage) flags |= VISION_OBS_FLAG_FULL_COVERAGE;
    observation.frame_seq = frame_seq;
    observation.capture_ms = capture_ms;
    observation.capture_epoch = capture_epoch;
    observation.capture_gyro_yaw_deg = capture_yaw_deg;
    observation.line_position_px = local.front_offset_mm /
                                   PHOTOTUBE_ARRAY_SPACING_MM;
    observation.line_position_mm = local.front_offset_mm;
    observation.fit_mse_px2 = local.variance_mm2 /
                              (PHOTOTUBE_ARRAY_SPACING_MM *
                               PHOTOTUBE_ARRAY_SPACING_MM);
    observation.valid_rows = local.active_count;
    observation.candidate_rows = PHOTOTUBE_ARRAY_COUNT;
    observation.row_span = (first_active >= 0) ?
        (uint16)((float)(last_active - first_active) *
                 PHOTOTUBE_ARRAY_SPACING_MM + 0.5f) : 0u;
    observation.flags = flags;
    observation.white_percent = local.white_percent;
    observation.active_mask = local.active_mask;
    observation.peak_count = local.peak_count;
    observation.threshold_gray = local.threshold_gray;
    stats_version = vision_line_stats_version + 1u;
    if (0u == (stats_version & 1u))
    {
        stats_version++;
    }
    vision_line_stats_version = stats_version;
    __dsync();
    if (0u != local.active_mask)
    {
        vision_line_last_white_frame_seq = frame_seq;
        vision_line_last_white_epoch = capture_epoch;
    }
    __dsync();
    vision_line_stats_epoch = capture_epoch;
    vision_line_stats_seq = frame_seq;
    __dsync();
    vision_line_stats_version = stats_version + 1u;
    __dsync();
    phototube_publish_observation(&observation);
    phototube_publish_hits(&local, candidates, candidate_count, weights,
                           capture_epoch);
    phototube_publish_snapshot(&local);
}

static void phototube_set_channel(uint8 channel)
{
    gpio_set_level(PHOTOTUBE_ARRAY_S0_PIN, (uint8)(channel & 1u));
    gpio_set_level(PHOTOTUBE_ARRAY_S1_PIN, (uint8)((channel >> 1u) & 1u));
    gpio_set_level(PHOTOTUBE_ARRAY_S2_PIN, (uint8)((channel >> 2u) & 1u));
    gpio_set_level(PHOTOTUBE_ARRAY_S3_PIN, (uint8)((channel >> 3u) & 1u));
}

static uint8 phototube_capture(uint16 values[PHOTOTUBE_ARRAY_COUNT])
{
    uint8 channel;
    for (channel = 0u; channel < PHOTOTUBE_ARRAY_COUNT; channel++)
    {
        uint8 sensor_index = phototube_sensor_index(channel);
        uint16 sample;
        phototube_set_channel(channel);
        system_delay_us(PHOTOTUBE_ARRAY_MUX_SETTLE_US);
        if (!adc_try_convert(PHOTOTUBE_ARRAY_ADC_CHANNEL, &sample) ||
            !adc_mean_filter_try_convert(PHOTOTUBE_ARRAY_ADC_CHANNEL,
                                          PHOTOTUBE_ARRAY_ADC_SAMPLE_COUNT,
                                          &sample))
        {
            return 0u;
        }
        values[sensor_index] = sample;
    }
    return 1u;
}

uint8 phototube_array_get_snapshot(PhototubeArraySnapshot *snapshot)
{
    uint8 retry;
    if (NULL == snapshot) return 0u;
    for (retry = 0u; retry < 3u; retry++)
    {
        uint32 before = phototube_snapshot_version;
        uint32 after;
        if ((0u == before) || (0u != (before & 1u))) continue;
        __dsync();
        *snapshot = phototube_snapshot;
        __dsync();
        after = phototube_snapshot_version;
        if ((before == after) && (0u == (after & 1u))) return 1u;
    }
    return 0u;
}

void phototube_array_init(void)
{
    gpio_init(PHOTOTUBE_ARRAY_S0_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(PHOTOTUBE_ARRAY_S1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(PHOTOTUBE_ARRAY_S2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(PHOTOTUBE_ARRAY_S3_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    adc_init(PHOTOTUBE_ARRAY_ADC_CHANNEL, ADC_12BIT);
    memset((void *)&phototube_snapshot, 0, sizeof(PhototubeArraySnapshot));
    memset(&phototube_track, 0, sizeof(phototube_track));
    phototube_snapshot_version = 0u;
    phototube_frame_seq = 0u;
    phototube_last_scan_tick = system_getval() -
                               PHOTOTUBE_ARRAY_SCAN_PERIOD_TICKS;
    phototube_timing_epoch_seq = imu_telemetry_epoch_seq;
    phototube_scan_started = 1u;
    vision_capture_drop_count = 0u;
    vision_capture_skipped_count = 0u;
    vision_capture_adc_failure_count = 0u;
    time_cost = 0u;
    phototube_scan_exec_max_us = 0u;
    phototube_scan_overrun_count = 0u;
    vision_capture_seq = 0u;
    vision_processed_seq = 0u;
    vision_pose_publish_seq = 0u;
    vision_pose_publish_slot =
        (uint8)(VISION_OBSERVATION_QUEUE_CAPACITY - 1u);
    vision_scan_hit_publish_seq = 0u;
    vision_scan_hit_publish_slot =
        (uint8)(VISION_SCAN_HIT_QUEUE_CAPACITY - 1u);
    vision_line_stats_version = 1u;
    __dsync();
    vision_line_last_white_frame_seq = 0u;
    vision_line_last_white_epoch = 0u;
    vision_line_stats_seq = 0u;
    vision_line_stats_epoch = vision_scan_run_epoch;
    __dsync();
    vision_line_stats_version = 2u;
    vision_scan_run_epoch_ack = vision_scan_run_epoch;
}

void phototube_array_task(void)
{
    uint32 now_tick;
    uint32 elapsed_ticks;
    uint32 periods;
    uint32 capture_start_ms;
    uint32 capture_epoch;
    float capture_yaw_deg;
    uint16 values[PHOTOTUBE_ARRAY_COUNT];
    uint32 process_start_tick;
    if (!phototube_scan_started) return;
    if (phototube_timing_epoch_seq != imu_telemetry_epoch_seq)
    {
        phototube_timing_epoch_seq = imu_telemetry_epoch_seq;
        time_cost = 0u;
        phototube_scan_exec_max_us = 0u;
        phototube_scan_overrun_count = 0u;
        vision_capture_adc_failure_count = 0u;
    }
    /* Poll CPU1's local STM while idle.  The old loop repeatedly read the
     * CPU0-owned millisecond counter across the inter-core bus.  Read that
     * common clock only once when a real frame is about to be captured. */
    now_tick = system_getval();
    elapsed_ticks = now_tick - phototube_last_scan_tick;
    if (elapsed_ticks < PHOTOTUBE_ARRAY_SCAN_PERIOD_TICKS) return;
    periods = elapsed_ticks / PHOTOTUBE_ARRAY_SCAN_PERIOD_TICKS;
    phototube_last_scan_tick += periods *
                                PHOTOTUBE_ARRAY_SCAN_PERIOD_TICKS;
    if (periods > 1u)
    {
        vision_capture_drop_count++;
        vision_capture_skipped_count += periods - 1u;
    }
    phototube_frame_seq += periods;
    /* Latch the CPU0 run boundary before the first ADC channel.  The ack is
     * published immediately; when CPU0 later observes the distinct close
     * epoch, every frame from the active epoch has completed publication. */
    capture_epoch = vision_scan_run_epoch;
    vision_scan_run_epoch_ack = capture_epoch;
    __dsync();
    capture_start_ms = vision_clock_ms;
    capture_yaw_deg = gyro_cpu1_get_pure_yaw_deg();
    process_start_tick = system_getval();
    if (!phototube_capture(values))
    {
        /* Do not turn an ADC timeout into an all-black/all-white frame.  The
         * frame sequence still advances, so the missing source frame is
         * visible through vision_capture_drop_count and the next packet's
         * source_frame_gap. */
        if (vision_capture_drop_count < 0xFFFFFFFFu)
        {
            vision_capture_drop_count++;
        }
        if (vision_capture_adc_failure_count < 0xFFFFFFFFu)
        {
            vision_capture_adc_failure_count++;
        }
        vision_capture_ms = capture_start_ms;
        vision_capture_seq = phototube_frame_seq;
        return;
    }
    vision_capture_ms = capture_start_ms;
    vision_capture_seq = phototube_frame_seq;
    phototube_process(values, phototube_frame_seq, capture_start_ms,
                      capture_yaw_deg, capture_epoch);
    vision_processed_seq = phototube_frame_seq;
    time_cost = (system_getval() - process_start_tick) /
                (PHOTOTUBE_ARRAY_TIMER_TICKS_PER_SECOND / 1000000u);
    if (time_cost > phototube_scan_exec_max_us)
    {
        phototube_scan_exec_max_us = time_cost;
    }
    if ((time_cost > (1000000u / PHOTOTUBE_ARRAY_SCAN_HZ)) &&
        (phototube_scan_overrun_count < 0xFFFFFFFFu))
    {
        phototube_scan_overrun_count++;
    }
    phototube_menu_frame_seq++;
}
