/**
 * @file calibration.c
 * @brief 相机与姿态标定流程、样本管理和参数保存。
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

#include "calibration.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "camera_pipeline.h"
#include "control.h"
#include "globals.h"
#include "gyroscope.h"
#include "motor.h"
#include "navigation.h"
#include "prestart.h"

#pragma section all "cpu0_dsram"

enum
{
    CAMERA_SAMPLE_CAPACITY = 384,
    ENCODER_EARLY_MANUAL_RECORD_COUNT = 1,
    ENCODER_ROLLBACK_COMPAT_RECORD_COUNT = 2,
    ENCODER_LEGACY_RECORD_COUNT = 6,
    CALIBRATION_FLASH_SECTION = 0,
    CALIBRATION_FLASH_PAGE_A = 2,
    CALIBRATION_FLASH_PAGE_B = 3,
    CALIBRATION_RECORD_MAGIC = 0x43414C32u,
    CALIBRATION_RECORD_VERSION = 1u
};

typedef enum
{
    CALIBRATION_RECORD_CAMERA_VALID = 1u << 0,
    CALIBRATION_RECORD_ENCODER_VALID = 1u << 1,
    CALIBRATION_RECORD_CAMERA_FUSION_ENABLED = 1u << 2
} CalibrationRecordFlag;

/* 页 0 保持原来的 22 项完赛参数，标定数据独立放在页 2/3。 */
typedef struct
{
    uint32 magic;
    uint32 version;
    uint32 generation;
    uint32 flags;

    float camera_reference_position_px;
    float camera_reference_slope_px_per_row;
    float camera_intercept;
    float camera_position_gain;
    float camera_slope_gain;
    float camera_bias_deg;
    float camera_rmse_deg;
    float camera_p95_deg;
    float camera_feature_independence;
    float camera_valid_rate_percent;
    uint32 camera_training_samples;
    uint32 camera_validation_samples;

    float encoder_yaw_deg_per_diff_pulse;
    float encoder_scale_ratio;
    float encoder_relative_mad;
    float encoder_repeat_difference;
    float encoder_maximum_relative_error;
    uint32 encoder_trial_count;

    uint32 checksum;
} CalibrationRecord;

enum
{
    CALIBRATION_RECORD_WORDS = sizeof(CalibrationRecord) / sizeof(uint32)
};

typedef struct
{
    uint16 training_min_samples;
    uint16 validation_min_samples;
    uint32 sample_period_ms;
    float minimum_valid_rate_percent;
    float minimum_yaw_each_side_deg;
    float maximum_sample_yaw_deg;
    float minimum_feature_independence;
    float maximum_bias_deg;
    float maximum_rmse_deg;
    float maximum_p95_deg;
} CameraAcceptanceConfig;

typedef struct
{
    float minimum_scale_ratio;
    float maximum_scale_ratio;
    float maximum_relative_mad;
    float maximum_repeat_difference;
    float maximum_trial_relative_error;
} EncoderRecordAcceptanceConfig;

static const CameraAcceptanceConfig camera_acceptance = {
    150u,  /* training_min_samples */
    80u,   /* validation_min_samples */
    50u,   /* sample_period_ms: 20 Hz, 384 points cover about 19 s */
    70.0f, /* minimum_valid_rate_percent */
    8.0f,  /* minimum_yaw_each_side_deg */
    60.0f, /* maximum_sample_yaw_deg */
    0.05f, /* minimum_feature_independence */
    1.5f,  /* maximum_bias_deg */
    2.0f,  /* maximum_rmse_deg */
    4.0f   /* maximum_p95_deg */
};

/* Flash 完整性门兼容旧的两圈自动记录和更早的六次正反记录。 */
static const EncoderRecordAcceptanceConfig encoder_record_acceptance = {
    0.50f, /* minimum_scale_ratio */
    1.50f, /* maximum_scale_ratio */
    0.05f, /* maximum_relative_mad */
    0.05f, /* maximum_repeat_difference */
    0.10f  /* maximum_trial_relative_error */
};

static CalibrationRecord active_record;
static uint8 active_record_page = 0xFFu;
static CameraCalibrationModel active_camera_model;

static CameraCalibrationStatus camera_status;
static CameraCalibrationModel pending_camera_model;
static CameraCalibrationSample camera_samples[CAMERA_SAMPLE_CAPACITY];
static uint16 camera_sample_count = 0u;
static uint16 camera_sample_write_index = 0u;
static uint16 camera_observed_frames = 0u;
static uint16 camera_valid_observed_frames = 0u;
static uint16 camera_training_sample_count = 0u;
static uint32 camera_last_publish_seq = 0u;
static uint32 camera_last_sample_capture_ms = 0u;
static uint32 camera_phase_start_ms = 0u;
static uint32 camera_phase_start_frame_seq = 0u;
static float camera_gyro_zero_deg = 0.0f;
static uint8 camera_page_active = 0u;

/* 编码器 90° 标定已删除：航向刻度见 control.h 宏，记录内 encoder 字段
 * 仅保留布局兼容（完整性校验仍验证旧数据），不再有任何写入/消费路径。 */

static uint8 calibration_float_valid(float value)
{
    return (uint8)((value == value) && (value > -1000000.0f) &&
                   (value < 1000000.0f));
}

static void calibration_saturating_add_u16(uint16 *value, uint32 increment)
{
    uint32 available = 0xFFFFu - (uint32)(*value);

    *value = (increment >= available)
                 ? 0xFFFFu
                 : (uint16)((uint32)(*value) + increment);
}

static uint32 calibration_record_checksum(const CalibrationRecord *record)
{
    const uint8 *bytes = (const uint8 *)record;
    uint32 checksum = 2166136261u;
    uint32 index;

    for (index = 0u; index < (uint32)offsetof(CalibrationRecord, checksum); index++)
    {
        checksum ^= bytes[index];
        checksum *= 16777619u;
    }
    return checksum;
}

static uint8 calibration_record_valid(const CalibrationRecord *record)
{
    uint32 known_flags = CALIBRATION_RECORD_CAMERA_VALID |
                         CALIBRATION_RECORD_ENCODER_VALID |
                         CALIBRATION_RECORD_CAMERA_FUSION_ENABLED;

    if ((NULL == record) ||
        (CALIBRATION_RECORD_MAGIC != record->magic) ||
        (CALIBRATION_RECORD_VERSION != record->version) ||
        (0u != (record->flags & ~known_flags)) ||
        (record->checksum != calibration_record_checksum(record)))
    {
        return 0u;
    }

    if (0u != (record->flags & CALIBRATION_RECORD_CAMERA_FUSION_ENABLED))
    {
        if (0u == (record->flags & CALIBRATION_RECORD_CAMERA_VALID))
        {
            return 0u;
        }
    }

    if (0u != (record->flags & CALIBRATION_RECORD_CAMERA_VALID))
    {
        if (!calibration_float_valid(record->camera_reference_position_px) ||
            !calibration_float_valid(record->camera_reference_slope_px_per_row) ||
            !calibration_float_valid(record->camera_intercept) ||
            !calibration_float_valid(record->camera_position_gain) ||
            !calibration_float_valid(record->camera_slope_gain) ||
            !calibration_float_valid(record->camera_bias_deg) ||
            !calibration_float_valid(record->camera_rmse_deg) ||
            !calibration_float_valid(record->camera_p95_deg) ||
            !calibration_float_valid(record->camera_feature_independence) ||
            !calibration_float_valid(record->camera_valid_rate_percent) ||
            (fabsf(record->camera_bias_deg) > camera_acceptance.maximum_bias_deg) ||
            (record->camera_rmse_deg < 0.0f) ||
            (record->camera_rmse_deg > camera_acceptance.maximum_rmse_deg) ||
            (record->camera_p95_deg < 0.0f) ||
            (record->camera_p95_deg > camera_acceptance.maximum_p95_deg) ||
            (record->camera_feature_independence <
             camera_acceptance.minimum_feature_independence) ||
            (record->camera_feature_independence > 1.0001f) ||
            (record->camera_valid_rate_percent <
             camera_acceptance.minimum_valid_rate_percent) ||
            (record->camera_valid_rate_percent > 100.0f) ||
            (record->camera_training_samples < camera_acceptance.training_min_samples) ||
            (record->camera_training_samples > CAMERA_SAMPLE_CAPACITY) ||
            (record->camera_validation_samples < camera_acceptance.validation_min_samples) ||
            (record->camera_validation_samples > CAMERA_SAMPLE_CAPACITY))
        {
            return 0u;
        }
    }

    if (0u != (record->flags & CALIBRATION_RECORD_ENCODER_VALID))
    {
        if (!calibration_float_valid(record->encoder_yaw_deg_per_diff_pulse) ||
            !calibration_float_valid(record->encoder_scale_ratio) ||
            !calibration_float_valid(record->encoder_relative_mad) ||
            !calibration_float_valid(record->encoder_repeat_difference) ||
            !calibration_float_valid(record->encoder_maximum_relative_error) ||
            (record->encoder_yaw_deg_per_diff_pulse <= 0.0f) ||
            (record->encoder_yaw_deg_per_diff_pulse >= 1.0f) ||
            (record->encoder_scale_ratio < encoder_record_acceptance.minimum_scale_ratio) ||
            (record->encoder_scale_ratio > encoder_record_acceptance.maximum_scale_ratio) ||
            (record->encoder_relative_mad < 0.0f) ||
            (record->encoder_relative_mad >
             encoder_record_acceptance.maximum_relative_mad) ||
            (record->encoder_repeat_difference < 0.0f) ||
            (record->encoder_repeat_difference >
             encoder_record_acceptance.maximum_repeat_difference) ||
            (record->encoder_maximum_relative_error < 0.0f) ||
            (record->encoder_maximum_relative_error >
             encoder_record_acceptance.maximum_trial_relative_error) ||
            ((record->encoder_trial_count != ENCODER_EARLY_MANUAL_RECORD_COUNT) &&
             (record->encoder_trial_count != ENCODER_ROLLBACK_COMPAT_RECORD_COUNT) &&
             (record->encoder_trial_count != ENCODER_LEGACY_RECORD_COUNT)))
        {
            return 0u;
        }
    }
    return 1u;
}

static uint8 calibration_read_record(uint8 page, CalibrationRecord *record)
{
    uint32 words[CALIBRATION_RECORD_WORDS];

    if ((NULL == record) || !flash_check(CALIBRATION_FLASH_SECTION, page))
    {
        return 0u;
    }
    flash_read_page(CALIBRATION_FLASH_SECTION, page, words,
                    (uint16)CALIBRATION_RECORD_WORDS);
    memcpy(record, words, sizeof(*record));
    return calibration_record_valid(record);
}

static void calibration_build_camera_model(const CalibrationRecord *record,
                                           CameraCalibrationModel *model)
{
    memset(model, 0, sizeof(*model));
    if (0u != (record->flags & CALIBRATION_RECORD_CAMERA_VALID))
    {
        model->status = CALIBRATION_FIT_OK;
        model->reference_position_px = record->camera_reference_position_px;
        model->reference_slope_px_per_row = record->camera_reference_slope_px_per_row;
        model->intercept = record->camera_intercept;
        model->position_gain = record->camera_position_gain;
        model->slope_gain = record->camera_slope_gain;
        model->bias_deg = record->camera_bias_deg;
        model->rmse_deg = record->camera_rmse_deg;
        model->p95_deg = record->camera_p95_deg;
        model->feature_independence = record->camera_feature_independence;
        model->used_count = (unsigned int)record->camera_training_samples;
    }
    else
    {
        model->status = CALIBRATION_FIT_BAD_ARGUMENT;
    }
}

static void calibration_publish_record(const CalibrationRecord *record,
                                       uint8 page)
{
    CameraCalibrationModel model;
    uint32 interrupt_state;

    calibration_build_camera_model(record, &model);
    interrupt_state = interrupt_global_disable();
    active_record = *record;
    active_record_page = page;
    active_camera_model = model;
    interrupt_global_enable(interrupt_state);
}

static uint8 calibration_generation_newer(uint32 first, uint32 second)
{
    return (uint8)((int32)(first - second) > 0);
}

static uint8 calibration_save_record(CalibrationRecord *record)
{
    CalibrationRecord readback;
    uint32 words[CALIBRATION_RECORD_WORDS];
    uint8 target_page = (CALIBRATION_FLASH_PAGE_A == active_record_page)
                            ? CALIBRATION_FLASH_PAGE_B
                            : CALIBRATION_FLASH_PAGE_A;

    record->magic = CALIBRATION_RECORD_MAGIC;
    record->version = CALIBRATION_RECORD_VERSION;
    record->generation = active_record.generation + 1u;
    if (0u == record->generation)
    {
        record->generation = 1u;
    }
    record->checksum = calibration_record_checksum(record);
    if (!calibration_record_valid(record))
    {
        return 0u;
    }
    memcpy(words, record, sizeof(*record));

    flash_erase_page(CALIBRATION_FLASH_SECTION, target_page);
    flash_write_page(CALIBRATION_FLASH_SECTION, target_page, words,
                     (uint16)CALIBRATION_RECORD_WORDS);
    if (!calibration_read_record(target_page, &readback) ||
        (0 != memcmp(record, &readback, sizeof(readback))))
    {
        return 0u;
    }

    calibration_publish_record(&readback, target_page);
    return 1u;
}

uint8 camera_heading_model_valid(void)
{
    return (uint8)(0u != (active_record.flags & CALIBRATION_RECORD_CAMERA_VALID));
}

float camera_heading_estimate_deg(float position_px, float slope_px_per_row)
{
    if (!camera_heading_model_valid())
    {
        return 0.0f;
    }
    return calibration_camera_heading_deg(&active_camera_model,
                                          position_px,
                                          slope_px_per_row);
}

uint8 camera_heading_fusion_enabled(void)
{
    return (uint8)(0u !=
                   (active_record.flags & CALIBRATION_RECORD_CAMERA_FUSION_ENABLED));
}

uint8 camera_heading_set_fusion_enabled(uint8 enabled)
{
    CalibrationRecord changed = active_record;

    if (enabled && !camera_heading_model_valid())
    {
        return 0u;
    }
    if (enabled)
    {
        changed.flags |= CALIBRATION_RECORD_CAMERA_FUSION_ENABLED;
    }
    else
    {
        changed.flags &= ~CALIBRATION_RECORD_CAMERA_FUSION_ENABLED;
    }
    if (changed.flags == active_record.flags)
    {
        return 1u;
    }
    return calibration_save_record(&changed);
}

void calibration_init(void)
{
    CalibrationRecord record_a;
    CalibrationRecord record_b;
    uint8 valid_a = calibration_read_record(CALIBRATION_FLASH_PAGE_A, &record_a);
    uint8 valid_b = calibration_read_record(CALIBRATION_FLASH_PAGE_B, &record_b);

    memset(&active_record, 0, sizeof(active_record));
    active_record.magic = CALIBRATION_RECORD_MAGIC;
    active_record.version = CALIBRATION_RECORD_VERSION;
    active_record_page = 0xFFu;

    if (valid_a && valid_b)
    {
        if (calibration_generation_newer(record_b.generation, record_a.generation))
        {
            active_record = record_b;
            active_record_page = CALIBRATION_FLASH_PAGE_B;
        }
        else
        {
            active_record = record_a;
            active_record_page = CALIBRATION_FLASH_PAGE_A;
        }
    }
    else if (valid_a)
    {
        active_record = record_a;
        active_record_page = CALIBRATION_FLASH_PAGE_A;
    }
    else if (valid_b)
    {
        active_record = record_b;
        active_record_page = CALIBRATION_FLASH_PAGE_B;
    }
    calibration_publish_record(&active_record, active_record_page);

    memset(&camera_status, 0, sizeof(camera_status));
    camera_status.phase = CAMERA_CAL_INACTIVE;
    camera_status.fit_status = CALIBRATION_FIT_BAD_ARGUMENT;
}

static void camera_reset_samples(void)
{
    camera_sample_count = 0u;
    camera_sample_write_index = 0u;
    camera_observed_frames = 0u;
    camera_valid_observed_frames = 0u;
    camera_last_sample_capture_ms = 0u;
    camera_status.valid_samples = 0u;
    camera_status.observed_frames = 0u;
    camera_status.valid_rate_percent = 0.0f;
    camera_status.minimum_gyro_deg = 0.0f;
    camera_status.maximum_gyro_deg = 0.0f;
    camera_status.live_position_px = 0.0f;
    camera_status.live_slope_px_per_row = 0.0f;
    camera_status.live_gyro_deg = 0.0f;
    camera_status.predicted_heading_deg = 0.0f;
    camera_status.live_fit_mse_px2 = 0.0f;
    camera_status.live_valid_rows = 0u;
    camera_status.live_candidate_rows = 0u;
    camera_status.live_frame_valid = 0u;
}

static void camera_update_sample_status(void)
{
    camera_status.valid_samples = camera_sample_count;
    camera_status.observed_frames = camera_observed_frames;
    camera_status.valid_rate_percent =
        (0u == camera_observed_frames)
            ? 0.0f
            : (100.0f * (float)camera_valid_observed_frames /
               (float)camera_observed_frames);
}

static void camera_fail(CameraCalibrationError error,
                        CalibrationFitStatus fit_status)
{
    camera_status.phase = CAMERA_CAL_FAILED;
    camera_status.error = error;
    camera_status.fit_status = fit_status;
}

static uint8 camera_sampling_prerequisites(void)
{
    if (!prestart_is_idle())
    {
        camera_fail(CAMERA_CAL_ERROR_NOT_IDLE,
                    CALIBRATION_FIT_BAD_ARGUMENT);
        return 0u;
    }
    if (!gyro_zero_calibration_done())
    {
        camera_fail(CAMERA_CAL_ERROR_IMU_NOT_READY,
                    CALIBRATION_FIT_BAD_ARGUMENT);
        return 0u;
    }
    return 1u;
}

static void camera_begin_sampling(CameraCalibrationPhase phase)
{
    if (!camera_sampling_prerequisites())
    {
        return;
    }

    control_disable();
    camera_reset_samples();
    camera_gyro_zero_deg = gyro_get_pure_yaw_deg();
    camera_phase_start_ms = vision_clock_ms;
    camera_phase_start_frame_seq = vision_capture_seq;
    camera_last_publish_seq = vision_pose_publish_seq;
    camera_status.phase = phase;
    camera_status.error = CAMERA_CAL_ERROR_NONE;
    camera_status.fit_status = CALIBRATION_FIT_OK;
    camera_status.minimum_samples =
        (CAMERA_CAL_TRAINING == phase)
            ? camera_acceptance.training_min_samples
            : camera_acceptance.validation_min_samples;
}

static uint8 camera_quality_valid(const VisionPoseObservation *observation)
{
    uint16 required_flags = VISION_OBS_FLAG_LATERAL_VALID |
                            VISION_OBS_FLAG_FIT_VALID |
                            VISION_OBS_FLAG_COVERAGE |
                            VISION_OBS_FLAG_EDGE_VALID;

    return (uint8)(((observation->flags & required_flags) == required_flags) &&
                   (0u == (observation->flags & VISION_OBS_FLAG_SIDE_OPEN)));
}

static uint8 camera_sample_values_valid(const VisionPoseObservation *observation,
                                        float relative_gyro_deg)
{
    return (uint8)(camera_quality_valid(observation) &&
                   calibration_float_valid(observation->line_position_px) &&
                   calibration_float_valid(observation->line_slope_px_per_row) &&
                   calibration_float_valid(observation->fit_mse_px2) &&
                   (observation->fit_mse_px2 >= 0.0f) &&
                   calibration_float_valid(relative_gyro_deg) &&
                   (fabsf(relative_gyro_deg) <=
                    camera_acceptance.maximum_sample_yaw_deg));
}

static void camera_recompute_sample_yaw_range(void)
{
    uint16 index;

    if (0u == camera_sample_count)
    {
        camera_status.minimum_gyro_deg = 0.0f;
        camera_status.maximum_gyro_deg = 0.0f;
        return;
    }
    camera_status.minimum_gyro_deg = camera_samples[0].gyro_deg;
    camera_status.maximum_gyro_deg = camera_samples[0].gyro_deg;
    for (index = 1u; index < camera_sample_count; index++)
    {
        if (camera_samples[index].gyro_deg < camera_status.minimum_gyro_deg)
        {
            camera_status.minimum_gyro_deg = camera_samples[index].gyro_deg;
        }
        if (camera_samples[index].gyro_deg > camera_status.maximum_gyro_deg)
        {
            camera_status.maximum_gyro_deg = camera_samples[index].gyro_deg;
        }
    }
}

static void camera_consume_observation(void)
{
    VisionPoseObservation observation;
    float relative_gyro_deg;
    uint32 previous_publish_seq = camera_last_publish_seq;
    uint32 published_frame_gap;

    if (!vision_observation_read_new(&camera_last_publish_seq, &observation))
    {
        return;
    }
    if ((int32)(observation.capture_ms - camera_phase_start_ms) < 0)
    {
        return;
    }
    if ((int32)(observation.frame_seq - camera_phase_start_frame_seq) <= 0)
    {
        return;
    }

    relative_gyro_deg = observation.capture_gyro_yaw_deg -
                        camera_gyro_zero_deg;
    camera_status.live_position_px = observation.line_position_px;
    camera_status.live_slope_px_per_row =
        observation.line_slope_px_per_row;
    camera_status.live_gyro_deg = relative_gyro_deg;
    camera_status.live_fit_mse_px2 = observation.fit_mse_px2;
    camera_status.live_valid_rows = observation.valid_rows;
    camera_status.live_candidate_rows = observation.candidate_rows;
    camera_status.live_frame_valid =
        camera_sample_values_valid(&observation, relative_gyro_deg);
    if (CALIBRATION_FIT_OK == pending_camera_model.status)
    {
        camera_status.predicted_heading_deg =
            calibration_camera_heading_deg(&pending_camera_model,
                                           observation.line_position_px,
                                           observation.line_slope_px_per_row);
    }

    published_frame_gap = observation.publish_seq - previous_publish_seq;
    calibration_saturating_add_u16(&camera_observed_frames,
                                   (0u == published_frame_gap)
                                       ? 1u
                                       : published_frame_gap);
    if (camera_status.live_frame_valid)
    {
        calibration_saturating_add_u16(&camera_valid_observed_frames, 1u);
        if ((0u == camera_sample_count) ||
            ((observation.capture_ms - camera_last_sample_capture_ms) >=
             camera_acceptance.sample_period_ms))
        {
            CameraCalibrationSample *sample =
                &camera_samples[camera_sample_write_index];

            sample->position_px = observation.line_position_px;
            sample->slope_px_per_row = observation.line_slope_px_per_row;
            sample->gyro_deg = relative_gyro_deg;
            camera_sample_write_index++;
            if (camera_sample_write_index >= CAMERA_SAMPLE_CAPACITY)
            {
                camera_sample_write_index = 0u;
            }
            if (camera_sample_count < CAMERA_SAMPLE_CAPACITY)
            {
                camera_sample_count++;
            }
            camera_last_sample_capture_ms = observation.capture_ms;
            camera_recompute_sample_yaw_range();
        }
    }
    camera_update_sample_status();
}

static uint8 camera_sample_set_acceptable(uint16 minimum_samples,
                                          uint8 require_yaw_coverage)
{
    camera_recompute_sample_yaw_range();
    camera_update_sample_status();
    if (camera_sample_count < minimum_samples)
    {
        camera_fail(CAMERA_CAL_ERROR_TOO_FEW_FRAMES,
                    CALIBRATION_FIT_TOO_FEW_SAMPLES);
        return 0u;
    }
    if (camera_status.valid_rate_percent <
        camera_acceptance.minimum_valid_rate_percent)
    {
        camera_fail(CAMERA_CAL_ERROR_VALID_RATE,
                    CALIBRATION_FIT_INVALID_SAMPLE);
        return 0u;
    }
    if (require_yaw_coverage &&
        ((camera_status.minimum_gyro_deg >
          -camera_acceptance.minimum_yaw_each_side_deg) ||
         (camera_status.maximum_gyro_deg <
          camera_acceptance.minimum_yaw_each_side_deg)))
    {
        camera_fail(CAMERA_CAL_ERROR_YAW_COVERAGE,
                    CALIBRATION_FIT_FEATURE_RANGE_TOO_SMALL);
        return 0u;
    }
    return 1u;
}

static void camera_finish_training(void)
{
    float reference_position = 0.0f;
    float reference_slope = 0.0f;
    uint16 index;

    if (!camera_sample_set_acceptable(
            camera_acceptance.training_min_samples, 1u))
    {
        return;
    }
    for (index = 0u; index < camera_sample_count; index++)
    {
        reference_position += camera_samples[index].position_px;
        reference_slope += camera_samples[index].slope_px_per_row;
    }
    reference_position /= (float)camera_sample_count;
    reference_slope /= (float)camera_sample_count;

    memset(&pending_camera_model, 0, sizeof(pending_camera_model));
    if (!calibration_fit_camera(camera_samples,
                                camera_sample_count,
                                reference_position,
                                reference_slope,
                                &pending_camera_model))
    {
        camera_fail(CAMERA_CAL_ERROR_FIT,
                    pending_camera_model.status);
        return;
    }
    if (pending_camera_model.feature_independence <
        camera_acceptance.minimum_feature_independence)
    {
        camera_fail(CAMERA_CAL_ERROR_CONDITIONING,
                    CALIBRATION_FIT_FEATURES_COUPLED);
        return;
    }

    camera_training_sample_count = camera_sample_count;
    camera_status.bias_deg = pending_camera_model.bias_deg;
    camera_status.rmse_deg = pending_camera_model.rmse_deg;
    camera_status.p95_deg = pending_camera_model.p95_deg;
    camera_status.feature_independence =
        pending_camera_model.feature_independence;
    camera_status.phase = CAMERA_CAL_WAIT_VALIDATION_ZERO;
    camera_status.error = CAMERA_CAL_ERROR_NONE;
    camera_status.minimum_samples = camera_acceptance.validation_min_samples;
}

static void camera_finish_validation(void)
{
    CameraCalibrationMetrics metrics;

    if (!camera_sample_set_acceptable(
            camera_acceptance.validation_min_samples, 1u))
    {
        return;
    }
    if (!calibration_evaluate_camera(camera_samples,
                                     camera_sample_count,
                                     &pending_camera_model,
                                     &metrics))
    {
        camera_fail(CAMERA_CAL_ERROR_FIT,
                    CALIBRATION_FIT_INVALID_SAMPLE);
        return;
    }
    if ((metrics.valid_count != camera_sample_count) ||
        (metrics.valid_count < camera_acceptance.validation_min_samples))
    {
        camera_fail(CAMERA_CAL_ERROR_FIT,
                    CALIBRATION_FIT_INVALID_SAMPLE);
        return;
    }

    camera_status.bias_deg = metrics.bias_deg;
    camera_status.rmse_deg = metrics.rmse_deg;
    camera_status.p95_deg = metrics.p95_deg;
    if (fabsf(metrics.bias_deg) > camera_acceptance.maximum_bias_deg)
    {
        camera_fail(CAMERA_CAL_ERROR_VALIDATION_BIAS,
                    CALIBRATION_FIT_RESULTS_UNSTABLE);
        return;
    }
    if (metrics.rmse_deg > camera_acceptance.maximum_rmse_deg)
    {
        camera_fail(CAMERA_CAL_ERROR_VALIDATION_RMSE,
                    CALIBRATION_FIT_RESULTS_UNSTABLE);
        return;
    }
    if (metrics.p95_deg > camera_acceptance.maximum_p95_deg)
    {
        camera_fail(CAMERA_CAL_ERROR_VALIDATION_P95,
                    CALIBRATION_FIT_RESULTS_UNSTABLE);
        return;
    }

    camera_status.phase = CAMERA_CAL_PASS_NOT_SAVED;
    camera_status.error = CAMERA_CAL_ERROR_NONE;
    camera_status.fit_status = CALIBRATION_FIT_OK;
}

static void camera_save_pending_model(void)
{
    CalibrationRecord changed = active_record;

    changed.flags |= CALIBRATION_RECORD_CAMERA_VALID;
    /* 新模型第一次保存保持影子模式，必须由页面再次明确开启融合。 */
    changed.flags &= ~CALIBRATION_RECORD_CAMERA_FUSION_ENABLED;
    changed.camera_reference_position_px =
        pending_camera_model.reference_position_px;
    changed.camera_reference_slope_px_per_row =
        pending_camera_model.reference_slope_px_per_row;
    changed.camera_intercept = pending_camera_model.intercept;
    changed.camera_position_gain = pending_camera_model.position_gain;
    changed.camera_slope_gain = pending_camera_model.slope_gain;
    changed.camera_bias_deg = camera_status.bias_deg;
    changed.camera_rmse_deg = camera_status.rmse_deg;
    changed.camera_p95_deg = camera_status.p95_deg;
    changed.camera_feature_independence =
        pending_camera_model.feature_independence;
    changed.camera_valid_rate_percent = camera_status.valid_rate_percent;
    changed.camera_training_samples = camera_training_sample_count;
    changed.camera_validation_samples = camera_sample_count;

    if (!calibration_save_record(&changed))
    {
        camera_fail(CAMERA_CAL_ERROR_FLASH,
                    CALIBRATION_FIT_BAD_ARGUMENT);
        return;
    }
    camera_status.phase = CAMERA_CAL_SAVED;
    camera_status.model_valid = 1u;
    camera_status.fusion_enabled = 0u;
}

void camera_calibration_enter(void)
{
    if (!prestart_is_idle())
    {
        prestart_stop();
    }
    control_disable();
    nav_stop();
    camera_page_active = 1u;
    camera_status.phase = CAMERA_CAL_WAIT_TRAIN_ZERO;
    camera_status.error = CAMERA_CAL_ERROR_NONE;
    camera_status.fit_status = CALIBRATION_FIT_BAD_ARGUMENT;
    camera_status.minimum_samples = camera_acceptance.training_min_samples;
    camera_status.model_valid = camera_heading_model_valid();
    camera_status.fusion_enabled = camera_heading_fusion_enabled();
    memset(&pending_camera_model, 0, sizeof(pending_camera_model));
    pending_camera_model.status = CALIBRATION_FIT_BAD_ARGUMENT;
    camera_reset_samples();
}

void camera_calibration_leave(void)
{
    camera_page_active = 0u;
    control_disable();
    nav_stop();
    camera_status.phase = CAMERA_CAL_INACTIVE;
}

void camera_calibration_abort(void)
{
    control_disable();
    nav_stop();
    camera_status.phase = CAMERA_CAL_ABORTED;
    camera_status.error = CAMERA_CAL_ERROR_NONE;
}

void camera_calibration_confirm(void)
{
    switch (camera_status.phase)
    {
    case CAMERA_CAL_WAIT_TRAIN_ZERO:
        camera_begin_sampling(CAMERA_CAL_TRAINING);
        break;
    case CAMERA_CAL_TRAINING:
        camera_finish_training();
        break;
    case CAMERA_CAL_WAIT_VALIDATION_ZERO:
        camera_begin_sampling(CAMERA_CAL_VALIDATING);
        break;
    case CAMERA_CAL_VALIDATING:
        camera_finish_validation();
        break;
    case CAMERA_CAL_PASS_NOT_SAVED:
        camera_save_pending_model();
        break;
    case CAMERA_CAL_FAILED:
    case CAMERA_CAL_ABORTED:
    case CAMERA_CAL_SAVED:
        camera_status.phase = CAMERA_CAL_WAIT_TRAIN_ZERO;
        camera_status.error = CAMERA_CAL_ERROR_NONE;
        camera_status.fit_status = CALIBRATION_FIT_BAD_ARGUMENT;
        camera_status.minimum_samples = camera_acceptance.training_min_samples;
        camera_reset_samples();
        break;
    default:
        break;
    }
}

void camera_calibration_get_status(CameraCalibrationStatus *status)
{
    if (NULL == status)
    {
        return;
    }
    camera_status.model_valid = camera_heading_model_valid();
    camera_status.fusion_enabled = camera_heading_fusion_enabled();
    *status = camera_status;
}

void camera_calibration_get_model_details(CameraCalibrationModelDetails *details)
{
    uint8 pending_selected;

    if (NULL == details)
    {
        return;
    }
    memset(details, 0, sizeof(*details));

    pending_selected = (uint8)(
        (CALIBRATION_FIT_OK == pending_camera_model.status) &&
        ((CAMERA_CAL_WAIT_VALIDATION_ZERO == camera_status.phase) ||
         (CAMERA_CAL_VALIDATING == camera_status.phase) ||
         (CAMERA_CAL_PASS_NOT_SAVED == camera_status.phase)));

    if (pending_selected)
    {
        details->available = 1u;
        details->saved = 0u;
        details->reference_position_px =
            pending_camera_model.reference_position_px;
        details->reference_slope_px_per_row =
            pending_camera_model.reference_slope_px_per_row;
        details->intercept = pending_camera_model.intercept;
        details->position_gain = pending_camera_model.position_gain;
        details->slope_gain = pending_camera_model.slope_gain;
        details->bias_deg = camera_status.bias_deg;
        details->rmse_deg = camera_status.rmse_deg;
        details->p95_deg = camera_status.p95_deg;
        details->feature_independence =
            pending_camera_model.feature_independence;
        details->valid_rate_percent = camera_status.valid_rate_percent;
        details->training_samples = camera_training_sample_count;
        details->validation_samples =
            (CAMERA_CAL_WAIT_VALIDATION_ZERO == camera_status.phase)
                ? 0u
                : camera_sample_count;
        return;
    }

    if (0u != (active_record.flags & CALIBRATION_RECORD_CAMERA_VALID))
    {
        details->available = 1u;
        details->saved = 1u;
        details->reference_position_px =
            active_record.camera_reference_position_px;
        details->reference_slope_px_per_row =
            active_record.camera_reference_slope_px_per_row;
        details->intercept = active_record.camera_intercept;
        details->position_gain = active_record.camera_position_gain;
        details->slope_gain = active_record.camera_slope_gain;
        details->bias_deg = active_record.camera_bias_deg;
        details->rmse_deg = active_record.camera_rmse_deg;
        details->p95_deg = active_record.camera_p95_deg;
        details->feature_independence =
            active_record.camera_feature_independence;
        details->valid_rate_percent =
            active_record.camera_valid_rate_percent;
        details->training_samples = active_record.camera_training_samples;
        details->validation_samples = active_record.camera_validation_samples;
    }
}

void calibration_task_5ms(void)
{
    if (camera_page_active &&
        ((CAMERA_CAL_TRAINING == camera_status.phase) ||
         (CAMERA_CAL_VALIDATING == camera_status.phase)))
    {
        camera_consume_observation();
    }
}

const char *camera_calibration_phase_text(CameraCalibrationPhase phase)
{
    switch (phase)
    {
    case CAMERA_CAL_WAIT_TRAIN_ZERO:
        return "PUT STRAIGHT / K3";
    case CAMERA_CAL_TRAINING:
        return "TRAIN: PUSH S PATH";
    case CAMERA_CAL_WAIT_VALIDATION_ZERO:
        return "STRAIGHT AGAIN / K3";
    case CAMERA_CAL_VALIDATING:
        return "VERIFY: PUSH S PATH";
    case CAMERA_CAL_PASS_NOT_SAVED:
        return "PASS / K3 SAVE";
    case CAMERA_CAL_SAVED:
        return "SAVED / R FUSION";
    case CAMERA_CAL_FAILED:
        return "FAILED / K3 RETRY";
    case CAMERA_CAL_ABORTED:
        return "ABORTED / K3 RETRY";
    default:
        return "CAMERA CAL OFF";
    }
}

static const char *calibration_fit_status_text(CalibrationFitStatus status)
{
    switch (status)
    {
    case CALIBRATION_FIT_TOO_FEW_SAMPLES:
        return "TOO FEW";
    case CALIBRATION_FIT_INVALID_SAMPLE:
        return "BAD SAMPLE";
    case CALIBRATION_FIT_FEATURE_RANGE_TOO_SMALL:
        return "RANGE SMALL";
    case CALIBRATION_FIT_FEATURES_COUPLED:
        return "P/S COUPLED";
    case CALIBRATION_FIT_TOO_MANY_OUTLIERS:
        return "OUTLIERS";
    case CALIBRATION_FIT_DIRECTION_MISMATCH:
        return "DIR WRONG";
    case CALIBRATION_FIT_DIRECTION_MISSING:
        return "DIR MISSING";
    case CALIBRATION_FIT_RESULTS_UNSTABLE:
        return "UNSTABLE";
    case CALIBRATION_FIT_BAD_ARGUMENT:
        return "NO RESULT";
    default:
        return "OK";
    }
}

const char *camera_calibration_error_text(CameraCalibrationError error,
                                          CalibrationFitStatus fit_status)
{
    switch (error)
    {
    case CAMERA_CAL_ERROR_NOT_IDLE:
        return "WAIT MOTOR IDLE";
    case CAMERA_CAL_ERROR_IMU_NOT_READY:
        return "WAIT IMU ZERO";
    case CAMERA_CAL_ERROR_TOO_FEW_FRAMES:
        return "NEED MORE FRAMES";
    case CAMERA_CAL_ERROR_VALID_RATE:
        return "LINE VALID <70%";
    case CAMERA_CAL_ERROR_YAW_COVERAGE:
        return "BOTH SIDES <8DEG";
    case CAMERA_CAL_ERROR_FIT:
        return calibration_fit_status_text(fit_status);
    case CAMERA_CAL_ERROR_CONDITIONING:
        return "P/S COUPLED";
    case CAMERA_CAL_ERROR_VALIDATION_BIAS:
        return "BIAS >1.5DEG";
    case CAMERA_CAL_ERROR_VALIDATION_RMSE:
        return "RMSE >2DEG";
    case CAMERA_CAL_ERROR_VALIDATION_P95:
        return "P95 >4DEG";
    case CAMERA_CAL_ERROR_FLASH:
        return "FLASH VERIFY";
    default:
        return "READY";
    }
}

#pragma section all restore
