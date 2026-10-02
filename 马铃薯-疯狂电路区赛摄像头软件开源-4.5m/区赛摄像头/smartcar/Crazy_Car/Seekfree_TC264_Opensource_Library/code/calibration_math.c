/**
 * @file calibration_math.c
 * @brief 相机航向标定使用的几何计算和鲁棒拟合。
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

#include "calibration_math.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

enum
{
    CALIBRATION_MAX_SAMPLES = 384,
    CAMERA_MIN_FIT_SAMPLES = 8,
    CAMERA_ROBUST_ITERATIONS = 3
};

static const float degrees_to_radians = 0.01745329251994329577f;
static const float radians_to_degrees = 57.295779513082320876f;

/* 相机拟合：先检查特征覆盖，再用 MAD 剔除少量异常帧。 */
static const float maximum_camera_angle_deg = 60.0f;
static const float minimum_position_variance = 0.0001f;
static const float minimum_slope_variance = 0.00000001f;
static const float minimum_feature_independence = 0.02f;
static const float minimum_outlier_limit_deg = 0.50f;
static const float mad_to_sigma = 1.4826f;
static const float outlier_sigma_limit = 4.5f;

/* 编码器标定已删除(2026-07-12)：航向刻度改为 control.h 的
 * ENCODER_YAW_DEG_PER_DIFF_PULSE 宏，本文件只保留相机模型拟合。 */

/* 标定只在菜单页运行，复用静态缓冲可避免占用较大的任务栈。 */
static float sort_workspace[CALIBRATION_MAX_SAMPLES];

typedef struct
{
    float intercept;
    float position_gain;
    float slope_gain;
    float feature_independence;
    unsigned int used_count;
} CameraCoefficients;

static int value_is_finite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static void clear_camera_model(CameraCalibrationModel *model)
{
    model->status = CALIBRATION_FIT_BAD_ARGUMENT;
    model->reference_position_px = 0.0f;
    model->reference_slope_px_per_row = 0.0f;
    model->intercept = 0.0f;
    model->position_gain = 0.0f;
    model->slope_gain = 0.0f;
    model->bias_deg = 0.0f;
    model->rmse_deg = 0.0f;
    model->p95_deg = 0.0f;
    model->feature_independence = 0.0f;
    model->used_count = 0u;
}

static void sort_values(float *values, unsigned int count)
{
    unsigned int i;

    for (i = 1u; i < count; i++)
    {
        float value = values[i];
        unsigned int insertion_index = i;

        while ((insertion_index > 0u) &&
               (values[insertion_index - 1u] > value))
        {
            values[insertion_index] = values[insertion_index - 1u];
            insertion_index--;
        }
        values[insertion_index] = value;
    }
}

static float workspace_median(unsigned int count)
{
    unsigned int middle;

    sort_values(sort_workspace, count);
    middle = count / 2u;
    if ((count & 1u) != 0u)
    {
        return sort_workspace[middle];
    }
    return 0.5f * (sort_workspace[middle - 1u] + sort_workspace[middle]);
}

static int camera_sample_is_valid(const CameraCalibrationSample *sample)
{
    return value_is_finite(sample->position_px) &&
           value_is_finite(sample->slope_px_per_row) &&
           value_is_finite(sample->gyro_deg) &&
           (fabsf(sample->gyro_deg) <= maximum_camera_angle_deg);
}

static float camera_heading_from_coefficients(const CameraCoefficients *coefficients,
                                              float position_offset,
                                              float slope_offset)
{
    float tangent = coefficients->intercept +
                    coefficients->position_gain * position_offset +
                    coefficients->slope_gain * slope_offset;
    return atanf(tangent) * radians_to_degrees;
}

static float camera_residual_deg(const CameraCalibrationSample *sample,
                                 float reference_position_px,
                                 float reference_slope_px_per_row,
                                 const CameraCoefficients *coefficients)
{
    float predicted_deg = camera_heading_from_coefficients(
        coefficients,
        sample->position_px - reference_position_px,
        sample->slope_px_per_row - reference_slope_px_per_row);
    return sample->gyro_deg - predicted_deg;
}

static int sample_is_inlier(const CameraCalibrationSample *sample,
                            float reference_position_px,
                            float reference_slope_px_per_row,
                            const CameraCoefficients *previous,
                            float residual_center_deg,
                            float residual_limit_deg)
{
    float residual;

    if (!camera_sample_is_valid(sample))
    {
        return 0;
    }
    if (previous == NULL)
    {
        return 1;
    }

    residual = camera_residual_deg(sample,
                                   reference_position_px,
                                   reference_slope_px_per_row,
                                   previous);
    return fabsf(residual - residual_center_deg) <= residual_limit_deg;
}

static CalibrationFitStatus solve_camera_coefficients(
    const CameraCalibrationSample *samples,
    unsigned int sample_count,
    float reference_position_px,
    float reference_slope_px_per_row,
    const CameraCoefficients *previous,
    float residual_center_deg,
    float residual_limit_deg,
    CameraCoefficients *coefficients)
{
    float position_mean = 0.0f;
    float slope_mean = 0.0f;
    float tangent_mean = 0.0f;
    float position_sum_square = 0.0f;
    float slope_sum_square = 0.0f;
    float position_slope_sum = 0.0f;
    float position_tangent_sum = 0.0f;
    float slope_tangent_sum = 0.0f;
    float determinant;
    unsigned int used_count = 0u;
    unsigned int i;

    for (i = 0u; i < sample_count; i++)
    {
        if (sample_is_inlier(&samples[i],
                             reference_position_px,
                             reference_slope_px_per_row,
                             previous,
                             residual_center_deg,
                             residual_limit_deg))
        {
            position_mean += samples[i].position_px - reference_position_px;
            slope_mean += samples[i].slope_px_per_row - reference_slope_px_per_row;
            tangent_mean += tanf(samples[i].gyro_deg * degrees_to_radians);
            used_count++;
        }
    }

    if (used_count < CAMERA_MIN_FIT_SAMPLES)
    {
        return (previous == NULL) ? CALIBRATION_FIT_TOO_FEW_SAMPLES
                                  : CALIBRATION_FIT_TOO_MANY_OUTLIERS;
    }

    position_mean /= (float)used_count;
    slope_mean /= (float)used_count;
    tangent_mean /= (float)used_count;

    for (i = 0u; i < sample_count; i++)
    {
        if (sample_is_inlier(&samples[i],
                             reference_position_px,
                             reference_slope_px_per_row,
                             previous,
                             residual_center_deg,
                             residual_limit_deg))
        {
            float position = samples[i].position_px - reference_position_px -
                             position_mean;
            float slope = samples[i].slope_px_per_row -
                          reference_slope_px_per_row - slope_mean;
            float tangent = tanf(samples[i].gyro_deg * degrees_to_radians) -
                            tangent_mean;

            position_sum_square += position * position;
            slope_sum_square += slope * slope;
            position_slope_sum += position * slope;
            position_tangent_sum += position * tangent;
            slope_tangent_sum += slope * tangent;
        }
    }

    if ((position_sum_square < minimum_position_variance * (float)used_count) ||
        (slope_sum_square < minimum_slope_variance * (float)used_count))
    {
        return CALIBRATION_FIT_FEATURE_RANGE_TOO_SMALL;
    }

    coefficients->feature_independence =
        1.0f - (position_slope_sum * position_slope_sum) /
                   (position_sum_square * slope_sum_square);
    if (coefficients->feature_independence < minimum_feature_independence)
    {
        return CALIBRATION_FIT_FEATURES_COUPLED;
    }

    determinant = position_sum_square * slope_sum_square -
                  position_slope_sum * position_slope_sum;
    coefficients->position_gain =
        (position_tangent_sum * slope_sum_square -
         slope_tangent_sum * position_slope_sum) /
        determinant;
    coefficients->slope_gain =
        (slope_tangent_sum * position_sum_square -
         position_tangent_sum * position_slope_sum) /
        determinant;
    coefficients->intercept = tangent_mean -
                              coefficients->position_gain * position_mean -
                              coefficients->slope_gain * slope_mean;
    coefficients->used_count = used_count;

    if (!value_is_finite(coefficients->intercept) ||
        !value_is_finite(coefficients->position_gain) ||
        !value_is_finite(coefficients->slope_gain))
    {
        return CALIBRATION_FIT_FEATURES_COUPLED;
    }
    return CALIBRATION_FIT_OK;
}

static int find_camera_outlier_limit(const CameraCalibrationSample *samples,
                                     unsigned int sample_count,
                                     float reference_position_px,
                                     float reference_slope_px_per_row,
                                     const CameraCoefficients *coefficients,
                                     float *residual_center_deg,
                                     float *residual_limit_deg)
{
    float median;
    float median_absolute_deviation;
    unsigned int valid_count = 0u;
    unsigned int write_index = 0u;
    unsigned int i;

    for (i = 0u; i < sample_count; i++)
    {
        if (camera_sample_is_valid(&samples[i]))
        {
            sort_workspace[valid_count] = camera_residual_deg(
                &samples[i],
                reference_position_px,
                reference_slope_px_per_row,
                coefficients);
            valid_count++;
        }
    }
    if (valid_count < CAMERA_MIN_FIT_SAMPLES)
    {
        return 0;
    }

    median = workspace_median(valid_count);
    for (i = 0u; i < sample_count; i++)
    {
        if (camera_sample_is_valid(&samples[i]))
        {
            float residual = camera_residual_deg(&samples[i],
                                                 reference_position_px,
                                                 reference_slope_px_per_row,
                                                 coefficients);
            sort_workspace[write_index] = fabsf(residual - median);
            write_index++;
        }
    }
    median_absolute_deviation = workspace_median(valid_count);
    *residual_center_deg = median;
    *residual_limit_deg = outlier_sigma_limit * mad_to_sigma *
                          median_absolute_deviation;
    if (*residual_limit_deg < minimum_outlier_limit_deg)
    {
        *residual_limit_deg = minimum_outlier_limit_deg;
    }
    return 1;
}

static void calculate_camera_metrics(const CameraCalibrationSample *samples,
                                     unsigned int sample_count,
                                     float reference_position_px,
                                     float reference_slope_px_per_row,
                                     const CameraCoefficients *coefficients,
                                     const CameraCoefficients *inlier_reference,
                                     float residual_center_deg,
                                     float residual_limit_deg,
                                     CameraCalibrationMetrics *metrics)
{
    float residual_sum = 0.0f;
    float residual_square_sum = 0.0f;
    unsigned int used_count = 0u;
    unsigned int i;
    unsigned int p95_index;

    for (i = 0u; i < sample_count; i++)
    {
        if (sample_is_inlier(&samples[i],
                             reference_position_px,
                             reference_slope_px_per_row,
                             inlier_reference,
                             residual_center_deg,
                             residual_limit_deg))
        {
            float residual = camera_residual_deg(&samples[i],
                                                 reference_position_px,
                                                 reference_slope_px_per_row,
                                                 coefficients);
            residual_sum += residual;
            residual_square_sum += residual * residual;
            sort_workspace[used_count] = fabsf(residual);
            used_count++;
        }
    }

    metrics->valid_count = used_count;
    metrics->bias_deg = residual_sum / (float)used_count;
    metrics->rmse_deg = sqrtf(residual_square_sum / (float)used_count);
    sort_values(sort_workspace, used_count);
    p95_index = ((95u * used_count + 99u) / 100u) - 1u;
    metrics->p95_deg = sort_workspace[p95_index];
}

int calibration_fit_camera(const CameraCalibrationSample *samples,
                           unsigned int sample_count,
                           float reference_position_px,
                           float reference_slope_px_per_row,
                           CameraCalibrationModel *model)
{
    CameraCoefficients coefficients;
    CameraCoefficients previous;
    CameraCalibrationMetrics metrics;
    CalibrationFitStatus status;
    float residual_center_deg = 0.0f;
    float residual_limit_deg = 0.0f;
    unsigned int iteration;

    if (model == NULL)
    {
        return 0;
    }
    clear_camera_model(model);
    if ((samples == NULL) ||
        (sample_count > CALIBRATION_MAX_SAMPLES) ||
        !value_is_finite(reference_position_px) ||
        !value_is_finite(reference_slope_px_per_row))
    {
        return 0;
    }
    model->reference_position_px = reference_position_px;
    model->reference_slope_px_per_row = reference_slope_px_per_row;

    if (sample_count < CAMERA_MIN_FIT_SAMPLES)
    {
        model->status = CALIBRATION_FIT_TOO_FEW_SAMPLES;
        return 0;
    }

    status = solve_camera_coefficients(samples,
                                       sample_count,
                                       reference_position_px,
                                       reference_slope_px_per_row,
                                       NULL,
                                       0.0f,
                                       0.0f,
                                       &coefficients);
    if (status != CALIBRATION_FIT_OK)
    {
        model->status = status;
        return 0;
    }

    for (iteration = 0u; iteration < CAMERA_ROBUST_ITERATIONS; iteration++)
    {
        previous = coefficients;
        if (!find_camera_outlier_limit(samples,
                                       sample_count,
                                       reference_position_px,
                                       reference_slope_px_per_row,
                                       &previous,
                                       &residual_center_deg,
                                       &residual_limit_deg))
        {
            model->status = CALIBRATION_FIT_TOO_FEW_SAMPLES;
            return 0;
        }
        status = solve_camera_coefficients(samples,
                                           sample_count,
                                           reference_position_px,
                                           reference_slope_px_per_row,
                                           &previous,
                                           residual_center_deg,
                                           residual_limit_deg,
                                           &coefficients);
        if (status != CALIBRATION_FIT_OK)
        {
            model->status = status;
            return 0;
        }
    }

    previous = coefficients;
    if (!find_camera_outlier_limit(samples,
                                   sample_count,
                                   reference_position_px,
                                   reference_slope_px_per_row,
                                   &previous,
                                   &residual_center_deg,
                                   &residual_limit_deg))
    {
        model->status = CALIBRATION_FIT_TOO_FEW_SAMPLES;
        return 0;
    }
    status = solve_camera_coefficients(samples,
                                       sample_count,
                                       reference_position_px,
                                       reference_slope_px_per_row,
                                       &previous,
                                       residual_center_deg,
                                       residual_limit_deg,
                                       &coefficients);
    if (status != CALIBRATION_FIT_OK)
    {
        model->status = status;
        return 0;
    }

    calculate_camera_metrics(samples,
                             sample_count,
                             reference_position_px,
                             reference_slope_px_per_row,
                             &coefficients,
                             &previous,
                             residual_center_deg,
                             residual_limit_deg,
                             &metrics);

    model->status = CALIBRATION_FIT_OK;
    model->intercept = coefficients.intercept;
    model->position_gain = coefficients.position_gain;
    model->slope_gain = coefficients.slope_gain;
    model->bias_deg = metrics.bias_deg;
    model->rmse_deg = metrics.rmse_deg;
    model->p95_deg = metrics.p95_deg;
    model->feature_independence = coefficients.feature_independence;
    model->used_count = metrics.valid_count;
    return 1;
}

float calibration_camera_heading_deg(const CameraCalibrationModel *model,
                                     float position_px,
                                     float slope_px_per_row)
{
    float tangent;

    if ((model == NULL) ||
        (model->status != CALIBRATION_FIT_OK) ||
        !value_is_finite(position_px) ||
        !value_is_finite(slope_px_per_row))
    {
        return 0.0f;
    }

    tangent = model->intercept +
              model->position_gain *
                  (position_px - model->reference_position_px) +
              model->slope_gain *
                  (slope_px_per_row - model->reference_slope_px_per_row);
    return atanf(tangent) * radians_to_degrees;
}

int calibration_evaluate_camera(const CameraCalibrationSample *samples,
                                unsigned int sample_count,
                                const CameraCalibrationModel *model,
                                CameraCalibrationMetrics *metrics)
{
    CameraCoefficients coefficients;
    unsigned int valid_count = 0u;
    unsigned int i;

    if (metrics != NULL)
    {
        metrics->valid_count = 0u;
        metrics->bias_deg = 0.0f;
        metrics->rmse_deg = 0.0f;
        metrics->p95_deg = 0.0f;
    }
    if ((samples == NULL) || (model == NULL) || (metrics == NULL) ||
        (model->status != CALIBRATION_FIT_OK) ||
        (sample_count == 0u) || (sample_count > CALIBRATION_MAX_SAMPLES))
    {
        return 0;
    }

    coefficients.intercept = model->intercept;
    coefficients.position_gain = model->position_gain;
    coefficients.slope_gain = model->slope_gain;
    coefficients.feature_independence = model->feature_independence;
    coefficients.used_count = model->used_count;

    for (i = 0u; i < sample_count; i++)
    {
        if (camera_sample_is_valid(&samples[i]))
        {
            float residual = camera_residual_deg(&samples[i],
                                                 model->reference_position_px,
                                                 model->reference_slope_px_per_row,
                                                 &coefficients);
            metrics->bias_deg += residual;
            metrics->rmse_deg += residual * residual;
            sort_workspace[valid_count] = fabsf(residual);
            valid_count++;
        }
    }
    if (valid_count == 0u)
    {
        return 0;
    }

    metrics->valid_count = valid_count;
    metrics->bias_deg /= (float)valid_count;
    metrics->rmse_deg = sqrtf(metrics->rmse_deg / (float)valid_count);
    sort_values(sort_workspace, valid_count);
    metrics->p95_deg =
        sort_workspace[((95u * valid_count + 99u) / 100u) - 1u];
    return 1;
}
