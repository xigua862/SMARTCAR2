/**
 * @file calibration_math.h
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

#ifndef CALIBRATION_MATH_H
#define CALIBRATION_MATH_H

/* 标定拟合结果。失败时可直接用于页面提示。 */
typedef enum
{
    CALIBRATION_FIT_OK = 0,
    CALIBRATION_FIT_BAD_ARGUMENT,
    CALIBRATION_FIT_TOO_FEW_SAMPLES,
    CALIBRATION_FIT_INVALID_SAMPLE,
    CALIBRATION_FIT_FEATURE_RANGE_TOO_SMALL,
    CALIBRATION_FIT_FEATURES_COUPLED,
    CALIBRATION_FIT_TOO_MANY_OUTLIERS,
    CALIBRATION_FIT_DIRECTION_MISMATCH,
    CALIBRATION_FIT_DIRECTION_MISSING,
    CALIBRATION_FIT_RESULTS_UNSTABLE
} CalibrationFitStatus;

typedef struct
{
    float position_px;
    float slope_px_per_row;
    float gyro_deg;
} CameraCalibrationSample;

typedef struct
{
    CalibrationFitStatus status;
    float reference_position_px;
    float reference_slope_px_per_row;

    /* tan(航向角) = intercept + position_gain*(P-P0) + slope_gain*(S-S0) */
    float intercept;
    float position_gain;
    float slope_gain;

    float bias_deg;
    float rmse_deg;
    float p95_deg;
    float feature_independence;
    unsigned int used_count;
} CameraCalibrationModel;

typedef struct
{
    unsigned int valid_count;
    float bias_deg;
    float rmse_deg;
    float p95_deg;
} CameraCalibrationMetrics;

/*
 * 用陀螺仪航向拟合相机模型。函数内部会剔除少量离群点，
 * 但 P/S 变化范围不足或高度耦合时会直接失败。
 */
int calibration_fit_camera(const CameraCalibrationSample *samples,
                           unsigned int sample_count,
                           float reference_position_px,
                           float reference_slope_px_per_row,
                           CameraCalibrationModel *model);

/* 用独立样本评估已拟合模型，评估阶段不剔除误差较大的有效样本。 */
int calibration_evaluate_camera(const CameraCalibrationSample *samples,
                                unsigned int sample_count,
                                const CameraCalibrationModel *model,
                                CameraCalibrationMetrics *metrics);

float calibration_camera_heading_deg(const CameraCalibrationModel *model,
                                     float position_px,
                                     float slope_px_per_row);

#endif
