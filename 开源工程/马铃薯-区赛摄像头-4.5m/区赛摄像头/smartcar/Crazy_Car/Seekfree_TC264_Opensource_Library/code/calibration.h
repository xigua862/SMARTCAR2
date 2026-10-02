/**
 * @file calibration.h
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

#ifndef CODE_CALIBRATION_H_
#define CODE_CALIBRATION_H_

#include "zf_common_headfile.h"
#include "calibration_math.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* 相机标定页的当前步骤。 */
typedef enum
{
    CAMERA_CAL_INACTIVE = 0,
    CAMERA_CAL_WAIT_TRAIN_ZERO,
    CAMERA_CAL_TRAINING,
    CAMERA_CAL_WAIT_VALIDATION_ZERO,
    CAMERA_CAL_VALIDATING,
    CAMERA_CAL_PASS_NOT_SAVED,
    CAMERA_CAL_SAVED,
    CAMERA_CAL_FAILED,
    CAMERA_CAL_ABORTED
} CameraCalibrationPhase;

/* 页面直接显示这些原因，避免只给一个模糊的“标定失败”。 */
typedef enum
{
    CAMERA_CAL_ERROR_NONE = 0,
    CAMERA_CAL_ERROR_NOT_IDLE,
    CAMERA_CAL_ERROR_IMU_NOT_READY,
    CAMERA_CAL_ERROR_TOO_FEW_FRAMES,
    CAMERA_CAL_ERROR_VALID_RATE,
    CAMERA_CAL_ERROR_YAW_COVERAGE,
    CAMERA_CAL_ERROR_FIT,
    CAMERA_CAL_ERROR_CONDITIONING,
    CAMERA_CAL_ERROR_VALIDATION_BIAS,
    CAMERA_CAL_ERROR_VALIDATION_RMSE,
    CAMERA_CAL_ERROR_VALIDATION_P95,
    CAMERA_CAL_ERROR_FLASH
} CameraCalibrationError;

typedef struct
{
    CameraCalibrationPhase phase;
    CameraCalibrationError error;
    CalibrationFitStatus fit_status;
    uint16 valid_samples;
    uint16 observed_frames;
    uint16 minimum_samples;
    float valid_rate_percent;
    float minimum_gyro_deg;
    float maximum_gyro_deg;
    float live_position_px;
    float live_slope_px_per_row;
    float live_gyro_deg;
    float predicted_heading_deg;
    float live_fit_mse_px2;
    uint16 live_valid_rows;
    uint16 live_candidate_rows;
    uint8 live_frame_valid;
    float bias_deg;
    float rmse_deg;
    float p95_deg;
    float feature_independence;
    uint8 model_valid;
    uint8 fusion_enabled;
} CameraCalibrationStatus;

/* 相机模型详情页显示的拟合系数和验收结果。 */
typedef struct
{
    uint8 available;
    uint8 saved;
    float reference_position_px;
    float reference_slope_px_per_row;
    float intercept;
    float position_gain;
    float slope_gain;
    float bias_deg;
    float rmse_deg;
    float p95_deg;
    float feature_independence;
    float valid_rate_percent;
    uint32 training_samples;
    uint32 validation_samples;
} CameraCalibrationModelDetails;

/* 编码器 90° 航向标定已删除(2026-07-12 用户定)：航向刻度改为
 * control.h 的 ENCODER_YAW_DEG_PER_DIFF_PULSE 宏，与轮径解耦。
 * Flash 记录中的 encoder 字段仅为保持记录布局兼容而保留，不再消费。 */

/* 上电加载标定记录。必须在 gyro_init() 和 menu_init() 前调用。 */
void calibration_init(void);
/* 5ms 后台任务：消费新相机帧并维持标定页负压。 */
void calibration_task_5ms(void);

void camera_calibration_enter(void);
void camera_calibration_leave(void);
void camera_calibration_confirm(void);
void camera_calibration_abort(void);
void camera_calibration_get_status(CameraCalibrationStatus *status);
void camera_calibration_get_model_details(CameraCalibrationModelDetails *details);

/* 运行期只读接口。标定无效时不会给出伪造的相机航向。 */
uint8 camera_heading_model_valid(void);
float camera_heading_estimate_deg(float position_px, float slope_px_per_row);
uint8 camera_heading_fusion_enabled(void);
uint8 camera_heading_set_fusion_enabled(uint8 enabled);

const char *camera_calibration_phase_text(CameraCalibrationPhase phase);
const char *camera_calibration_error_text(CameraCalibrationError error,
                                          CalibrationFitStatus fit_status);

#ifdef __cplusplus
}
#endif

#endif /* CODE_CALIBRATION_H_ */
