/**
 * @file telemetry.h
 * @brief 单次运行日志采集、冻结、EMEM 管理和串口导出。
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

#ifndef CODE_TELEMETRY_H_
#define CODE_TELEMETRY_H_

#include "zf_common_headfile.h"
#include "telemetry_format.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    uint32 timestamp_ms;
    float progress_points;
    float actual_x_mm;
    float actual_y_mm;
    float actual_yaw_deg;
    float raw_x_mm;
    float raw_y_mm;
    float raw_yaw_deg;
    uint16 localization_update_seq;
} TelemetryNavigationSample;

void telemetry_init(void);
/* CPU0 main-loop call: performs the EMEM session-header writes; never call
 * from a timer/control ISR. */
void telemetry_begin_run(uint8 route_index, uint16 route_point_count, uint32 route_hash);
void telemetry_activate_run(void);       // 主循环完成控制/导航切换后，开放 ISR 记录
void telemetry_freeze(uint8 stop_reason, uint8 final_prestart_state, uint8 final_nav_state);
void telemetry_service(void);
void telemetry_export_service(void);
void telemetry_link_service(void);
/* Request export from the one-key bench workflow.  A frozen session may be
 * requested during the short stop-drain window; the main loop waits for the
 * final CRC/header commit before sending. */
uint8 telemetry_request_export(void);

uint8 telemetry_is_capturing(void);
TelemetryState telemetry_get_state(void);
uint16 telemetry_relative_timestamp_ms(uint32 timestamp_ms);

void telemetry_capture_control_isr(int16 encoder_l,
                                   int16 encoder_r,
                                   float target_l_cmps,
                                   float target_r_cmps,
                                   float accel_l_cmps2,
                                   float accel_r_cmps2,
                                   int16 pwm_l,
                                   int16 pwm_r,
                                   float yaw_rate_error_degps,
                                   float tracking_turn_cmps);
void telemetry_capture_navigation_isr(const TelemetryNavigationSample *sample);
/* CPU1 IMU DRDY producer.  This is a bounded single-producer enqueue into a
 * CPU1-local, CPU0-drained DSRAM queue; it never writes EMEM directly. */
void telemetry_capture_imu_isr(float acc_x_g,
                               float acc_y_g,
                               float acc_z_g,
                               float gyro_x_rad_s,
                               float gyro_y_rad_s,
                               float gyro_z_rad_s,
                               float quaternion_q0,
                               float quaternion_q1,
                               float quaternion_q2,
                               float quaternion_q3,
                               uint32 sample_us);
void telemetry_capture_vision_isr(uint32 capture_ms,
                                  uint32 frame_seq,
                                  float line_position_mm,
                                  uint16 active_mask,
                                  uint16 observation_flags,
                                  uint8 peak_count,
                                  uint8 white_percent,
                                  uint8 threshold_gray);
void telemetry_capture_event_isr(TelemetryEventType type,
                                 TelemetryEventSource source,
                                 uint16 code,
                                 int32 value0,
                                 int32 value1);
/* CPU0 scan service archives each accepted raw-odom hit while the run is
 * active.  The online 1280-point ring remains independent and may overwrite
 * old points without deleting this EMEM history. */
void telemetry_record_phototube_scan_point(uint32 frame_seq,
                                           uint32 capture_ms,
                                           float x_mm,
                                           float y_mm,
                                           float lateral_mm,
                                           float raw_yaw_rad,
                                           float travel_distance_mm,
                                           uint8 confidence,
                                           uint8 point_flags);
/* CPU0 navigation service records one accepted or rejected localization transaction.
 * This is a main-loop API, not an ISR producer, and is persisted in the
 * dedicated schema-21 localization ledger.  record.timestamp_ms is the
 * boot-clock commit/decision instant and is stored as run-relative time. */
void telemetry_record_localization(const TelemetryLocalizationRecord *record);
/* Lossless per-feature aggregate.  The slot is selected directly by
 * route_feature_occurrence, so later corners cannot be displaced by early
 * repeated failures. */
void telemetry_record_localization_feature(
    const TelemetryLocalizationFeatureRecord *record);

#ifdef __cplusplus
}
#endif

#endif /* CODE_TELEMETRY_H_ */
