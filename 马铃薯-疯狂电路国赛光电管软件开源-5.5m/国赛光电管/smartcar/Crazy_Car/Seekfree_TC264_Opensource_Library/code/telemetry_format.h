/**
 * @file telemetry_format.h
 * @brief EMEM 遥测记录的字节序、校验和格式编码。
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

#ifndef CODE_TELEMETRY_FORMAT_H_
#define CODE_TELEMETRY_FORMAT_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define TELEMETRY_SCHEMA_VERSION (21u)
#define TELEMETRY_PROTOCOL_VERSION (2u)

/*
 * This is the exported logical image size, not a claim about the physical
 * EMEM capacity of every TC26x derivative.  The image is used only after
 * telemetry_init() has positively identified an EMEM-capable device.
 */
#define TELEMETRY_EMEM_SIZE (0x50000u)
#define TELEMETRY_HEADER_OFFSET (0x00000u)
#define TELEMETRY_HEADER_REGION_SIZE (0x01000u)
#define TELEMETRY_CONTROL_OFFSET (0x01000u)
#define TELEMETRY_CONTROL_REGION_SIZE (0x11B40u)
#define TELEMETRY_NAVIGATION_OFFSET (0x12B40u)
#define TELEMETRY_NAVIGATION_REGION_SIZE (0x073E0u)
#define TELEMETRY_VISION_OFFSET (0x19F20u)
#define TELEMETRY_VISION_REGION_SIZE (0x080C0u)
#define TELEMETRY_EVENT_OFFSET (0x21FE0u)
#define TELEMETRY_EVENT_REGION_SIZE (0x01000u)
#define TELEMETRY_EVENT_CAPACITY (256u)
#define TELEMETRY_IMU_OFFSET (0x22FE0u)
#define TELEMETRY_IMU_REGION_SIZE (0x13560u)
#define TELEMETRY_RESERVED_OFFSET (0x36540u)
#define TELEMETRY_RESERVED_REGION_SIZE (0x19AC0u)

/* The tail is split into a compact localization ledger followed by the raw
 * odom phototube scan.  The two blocks share the old reserved tail without
 * changing the control/navigation/vision/IMU offsets. */
#define TELEMETRY_LOCALIZATION_OFFSET (TELEMETRY_RESERVED_OFFSET)
#define TELEMETRY_LOCALIZATION_CAPACITY (512u)
#define TELEMETRY_LOCALIZATION_RECORD_SIZE (44u)

#define TELEMETRY_PHOTOTUBE_SCAN_OFFSET \
    (TELEMETRY_LOCALIZATION_OFFSET + \
     TELEMETRY_LOCALIZATION_CAPACITY * TELEMETRY_LOCALIZATION_RECORD_SIZE)
#define TELEMETRY_PHOTOTUBE_SCAN_REGION_SIZE \
    (TELEMETRY_RESERVED_OFFSET + TELEMETRY_RESERVED_REGION_SIZE - \
     TELEMETRY_PHOTOTUBE_SCAN_OFFSET)
#define TELEMETRY_PHOTOTUBE_SCAN_HEADER_SIZE (56u)
#define TELEMETRY_PHOTOTUBE_SCAN_RECORD_SIZE (16u)
#define TELEMETRY_PHOTOTUBE_SCAN_CAPACITY (5120u)

#define TELEMETRY_SESSION_HEADER_SIZE (256u)
#define TELEMETRY_LOCALIZATION_CONFIG_OFFSET (0x00100u)
#define TELEMETRY_LOCALIZATION_CONFIG_SIZE (64u)
#define TELEMETRY_LOCALIZATION_CONFIG_VERSION (2u)
#define TELEMETRY_LOCALIZATION_FEATURE_OFFSET (0x00140u)
#define TELEMETRY_LOCALIZATION_FEATURE_HEADER_SIZE (16u)
#define TELEMETRY_LOCALIZATION_FEATURE_RECORD_SIZE (32u)
#define TELEMETRY_LOCALIZATION_FEATURE_CAPACITY (116u)
#define TELEMETRY_LOCALIZATION_FEATURE_VERSION (3u)
#define TELEMETRY_TRACKING_CONFIG_OFFSET (0x00FD0u)
#define TELEMETRY_TRACKING_CONFIG_SIZE (48u)
#define TELEMETRY_TRACKING_CONFIG_VERSION (2u)
#define TELEMETRY_CONTROL_CAPACITY (3296u)
#define TELEMETRY_NAVIGATION_CAPACITY (1648u)
#define TELEMETRY_VISION_CAPACITY (3296u)
#define TELEMETRY_IMU_CAPACITY (3960u)
#define TELEMETRY_IMU_SOURCE_SAMPLE_HZ (480u)
#define TELEMETRY_IMU_SAMPLE_HZ (240u)
#define TELEMETRY_IMU_DECIMATION (2u)
#define TELEMETRY_IMU_RECORD_SIZE (20u)
#define TELEMETRY_CONTROL_LOG_DIVIDER (5u)
#define TELEMETRY_CONTROL_LOG_RATE_HZ (200u)
#define TELEMETRY_VISION_SOURCE_SAMPLE_HZ (1000u)
#define TELEMETRY_VISION_LOG_DIVIDER (5u)
#define TELEMETRY_VISION_LOG_RATE_HZ (200u)

#if TELEMETRY_VISION_LOG_DIVIDER == 0u
#error "Telemetry vision log divider must be greater than zero"
#endif
#define TELEMETRY_PARAMETER_WORD_COUNT (20u)
#define TELEMETRY_HEADER_CRC_OFFSET (24u)

#define TELEMETRY_SESSION_MAGIC_0 ('T')
#define TELEMETRY_SESSION_MAGIC_1 ('L')
#define TELEMETRY_SESSION_MAGIC_2 ('M')
#define TELEMETRY_SESSION_MAGIC_3 ('1')
#define TELEMETRY_LOCALIZATION_CONFIG_MAGIC_0 ('L')
#define TELEMETRY_LOCALIZATION_CONFIG_MAGIC_1 ('C')
#define TELEMETRY_LOCALIZATION_CONFIG_MAGIC_2 ('F')
#define TELEMETRY_LOCALIZATION_CONFIG_MAGIC_3 ('1')
#define TELEMETRY_LOCALIZATION_FEATURE_MAGIC_0 ('L')
#define TELEMETRY_LOCALIZATION_FEATURE_MAGIC_1 ('F')
#define TELEMETRY_LOCALIZATION_FEATURE_MAGIC_2 ('D')
#define TELEMETRY_LOCALIZATION_FEATURE_MAGIC_3 ('1')
#define TELEMETRY_TRACKING_CONFIG_MAGIC_0 ('T')
#define TELEMETRY_TRACKING_CONFIG_MAGIC_1 ('R')
#define TELEMETRY_TRACKING_CONFIG_MAGIC_2 ('K')
#define TELEMETRY_TRACKING_CONFIG_MAGIC_3 ('1')
#define TELEMETRY_WIRE_MAGIC_0 ('T')
#define TELEMETRY_WIRE_MAGIC_1 ('L')
#define TELEMETRY_WIRE_MAGIC_2 ('G')
#define TELEMETRY_WIRE_MAGIC_3 ('1')
#define TELEMETRY_LINK_MAGIC_0 ('T')
#define TELEMETRY_LINK_MAGIC_1 ('L')
#define TELEMETRY_LINK_MAGIC_2 ('D')
#define TELEMETRY_LINK_MAGIC_3 ('1')
#define TELEMETRY_PHOTOTUBE_SCAN_MAGIC_0 ('P')
#define TELEMETRY_PHOTOTUBE_SCAN_MAGIC_1 ('T')
#define TELEMETRY_PHOTOTUBE_SCAN_MAGIC_2 ('S')
#define TELEMETRY_PHOTOTUBE_SCAN_MAGIC_3 ('1')
#define TELEMETRY_PHOTOTUBE_SCAN_VERSION (14u)

#define TELEMETRY_LINK_FLAG_LOGGER_ENABLED (1u)
#define TELEMETRY_LINK_FLAG_RUN_FINALIZED (2u)
#define TELEMETRY_LINK_FLAG_IDLE (4u)

/* Wire protocol v2 keeps the logical EMEM image unchanged, but transports it
 * as a stream of little-endian 16-bit blocks.  Bit 15 selects a zero run;
 * bits 14:0 store length - 1.  Literal blocks carry that many bytes after the
 * header.  The receiver stops after reconstructing payload_size bytes. */
#define TELEMETRY_WIRE_FLAG_ZERO_RLE (1u)
#define TELEMETRY_WIRE_BLOCK_ZERO_FLAG (0x8000u)
#define TELEMETRY_WIRE_BLOCK_LENGTH_MASK (0x7FFFu)
#define TELEMETRY_WIRE_BLOCK_MAX_LENGTH (0x8000u)

typedef enum
{
    TELEMETRY_STATE_DISABLED = 0u,
    TELEMETRY_STATE_READY = 1u,
    TELEMETRY_STATE_RECORDING = 2u,
    TELEMETRY_STATE_FROZEN = 3u,
    TELEMETRY_STATE_EXPORTING = 4u
} TelemetryState;

typedef enum
{
    TELEMETRY_EVENT_RUN_START = 1u,
    TELEMETRY_EVENT_RUN_STOP = 2u,
    TELEMETRY_EVENT_PRESTART_STATE = 3u,
    TELEMETRY_EVENT_NAV_STATE = 4u,
    TELEMETRY_EVENT_VISION_DECISION = 5u,
    TELEMETRY_EVENT_BUFFER_DROP = 6u,
    TELEMETRY_EVENT_POSITION_SATURATION = 7u,
    TELEMETRY_EVENT_CONTROL_LIMIT = 8u,
    TELEMETRY_EVENT_PROTECTION = 9u,
    TELEMETRY_EVENT_RUNTIME_BUDGET = 10u
} TelemetryEventType;

typedef enum
{
    TELEMETRY_RUNTIME_CONTROL_EXEC = 1u,
    TELEMETRY_RUNTIME_CONTROL_PERIOD = 2u,
    TELEMETRY_RUNTIME_PHOTOTUBE_SCAN = 3u,
    TELEMETRY_RUNTIME_DUTY_10MS = 4u,
    TELEMETRY_RUNTIME_NAV_MATCH = 5u,
    TELEMETRY_RUNTIME_EMEM_SERVICE = 6u,
    TELEMETRY_RUNTIME_IMU_SNAPSHOT_FAILURE = 7u,
    TELEMETRY_RUNTIME_IMU_SNAPSHOT_AGE = 8u,
    TELEMETRY_RUNTIME_SCAN_EPOCH = 9u,
    TELEMETRY_RUNTIME_IMU_ISR = 10u,
    TELEMETRY_RUNTIME_IMU_SAMPLE_FAILURE = 11u,
    TELEMETRY_RUNTIME_PHOTOTUBE_ADC_FAILURE = 12u
} TelemetryRuntimeBudgetCode;

typedef enum
{
    TELEMETRY_EVENT_SOURCE_NONE = 0u,
    TELEMETRY_EVENT_SOURCE_PRESTART = 1u,
    TELEMETRY_EVENT_SOURCE_LOGGER = 2u,
    TELEMETRY_EVENT_SOURCE_NAVIGATION = 3u,
    TELEMETRY_EVENT_SOURCE_CONTROL = 4u,
    TELEMETRY_EVENT_SOURCE_VISION = 5u,
    TELEMETRY_EVENT_SOURCE_SAFETY = 6u
} TelemetryEventSource;

typedef enum
{
    TELEMETRY_CONTROL_LIMIT_NONE = 0u,
    TELEMETRY_CONTROL_LIMIT_FEEDBACK_AUTHORITY = 1u,
    TELEMETRY_CONTROL_LIMIT_TURN = 2u,
    TELEMETRY_CONTROL_LIMIT_DUTY_LEFT = 4u,
    TELEMETRY_CONTROL_LIMIT_DUTY_RIGHT = 8u,
    TELEMETRY_CONTROL_LIMIT_SLIP_GUARD = 16u
} TelemetryControlLimitFlag;

/* TASKING TriCore only accepts pack 2 or pack 0.  Keep pack 2 local to the
 * four compact all-16-bit records whose natural aggregate alignment would add
 * two bytes of padding.  32-bit/float-bearing records intentionally retain
 * their default alignment. */
#pragma pack 2

typedef struct
{
    uint16_t tick_ms;
    int16_t encoder_l;
    int16_t encoder_r;
    int16_t target_l_x10;
    int16_t target_r_x10;
    int16_t accel_l_cmps2;
    int16_t accel_r_cmps2;
    int16_t pwm_l;
    int16_t pwm_r;
    /* Exact yaw-rate loop error: target - measured, 0.1 deg/s. */
    int16_t yaw_rate_error_degps_x10;
    int16_t tracking_turn_x10;
} TelemetryControlRecord;

typedef struct
{
    uint16_t timestamp_ms;
    uint16_t progress_quarter_points;
    int16_t actual_x_mm;
    int16_t actual_y_mm;
    int16_t actual_yaw_cdeg; /* wrapped (-180, 180] heading, 0.01 degree */
    int16_t raw_x_mm;
    int16_t raw_y_mm;
    int16_t raw_yaw_cdeg;
    uint16_t localization_update_seq;
} TelemetryNavigationRecord;

#pragma pack 0

/* Compact dynamic-preview summary in the fixed session header.  The complete
 * self-describing controller snapshot is TelemetryTrackingConfigBlock. */
typedef struct
{
    float preview_time_s;
    float min_lookahead_mm;
    float max_lookahead_mm;
    float kd_cmps_per_degps_per_speed_scale;
} TelemetryControlConfig;

#pragma pack 2

typedef struct
{
    uint16_t tick_ms;
    uint16_t frame_seq;
    int16_t line_position_mm_x100;
    uint16_t active_mask;
    /* bits 0..6 threshold; 7..10 peak count; 11..14 white ratio/15;
     * bit 15 is the exact full-coverage decision.  Remaining flags are
     * reconstructed from these fields, active_mask and the line sentinel. */
    uint16_t diagnostics;
} TelemetryVisionRecord;

#pragma pack 0

typedef struct
{
    uint32_t timestamp_ms;
    uint8_t type;
    uint8_t source;
    uint16_t code;
    int32_t value0;
    int32_t value1;
} TelemetryEventRecord;

#define TELEMETRY_LOCALIZATION_FEATURE_REGISTERED (1u << 0)
#define TELEMETRY_LOCALIZATION_FEATURE_ENCOUNTERED (1u << 1)
#define TELEMETRY_LOCALIZATION_FEATURE_ACCEPTED (1u << 2)
/* ACCEPTED|PARTIAL is a valid single-edge calibration. */
#define TELEMETRY_LOCALIZATION_FEATURE_PARTIAL (1u << 3)
#define TELEMETRY_LOCALIZATION_MATCH_STRAIGHT (1u)
#define TELEMETRY_LOCALIZATION_MATCH_CORNER (2u)
#define TELEMETRY_LOCALIZATION_MATCH_JUNCTION (3u)

#define TELEMETRY_LOCALIZATION_DETAIL_MAP_GEOMETRY (1u << 0)
#define TELEMETRY_LOCALIZATION_DETAIL_TOTAL_POINTS (1u << 1)
#define TELEMETRY_LOCALIZATION_DETAIL_TOTAL_WEIGHT (1u << 2)
#define TELEMETRY_LOCALIZATION_DETAIL_LINE0_POINTS (1u << 3)
#define TELEMETRY_LOCALIZATION_DETAIL_LINE1_POINTS (1u << 4)
#define TELEMETRY_LOCALIZATION_DETAIL_LINE2_POINTS (1u << 5)
#define TELEMETRY_LOCALIZATION_DETAIL_LINE0_WEIGHT (1u << 6)
#define TELEMETRY_LOCALIZATION_DETAIL_LINE1_WEIGHT (1u << 7)
#define TELEMETRY_LOCALIZATION_DETAIL_LINE2_WEIGHT (1u << 8)
#define TELEMETRY_LOCALIZATION_DETAIL_LINE0_SPAN (1u << 9)
#define TELEMETRY_LOCALIZATION_DETAIL_LINE1_SPAN (1u << 10)
#define TELEMETRY_LOCALIZATION_DETAIL_LINE2_SPAN (1u << 11)
#define TELEMETRY_LOCALIZATION_DETAIL_LINE0_FRAMES (1u << 12)
#define TELEMETRY_LOCALIZATION_DETAIL_LINE1_FRAMES (1u << 13)
#define TELEMETRY_LOCALIZATION_DETAIL_LINE2_FRAMES (1u << 14)
#define TELEMETRY_LOCALIZATION_DETAIL_NONCOLLINEAR (1u << 15)
#define TELEMETRY_LOCALIZATION_DETAIL_SOLVER (1u << 16)
#define TELEMETRY_LOCALIZATION_DETAIL_RMS (1u << 17)
#define TELEMETRY_LOCALIZATION_DETAIL_TRANSLATION (1u << 18)
#define TELEMETRY_LOCALIZATION_DETAIL_YAW (1u << 19)
#define TELEMETRY_LOCALIZATION_DETAIL_AMBIGUOUS (1u << 20)
#define TELEMETRY_LOCALIZATION_DETAIL_CONFIRMATION_WAIT (1u << 21)
#define TELEMETRY_LOCALIZATION_DETAIL_CONFIRMATION_INCONSISTENT (1u << 22)
#define TELEMETRY_LOCALIZATION_DETAIL_NO_IMPROVEMENT (1u << 23)
#define TELEMETRY_LOCALIZATION_DETAIL_PROJECTION (1u << 24)
#define TELEMETRY_LOCALIZATION_DETAIL_COMMIT_BUSY (1u << 25)
#define TELEMETRY_LOCALIZATION_DETAIL_FEATURE_ORDER (1u << 26)
#define TELEMETRY_LOCALIZATION_DETAIL_FEATURE_PROGRESS (1u << 27)
#define TELEMETRY_LOCALIZATION_DETAIL_FEATURE_CONSUMED (1u << 28)
#define TELEMETRY_LOCALIZATION_DETAIL_NO_FRESH_EVIDENCE (1u << 29)
#define TELEMETRY_LOCALIZATION_DETAIL_PROGRESS_REGRESSION (1u << 30)

/* Compact schema-21 localization transaction.  T_before translation,
 * delta and raw pose use whole millimetres; yaw uses centidegrees, progress
 * uses quarter path points, and RMS uses mm x10.  Accepted and rejected
 * attempts share this chronological ledger; localization_update_seq remains
 * the sequence of committed transforms only.
 * For an accepted record, T_after and map-after pose are reconstructed from
 * T_before, delta and raw pose.  For a rejected record the actual T_after is
 * T_before; the same fields reconstruct the refused candidate separately. */
#define TELEMETRY_LOCALIZATION_STATUS_REJECTED (0u)
#define TELEMETRY_LOCALIZATION_STATUS_ACCEPTED (1u)
typedef struct
{
    uint32_t timestamp_ms;
    uint32_t capture_frame_seq;
    int16_t t_before_tx_mm;
    int16_t t_before_ty_mm;
    uint16_t travel_distance_mm;
    uint16_t localization_update_seq;
    uint8_t route_feature_occurrence;
    uint8_t match_type;
    int16_t t_before_yaw_cdeg;
    int16_t delta_odom_dx_mm;
    int16_t delta_odom_dy_mm;
    int16_t delta_odom_dyaw_cdeg;
    int16_t raw_pose_x_mm;
    int16_t raw_pose_y_mm;
    int16_t raw_pose_yaw_cdeg;
    uint16_t progress_before_quarter_points;
    uint16_t progress_after_quarter_points;
    int16_t rms_before_mm_x10;
    int16_t rms_after_mm_x10;
    uint16_t inlier_count;
    uint8_t status;
    uint8_t reject_reason;
} TelemetryLocalizationRecord;

typedef struct
{
    uint8_t magic[4];
    uint16_t version;
    uint16_t header_size;
    uint16_t record_size;
    uint16_t capacity;
    uint16_t registered_count;
    uint16_t diagnostic_drop_count;
} TelemetryLocalizationFeatureHeader;

/* One fixed slot per nav_path physical vertex.  Only CORNER/JUNCTION slots
 * are registered.  Repeated attempts OR their reason/detail masks into the
 * same record, so a long run cannot discard a later feature's diagnosis.
 * ACCEPTED|PARTIAL means a single finite route edge was used; the unobservable
 * along-edge degree of freedom is intentionally left unchanged. */
typedef struct
{
    uint8_t route_feature_occurrence;
    uint8_t status_flags;
    uint8_t last_reject_reason;
    uint8_t reserved0;
    uint32_t reject_reason_mask;
    uint32_t detail_flags;
    uint32_t last_detail_flags;
    uint16_t attempt_count;
    uint16_t last_timestamp_ms;
    uint16_t observed_progress_points;
    uint8_t line_point_count[3];
    uint8_t line_frame_count[3];
    int16_t failure_value_x10;
    uint16_t reserved1;
} TelemetryLocalizationFeatureRecord;

/*
 * Full-rate posture sample.  timestamp_delta_4us is accumulated by the host.
 * The unit quaternion uses "smallest three" compression: the largest absolute
 * component is made positive and omitted; its 2-bit index occupies bits 15..14
 * of quaternion_component0_index, while all three stored components are signed
 * 14-bit values scaled by TELEMETRY_QUATERNION_S14_SCALE.
 */
#define TELEMETRY_QUATERNION_S14_SCALE (11583.0f)
typedef struct
{
    uint16_t timestamp_delta_4us;
    int16_t acc_x_mg;
    int16_t acc_y_mg;
    int16_t acc_z_mg;
    int16_t gyro_x_dps_x10;
    int16_t gyro_y_dps_x10;
    int16_t gyro_z_dps_x10;
    uint16_t quaternion_component0_index;
    int16_t quaternion_component1_s14;
    int16_t quaternion_component2_s14;
} TelemetryImuRecord;

#pragma pack 2
typedef struct
{
    uint16_t frame_seq_delta;
    uint16_t capture_ms;
    int16_t x_mm;
    int16_t y_mm;
    int16_t lateral_mm_x10;
    int16_t raw_yaw_cdeg;
    uint16_t travel_distance_mm;
    uint8_t confidence;
    uint8_t point_flags;
} TelemetryPhototubeScanRecord;

#pragma pack 0

typedef struct
{
    uint8_t magic[4];
    uint16_t version;
    uint16_t header_size;
    uint16_t point_count;
    uint16_t record_size;
    uint32_t first_frame_seq;
    uint16_t pose_interpolated_count;
    uint16_t pose_rejected_count;
    uint16_t mailbox_skipped_count;
    uint16_t source_frame_skipped_count;
    uint16_t source_schedule_drop_count;
    uint16_t source_schedule_skipped_count;
    uint16_t max_processing_latency_ms;
    uint16_t max_capture_to_cloud_latency_ms;
    uint16_t received_frame_count;
    uint16_t accepted_frame_count;
    uint16_t accepted_hit_count;
    uint16_t spatial_dedup_frame_count;
    uint16_t travel_reversal_frame_count;
    uint16_t online_ring_overwrite_point_count;
    uint16_t empty_hit_frame_count;
    /* Exact oldest slot when the archive is full.  The saturating overwrite
     * count is kept in BUFFER_DROP code 7 so it cannot corrupt ring order. */
    uint16_t archive_oldest_storage_slot;
    uint16_t staging_backpressure_count;
    uint16_t staging_high_watermark;
    uint16_t pose_wait_count;
    uint16_t emem_queue_drop_count;
} TelemetryPhototubeScanHeader;

typedef struct
{
    uint8_t magic[4];
    uint16_t version;
    uint16_t size;
    uint32_t config_hash;
    uint32_t algorithm_version;
    uint8_t sensor_count;
    uint8_t adc_group;
    uint8_t adc_channel;
    uint8_t mux_selector_count;
    uint32_t scan_hz;
    float sensor_spacing_mm;
    float sensor_forward_mm;
    float scan_min_step_mm;
    float match_travel_window_mm;
    float feature_radius_mm;
    float max_feature_point_residual_mm;
    float max_rms_mm;
    float max_correction_mm;
    float max_correction_yaw_deg;
    float black_scan_stop_distance_mm;
} TelemetryLocalizationConfigBlock;

typedef struct
{
    uint8_t magic[4];
    uint16_t version;
    uint16_t size;
    uint32_t config_hash;
    uint32_t algorithm_version;
    float wheel_track_mm;
    float preview_time_s;
    float min_lookahead_mm;
    float max_lookahead_mm;
    /* Fixed slots: algorithm >=16 stores 0 then K_omega; v17 stores the
     * curvature-feedforward preview time in target_rate_window_ms. */
    float kd_cmps_per_degps_per_speed_scale;
    float gkd_cmps_per_radps;
    float turn_limit_ratio;
    uint16_t path_sample_mm_x100;
    uint16_t target_rate_window_ms;
} TelemetryTrackingConfigBlock;

typedef struct
{
    uint8_t magic[4];
    uint16_t schema_version;
    uint16_t header_size;
    uint8_t state;
    uint8_t init_status;
    uint8_t stop_reason;
    uint8_t route_index;
    uint32_t session_sequence;
    uint32_t start_ms;
    uint32_t end_ms;
    uint32_t payload_crc32;
    uint32_t route_hash;
    uint16_t route_point_count;
    uint8_t final_prestart_state;
    uint8_t final_nav_state;
    uint16_t control_count;
    uint16_t navigation_count;
    uint16_t vision_count;
    uint16_t event_count;
    uint16_t control_drop_count;
    uint16_t navigation_drop_count;
    uint16_t vision_drop_count;
    uint16_t event_drop_count;
    uint32_t control_offset;
    uint32_t navigation_offset;
    uint32_t vision_offset;
    uint32_t event_offset;
    uint16_t control_record_size;
    uint16_t navigation_record_size;
    uint16_t vision_record_size;
    uint16_t event_record_size;
    uint16_t control_capacity;
    uint16_t navigation_capacity;
    uint16_t vision_capacity;
    uint16_t event_capacity;
    uint32_t firmware_id;
    float wheel_diameter_mm;
    float gear_ratio;
    float distance_per_pulse_mm;
    float cmps_per_count;
    float pid_sample_s;
    float nav_sample_mm;
    float nav_min_lookahead_mm;
    uint16_t encoder_ppr;
    uint16_t imu_flags;
    float imu_rate_hz;
    uint32_t parameter_words[TELEMETRY_PARAMETER_WORD_COUNT];
    uint16_t imu_count;
    uint16_t imu_drop_count;
    uint32_t imu_offset;
    uint16_t imu_record_size;
    uint16_t imu_capacity;
    uint16_t imu_sample_rate_hz;
    uint16_t imu_decimation;
    uint16_t control_log_divider;
    uint16_t control_log_rate_hz;
    /* Exact map-heading controller gains used by this run. */
    TelemetryControlConfig control_config;
    uint16_t localization_count;
    uint16_t localization_overwrite_count;
    uint32_t localization_offset;
    uint16_t localization_record_size;
    uint16_t localization_capacity;
    uint32_t vision_first_frame_seq;
} TelemetrySessionHeader;

typedef struct
{
    uint8_t magic[4];
    uint16_t protocol_version;
    uint16_t flags;
    uint32_t payload_size;
    uint32_t payload_crc32;
} TelemetryWirePrefix;

typedef struct
{
    uint8_t magic[4];
    uint16_t protocol_version;
    uint16_t flags;
    uint32_t payload_size;
    uint32_t session_sequence;
} TelemetryLinkBeacon;

#pragma pack 0

#define TELEMETRY_OFFSET_CHECK(type_name, field_name, expected_offset) \
    typedef char telemetry_##type_name##_##field_name##_offset_check[ \
        (offsetof(type_name, field_name) == (expected_offset)) ? 1 : -1]

TELEMETRY_OFFSET_CHECK(TelemetryControlRecord, tick_ms, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryControlRecord, encoder_l, 2u);
TELEMETRY_OFFSET_CHECK(TelemetryControlRecord, encoder_r, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryControlRecord, target_l_x10, 6u);
TELEMETRY_OFFSET_CHECK(TelemetryControlRecord, target_r_x10, 8u);
TELEMETRY_OFFSET_CHECK(TelemetryControlRecord, accel_l_cmps2, 10u);
TELEMETRY_OFFSET_CHECK(TelemetryControlRecord, accel_r_cmps2, 12u);
TELEMETRY_OFFSET_CHECK(TelemetryControlRecord, pwm_l, 14u);
TELEMETRY_OFFSET_CHECK(TelemetryControlRecord, pwm_r, 16u);
TELEMETRY_OFFSET_CHECK(TelemetryControlRecord, yaw_rate_error_degps_x10, 18u);
TELEMETRY_OFFSET_CHECK(TelemetryControlRecord, tracking_turn_x10, 20u);

TELEMETRY_OFFSET_CHECK(TelemetryNavigationRecord, timestamp_ms, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryNavigationRecord, progress_quarter_points, 2u);
TELEMETRY_OFFSET_CHECK(TelemetryNavigationRecord, actual_x_mm, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryNavigationRecord, actual_y_mm, 6u);
TELEMETRY_OFFSET_CHECK(TelemetryNavigationRecord, actual_yaw_cdeg, 8u);
TELEMETRY_OFFSET_CHECK(TelemetryNavigationRecord, raw_x_mm, 10u);
TELEMETRY_OFFSET_CHECK(TelemetryNavigationRecord, raw_y_mm, 12u);
TELEMETRY_OFFSET_CHECK(TelemetryNavigationRecord, raw_yaw_cdeg, 14u);
TELEMETRY_OFFSET_CHECK(TelemetryNavigationRecord, localization_update_seq, 16u);

TELEMETRY_OFFSET_CHECK(TelemetryVisionRecord, tick_ms, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryVisionRecord, frame_seq, 2u);
TELEMETRY_OFFSET_CHECK(TelemetryVisionRecord, line_position_mm_x100, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryVisionRecord, active_mask, 6u);
TELEMETRY_OFFSET_CHECK(TelemetryVisionRecord, diagnostics, 8u);

TELEMETRY_OFFSET_CHECK(TelemetryEventRecord, timestamp_ms, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryEventRecord, type, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryEventRecord, source, 5u);
TELEMETRY_OFFSET_CHECK(TelemetryEventRecord, code, 6u);
TELEMETRY_OFFSET_CHECK(TelemetryEventRecord, value0, 8u);
TELEMETRY_OFFSET_CHECK(TelemetryEventRecord, value1, 12u);

TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, timestamp_ms, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, capture_frame_seq, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, t_before_tx_mm, 8u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, t_before_ty_mm, 10u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, travel_distance_mm, 12u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, localization_update_seq, 14u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, route_feature_occurrence, 16u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, match_type, 17u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, t_before_yaw_cdeg, 18u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, delta_odom_dx_mm, 20u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, delta_odom_dy_mm, 22u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, delta_odom_dyaw_cdeg, 24u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, raw_pose_x_mm, 26u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, raw_pose_y_mm, 28u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, raw_pose_yaw_cdeg, 30u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, progress_before_quarter_points, 32u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, progress_after_quarter_points, 34u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, rms_before_mm_x10, 36u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, rms_after_mm_x10, 38u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, inlier_count, 40u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, status, 42u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationRecord, reject_reason, 43u);

TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureHeader, magic, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureHeader, version, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureHeader, header_size, 6u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureHeader, record_size, 8u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureHeader, capacity, 10u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureHeader, registered_count, 12u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureHeader, diagnostic_drop_count, 14u);

TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureRecord, route_feature_occurrence, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureRecord, status_flags, 1u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureRecord, last_reject_reason, 2u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureRecord, reject_reason_mask, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureRecord, detail_flags, 8u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureRecord, last_detail_flags, 12u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureRecord, attempt_count, 16u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureRecord, last_timestamp_ms, 18u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureRecord, observed_progress_points, 20u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureRecord, line_point_count, 22u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureRecord, line_frame_count, 25u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureRecord, failure_value_x10, 28u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationFeatureRecord, reserved1, 30u);

TELEMETRY_OFFSET_CHECK(TelemetryImuRecord, timestamp_delta_4us, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryImuRecord, acc_x_mg, 2u);
TELEMETRY_OFFSET_CHECK(TelemetryImuRecord, acc_y_mg, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryImuRecord, acc_z_mg, 6u);
TELEMETRY_OFFSET_CHECK(TelemetryImuRecord, gyro_x_dps_x10, 8u);
TELEMETRY_OFFSET_CHECK(TelemetryImuRecord, gyro_y_dps_x10, 10u);
TELEMETRY_OFFSET_CHECK(TelemetryImuRecord, gyro_z_dps_x10, 12u);
TELEMETRY_OFFSET_CHECK(TelemetryImuRecord, quaternion_component0_index, 14u);
TELEMETRY_OFFSET_CHECK(TelemetryImuRecord, quaternion_component1_s14, 16u);
TELEMETRY_OFFSET_CHECK(TelemetryImuRecord, quaternion_component2_s14, 18u);

TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanRecord, frame_seq_delta, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanRecord, capture_ms, 2u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanRecord, x_mm, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanRecord, y_mm, 6u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanRecord, lateral_mm_x10, 8u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanRecord, raw_yaw_cdeg, 10u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanRecord, travel_distance_mm, 12u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanRecord, confidence, 14u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanRecord, point_flags, 15u);

TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, magic, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, version, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, header_size, 6u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, point_count, 8u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, record_size, 10u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, first_frame_seq, 12u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, pose_interpolated_count, 16u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, pose_rejected_count, 18u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, received_frame_count, 32u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, accepted_frame_count, 34u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, spatial_dedup_frame_count, 38u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, online_ring_overwrite_point_count, 42u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, empty_hit_frame_count, 44u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, archive_oldest_storage_slot, 46u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, staging_backpressure_count, 48u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, staging_high_watermark, 50u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, pose_wait_count, 52u);
TELEMETRY_OFFSET_CHECK(TelemetryPhototubeScanHeader, emem_queue_drop_count, 54u);

TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, magic, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, version, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, size, 6u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, config_hash, 8u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, algorithm_version, 12u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, sensor_count, 16u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, adc_group, 17u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, adc_channel, 18u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, mux_selector_count, 19u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, scan_hz, 20u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, sensor_spacing_mm, 24u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, sensor_forward_mm, 28u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, scan_min_step_mm, 32u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, match_travel_window_mm, 36u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, feature_radius_mm, 40u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, max_feature_point_residual_mm, 44u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, max_rms_mm, 48u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, max_correction_mm, 52u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, max_correction_yaw_deg, 56u);
TELEMETRY_OFFSET_CHECK(TelemetryLocalizationConfigBlock, black_scan_stop_distance_mm, 60u);

TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, magic, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, version, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, size, 6u);
TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, config_hash, 8u);
TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, algorithm_version, 12u);
TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, wheel_track_mm, 16u);
TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, preview_time_s, 20u);
TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, min_lookahead_mm, 24u);
TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, max_lookahead_mm, 28u);
TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, kd_cmps_per_degps_per_speed_scale, 32u);
TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, gkd_cmps_per_radps, 36u);
TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, turn_limit_ratio, 40u);
TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, path_sample_mm_x100, 44u);
TELEMETRY_OFFSET_CHECK(TelemetryTrackingConfigBlock, target_rate_window_ms, 46u);

TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, magic, 0u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, schema_version, 4u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, header_size, 6u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, state, 8u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, init_status, 9u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, stop_reason, 10u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, route_index, 11u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, session_sequence, 12u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, start_ms, 16u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, end_ms, 20u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, payload_crc32, 24u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, route_hash, 28u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, route_point_count, 32u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, final_prestart_state, 34u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, final_nav_state, 35u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, control_count, 36u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, navigation_count, 38u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, vision_count, 40u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, event_count, 42u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, control_drop_count, 44u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, navigation_drop_count, 46u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, vision_drop_count, 48u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, event_drop_count, 50u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, control_offset, 52u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, navigation_offset, 56u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, vision_offset, 60u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, event_offset, 64u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, control_record_size, 68u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, navigation_record_size, 70u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, vision_record_size, 72u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, event_record_size, 74u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, control_capacity, 76u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, navigation_capacity, 78u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, vision_capacity, 80u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, event_capacity, 82u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, firmware_id, 84u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, wheel_diameter_mm, 88u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, gear_ratio, 92u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, distance_per_pulse_mm, 96u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, cmps_per_count, 100u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, pid_sample_s, 104u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, nav_sample_mm, 108u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, nav_min_lookahead_mm, 112u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, encoder_ppr, 116u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, imu_flags, 118u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, imu_rate_hz, 120u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, parameter_words, 124u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, imu_count, 204u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, imu_drop_count, 206u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, imu_offset, 208u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, imu_record_size, 212u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, imu_capacity, 214u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, imu_sample_rate_hz, 216u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, imu_decimation, 218u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, control_log_divider, 220u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, control_log_rate_hz, 222u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, control_config, 224u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, localization_count, 240u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, localization_overwrite_count, 242u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, localization_offset, 244u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, localization_record_size, 248u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, localization_capacity, 250u);
TELEMETRY_OFFSET_CHECK(TelemetrySessionHeader, vision_first_frame_seq, 252u);

TELEMETRY_OFFSET_CHECK(TelemetryWirePrefix, magic, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryWirePrefix, protocol_version, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryWirePrefix, flags, 6u);
TELEMETRY_OFFSET_CHECK(TelemetryWirePrefix, payload_size, 8u);
TELEMETRY_OFFSET_CHECK(TelemetryWirePrefix, payload_crc32, 12u);

TELEMETRY_OFFSET_CHECK(TelemetryLinkBeacon, magic, 0u);
TELEMETRY_OFFSET_CHECK(TelemetryLinkBeacon, protocol_version, 4u);
TELEMETRY_OFFSET_CHECK(TelemetryLinkBeacon, flags, 6u);
TELEMETRY_OFFSET_CHECK(TelemetryLinkBeacon, payload_size, 8u);
TELEMETRY_OFFSET_CHECK(TelemetryLinkBeacon, session_sequence, 12u);

#undef TELEMETRY_OFFSET_CHECK

typedef char telemetry_control_record_size_check[
    (sizeof(TelemetryControlRecord) == 22u) ? 1 : -1];
typedef char telemetry_navigation_record_size_check[
    (sizeof(TelemetryNavigationRecord) == 18u) ? 1 : -1];
typedef char telemetry_control_config_size_check[
    (sizeof(TelemetryControlConfig) == 16u) ? 1 : -1];
typedef char telemetry_vision_record_size_check[
    (sizeof(TelemetryVisionRecord) == 10u) ? 1 : -1];
typedef char telemetry_event_record_size_check[
    (sizeof(TelemetryEventRecord) == 16u) ? 1 : -1];
typedef char telemetry_localization_record_size_check[
    (sizeof(TelemetryLocalizationRecord) == TELEMETRY_LOCALIZATION_RECORD_SIZE) ? 1 : -1];
typedef char telemetry_localization_feature_header_size_check[
    (sizeof(TelemetryLocalizationFeatureHeader) ==
     TELEMETRY_LOCALIZATION_FEATURE_HEADER_SIZE) ? 1 : -1];
typedef char telemetry_localization_feature_record_size_check[
    (sizeof(TelemetryLocalizationFeatureRecord) ==
     TELEMETRY_LOCALIZATION_FEATURE_RECORD_SIZE) ? 1 : -1];
typedef char telemetry_imu_record_size_check[
    (sizeof(TelemetryImuRecord) == TELEMETRY_IMU_RECORD_SIZE) ? 1 : -1];
typedef char telemetry_phototube_scan_record_size_check[
    (sizeof(TelemetryPhototubeScanRecord) == TELEMETRY_PHOTOTUBE_SCAN_RECORD_SIZE) ? 1 : -1];
typedef char telemetry_phototube_scan_header_size_check[
    (sizeof(TelemetryPhototubeScanHeader) == TELEMETRY_PHOTOTUBE_SCAN_HEADER_SIZE) ? 1 : -1];
typedef char telemetry_phototube_scan_region_check[
    ((TELEMETRY_PHOTOTUBE_SCAN_HEADER_SIZE +
      TELEMETRY_PHOTOTUBE_SCAN_CAPACITY * TELEMETRY_PHOTOTUBE_SCAN_RECORD_SIZE) <=
     TELEMETRY_PHOTOTUBE_SCAN_REGION_SIZE) ? 1 : -1];
typedef char telemetry_localization_config_block_size_check[
    (sizeof(TelemetryLocalizationConfigBlock) ==
     TELEMETRY_LOCALIZATION_CONFIG_SIZE) ? 1 : -1];
typedef char telemetry_tracking_config_block_size_check[
    (sizeof(TelemetryTrackingConfigBlock) ==
     TELEMETRY_TRACKING_CONFIG_SIZE) ? 1 : -1];
typedef char telemetry_localization_config_header_region_check[
    ((TELEMETRY_LOCALIZATION_CONFIG_OFFSET +
      sizeof(TelemetryLocalizationConfigBlock)) <=
     TELEMETRY_HEADER_REGION_SIZE) ? 1 : -1];
typedef char telemetry_localization_feature_header_region_check[
    ((TELEMETRY_LOCALIZATION_FEATURE_OFFSET +
      TELEMETRY_LOCALIZATION_FEATURE_HEADER_SIZE +
      TELEMETRY_LOCALIZATION_FEATURE_CAPACITY *
          TELEMETRY_LOCALIZATION_FEATURE_RECORD_SIZE) <=
     TELEMETRY_HEADER_REGION_SIZE) ? 1 : -1];
typedef char telemetry_localization_feature_tracking_contiguous_check[
    ((TELEMETRY_LOCALIZATION_FEATURE_OFFSET +
      TELEMETRY_LOCALIZATION_FEATURE_HEADER_SIZE +
      TELEMETRY_LOCALIZATION_FEATURE_CAPACITY *
          TELEMETRY_LOCALIZATION_FEATURE_RECORD_SIZE) ==
     TELEMETRY_TRACKING_CONFIG_OFFSET) ? 1 : -1];
typedef char telemetry_tracking_config_header_region_check[
    ((TELEMETRY_TRACKING_CONFIG_OFFSET +
      sizeof(TelemetryTrackingConfigBlock)) <=
     TELEMETRY_HEADER_REGION_SIZE) ? 1 : -1];
typedef char telemetry_localization_config_feature_contiguous_check[
    ((TELEMETRY_LOCALIZATION_CONFIG_OFFSET +
      TELEMETRY_LOCALIZATION_CONFIG_SIZE) ==
     TELEMETRY_LOCALIZATION_FEATURE_OFFSET) ? 1 : -1];
typedef char telemetry_session_header_size_check[
    (sizeof(TelemetrySessionHeader) == TELEMETRY_SESSION_HEADER_SIZE) ? 1 : -1];
typedef char telemetry_telemetry_image_layout_check[
    ((TELEMETRY_HEADER_OFFSET + TELEMETRY_HEADER_REGION_SIZE <= TELEMETRY_CONTROL_OFFSET) &&
     (TELEMETRY_CONTROL_OFFSET + TELEMETRY_CONTROL_REGION_SIZE <= TELEMETRY_NAVIGATION_OFFSET) &&
     (TELEMETRY_NAVIGATION_OFFSET + TELEMETRY_NAVIGATION_REGION_SIZE <= TELEMETRY_VISION_OFFSET) &&
     (TELEMETRY_VISION_OFFSET + TELEMETRY_VISION_REGION_SIZE <= TELEMETRY_EVENT_OFFSET) &&
     (TELEMETRY_EVENT_OFFSET + TELEMETRY_EVENT_REGION_SIZE <= TELEMETRY_IMU_OFFSET) &&
     (TELEMETRY_IMU_OFFSET + TELEMETRY_IMU_REGION_SIZE <= TELEMETRY_RESERVED_OFFSET) &&
     (TELEMETRY_RESERVED_OFFSET + TELEMETRY_RESERVED_REGION_SIZE <= TELEMETRY_EMEM_SIZE)) ? 1 : -1];
typedef char telemetry_wire_prefix_size_check[
    (sizeof(TelemetryWirePrefix) == 16u) ? 1 : -1];
typedef char telemetry_link_beacon_size_check[
    (sizeof(TelemetryLinkBeacon) == 16u) ? 1 : -1];
typedef char telemetry_memory_map_size_check[
    ((TELEMETRY_RESERVED_OFFSET + TELEMETRY_RESERVED_REGION_SIZE) == TELEMETRY_EMEM_SIZE) ? 1 : -1];
typedef char telemetry_header_control_contiguous_check[
    ((TELEMETRY_HEADER_OFFSET + TELEMETRY_HEADER_REGION_SIZE) ==
     TELEMETRY_CONTROL_OFFSET) ? 1 : -1];
typedef char telemetry_control_navigation_contiguous_check[
    ((TELEMETRY_CONTROL_OFFSET + TELEMETRY_CONTROL_REGION_SIZE) ==
     TELEMETRY_NAVIGATION_OFFSET) ? 1 : -1];
typedef char telemetry_navigation_vision_contiguous_check[
    ((TELEMETRY_NAVIGATION_OFFSET + TELEMETRY_NAVIGATION_REGION_SIZE) ==
     TELEMETRY_VISION_OFFSET) ? 1 : -1];
typedef char telemetry_vision_event_contiguous_check[
    ((TELEMETRY_VISION_OFFSET + TELEMETRY_VISION_REGION_SIZE) ==
     TELEMETRY_EVENT_OFFSET) ? 1 : -1];
typedef char telemetry_event_imu_contiguous_check[
    ((TELEMETRY_EVENT_OFFSET + TELEMETRY_EVENT_REGION_SIZE) ==
     TELEMETRY_IMU_OFFSET) ? 1 : -1];
typedef char telemetry_imu_reserved_contiguous_check[
    ((TELEMETRY_IMU_OFFSET + TELEMETRY_IMU_REGION_SIZE) ==
     TELEMETRY_RESERVED_OFFSET) ? 1 : -1];
typedef char telemetry_control_capacity_region_check[
    ((TELEMETRY_CONTROL_CAPACITY * sizeof(TelemetryControlRecord)) ==
     TELEMETRY_CONTROL_REGION_SIZE) ? 1 : -1];
typedef char telemetry_navigation_capacity_region_check[
    ((TELEMETRY_NAVIGATION_CAPACITY * sizeof(TelemetryNavigationRecord)) ==
     TELEMETRY_NAVIGATION_REGION_SIZE) ? 1 : -1];
typedef char telemetry_vision_capacity_region_check[
    ((TELEMETRY_VISION_CAPACITY * sizeof(TelemetryVisionRecord)) ==
     TELEMETRY_VISION_REGION_SIZE) ? 1 : -1];
typedef char telemetry_event_capacity_region_check[
    ((TELEMETRY_EVENT_CAPACITY * sizeof(TelemetryEventRecord)) ==
     TELEMETRY_EVENT_REGION_SIZE) ? 1 : -1];
typedef char telemetry_localization_capacity_region_check[
    ((TELEMETRY_LOCALIZATION_OFFSET +
      TELEMETRY_LOCALIZATION_CAPACITY * TELEMETRY_LOCALIZATION_RECORD_SIZE) <=
     (TELEMETRY_RESERVED_OFFSET + TELEMETRY_RESERVED_REGION_SIZE)) ? 1 : -1];
typedef char telemetry_localization_scan_contiguous_check[
    (TELEMETRY_PHOTOTUBE_SCAN_OFFSET ==
     (TELEMETRY_LOCALIZATION_OFFSET +
      TELEMETRY_LOCALIZATION_CAPACITY * TELEMETRY_LOCALIZATION_RECORD_SIZE)) ? 1 : -1];
typedef char telemetry_imu_capacity_region_check[
    ((TELEMETRY_IMU_CAPACITY * sizeof(TelemetryImuRecord)) <=
     TELEMETRY_IMU_REGION_SIZE) ? 1 : -1];
typedef char telemetry_imu_rate_decimation_check[
    ((TELEMETRY_IMU_SAMPLE_HZ * TELEMETRY_IMU_DECIMATION) ==
     TELEMETRY_IMU_SOURCE_SAMPLE_HZ) ? 1 : -1];
typedef char telemetry_vision_rate_decimation_check[
    ((TELEMETRY_VISION_LOG_RATE_HZ * TELEMETRY_VISION_LOG_DIVIDER) ==
     TELEMETRY_VISION_SOURCE_SAMPLE_HZ) ? 1 : -1];

uint32_t telemetry_crc32_update(uint32_t state, const void *data, uint32_t size);
int16_t telemetry_float_to_i16(float value, float scale);
uint16_t telemetry_float_to_u16(float value, float scale);
int16_t telemetry_yaw_to_cdeg(float yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* CODE_TELEMETRY_FORMAT_H_ */
