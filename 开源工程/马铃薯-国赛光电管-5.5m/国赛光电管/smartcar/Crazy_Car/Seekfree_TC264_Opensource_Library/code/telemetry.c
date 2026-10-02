/**
 * @file telemetry.c
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

#include "telemetry.h"
#include "competition_config.h"

#include <string.h>

#include "control.h"
#include "flash.h"
#include "globals.h"
#include "gyroscope.h"
#include "navigation.h"
#include "phototube_scan.h"
#include "prestart.h"
#include "phototube_array.h"
#include "Emem/Std/IfxEmem.h"
#include "IfxCbs_reg.h"
#include "IfxScu_reg.h"
#include "IfxScuWdt.h"

#define TELEMETRY_CONTROL_QUEUE_CAPACITY (64u)
#define TELEMETRY_NAVIGATION_QUEUE_CAPACITY (16u)
#define TELEMETRY_VISION_QUEUE_CAPACITY (32u)
#define TELEMETRY_EVENT_QUEUE_CAPACITY (16u)
#define TELEMETRY_PHOTOTUBE_SCAN_QUEUE_CAPACITY (256u)
#define TELEMETRY_LOCALIZATION_QUEUE_CAPACITY (16u)
#define TELEMETRY_EVENT_QUEUE_CRITICAL_RESERVE (8u)
#define TELEMETRY_EVENT_STORAGE_CRITICAL_RESERVE (10u)
#define TELEMETRY_RUNTIME_LEDGER_EVENT_COUNT (12u)
#define TELEMETRY_DROP_SUMMARY_MAX_EVENT_COUNT (10u)
#define TELEMETRY_TERMINAL_EVENT_COUNT \
    (TELEMETRY_RUNTIME_LEDGER_EVENT_COUNT + 1u)
#define TELEMETRY_EVENT_STORAGE_FINAL_RESERVE \
    (TELEMETRY_DROP_SUMMARY_MAX_EVENT_COUNT + TELEMETRY_TERMINAL_EVENT_COUNT)
#define TELEMETRY_CONTROL_DRAIN_LIMIT (16u)
#define TELEMETRY_NAVIGATION_DRAIN_LIMIT (4u)
#define TELEMETRY_VISION_DRAIN_LIMIT (8u)
#define TELEMETRY_EVENT_DRAIN_LIMIT (4u)
#define TELEMETRY_IMU_DRAIN_LIMIT (16u)
#define TELEMETRY_PHOTOTUBE_SCAN_DRAIN_LIMIT (16u)
#define TELEMETRY_LOCALIZATION_DRAIN_LIMIT (4u)
#define TELEMETRY_SERVICE_PHASE_COUNT (7u)
#define TELEMETRY_SERVICE_SOFT_BUDGET_US (500u)
/* Export only runs while the vehicle is fully idle.  Literal blocks stay
 * small enough for bounded scanning; long cleared EMEM regions collapse into
 * two-byte zero-run headers on the wire. */
#define TELEMETRY_EXPORT_CHUNK_BYTES (256u)
#define TELEMETRY_EXPORT_ZERO_MIN_RUN (8u)
#define TELEMETRY_LINK_BEACON_INTERVAL_MS (500u)
#define TELEMETRY_IMU_PRODUCER_WAIT_LIMIT (10000u)
#define TELEMETRY_SELF_TEST_OFFSET (TELEMETRY_EMEM_SIZE - 32u)
/* CHID is bits [15:8] of the complete TC264DA CHIPID 0x05476852. */
#define TELEMETRY_EMEM_REQUIRED_CHIP_ID (0x68u)
#define TELEMETRY_EMEM_TILESTATE_MASK (0x0000FFFFu)

#if TELEMETRY_EMEM_SIZE > IFXEMEM_SIZE
#error "Telemetry image exceeds the EMEM size supplied by the iLLD"
#endif

#if (TELEMETRY_EMEM_SIZE % 32u) != 0u
#error "Telemetry EMEM image must contain complete 32-byte ECC lines"
#endif

#if (TELEMETRY_SELF_TEST_OFFSET % 32u) != 0u
#error "Telemetry self-test must start on a 32-byte ECC-line boundary"
#endif

#if (TELEMETRY_SELF_TEST_OFFSET + 32u) > TELEMETRY_EMEM_SIZE
#error "Telemetry self-test line exceeds the logical EMEM image"
#endif

#if (TELEMETRY_PHOTOTUBE_SCAN_OFFSET + TELEMETRY_PHOTOTUBE_SCAN_HEADER_SIZE + \
     TELEMETRY_PHOTOTUBE_SCAN_CAPACITY * TELEMETRY_PHOTOTUBE_SCAN_RECORD_SIZE) > \
    TELEMETRY_SELF_TEST_OFFSET
#error "Phototube archive overlaps the reserved EMEM self-test line"
#endif

#if TELEMETRY_EVENT_QUEUE_CRITICAL_RESERVE >= (TELEMETRY_EVENT_QUEUE_CAPACITY - 1u)
#error "Telemetry event queue reserve leaves no room for ordinary events"
#endif

#if TELEMETRY_EVENT_STORAGE_FINAL_RESERVE >= TELEMETRY_EVENT_CAPACITY
#error "Telemetry final event reserve exceeds event capacity"
#endif

#if TELEMETRY_EVENT_STORAGE_CRITICAL_RESERVE >= \
    (TELEMETRY_EVENT_CAPACITY - TELEMETRY_EVENT_STORAGE_FINAL_RESERVE)
#error "Telemetry active critical reserve exceeds event capacity"
#endif

#if TELEMETRY_IMU_DECIMATION == 0u
#error "Telemetry IMU decimation must be greater than zero"
#endif

#if ((TELEMETRY_PHOTOTUBE_SCAN_QUEUE_CAPACITY & \
      (TELEMETRY_PHOTOTUBE_SCAN_QUEUE_CAPACITY - 1u)) != 0u)
#error "Phototube telemetry queue capacity must be a power of two"
#endif

#if PHOTOTUBE_ARRAY_SCAN_HZ != TELEMETRY_VISION_SOURCE_SAMPLE_HZ
#error "Phototube acquisition and telemetry source rates must match"
#endif

#if TELEMETRY_EXPORT_CHUNK_BYTES > TELEMETRY_WIRE_BLOCK_MAX_LENGTH
#error "Telemetry export literal block exceeds the wire-format limit"
#endif

#if TELEMETRY_EXPORT_ZERO_MIN_RUN == 0u
#error "Telemetry export zero-run threshold must be greater than zero"
#endif

typedef union
{
    float value;
    uint32 word;
} TelemetryFloatWord;

/* CPU1-private compression/decimation state.  The cross-core SPSC mailbox
 * itself is declared centrally in globals.c/.h. */
#pragma section all "cpu1_dsram"

static uint32 imu_last_timestamp_4us;
static uint8 imu_have_timestamp;
static uint8 imu_log_decimation_countdown;
static uint32 imu_telemetry_local_epoch_seq;
static uint32 imu_telemetry_local_start_us;

#pragma section all restore
#pragma section all "cpu0_dsram"

static volatile TelemetryControlRecord control_queue[TELEMETRY_CONTROL_QUEUE_CAPACITY];
static volatile TelemetryNavigationRecord navigation_queue[TELEMETRY_NAVIGATION_QUEUE_CAPACITY];
static volatile TelemetryVisionRecord vision_queue[TELEMETRY_VISION_QUEUE_CAPACITY];
static volatile TelemetryEventRecord event_queue[TELEMETRY_EVENT_QUEUE_CAPACITY];
static TelemetryPhototubeScanRecord phototube_scan_queue[
    TELEMETRY_PHOTOTUBE_SCAN_QUEUE_CAPACITY];
static TelemetryLocalizationRecord localization_queue[
    TELEMETRY_LOCALIZATION_QUEUE_CAPACITY];
static TelemetryLocalizationFeatureRecord localization_feature_queue[
    TELEMETRY_LOCALIZATION_QUEUE_CAPACITY];

static volatile uint8 control_head;
static volatile uint8 control_tail;
static volatile uint8 navigation_head;
static volatile uint8 navigation_tail;
static volatile uint8 vision_head;
static volatile uint8 vision_tail;
static volatile uint8 event_head;
static volatile uint8 event_tail;
/* IMU records are produced by CPU1 and drained by CPU0.  Keep producer-side
 * drops separate from the CPU0-owned EMEM capacity counter so the two cores
 * never update the same session-header field concurrently. */
static uint16 phototube_scan_queue_head;
static uint16 phototube_scan_queue_tail;
static uint8 localization_queue_head;
static uint8 localization_queue_tail;
static uint8 localization_feature_queue_head;
static uint8 localization_feature_queue_tail;
static uint16 imu_storage_drop_count;

static volatile uint8 logger_enabled;
static volatile uint8 logger_accepting;
static volatile uint8 logger_finalized;
static volatile uint8 export_requested;
static volatile TelemetryState logger_state = TELEMETRY_STATE_DISABLED;
static uint16 position_saturation_count;
static volatile uint32 session_sequence;
static uint8 vision_log_decimation_countdown;
static uint8 vision_have_first_frame_seq;
static uint8 telemetry_service_phase;
static TelemetrySessionHeader session_header;
static TelemetryLocalizationConfigBlock localization_config_block;
static TelemetryTrackingConfigBlock tracking_config_block;
static TelemetryLocalizationFeatureHeader localization_feature_header;
static TelemetryWirePrefix wire_prefix;
static uint32 firmware_image_hash;
static uint32 export_prefix_offset;
static uint32 export_payload_offset;
static uint32 export_session_sequence;
static uint32 telemetry_link_last_ms;
static uint16 phototube_scan_archive_count;
static uint16 phototube_scan_archive_write_index;
static uint32 phototube_scan_archive_first_frame_seq;
static uint16 phototube_scan_archive_overwrite_count;
static uint16 phototube_scan_queue_drop_count;
static uint16 localization_queue_drop_count;
static uint8 phototube_scan_archive_have_first_frame;
static uint8 phototube_scan_header_finalized;
static const uint8 invalid_magic[4] = {0u, 0u, 0u, 0u};

static volatile uint8 *const telemetry_emem = (volatile uint8 *)IFXEMEM_START_ADDR_CPU;

static uint8 telemetry_next_index(uint8 index, uint8 capacity)
{
    index++;
    return (index >= capacity) ? 0u : index;
}

static void telemetry_increment_drop(volatile uint16_t *counter)
{
    if (*counter < 0xFFFFu)
    {
        (*counter)++;
    }
}

static void telemetry_emem_write(uint32 offset, const void *source, uint32 size)
{
    const uint8 *bytes = (const uint8 *)source;
    uint32 index;

    if ((offset > TELEMETRY_EMEM_SIZE) || (size > (TELEMETRY_EMEM_SIZE - offset)))
    {
        return;
    }
    for (index = 0u; index < size; index++)
    {
        telemetry_emem[offset + index] = bytes[index];
    }
    __dsync();
}

static void telemetry_emem_read(uint32 offset, void *destination, uint32 size)
{
    uint8 *bytes = (uint8 *)destination;
    uint32 index;

    if ((NULL == destination) || (offset > TELEMETRY_EMEM_SIZE) ||
        (size > (TELEMETRY_EMEM_SIZE - offset)))
    {
        return;
    }
    __dsync();
    for (index = 0u; index < size; index++)
    {
        bytes[index] = telemetry_emem[offset + index];
    }
}

static void telemetry_emem_clear(void)
{
    uint32 offset;

    /*
     * EMEM ECC is line based.  A byte/word write to a virgin line can perform
     * an internal read-modify-write and raise an ECC trap.  Start every
     * 32-byte line on its alignment boundary and issue four consecutive
     * 64-bit stores, covering both ECC halves without a preceding read.
     */
    for (offset = 0u; offset < TELEMETRY_EMEM_SIZE; offset += 32u)
    {
        volatile uint64 *line_words =
            (volatile uint64 *)(IFXEMEM_START_ADDR_CPU + offset);
        line_words[0] = 0u;
        line_words[1] = 0u;
        line_words[2] = 0u;
        line_words[3] = 0u;
    }
    __dsync();
}

static uint8 telemetry_emem_self_test(void)
{
    static const uint32 patterns[8] = {
        0x55AA33CCu,
        0xA55AC33Cu,
        0x01234567u,
        0x89ABCDEFu,
        0x13579BDFu,
        0x2468ACE0u,
        0x0F0FF0F0u,
        0xF0F00F0Fu};
    volatile uint32 *test_words =
        (volatile uint32 *)(IFXEMEM_START_ADDR_CPU + TELEMETRY_SELF_TEST_OFFSET);
    uint8 passed = 1u;
    uint8 index;

    for (index = 0u; index < 8u; index++)
    {
        test_words[index] = patterns[index];
    }
    __dsync();
    for (index = 0u; index < 8u; index++)
    {
        if (test_words[index] != patterns[index])
        {
            passed = 0u;
        }
        test_words[index] = 0u;
    }
    __dsync();
    return passed;
}

static void telemetry_reset_queues(void)
{
    control_head = 0u;
    control_tail = 0u;
    navigation_head = 0u;
    navigation_tail = 0u;
    vision_head = 0u;
    vision_tail = 0u;
    event_head = 0u;
    event_tail = 0u;
    imu_telemetry_head = 0u;
    imu_telemetry_tail = 0u;
    phototube_scan_queue_head = 0u;
    phototube_scan_queue_tail = 0u;
    localization_queue_head = 0u;
    localization_queue_tail = 0u;
    localization_feature_queue_head = 0u;
    localization_feature_queue_tail = 0u;
    imu_storage_drop_count = 0u;
}

static uint8 telemetry_wait_imu_producer_idle(void)
{
    uint32 wait_count = 0u;

    /* Freeze/begin-run is called from CPU0 context.  The CPU1 IMU ISR is
     * bounded and clears this flag on every exit.  Keep a guard anyway: a
     * broken peripheral must never make the CPU0 menu spin forever. */
    while (imu_telemetry_producer_active &&
           (wait_count++ < TELEMETRY_IMU_PRODUCER_WAIT_LIMIT))
    {
        __dsync();
    }
    return (uint8)(0u == imu_telemetry_producer_active);
}

static void telemetry_merge_imu_producer_drops(void)
{
    uint16 producer_drops;
    uint32 total_drops;

    __dsync();
    producer_drops =
        (imu_telemetry_producer_epoch_seq == imu_telemetry_epoch_seq) ?
            imu_telemetry_producer_drop_count : 0u;
    total_drops = (uint32)producer_drops + (uint32)imu_storage_drop_count;
    session_header.imu_drop_count = (uint16)((total_drops > 0xFFFFu) ?
                                             0xFFFFu : total_drops);
}

static void telemetry_store_float_word(uint8 index, float value)
{
    TelemetryFloatWord converted;

    converted.value = value;
    session_header.parameter_words[index] = converted.word;
}

static void telemetry_snapshot_parameters(void)
{
    /* Keep the fixed schema-21 parameter layout for host compatibility, but
     * publish zeroes for the retired motor-feedforward/anti-reversal slots.
     * The active firmware path is the pure incremental PI from the pre-race
     * finish version. */
    telemetry_store_float_word(0u, 0.0f);
    telemetry_store_float_word(1u, 0.0f);
    telemetry_store_float_word(2u, 0.0f);
    telemetry_store_float_word(3u, 0.0f);
    telemetry_store_float_word(4u, 0.0f);
    telemetry_store_float_word(5u, 0.0f);
    telemetry_store_float_word(6u, 0.0f);
    telemetry_store_float_word(7u, 0.0f);
    telemetry_store_float_word(8u, 0.0f);
    session_header.parameter_words[9] = 0u;
    telemetry_store_float_word(10u, g_flash_parameters.speed_KP);
    telemetry_store_float_word(11u, g_flash_parameters.speed_KI);
    telemetry_store_float_word(12u, 0.0f);
    telemetry_store_float_word(13u, g_flash_parameters.speed_IncrementLimit);
    telemetry_store_float_word(14u, 0.0f);
    telemetry_store_float_word(15u, nav_get_speed_profile_max_accel_cmps2());
    telemetry_store_float_word(16u, nav_get_speed_profile_max_decel_cmps2());
    telemetry_store_float_word(17u, nav_get_speed_profile_safety_margin_mm());
    telemetry_store_float_word(18u, 0.0f);
    telemetry_store_float_word(19u, 0.0f);
}

static uint32 telemetry_fnv1a_update(uint32 hash, const void *data, uint32 size)
{
    const uint8 *bytes = (const uint8 *)data;
    uint32 index;

    for (index = 0u; index < size; index++)
    {
        hash ^= bytes[index];
        hash *= 16777619u;
    }
    return hash;
}

static uint32 telemetry_calculate_firmware_config_fingerprint(
    const TelemetryLocalizationConfigBlock *localization_block,
    const TelemetryTrackingConfigBlock *tracking_block)
{
    const uint32 layout_constants[] = {
        TELEMETRY_SCHEMA_VERSION,
        TELEMETRY_PROTOCOL_VERSION,
        TELEMETRY_EMEM_SIZE,
        TELEMETRY_HEADER_REGION_SIZE,
        TELEMETRY_HEADER_CRC_OFFSET,
        TELEMETRY_LOCALIZATION_CONFIG_OFFSET,
        TELEMETRY_LOCALIZATION_CONFIG_SIZE,
        TELEMETRY_LOCALIZATION_CONFIG_VERSION,
        TELEMETRY_LOCALIZATION_FEATURE_OFFSET,
        TELEMETRY_LOCALIZATION_FEATURE_HEADER_SIZE,
        TELEMETRY_LOCALIZATION_FEATURE_RECORD_SIZE,
        TELEMETRY_LOCALIZATION_FEATURE_CAPACITY,
        TELEMETRY_LOCALIZATION_FEATURE_VERSION,
        TELEMETRY_TRACKING_CONFIG_OFFSET,
        TELEMETRY_TRACKING_CONFIG_SIZE,
        TELEMETRY_TRACKING_CONFIG_VERSION,
        TELEMETRY_CONTROL_OFFSET,
        TELEMETRY_CONTROL_REGION_SIZE,
        TELEMETRY_NAVIGATION_OFFSET,
        TELEMETRY_NAVIGATION_REGION_SIZE,
        TELEMETRY_VISION_OFFSET,
        TELEMETRY_VISION_REGION_SIZE,
        TELEMETRY_EVENT_OFFSET,
        TELEMETRY_EVENT_REGION_SIZE,
        TELEMETRY_LOCALIZATION_OFFSET,
        TELEMETRY_LOCALIZATION_RECORD_SIZE,
        TELEMETRY_LOCALIZATION_CAPACITY,
        TELEMETRY_RESERVED_OFFSET,
        TELEMETRY_RESERVED_REGION_SIZE,
        TELEMETRY_SESSION_HEADER_SIZE,
        TELEMETRY_PARAMETER_WORD_COUNT,
        TELEMETRY_CONTROL_CAPACITY,
        TELEMETRY_NAVIGATION_CAPACITY,
        TELEMETRY_VISION_CAPACITY,
        TELEMETRY_EVENT_CAPACITY,
        sizeof(TelemetryControlRecord),
        sizeof(TelemetryNavigationRecord),
        sizeof(TelemetryVisionRecord),
        sizeof(TelemetryEventRecord),
        sizeof(TelemetryLocalizationRecord),
        sizeof(TelemetryLocalizationFeatureHeader),
        sizeof(TelemetryLocalizationFeatureRecord),
        sizeof(TelemetryTrackingConfigBlock),
        sizeof(TelemetryImuRecord),
        TELEMETRY_IMU_OFFSET,
        TELEMETRY_IMU_REGION_SIZE,
        TELEMETRY_IMU_CAPACITY,
        TELEMETRY_IMU_SOURCE_SAMPLE_HZ,
        TELEMETRY_IMU_SAMPLE_HZ,
        TELEMETRY_IMU_DECIMATION,
        TELEMETRY_CONTROL_LOG_DIVIDER,
        TELEMETRY_CONTROL_LOG_RATE_HZ,
        TELEMETRY_VISION_SOURCE_SAMPLE_HZ,
        TELEMETRY_VISION_LOG_DIVIDER,
        TELEMETRY_VISION_LOG_RATE_HZ,
        TELEMETRY_PHOTOTUBE_SCAN_OFFSET,
        TELEMETRY_PHOTOTUBE_SCAN_REGION_SIZE,
        TELEMETRY_PHOTOTUBE_SCAN_HEADER_SIZE,
        TELEMETRY_PHOTOTUBE_SCAN_RECORD_SIZE,
        TELEMETRY_PHOTOTUBE_SCAN_CAPACITY,
        sizeof(TelemetryPhototubeScanHeader),
        sizeof(TelemetryPhototubeScanRecord)};
    uint32 hash = 2166136261u;

    hash = telemetry_fnv1a_update(hash, &firmware_image_hash,
                                   sizeof(firmware_image_hash));
    hash = telemetry_fnv1a_update(hash, layout_constants, sizeof(layout_constants));
    hash = telemetry_fnv1a_update(hash, &session_header.route_hash,
                                   sizeof(session_header.route_hash));
    hash = telemetry_fnv1a_update(hash, &session_header.route_point_count,
                                   sizeof(session_header.route_point_count));
    hash = telemetry_fnv1a_update(hash, &session_header.wheel_diameter_mm,
                                   sizeof(session_header.wheel_diameter_mm));
    hash = telemetry_fnv1a_update(hash, &session_header.gear_ratio,
                                   sizeof(session_header.gear_ratio));
    hash = telemetry_fnv1a_update(hash, &session_header.distance_per_pulse_mm,
                                   sizeof(session_header.distance_per_pulse_mm));
    hash = telemetry_fnv1a_update(hash, &session_header.cmps_per_count,
                                   sizeof(session_header.cmps_per_count));
    hash = telemetry_fnv1a_update(hash, &session_header.pid_sample_s,
                                   sizeof(session_header.pid_sample_s));
    hash = telemetry_fnv1a_update(hash, &session_header.nav_sample_mm,
                                   sizeof(session_header.nav_sample_mm));
    hash = telemetry_fnv1a_update(hash, &session_header.nav_min_lookahead_mm,
                                   sizeof(session_header.nav_min_lookahead_mm));
    hash = telemetry_fnv1a_update(hash, &session_header.encoder_ppr,
                                   sizeof(session_header.encoder_ppr));
    hash = telemetry_fnv1a_update(hash, &session_header.imu_flags,
                                   sizeof(session_header.imu_flags));
    hash = telemetry_fnv1a_update(hash, &session_header.imu_rate_hz,
                                   sizeof(session_header.imu_rate_hz));
    hash = telemetry_fnv1a_update(
        hash, &session_header.control_config.preview_time_s,
        sizeof(session_header.control_config.preview_time_s));
    hash = telemetry_fnv1a_update(
        hash, &session_header.control_config.min_lookahead_mm,
        sizeof(session_header.control_config.min_lookahead_mm));
    hash = telemetry_fnv1a_update(
        hash, &session_header.control_config.max_lookahead_mm,
        sizeof(session_header.control_config.max_lookahead_mm));
    hash = telemetry_fnv1a_update(
        hash, &session_header.control_config.kd_cmps_per_degps_per_speed_scale,
        sizeof(session_header.control_config.kd_cmps_per_degps_per_speed_scale));
    hash = telemetry_fnv1a_update(hash, session_header.parameter_words,
                                   sizeof(session_header.parameter_words));
    hash = telemetry_fnv1a_update(hash, &localization_block->config_hash,
                                   sizeof(localization_block->config_hash));
    hash = telemetry_fnv1a_update(hash, &localization_block->algorithm_version,
                                   sizeof(localization_block->algorithm_version));
    hash = telemetry_fnv1a_update(hash, &tracking_block->config_hash,
                                   sizeof(tracking_block->config_hash));
    hash = telemetry_fnv1a_update(hash, &tracking_block->algorithm_version,
                                   sizeof(tracking_block->algorithm_version));
    return hash;
}

static void telemetry_fill_header_layout(void)
{
    session_header.magic[0] = TELEMETRY_SESSION_MAGIC_0;
    session_header.magic[1] = TELEMETRY_SESSION_MAGIC_1;
    session_header.magic[2] = TELEMETRY_SESSION_MAGIC_2;
    session_header.magic[3] = TELEMETRY_SESSION_MAGIC_3;
    session_header.schema_version = TELEMETRY_SCHEMA_VERSION;
    session_header.header_size = TELEMETRY_SESSION_HEADER_SIZE;
    session_header.init_status = 1u;
    session_header.control_offset = TELEMETRY_CONTROL_OFFSET;
    session_header.navigation_offset = TELEMETRY_NAVIGATION_OFFSET;
    session_header.vision_offset = TELEMETRY_VISION_OFFSET;
    session_header.event_offset = TELEMETRY_EVENT_OFFSET;
    session_header.control_record_size = sizeof(TelemetryControlRecord);
    session_header.navigation_record_size = sizeof(TelemetryNavigationRecord);
    session_header.vision_record_size = sizeof(TelemetryVisionRecord);
    session_header.event_record_size = sizeof(TelemetryEventRecord);
    session_header.control_capacity = TELEMETRY_CONTROL_CAPACITY;
    session_header.navigation_capacity = TELEMETRY_NAVIGATION_CAPACITY;
    session_header.vision_capacity = TELEMETRY_VISION_CAPACITY;
    session_header.event_capacity = TELEMETRY_EVENT_CAPACITY;
    session_header.localization_offset = TELEMETRY_LOCALIZATION_OFFSET;
    session_header.localization_record_size = sizeof(TelemetryLocalizationRecord);
    session_header.localization_capacity = TELEMETRY_LOCALIZATION_CAPACITY;
    session_header.imu_offset = TELEMETRY_IMU_OFFSET;
    session_header.imu_record_size = sizeof(TelemetryImuRecord);
    session_header.imu_capacity = TELEMETRY_IMU_CAPACITY;
    session_header.imu_sample_rate_hz = TELEMETRY_IMU_SAMPLE_HZ;
    session_header.imu_decimation = TELEMETRY_IMU_DECIMATION;
    session_header.control_log_divider = TELEMETRY_CONTROL_LOG_DIVIDER;
    session_header.control_log_rate_hz = TELEMETRY_CONTROL_LOG_RATE_HZ;
    session_header.control_config.preview_time_s = NAV_PP_PREVIEW_TIME_S;
    session_header.control_config.min_lookahead_mm =
        NAV_PP_MIN_LOOKAHEAD_MM;
    session_header.control_config.max_lookahead_mm =
        NAV_PP_MAX_LOOKAHEAD_MM;
    /* v17 reuses the fixed header layout: legacy KD is unused. */
    session_header.control_config.kd_cmps_per_degps_per_speed_scale = 0.0f;
    session_header.wheel_diameter_mm = WHEEL_DIAMETER_MM;
    session_header.gear_ratio = GEAR_RATIO;
    session_header.distance_per_pulse_mm = DIST_PER_PULSE_MM;
    session_header.cmps_per_count = CMPS_PER_COUNT;
    session_header.pid_sample_s = PID_SAMPLE_S;
    session_header.nav_sample_mm = nav_get_path_sample_mm();
    session_header.nav_min_lookahead_mm = NAV_PP_MIN_LOOKAHEAD_MM;
    session_header.encoder_ppr = ENCODER_PPR;
    session_header.imu_flags =
        (uint16_t)(gyro_zero_calibration_done() ? 1u : 0u) |
        (uint16_t)(gyro_zero_calibration_running() ? 2u : 0u);
    session_header.imu_rate_hz = (float)TELEMETRY_IMU_SOURCE_SAMPLE_HZ;
}

static void telemetry_snapshot_localization_config(void)
{
    NavLocalizationConfigSnapshot config;

    nav_get_localization_config_snapshot(&config);
    localization_config_block.magic[0] =
        TELEMETRY_LOCALIZATION_CONFIG_MAGIC_0;
    localization_config_block.magic[1] =
        TELEMETRY_LOCALIZATION_CONFIG_MAGIC_1;
    localization_config_block.magic[2] =
        TELEMETRY_LOCALIZATION_CONFIG_MAGIC_2;
    localization_config_block.magic[3] =
        TELEMETRY_LOCALIZATION_CONFIG_MAGIC_3;
    localization_config_block.version =
        TELEMETRY_LOCALIZATION_CONFIG_VERSION;
    localization_config_block.size = TELEMETRY_LOCALIZATION_CONFIG_SIZE;
    localization_config_block.config_hash = nav_get_localization_config_hash();
    localization_config_block.algorithm_version = config.algorithm_version;
    localization_config_block.sensor_count = config.sensor_count;
    localization_config_block.adc_group = config.adc_group;
    localization_config_block.adc_channel = config.adc_channel;
    localization_config_block.mux_selector_count = config.mux_selector_count;
    localization_config_block.scan_hz = config.scan_hz;
    localization_config_block.sensor_spacing_mm = config.sensor_spacing_mm;
    localization_config_block.sensor_forward_mm = config.sensor_forward_mm;
    localization_config_block.scan_min_step_mm = config.scan_min_step_mm;
    localization_config_block.match_travel_window_mm =
        config.match_travel_window_mm;
    localization_config_block.feature_radius_mm = config.feature_radius_mm;
    localization_config_block.max_feature_point_residual_mm =
        config.max_feature_point_residual_mm;
    localization_config_block.max_rms_mm = config.max_rms_mm;
    localization_config_block.max_correction_mm = config.max_correction_mm;
    localization_config_block.max_correction_yaw_deg =
        config.max_correction_yaw_deg;
    localization_config_block.black_scan_stop_distance_mm =
        config.black_stop_distance_mm;
}

static void telemetry_snapshot_tracking_config(void)
{
    uint32 hash;

    memset(&tracking_config_block, 0, sizeof(tracking_config_block));
    tracking_config_block.magic[0] = TELEMETRY_TRACKING_CONFIG_MAGIC_0;
    tracking_config_block.magic[1] = TELEMETRY_TRACKING_CONFIG_MAGIC_1;
    tracking_config_block.magic[2] = TELEMETRY_TRACKING_CONFIG_MAGIC_2;
    tracking_config_block.magic[3] = TELEMETRY_TRACKING_CONFIG_MAGIC_3;
    tracking_config_block.version = TELEMETRY_TRACKING_CONFIG_VERSION;
    tracking_config_block.size = TELEMETRY_TRACKING_CONFIG_SIZE;
    tracking_config_block.algorithm_version =
        NAV_TRACKING_REFERENCE_ALGORITHM_VERSION;
    tracking_config_block.wheel_track_mm = DRIVE_WHEEL_TRACK_MM;
    tracking_config_block.preview_time_s = NAV_PP_PREVIEW_TIME_S;
    tracking_config_block.min_lookahead_mm = NAV_PP_MIN_LOOKAHEAD_MM;
    tracking_config_block.max_lookahead_mm = NAV_PP_MAX_LOOKAHEAD_MM;
    /* TRK1 v2 layout is fixed.  In v17 the legacy KD slot is zero, the
     * former GKD field carries K_omega, and target_rate_window_ms carries
     * the delayed curvature-feedforward preview time. */
    tracking_config_block.kd_cmps_per_degps_per_speed_scale = 0.0f;
    tracking_config_block.gkd_cmps_per_radps =
        NAV_TRACK_YAW_RATE_FB_CMPS_PER_RADPS;
    tracking_config_block.turn_limit_ratio = NAV_TURN_LIMIT_RATIO;
    tracking_config_block.path_sample_mm_x100 =
        (uint16)(nav_get_path_sample_mm() * 100.0f + 0.5f);
    tracking_config_block.target_rate_window_ms =
        (NAV_TRACKING_REFERENCE_ALGORITHM_VERSION >= 17u) ?
        (uint16)(NAV_TRACK_CURVATURE_PREVIEW_TIME_S * 1000.0f + 0.5f) : 0u;

    hash = 2166136261u;
    hash = telemetry_fnv1a_update(
        hash, &tracking_config_block.algorithm_version,
        (uint32)sizeof(tracking_config_block) -
            (uint32)offsetof(TelemetryTrackingConfigBlock, algorithm_version));
    tracking_config_block.config_hash = hash;
}

static void telemetry_initialize_localization_features(void)
{
    TelemetryLocalizationFeatureRecord record;
    uint16 occurrence;
    uint16 feature_count = nav_get_scan_feature_count();

    memset(&localization_feature_header, 0,
           sizeof(localization_feature_header));
    localization_feature_header.magic[0] =
        TELEMETRY_LOCALIZATION_FEATURE_MAGIC_0;
    localization_feature_header.magic[1] =
        TELEMETRY_LOCALIZATION_FEATURE_MAGIC_1;
    localization_feature_header.magic[2] =
        TELEMETRY_LOCALIZATION_FEATURE_MAGIC_2;
    localization_feature_header.magic[3] =
        TELEMETRY_LOCALIZATION_FEATURE_MAGIC_3;
    localization_feature_header.version =
        TELEMETRY_LOCALIZATION_FEATURE_VERSION;
    localization_feature_header.header_size =
        TELEMETRY_LOCALIZATION_FEATURE_HEADER_SIZE;
    localization_feature_header.record_size =
        TELEMETRY_LOCALIZATION_FEATURE_RECORD_SIZE;
    localization_feature_header.capacity =
        TELEMETRY_LOCALIZATION_FEATURE_CAPACITY;

    memset(&record, 0, sizeof(record));
    for (occurrence = 0u;
         occurrence < TELEMETRY_LOCALIZATION_FEATURE_CAPACITY;
         occurrence++)
    {
        telemetry_emem_write(
            TELEMETRY_LOCALIZATION_FEATURE_OFFSET +
                TELEMETRY_LOCALIZATION_FEATURE_HEADER_SIZE +
                (uint32)occurrence * sizeof(record),
            &record, sizeof(record));
    }

    if (feature_count > TELEMETRY_LOCALIZATION_FEATURE_CAPACITY)
    {
        feature_count = TELEMETRY_LOCALIZATION_FEATURE_CAPACITY;
    }
    for (occurrence = 0u; occurrence < feature_count; occurrence++)
    {
        uint8 feature_type;
        uint16 feature_progress_points;

        if (!nav_get_scan_feature_info(occurrence, &feature_type,
                                       &feature_progress_points) ||
            (0u == feature_type))
        {
            continue;
        }
        memset(&record, 0, sizeof(record));
        record.route_feature_occurrence = (uint8)occurrence;
        record.status_flags = TELEMETRY_LOCALIZATION_FEATURE_REGISTERED;
        (void)feature_progress_points;
        telemetry_emem_write(
            TELEMETRY_LOCALIZATION_FEATURE_OFFSET +
                TELEMETRY_LOCALIZATION_FEATURE_HEADER_SIZE +
                (uint32)occurrence * sizeof(record),
            &record, sizeof(record));
        localization_feature_header.registered_count++;
    }
    telemetry_emem_write(TELEMETRY_LOCALIZATION_FEATURE_OFFSET,
                         &localization_feature_header,
                         sizeof(localization_feature_header));
}

static uint8 telemetry_event_queue_count(uint8 head, uint8 tail)
{
    if (head >= tail)
    {
        return (uint8)(head - tail);
    }
    return (uint8)(TELEMETRY_EVENT_QUEUE_CAPACITY - tail + head);
}

static uint8 telemetry_event_is_critical(uint8 type)
{
    return (uint8)((TELEMETRY_EVENT_RUN_START == type) ||
                   (TELEMETRY_EVENT_RUN_STOP == type) ||
                   (TELEMETRY_EVENT_PRESTART_STATE == type) ||
                   (TELEMETRY_EVENT_PROTECTION == type) ||
                   (TELEMETRY_EVENT_RUNTIME_BUDGET == type));
}

static void telemetry_push_event(uint8 type, uint8 source, uint16 code, int32 value0, int32 value1)
{
    uint8 head;
    uint8 next;
    uint8 queued;
    uint32 interrupt_state;
    TelemetryEventRecord record;

    if (!logger_enabled)
    {
        return;
    }
    interrupt_state = interrupt_global_disable();
    head = event_head;
    next = telemetry_next_index(head, TELEMETRY_EVENT_QUEUE_CAPACITY);
    queued = telemetry_event_queue_count(head, event_tail);
    if ((next == event_tail) ||
        ((!telemetry_event_is_critical(type)) &&
         (queued >= (TELEMETRY_EVENT_QUEUE_CAPACITY - 1u -
                     TELEMETRY_EVENT_QUEUE_CRITICAL_RESERVE))))
    {
        telemetry_increment_drop(&session_header.event_drop_count);
        interrupt_global_enable(interrupt_state);
        return;
    }
    record.timestamp_ms = vision_clock_ms - session_header.start_ms;
    record.type = type;
    record.source = source;
    record.code = code;
    record.value0 = value0;
    record.value1 = value1;
    event_queue[head] = record;
    __dsync();
    event_head = next;
    interrupt_global_enable(interrupt_state);
}

static uint8 telemetry_queues_empty(void)
{
    return (uint8)((control_head == control_tail) &&
                   (navigation_head == navigation_tail) &&
                   (vision_head == vision_tail) &&
                   (event_head == event_tail) &&
                   (imu_telemetry_head == imu_telemetry_tail) &&
                   (phototube_scan_queue_head ==
                    phototube_scan_queue_tail) &&
                   (localization_queue_head == localization_queue_tail) &&
                   (localization_feature_queue_head ==
                    localization_feature_queue_tail) &&
                   (0u == imu_telemetry_producer_active));
}

static void telemetry_append_event_direct(uint8 type, uint8 source, uint16 code, int32 value0, int32 value1)
{
    TelemetryEventRecord record;

    if (session_header.event_count >= TELEMETRY_EVENT_CAPACITY)
    {
        telemetry_increment_drop(&session_header.event_drop_count);
        return;
    }
    record.timestamp_ms = session_header.end_ms - session_header.start_ms;
    record.type = type;
    record.source = source;
    record.code = code;
    record.value0 = value0;
    record.value1 = value1;
    telemetry_emem_write(
        TELEMETRY_EVENT_OFFSET +
            (uint32)session_header.event_count * sizeof(TelemetryEventRecord),
        &record,
        sizeof(record));
    session_header.event_count++;
}

static void telemetry_write_drop_summaries(void)
{
    if (session_header.control_drop_count)
    {
        telemetry_append_event_direct(TELEMETRY_EVENT_BUFFER_DROP,
                                      TELEMETRY_EVENT_SOURCE_LOGGER,
                                      1u,
                                      session_header.control_drop_count,
                                      0);
    }
    if (session_header.navigation_drop_count)
    {
        telemetry_append_event_direct(TELEMETRY_EVENT_BUFFER_DROP,
                                      TELEMETRY_EVENT_SOURCE_LOGGER,
                                      2u,
                                      session_header.navigation_drop_count,
                                      0);
    }
    if (session_header.vision_drop_count)
    {
        telemetry_append_event_direct(TELEMETRY_EVENT_BUFFER_DROP,
                                      TELEMETRY_EVENT_SOURCE_LOGGER,
                                      3u,
                                      session_header.vision_drop_count,
                                      0);
    }
    if (session_header.event_drop_count)
    {
        telemetry_append_event_direct(TELEMETRY_EVENT_BUFFER_DROP,
                                      TELEMETRY_EVENT_SOURCE_LOGGER,
                                      4u,
                                      session_header.event_drop_count,
                                      0);
    }
    if (session_header.localization_overwrite_count)
    {
        telemetry_append_event_direct(TELEMETRY_EVENT_BUFFER_DROP,
                                      TELEMETRY_EVENT_SOURCE_LOGGER,
                                      6u,
                                      session_header.localization_overwrite_count,
                                      0);
    }
    if (session_header.imu_drop_count)
    {
        telemetry_append_event_direct(TELEMETRY_EVENT_BUFFER_DROP,
                                      TELEMETRY_EVENT_SOURCE_LOGGER,
                                      5u,
                                      session_header.imu_drop_count,
                                      0);
    }
    if (phototube_scan_archive_overwrite_count)
    {
        telemetry_append_event_direct(
            TELEMETRY_EVENT_BUFFER_DROP,
            TELEMETRY_EVENT_SOURCE_LOGGER,
            7u,
            phototube_scan_archive_overwrite_count,
            0);
    }
    if (phototube_scan_queue_drop_count)
    {
        telemetry_append_event_direct(
            TELEMETRY_EVENT_BUFFER_DROP,
            TELEMETRY_EVENT_SOURCE_LOGGER,
            8u,
            phototube_scan_queue_drop_count,
            0);
    }
    if (localization_queue_drop_count)
    {
        telemetry_append_event_direct(
            TELEMETRY_EVENT_BUFFER_DROP,
            TELEMETRY_EVENT_SOURCE_LOGGER,
            9u,
            localization_queue_drop_count,
            0);
    }
    if (position_saturation_count)
    {
        telemetry_append_event_direct(TELEMETRY_EVENT_POSITION_SATURATION,
                                      TELEMETRY_EVENT_SOURCE_LOGGER,
                                      0u,
                                      position_saturation_count,
                                      0);
    }
}

static void telemetry_write_runtime_ledger(void)
{
    PhototubeScanSnapshot scan_snapshot;

    memset(&scan_snapshot, 0, sizeof(scan_snapshot));
    (void)phototube_scan_get_snapshot(&scan_snapshot);
    telemetry_append_event_direct(TELEMETRY_EVENT_RUNTIME_BUDGET,
                                  TELEMETRY_EVENT_SOURCE_CONTROL,
                                  TELEMETRY_RUNTIME_CONTROL_EXEC,
                                  (int32)control_exec_max_us,
                                  (int32)control_exec_overrun_count);
    telemetry_append_event_direct(TELEMETRY_EVENT_RUNTIME_BUDGET,
                                  TELEMETRY_EVENT_SOURCE_CONTROL,
                                  TELEMETRY_RUNTIME_CONTROL_PERIOD,
                                  (int32)control_period_max_us,
                                  (int32)control_period_overrun_count);
    telemetry_append_event_direct(TELEMETRY_EVENT_RUNTIME_BUDGET,
                                  TELEMETRY_EVENT_SOURCE_VISION,
                                  TELEMETRY_RUNTIME_PHOTOTUBE_SCAN,
                                  (int32)phototube_scan_exec_max_us,
                                  (int32)phototube_scan_overrun_count);
    telemetry_append_event_direct(TELEMETRY_EVENT_RUNTIME_BUDGET,
                                  TELEMETRY_EVENT_SOURCE_CONTROL,
                                  TELEMETRY_RUNTIME_DUTY_10MS,
                                  (int32)duty_10ms_exec_max_us,
                                  (int32)duty_10ms_overrun_count);
    telemetry_append_event_direct(TELEMETRY_EVENT_RUNTIME_BUDGET,
                                  TELEMETRY_EVENT_SOURCE_NAVIGATION,
                                  TELEMETRY_RUNTIME_NAV_MATCH,
                                  (int32)nav_match_exec_max_us,
                                  (int32)nav_match_overrun_count);
    telemetry_append_event_direct(TELEMETRY_EVENT_RUNTIME_BUDGET,
                                  TELEMETRY_EVENT_SOURCE_LOGGER,
                                  TELEMETRY_RUNTIME_EMEM_SERVICE,
                                  (int32)telemetry_service_exec_max_us,
                                  (int32)telemetry_service_overrun_count);
    telemetry_append_event_direct(TELEMETRY_EVENT_RUNTIME_BUDGET,
                                  TELEMETRY_EVENT_SOURCE_NAVIGATION,
                                  TELEMETRY_RUNTIME_IMU_SNAPSHOT_FAILURE,
                                  (int32)imu_snapshot_read_failure_count,
                                  (int32)imu_snapshot_max_consecutive_failure_count);
    telemetry_append_event_direct(TELEMETRY_EVENT_RUNTIME_BUDGET,
                                  TELEMETRY_EVENT_SOURCE_NAVIGATION,
                                  TELEMETRY_RUNTIME_IMU_SNAPSHOT_AGE,
                                  (int32)imu_snapshot_max_age_us,
                                  (int32)imu_snapshot_last_age_us);
    telemetry_append_event_direct(TELEMETRY_EVENT_RUNTIME_BUDGET,
                                  TELEMETRY_EVENT_SOURCE_VISION,
                                  TELEMETRY_RUNTIME_SCAN_EPOCH,
                                  (int32)scan_snapshot.epoch_filtered_frame_count,
                                  (int32)scan_snapshot.epoch_ack_timeout_count);
    telemetry_append_event_direct(TELEMETRY_EVENT_RUNTIME_BUDGET,
                                  TELEMETRY_EVENT_SOURCE_VISION,
                                  TELEMETRY_RUNTIME_IMU_ISR,
                                  (int32)imu_isr_exec_max_us,
                                  (int32)imu_isr_overrun_count);
    telemetry_append_event_direct(TELEMETRY_EVENT_RUNTIME_BUDGET,
                                  TELEMETRY_EVENT_SOURCE_VISION,
                                  TELEMETRY_RUNTIME_IMU_SAMPLE_FAILURE,
                                  (int32)imu_sample_failure_count,
                                  (int32)imu_sample_gap_count);
    telemetry_append_event_direct(TELEMETRY_EVENT_RUNTIME_BUDGET,
                                  TELEMETRY_EVENT_SOURCE_VISION,
                                  TELEMETRY_RUNTIME_PHOTOTUBE_ADC_FAILURE,
                                  (int32)vision_capture_adc_failure_count,
                                  0);
}

static void telemetry_store_phototube_scan_record(
    const TelemetryPhototubeScanRecord *record)
{
    uint32 record_offset;
    uint16 record_index;

    if (NULL == record)
    {
        return;
    }
    record_index = phototube_scan_archive_write_index;
    phototube_scan_archive_write_index++;
    if (phototube_scan_archive_write_index >=
        TELEMETRY_PHOTOTUBE_SCAN_CAPACITY)
    {
        phototube_scan_archive_write_index = 0u;
    }
    if (phototube_scan_archive_count < TELEMETRY_PHOTOTUBE_SCAN_CAPACITY)
    {
        phototube_scan_archive_count++;
    }
    else
    {
        telemetry_increment_drop(&phototube_scan_archive_overwrite_count);
    }
    record_offset = TELEMETRY_PHOTOTUBE_SCAN_OFFSET +
                    TELEMETRY_PHOTOTUBE_SCAN_HEADER_SIZE +
                    (uint32)record_index *
                        TELEMETRY_PHOTOTUBE_SCAN_RECORD_SIZE;
    telemetry_emem_write(record_offset, record, sizeof(*record));
}

static void telemetry_drain_phototube_scan(void)
{
    uint8 drained;

    for (drained = 0u;
         drained < TELEMETRY_PHOTOTUBE_SCAN_DRAIN_LIMIT;
         drained++)
    {
        TelemetryPhototubeScanRecord record;

        if (phototube_scan_queue_tail == phototube_scan_queue_head)
        {
            return;
        }
        record = phototube_scan_queue[phototube_scan_queue_tail];
        phototube_scan_queue_tail = (uint16)(
            (phototube_scan_queue_tail + 1u) &
            (TELEMETRY_PHOTOTUBE_SCAN_QUEUE_CAPACITY - 1u));
        telemetry_store_phototube_scan_record(&record);
    }
}

/* The online 1280-point matching ring and the larger EMEM diagnostic ring are
 * independent.  This hot-path API only compresses into CPU0 DSRAM; the main
 * telemetry service performs bounded EMEM writes after the scan tasks. */
void telemetry_record_phototube_scan_point(uint32 frame_seq,
                                           uint32 capture_ms,
                                           float x_mm,
                                           float y_mm,
                                           float lateral_mm,
                                           float raw_yaw_rad,
                                           float travel_distance_mm,
                                           uint8 confidence,
                                           uint8 point_flags)
{
    TelemetryPhototubeScanRecord record;
    uint32 frame_delta;
    uint16 next;

    if (!logger_accepting ||
        (TELEMETRY_STATE_RECORDING != logger_state))
    {
        return;
    }
    if (0u == phototube_scan_archive_have_first_frame)
    {
        phototube_scan_archive_first_frame_seq = frame_seq;
        phototube_scan_archive_have_first_frame = 1u;
    }
    frame_delta = frame_seq - phototube_scan_archive_first_frame_seq;
    memset(&record, 0, sizeof(record));
    /* The host unwraps this modulo-16-bit delta after restoring ring order. */
    record.frame_seq_delta = (uint16)frame_delta;
    record.capture_ms = (uint16)(capture_ms - session_header.start_ms);
    record.x_mm = telemetry_float_to_i16(x_mm, 1.0f);
    record.y_mm = telemetry_float_to_i16(y_mm, 1.0f);
    record.lateral_mm_x10 = telemetry_float_to_i16(lateral_mm, 10.0f);
    record.raw_yaw_cdeg = telemetry_yaw_to_cdeg(
        raw_yaw_rad * 57.29577951f);
    /* Store cumulative travel modulo 65536 instead of saturating at 65.535 m.
     * Records are restored in chronological ring order, so the host can
     * unwrap every retained inter-point delta without spending another two
     * bytes per point.  No synthetic points are introduced. */
    record.travel_distance_mm =
        (uint16)((travel_distance_mm < 0.0f) ? 0u :
                 (uint32)(travel_distance_mm + 0.5f));
    record.confidence = confidence;
    record.point_flags = point_flags;
    next = (uint16)((phototube_scan_queue_head + 1u) &
                    (TELEMETRY_PHOTOTUBE_SCAN_QUEUE_CAPACITY - 1u));
    if (next == phototube_scan_queue_tail)
    {
        telemetry_increment_drop(&phototube_scan_queue_drop_count);
        return;
    }
    phototube_scan_queue[phototube_scan_queue_head] = record;
    phototube_scan_queue_head = next;
}

static uint8 telemetry_write_phototube_scan_header(void)
{
    PhototubeScanSnapshot snapshot;
    TelemetryPhototubeScanHeader header;

    if (phototube_scan_header_finalized)
    {
        return 1u;
    }
    memset(&snapshot, 0, sizeof(snapshot));
    phototube_scan_get_snapshot(&snapshot);
    memset(&header, 0, sizeof(header));
    header.magic[0] = TELEMETRY_PHOTOTUBE_SCAN_MAGIC_0;
    header.magic[1] = TELEMETRY_PHOTOTUBE_SCAN_MAGIC_1;
    header.magic[2] = TELEMETRY_PHOTOTUBE_SCAN_MAGIC_2;
    header.magic[3] = TELEMETRY_PHOTOTUBE_SCAN_MAGIC_3;
    header.version = TELEMETRY_PHOTOTUBE_SCAN_VERSION;
    header.header_size = TELEMETRY_PHOTOTUBE_SCAN_HEADER_SIZE;
    header.point_count = phototube_scan_archive_count;
    header.record_size = TELEMETRY_PHOTOTUBE_SCAN_RECORD_SIZE;
    header.first_frame_seq = phototube_scan_archive_have_first_frame ?
        phototube_scan_archive_first_frame_seq : 0u;
    header.pose_interpolated_count = snapshot.pose_interpolated_count;
    header.pose_rejected_count = snapshot.pose_rejected_count;
    header.mailbox_skipped_count = snapshot.mailbox_skipped_count;
    header.source_frame_skipped_count = snapshot.source_frame_skipped_count;
    header.source_schedule_drop_count = snapshot.source_schedule_drop_count;
    header.source_schedule_skipped_count = snapshot.source_schedule_skipped_count;
    header.max_processing_latency_ms = snapshot.max_processing_latency_ms;
    header.max_capture_to_cloud_latency_ms =
        snapshot.max_capture_to_cloud_latency_ms;
    header.received_frame_count = snapshot.received_frame_count;
    header.accepted_frame_count = snapshot.accepted_frame_count;
    header.accepted_hit_count = snapshot.accepted_hit_count;
    header.spatial_dedup_frame_count = snapshot.spatial_dedup_frame_count;
    header.travel_reversal_frame_count =
        snapshot.travel_reversal_frame_count;
    header.online_ring_overwrite_point_count =
        snapshot.online_ring_overwrite_point_count;
    header.empty_hit_frame_count = snapshot.empty_hit_frame_count;
    header.archive_oldest_storage_slot =
        (phototube_scan_archive_count == TELEMETRY_PHOTOTUBE_SCAN_CAPACITY) ?
            phototube_scan_archive_write_index : 0u;
    header.staging_backpressure_count =
        snapshot.staging_backpressure_count;
    header.staging_high_watermark = snapshot.staging_high_watermark;
    header.pose_wait_count = snapshot.pose_wait_count;
    header.emem_queue_drop_count = phototube_scan_queue_drop_count;
    telemetry_emem_write(TELEMETRY_PHOTOTUBE_SCAN_OFFSET,
                         &header,
                         sizeof(header));
    phototube_scan_header_finalized = 1u;
    return 1u;
}

static void telemetry_drain_control(void)
{
    uint8 drained;

    for (drained = 0u; drained < TELEMETRY_CONTROL_DRAIN_LIMIT; drained++)
    {
        TelemetryControlRecord record;
        uint32 interrupt_state;
        uint32 record_session_sequence;
        uint32 record_offset = 0u;
        uint8 should_write = 0u;
        uint8 tail;

        interrupt_state = interrupt_global_disable();
        tail = control_tail;
        if (tail == control_head)
        {
            interrupt_global_enable(interrupt_state);
            break;
        }
        record = control_queue[tail];
        control_tail = telemetry_next_index(tail, TELEMETRY_CONTROL_QUEUE_CAPACITY);
        record_session_sequence = session_sequence;
        if (session_header.control_count < TELEMETRY_CONTROL_CAPACITY)
        {
            record_offset =
                TELEMETRY_CONTROL_OFFSET +
                    (uint32)session_header.control_count * sizeof(TelemetryControlRecord);
            session_header.control_count++;
            should_write = 1u;
        }
        else
        {
            telemetry_increment_drop(&session_header.control_drop_count);
        }
        interrupt_global_enable(interrupt_state);
        if (should_write && (record_session_sequence == session_sequence) &&
            (record_session_sequence == session_header.session_sequence))
        {
            telemetry_emem_write(record_offset, &record, sizeof(record));
        }
    }
}

static void telemetry_drain_navigation(void)
{
    uint8 drained;

    for (drained = 0u; drained < TELEMETRY_NAVIGATION_DRAIN_LIMIT; drained++)
    {
        TelemetryNavigationRecord record;
        uint32 interrupt_state;
        uint32 record_session_sequence;
        uint32 record_offset = 0u;
        uint8 should_write = 0u;
        uint8 tail;

        interrupt_state = interrupt_global_disable();
        tail = navigation_tail;
        if (tail == navigation_head)
        {
            interrupt_global_enable(interrupt_state);
            break;
        }
        record = navigation_queue[tail];
        navigation_tail = telemetry_next_index(tail, TELEMETRY_NAVIGATION_QUEUE_CAPACITY);
        record_session_sequence = session_sequence;
        if (session_header.navigation_count < TELEMETRY_NAVIGATION_CAPACITY)
        {
            record_offset =
                TELEMETRY_NAVIGATION_OFFSET +
                    (uint32)session_header.navigation_count * sizeof(TelemetryNavigationRecord);
            session_header.navigation_count++;
            should_write = 1u;
        }
        else
        {
            telemetry_increment_drop(&session_header.navigation_drop_count);
        }
        interrupt_global_enable(interrupt_state);
        if (should_write && (record_session_sequence == session_sequence) &&
            (record_session_sequence == session_header.session_sequence))
        {
            telemetry_emem_write(record_offset, &record, sizeof(record));
        }
    }
}

static void telemetry_drain_vision(void)
{
    uint8 drained;

    for (drained = 0u; drained < TELEMETRY_VISION_DRAIN_LIMIT; drained++)
    {
        TelemetryVisionRecord record;
        uint32 interrupt_state;
        uint32 record_session_sequence;
        uint32 record_offset = 0u;
        uint8 should_write = 0u;
        uint8 tail;

        interrupt_state = interrupt_global_disable();
        tail = vision_tail;
        if (tail == vision_head)
        {
            interrupt_global_enable(interrupt_state);
            break;
        }
        record = vision_queue[tail];
        vision_tail = telemetry_next_index(tail, TELEMETRY_VISION_QUEUE_CAPACITY);
        record_session_sequence = session_sequence;
        if (session_header.vision_count < TELEMETRY_VISION_CAPACITY)
        {
            record_offset =
                TELEMETRY_VISION_OFFSET +
                    (uint32)session_header.vision_count * sizeof(TelemetryVisionRecord);
            session_header.vision_count++;
            should_write = 1u;
        }
        else
        {
            telemetry_increment_drop(&session_header.vision_drop_count);
        }
        interrupt_global_enable(interrupt_state);
        if (should_write && (record_session_sequence == session_sequence) &&
            (record_session_sequence == session_header.session_sequence))
        {
            telemetry_emem_write(record_offset, &record, sizeof(record));
        }
    }
}

static void telemetry_drain_events(void)
{
    uint8 drained;

    for (drained = 0u; drained < TELEMETRY_EVENT_DRAIN_LIMIT; drained++)
    {
        TelemetryEventRecord record;
        uint32 interrupt_state;
        uint32 record_session_sequence;
        uint32 record_offset = 0u;
        uint16 storage_limit;
        uint8 should_write = 0u;
        uint8 tail;

        interrupt_state = interrupt_global_disable();
        tail = event_tail;
        if (tail == event_head)
        {
            interrupt_global_enable(interrupt_state);
            break;
        }
        record = event_queue[tail];
        event_tail = telemetry_next_index(tail, TELEMETRY_EVENT_QUEUE_CAPACITY);
        record_session_sequence = session_sequence;
        storage_limit = (uint16)(TELEMETRY_EVENT_CAPACITY -
                                 TELEMETRY_EVENT_STORAGE_FINAL_RESERVE);
        if (!telemetry_event_is_critical(record.type))
        {
            storage_limit = (uint16)(storage_limit -
                TELEMETRY_EVENT_STORAGE_CRITICAL_RESERVE);
        }
        if (session_header.event_count < storage_limit)
        {
            record_offset =
                TELEMETRY_EVENT_OFFSET +
                    (uint32)session_header.event_count * sizeof(TelemetryEventRecord);
            session_header.event_count++;
            should_write = 1u;
        }
        else
        {
            telemetry_increment_drop(&session_header.event_drop_count);
        }
        interrupt_global_enable(interrupt_state);
        if (should_write && (record_session_sequence == session_sequence) &&
            (record_session_sequence == session_header.session_sequence))
        {
            telemetry_emem_write(record_offset, &record, sizeof(record));
        }
    }
}

static void telemetry_drain_imu(void)
{
    uint8 drained;

    for (drained = 0u; drained < TELEMETRY_IMU_DRAIN_LIMIT; drained++)
    {
        TelemetryImuRecord record;
        uint32 interrupt_state;
        uint32 record_session_sequence;
        uint32 record_offset = 0u;
        uint8 should_write = 0u;
        uint8 tail;

        interrupt_state = interrupt_global_disable();
        tail = imu_telemetry_tail;
        __dsync();
        if (tail == imu_telemetry_head)
        {
            interrupt_global_enable(interrupt_state);
            break;
        }
        __dsync();
        record = imu_telemetry_queue[tail];
        imu_telemetry_tail = telemetry_next_index(
            tail, IMU_TELEMETRY_QUEUE_CAPACITY);
        __dsync();
        record_session_sequence = session_sequence;
        if (session_header.imu_count < TELEMETRY_IMU_CAPACITY)
        {
            record_offset = TELEMETRY_IMU_OFFSET +
                            (uint32)session_header.imu_count * sizeof(TelemetryImuRecord);
            session_header.imu_count++;
            should_write = 1u;
        }
        else
        {
            telemetry_increment_drop(&imu_storage_drop_count);
        }
        interrupt_global_enable(interrupt_state);
        if (should_write && (record_session_sequence == session_sequence) &&
            (record_session_sequence == session_header.session_sequence))
        {
            telemetry_emem_write(record_offset, &record, sizeof(record));
        }
    }
    telemetry_merge_imu_producer_drops();
}

static uint32 telemetry_compute_payload_crc_for_commit(void)
{
    uint32 crc = 0xFFFFFFFFu;
    const void *after_crc =
        (const void *)(IFXEMEM_START_ADDR_CPU + TELEMETRY_HEADER_CRC_OFFSET + sizeof(session_header.payload_crc32));

    crc = telemetry_crc32_update(
        crc,
        &session_header,
        TELEMETRY_HEADER_CRC_OFFSET);
    crc = telemetry_crc32_update(
        crc,
        after_crc,
        TELEMETRY_EMEM_SIZE - TELEMETRY_HEADER_CRC_OFFSET -
            sizeof(session_header.payload_crc32));
    return crc ^ 0xFFFFFFFFu;
}

static uint8 telemetry_commit_session_header(void)
{
    const uint8 *header_bytes = (const uint8 *)&session_header;
    uint32 crc;

    session_header.payload_crc32 = 0u;
    telemetry_emem_write(
        TELEMETRY_HEADER_OFFSET + sizeof(session_header.magic),
        header_bytes + sizeof(session_header.magic),
        sizeof(session_header) - sizeof(session_header.magic));

    crc = telemetry_compute_payload_crc_for_commit();
    /* CRC32 has no reserved non-zero value.  A mathematically valid zero
     * checksum must still finalize the session; otherwise the next service
     * pass would append the terminal ledger a second time. */
    session_header.payload_crc32 = crc;
    telemetry_emem_write(TELEMETRY_HEADER_CRC_OFFSET,
                         &session_header.payload_crc32,
                         sizeof(session_header.payload_crc32));
    telemetry_emem_write(TELEMETRY_HEADER_OFFSET,
                         session_header.magic,
                         sizeof(session_header.magic));
    return 1u;
}

void telemetry_init(void)
{
    uint16 cpu_wdt_password;

    memset(&session_header, 0, sizeof(session_header));
    memset(&localization_config_block, 0, sizeof(localization_config_block));
    memset(&tracking_config_block, 0, sizeof(tracking_config_block));
    memset(&localization_feature_header, 0,
           sizeof(localization_feature_header));
    telemetry_reset_queues();
    logger_enabled = 0u;
    logger_accepting = 0u;
    imu_telemetry_capture_enabled = 0u;
    logger_finalized = 0u;
    export_requested = 0u;
    export_session_sequence = 0u;
    phototube_scan_archive_count = 0u;
    phototube_scan_archive_write_index = 0u;
    phototube_scan_archive_first_frame_seq = 0u;
    phototube_scan_archive_overwrite_count = 0u;
    phototube_scan_queue_drop_count = 0u;
    localization_queue_drop_count = 0u;
    phototube_scan_archive_have_first_frame = 0u;
    phototube_scan_header_finalized = 0u;
    telemetry_link_last_ms = 0u;
    session_sequence = 0u;
    imu_telemetry_epoch_seq = 0u;
    imu_telemetry_session_start_us = 0u;
    vision_log_decimation_countdown = 0u;
    vision_have_first_frame_seq = 0u;
    telemetry_service_phase = 0u;
    telemetry_service_exec_time_us = 0u;
    telemetry_service_exec_max_us = 0u;
    telemetry_service_overrun_count = 0u;
    /*
     * Do not scan PFlash in the launch path.  The former one-megabyte
     * byte-by-byte scan ran synchronously from the first key press and
     * stopped the menu refresh, key scanner, and UART service long enough to
     * look like a dead screen.  The fingerprint remains deterministic and
     * still includes the complete layout/parameter/route/config snapshot;
     * the optional binary-image component is represented by zero.
     */
    firmware_image_hash = 0u;
    logger_state = TELEMETRY_STATE_DISABLED;

    /*
     * Ordinary TC264D devices do not implement EMEM.  Do not touch either
     * the EMEM registers or the 0xBF... alias until the chip advertises the
     * EMEM extension; on such a device telemetry is deliberately disabled.
     */
    if ((0u == MODULE_SCU.CHIPID.B.EEA) ||
        (TELEMETRY_EMEM_REQUIRED_CHIP_ID != MODULE_SCU.CHIPID.B.CHID))
    {
        return;
    }

    /* EECDIS is read-only.  If the startup/debug boundary is still closed,
     * the first EMEM access can hang; telemetry must remain optional. */
    if (0u != MODULE_CBS.OSTATE.B.EECDIS)
    {
        return;
    }

    cpu_wdt_password = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(cpu_wdt_password);
    MODULE_EMEM.CLC.B.DISR = 0u;
    IfxScuWdt_setCpuEndinit(cpu_wdt_password);
    __nop();
    if (0u != MODULE_EMEM.CLC.B.DISS)
    {
        return;
    }

    if (0u == MODULE_EMEM.SBRCTR.B.STBLOCK)
    {
        IfxEmem_setUnlockStandbyLockFlag(0x1u);
        IfxEmem_setUnlockStandbyLockFlag(0x3u);
        IfxEmem_setUnlockStandbyLockFlag(0x7u);
        __nop();
    }

    /*
     * Configure the implemented EMEM tiles in common/calibration mode in one
     * masked write.  The old per-tile loop assumed ten 32 KiB tiles and read
     * TILESTATE before the first write; both assumptions are unsafe on TC26x
     * (TILESTATE may be random after reset/ECC is not initialized).
     */
    if (0u == MODULE_EMEM.SBRCTR.B.STBLOCK)
    {
        return;
    }
    MODULE_EMEM.TILECONFIG.U = 0u;
    __dsync();
    __nop();
    if (0u != (MODULE_EMEM.TILESTATE.U & TELEMETRY_EMEM_TILESTATE_MASK))
    {
        return;
    }

    /* Initialize every ECC line before any EMEM read (header/CRC/self-test). */
    telemetry_emem_clear();
    if (!telemetry_emem_self_test())
    {
        logger_enabled = 0u;
        logger_state = TELEMETRY_STATE_DISABLED;
        return;
    }

    /* The launch path is deliberately free of a synchronous PFlash scan. */
    logger_enabled = 1u;
    logger_finalized = 0u;
    logger_state = TELEMETRY_STATE_READY;
}

void telemetry_begin_run(uint8 route_index, uint16 route_point_count, uint32 route_hash)
{
    if (!logger_enabled)
    {
        return;
    }

    logger_accepting = 0u;
    imu_telemetry_capture_enabled = 0u;
    __dsync();
    if (!telemetry_wait_imu_producer_idle())
    {
        /* Do not reset a queue while its CPU1 producer still owns a slot.
         * The run remains non-recording and the next launch can retry. */
        logger_state = TELEMETRY_STATE_READY;
        logger_finalized = 0u;
        return;
    }
    logger_state = TELEMETRY_STATE_READY;
    telemetry_emem_write(TELEMETRY_HEADER_OFFSET,
                         invalid_magic,
                         sizeof(invalid_magic));
    telemetry_emem_write(TELEMETRY_PHOTOTUBE_SCAN_OFFSET,
                         invalid_magic,
                         sizeof(invalid_magic));
    telemetry_reset_queues();
    memset(&session_header, 0, sizeof(session_header));
    memset(&localization_config_block, 0, sizeof(localization_config_block));
    memset(&tracking_config_block, 0, sizeof(tracking_config_block));
    memset(&localization_feature_header, 0,
           sizeof(localization_feature_header));
    position_saturation_count = 0u;
    logger_finalized = 0u;
    phototube_scan_archive_count = 0u;
    phototube_scan_archive_write_index = 0u;
    phototube_scan_archive_first_frame_seq = 0u;
    phototube_scan_archive_overwrite_count = 0u;
    phototube_scan_queue_drop_count = 0u;
    localization_queue_drop_count = 0u;
    phototube_scan_archive_have_first_frame = 0u;
    phototube_scan_header_finalized = 0u;
    export_requested = 0u;
    export_prefix_offset = 0u;
    export_payload_offset = 0u;
    export_session_sequence = 0u;
    session_sequence++;
    imu_telemetry_epoch_seq++;
    if (0u == imu_telemetry_epoch_seq)
    {
        imu_telemetry_epoch_seq = 1u;
    }
    imu_telemetry_session_start_us = 0u;
    vision_log_decimation_countdown = 0u;
    vision_have_first_frame_seq = 0u;
    telemetry_fill_header_layout();
    telemetry_snapshot_parameters();
    session_header.state = TELEMETRY_STATE_RECORDING;
    session_header.session_sequence = session_sequence;
    session_header.start_ms = vision_clock_ms;
    session_header.route_index = route_index;
    session_header.route_point_count = route_point_count;
    session_header.route_hash = route_hash;
    telemetry_snapshot_localization_config();
    telemetry_snapshot_tracking_config();
    session_header.firmware_id =
        telemetry_calculate_firmware_config_fingerprint(
            &localization_config_block, &tracking_config_block);
    telemetry_emem_write(TELEMETRY_LOCALIZATION_CONFIG_OFFSET,
                         &localization_config_block,
                         sizeof(localization_config_block));
    telemetry_emem_write(TELEMETRY_TRACKING_CONFIG_OFFSET,
                         &tracking_config_block,
                         sizeof(tracking_config_block));
    telemetry_initialize_localization_features();
    logger_state = TELEMETRY_STATE_RECORDING;
    /* Producers remain closed until control and navigation are enabled as
     * one final atomic launch step.  The RUN_START event is emitted by
     * prestart_service() immediately after telemetry_activate_run(). */
    logger_accepting = 0u;
}

void telemetry_activate_run(void)
{
    ImuSnapshot imu_snapshot;

    if (logger_enabled && (TELEMETRY_STATE_RECORDING == logger_state))
    {
        /* The header is still in RAM at this point.  Take the timestamp at
         * the same atomic boundary as control_enable/nav_start, rather than
         * at the beginning of the launch metadata transaction. */
        session_header.start_ms = vision_clock_ms;
        /* IMU records are timestamped on CPU1's STM.  Anchor their relative
         * clock with a coherent CPU1 snapshot instead of assuming STM0 and
         * STM1 software epochs are identical. */
        imu_telemetry_session_start_us =
            imu_snapshot_read(&imu_snapshot) ? imu_snapshot.sample_us : 0u;
        imu_telemetry_capture_enabled = 1u;
        __dsync();
        logger_accepting = 1u;
    }
}

void telemetry_freeze(uint8 stop_reason, uint8 final_prestart_state, uint8 final_nav_state)
{
    if (!logger_enabled || (TELEMETRY_STATE_RECORDING != logger_state))
    {
        return;
    }

    logger_accepting = 0u;
    imu_telemetry_capture_enabled = 0u;
    __dsync();
    (void)telemetry_wait_imu_producer_idle();
    telemetry_merge_imu_producer_drops();
    /* Preserve all run events ahead of the terminal ledger.  Runtime/drop
     * summaries and RUN_STOP are written only after every data queue drains,
     * so RUN_STOP carries final record counts and cannot consume queue slots. */
    while (event_tail != event_head)
    {
        telemetry_drain_events();
    }
    if (STOP_REASON_NONE == stop_reason)
    {
        stop_reason = STOP_REASON_MANUAL;
    }
    session_header.end_ms = vision_clock_ms;
    session_header.stop_reason = stop_reason;
    session_header.final_prestart_state = final_prestart_state;
    session_header.final_nav_state = final_nav_state;
    logger_state = TELEMETRY_STATE_FROZEN;
}

void telemetry_capture_control_isr(int16 encoder_l,
                                   int16 encoder_r,
                                   float target_l_cmps,
                                   float target_r_cmps,
                                   float accel_l_cmps2,
                                   float accel_r_cmps2,
                                   int16 pwm_l,
                                   int16 pwm_r,
                                   float yaw_rate_error_degps,
                                   float tracking_turn_cmps)
{
    uint8 head;
    uint8 next;
    TelemetryControlRecord record;

    if (!logger_accepting)
    {
        return;
    }
    /* Navigation is captured earlier in this same control ISR.  Timestamp
     * both streams from the shared launch clock so the host can associate
     * this tick's plan_v with the exact pose/progress used by the tracker. */
    record.tick_ms = (uint16)(vision_clock_ms - session_header.start_ms);
    if ((record.tick_ms % TELEMETRY_CONTROL_LOG_DIVIDER) != 0u)
    {
        return;
    }
    head = control_head;
    next = telemetry_next_index(head, TELEMETRY_CONTROL_QUEUE_CAPACITY);
    if (next == control_tail)
    {
        telemetry_increment_drop(&session_header.control_drop_count);
        return;
    }
    record.encoder_l = encoder_l;
    record.encoder_r = encoder_r;
    record.target_l_x10 = telemetry_float_to_i16(target_l_cmps, 10.0f);
    record.target_r_x10 = telemetry_float_to_i16(target_r_cmps, 10.0f);
    record.accel_l_cmps2 = telemetry_float_to_i16(accel_l_cmps2, 1.0f);
    record.accel_r_cmps2 = telemetry_float_to_i16(accel_r_cmps2, 1.0f);
    record.pwm_l = pwm_l;
    record.pwm_r = pwm_r;
    record.yaw_rate_error_degps_x10 =
        telemetry_float_to_i16(yaw_rate_error_degps, 10.0f);
    record.tracking_turn_x10 =
        telemetry_float_to_i16(tracking_turn_cmps, 10.0f);

    control_queue[head] = record;
    __dsync();
    control_head = next;
}

void telemetry_capture_navigation_isr(const TelemetryNavigationSample *sample)
{
    uint8 head;
    uint8 next;
    TelemetryNavigationRecord record;

    if (!logger_accepting || (NULL == sample))
    {
        return;
    }
    head = navigation_head;
    next = telemetry_next_index(head, TELEMETRY_NAVIGATION_QUEUE_CAPACITY);
    if (next == navigation_tail)
    {
        telemetry_increment_drop(&session_header.navigation_drop_count);
        return;
    }
    if ((sample->actual_x_mm > 32767.0f) || (sample->actual_x_mm < -32768.0f) ||
        (sample->actual_y_mm > 32767.0f) || (sample->actual_y_mm < -32768.0f) ||
        (sample->raw_x_mm > 32767.0f) || (sample->raw_x_mm < -32768.0f) ||
        (sample->raw_y_mm > 32767.0f) || (sample->raw_y_mm < -32768.0f))
    {
        if (position_saturation_count < 0xFFFFu)
        {
            position_saturation_count++;
        }
    }
    record.timestamp_ms = (uint16)(sample->timestamp_ms -
                                   session_header.start_ms);
    record.progress_quarter_points = telemetry_float_to_u16(
        sample->progress_points, 4.0f);
    record.actual_x_mm = telemetry_float_to_i16(sample->actual_x_mm, 1.0f);
    record.actual_y_mm = telemetry_float_to_i16(sample->actual_y_mm, 1.0f);
    record.actual_yaw_cdeg = telemetry_yaw_to_cdeg(sample->actual_yaw_deg);
    record.raw_x_mm = telemetry_float_to_i16(sample->raw_x_mm, 1.0f);
    record.raw_y_mm = telemetry_float_to_i16(sample->raw_y_mm, 1.0f);
    record.raw_yaw_cdeg = telemetry_yaw_to_cdeg(sample->raw_yaw_deg);
    record.localization_update_seq = sample->localization_update_seq;

    navigation_queue[head] = record;
    __dsync();
    navigation_head = next;
}

/* The IMU producer runs from the CPU1 DRDY ISR.  Keep its conversion helpers
 * beside the producer so a high-rate sample never executes CPU0 DSRAM code. */
#pragma section all restore
#pragma section all "cpu1_dsram"

static int16 telemetry_imu_float_to_i16(float value, float scale)
{
    float scaled;

    if ((value != value) || (scale <= 0.0f))
    {
        return 0;
    }
    scaled = value * scale;
    if (scaled >= 32767.0f) return 32767;
    if (scaled <= -32768.0f) return -32768;
    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    return (int16)scaled;
}

static uint8 telemetry_imu_next_index(uint8 index)
{
    index++;
    return (index >= IMU_TELEMETRY_QUEUE_CAPACITY) ? 0u : index;
}

static int16 telemetry_quaternion_to_s14(float value)
{
    float scaled = value * TELEMETRY_QUATERNION_S14_SCALE;
    int32 rounded = (int32)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));

    if (rounded > 8191) rounded = 8191;
    if (rounded < -8192) rounded = -8192;
    return (int16)rounded;
}

static void telemetry_pack_quaternion(TelemetryImuRecord *record,
                                      float q0, float q1,
                                      float q2, float q3)
{
    float q[4];
    float largest_abs;
    float sign;
    int16 packed[3];
    uint8 largest = 0u;
    uint8 source;
    uint8 destination = 0u;

    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
    largest_abs = (q[0] < 0.0f) ? -q[0] : q[0];
    for (source = 1u; source < 4u; source++)
    {
        float value_abs = (q[source] < 0.0f) ? -q[source] : q[source];
        if (value_abs > largest_abs)
        {
            largest_abs = value_abs;
            largest = source;
        }
    }
    sign = (q[largest] < 0.0f) ? -1.0f : 1.0f;
    for (source = 0u; source < 4u; source++)
    {
        if (source != largest)
        {
            packed[destination++] = telemetry_quaternion_to_s14(
                sign * q[source]);
        }
    }
    record->quaternion_component0_index =
        (uint16)(((uint16)largest << 14u) |
                 ((uint16)packed[0] & 0x3FFFu));
    record->quaternion_component1_s14 = packed[1];
    record->quaternion_component2_s14 = packed[2];
}

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
                               uint32 sample_us)
{
    uint8 head;
    uint8 next;
    uint32 timestamp_4us;
    uint32 timestamp_delta_4us;
    TelemetryImuRecord record;

    if (!imu_telemetry_capture_enabled)
    {
        return;
    }
    imu_telemetry_producer_active = 1u;
    __dsync();
    if (!imu_telemetry_capture_enabled)
    {
        goto imu_capture_done;
    }
    if (imu_telemetry_local_epoch_seq != imu_telemetry_epoch_seq)
    {
        imu_telemetry_local_epoch_seq = imu_telemetry_epoch_seq;
        imu_telemetry_producer_epoch_seq = imu_telemetry_epoch_seq;
        imu_telemetry_producer_drop_count = 0u;
        imu_last_timestamp_4us = 0u;
        imu_have_timestamp = 0u;
        imu_log_decimation_countdown = 0u;
        imu_telemetry_local_start_us = imu_telemetry_session_start_us;
    }
    if (imu_log_decimation_countdown > 0u)
    {
        imu_log_decimation_countdown--;
        goto imu_capture_done;
    }
    imu_log_decimation_countdown = (uint8)(TELEMETRY_IMU_DECIMATION - 1u);
    __dsync();
    head = imu_telemetry_head;
    next = telemetry_imu_next_index(head);
    if (next == imu_telemetry_tail)
    {
        if (imu_telemetry_producer_drop_count != 0xFFFFu)
        {
            imu_telemetry_producer_drop_count++;
        }
        goto imu_capture_done;
    }

    if (0u == imu_telemetry_local_start_us)
    {
        /* Snapshot contention at activation is rare.  Establish the missing
         * epoch on CPU1's own first accepted sample rather than mixing STM0
         * and STM1 time bases.  Keep this fallback local: CPU0 owns the
         * shared activation epoch. */
        imu_telemetry_local_start_us = sample_us;
    }
    timestamp_4us = (sample_us -
                     imu_telemetry_local_start_us + 2u) / 4u;
    timestamp_delta_4us = imu_have_timestamp ?
        (timestamp_4us - imu_last_timestamp_4us) : timestamp_4us;
    if (timestamp_delta_4us > 0xFFFFu)
    {
        timestamp_delta_4us = 0xFFFFu;
        if (imu_telemetry_producer_drop_count != 0xFFFFu)
        {
            imu_telemetry_producer_drop_count++;
        }
    }
    record.timestamp_delta_4us = (uint16)timestamp_delta_4us;
    imu_last_timestamp_4us = timestamp_4us;
    imu_have_timestamp = 1u;
    record.acc_x_mg = telemetry_imu_float_to_i16(acc_x_g, 1000.0f);
    record.acc_y_mg = telemetry_imu_float_to_i16(acc_y_g, 1000.0f);
    record.acc_z_mg = telemetry_imu_float_to_i16(acc_z_g, 1000.0f);
    record.gyro_x_dps_x10 =
        telemetry_imu_float_to_i16(gyro_x_rad_s * 57.29577951f, 10.0f);
    record.gyro_y_dps_x10 =
        telemetry_imu_float_to_i16(gyro_y_rad_s * 57.29577951f, 10.0f);
    record.gyro_z_dps_x10 =
        telemetry_imu_float_to_i16(gyro_z_rad_s * 57.29577951f, 10.0f);
    telemetry_pack_quaternion(&record, quaternion_q0, quaternion_q1,
                              quaternion_q2, quaternion_q3);

    imu_telemetry_queue[head] = record;
    __dsync();
    imu_telemetry_head = next;

imu_capture_done:
    __dsync();
    imu_telemetry_producer_active = 0u;
}

#pragma section all restore
#pragma section all "cpu0_dsram"

void telemetry_capture_vision_isr(uint32 capture_ms,
                                  uint32 frame_seq,
                                  float line_position_mm,
                                  uint16 active_mask,
                                  uint16 observation_flags,
                                  uint8 peak_count,
                                  uint8 white_percent,
                                  uint8 threshold_gray)
{
    uint8 head;
    uint8 next;
    TelemetryVisionRecord record;

    if (!logger_accepting)
    {
        return;
    }
    if (vision_log_decimation_countdown > 0u)
    {
        vision_log_decimation_countdown--;
        return;
    }
    vision_log_decimation_countdown =
        (uint8)(TELEMETRY_VISION_LOG_DIVIDER - 1u);
    head = vision_head;
    next = telemetry_next_index(head, TELEMETRY_VISION_QUEUE_CAPACITY);
    if (next == vision_tail)
    {
        telemetry_increment_drop(&session_header.vision_drop_count);
        return;
    }
    if (white_percent > 100u)
    {
        white_percent = 100u;
    }
    if (threshold_gray > 127u)
    {
        threshold_gray = 127u;
    }
    record.tick_ms = (uint16)(capture_ms - session_header.start_ms);
    if (!vision_have_first_frame_seq)
    {
        session_header.vision_first_frame_seq = frame_seq;
        vision_have_first_frame_seq = 1u;
    }
    record.frame_seq = (uint16)frame_seq;
    record.line_position_mm_x100 =
        ((observation_flags &
          (VISION_OBS_FLAG_VIRTUAL_LINE |
           VISION_OBS_FLAG_LATERAL_VALID |
           VISION_OBS_FLAG_FIT_VALID |
           VISION_OBS_FLAG_COVERAGE)) ==
         (VISION_OBS_FLAG_VIRTUAL_LINE |
          VISION_OBS_FLAG_LATERAL_VALID |
          VISION_OBS_FLAG_FIT_VALID |
          VISION_OBS_FLAG_COVERAGE)) ?
            telemetry_float_to_i16(line_position_mm, 100.0f) :
            (int16)(-32767 - 1);
    record.active_mask = active_mask;
    if (peak_count > 15u) peak_count = 15u;
    record.diagnostics =
        (uint16)((uint16)threshold_gray |
                 ((uint16)peak_count << 7u) |
                 ((uint16)((white_percent * 15u + 50u) / 100u) << 11u) |
                 ((0u != (observation_flags & VISION_OBS_FLAG_FULL_COVERAGE)) ?
                      0x8000u : 0u));

    vision_queue[head] = record;
    __dsync();
    vision_head = next;
}

void telemetry_capture_event_isr(TelemetryEventType type,
                                 TelemetryEventSource source,
                                 uint16 code,
                                 int32 value0,
                                 int32 value1)
{
    uint32 interrupt_state = interrupt_global_disable();

    if (logger_accepting && (TELEMETRY_STATE_RECORDING == logger_state))
    {
        telemetry_push_event((uint8)type, (uint8)source, code, value0, value1);
    }
    interrupt_global_enable(interrupt_state);
}

static void telemetry_store_localization_record(
    const TelemetryLocalizationRecord *record)
{
    uint32 record_offset;
    uint16 record_index;

    if (session_header.localization_count < TELEMETRY_LOCALIZATION_CAPACITY)
    {
        record_index = session_header.localization_count;
        session_header.localization_count++;
    }
    else
    {
        record_index = (uint16)(session_header.localization_overwrite_count %
                                TELEMETRY_LOCALIZATION_CAPACITY);
        telemetry_increment_drop(&session_header.localization_overwrite_count);
    }
    record_offset = TELEMETRY_LOCALIZATION_OFFSET +
                    (uint32)record_index * sizeof(TelemetryLocalizationRecord);
    telemetry_emem_write(record_offset, record, sizeof(*record));
}

static void telemetry_store_localization_feature(
    const TelemetryLocalizationFeatureRecord *record)
{
    TelemetryLocalizationFeatureRecord stored;
    uint32 record_offset;
    uint16 increment;
    uint8 index;

    record_offset = TELEMETRY_LOCALIZATION_FEATURE_OFFSET +
                    TELEMETRY_LOCALIZATION_FEATURE_HEADER_SIZE +
                    (uint32)record->route_feature_occurrence *
                        sizeof(TelemetryLocalizationFeatureRecord);
    telemetry_emem_read(record_offset, &stored, sizeof(stored));
    if (0u == (stored.status_flags &
               TELEMETRY_LOCALIZATION_FEATURE_REGISTERED))
    {
        telemetry_increment_drop(
            &localization_feature_header.diagnostic_drop_count);
        return;
    }
    stored.status_flags |= TELEMETRY_LOCALIZATION_FEATURE_ENCOUNTERED;
    stored.status_flags |= (record->status_flags &
                            TELEMETRY_LOCALIZATION_FEATURE_ACCEPTED);
    if (record->status_flags & TELEMETRY_LOCALIZATION_FEATURE_ACCEPTED)
    {
        if (record->status_flags & TELEMETRY_LOCALIZATION_FEATURE_PARTIAL)
        {
            stored.status_flags |= TELEMETRY_LOCALIZATION_FEATURE_PARTIAL;
        }
        else
        {
            stored.status_flags &=
                (uint8)~TELEMETRY_LOCALIZATION_FEATURE_PARTIAL;
        }
    }
    stored.last_timestamp_ms = record->last_timestamp_ms;
    stored.last_reject_reason = record->last_reject_reason;
    stored.reject_reason_mask |= record->reject_reason_mask;
    stored.detail_flags |= record->detail_flags;
    stored.last_detail_flags = record->detail_flags;
    increment = record->attempt_count;
    if (increment > (uint16)(0xFFFFu - stored.attempt_count))
        stored.attempt_count = 0xFFFFu;
    else
        stored.attempt_count = (uint16)(stored.attempt_count + increment);
    stored.observed_progress_points = record->observed_progress_points;
    for (index = 0u; index < 3u; index++)
    {
        stored.line_point_count[index] = record->line_point_count[index];
        stored.line_frame_count[index] = record->line_frame_count[index];
    }
    stored.failure_value_x10 = record->failure_value_x10;
    telemetry_emem_write(record_offset, &stored, sizeof(stored));
}

static void telemetry_drain_localization(void)
{
    uint8 drained;

    for (drained = 0u; drained < TELEMETRY_LOCALIZATION_DRAIN_LIMIT;
         drained++)
    {
        if (localization_queue_tail == localization_queue_head)
            break;
        telemetry_store_localization_record(
            &localization_queue[localization_queue_tail]);
        localization_queue_tail = (uint8)(
            (localization_queue_tail + 1u) &
            (TELEMETRY_LOCALIZATION_QUEUE_CAPACITY - 1u));
    }
    for (drained = 0u; drained < TELEMETRY_LOCALIZATION_DRAIN_LIMIT;
         drained++)
    {
        if (localization_feature_queue_tail ==
            localization_feature_queue_head)
            break;
        telemetry_store_localization_feature(
            &localization_feature_queue[localization_feature_queue_tail]);
        localization_feature_queue_tail = (uint8)(
            (localization_feature_queue_tail + 1u) &
            (TELEMETRY_LOCALIZATION_QUEUE_CAPACITY - 1u));
    }
}

void telemetry_record_localization(const TelemetryLocalizationRecord *record)
{
    TelemetryLocalizationRecord stored;
    uint8 next;

    if ((NULL == record) || !logger_accepting ||
        (TELEMETRY_STATE_RECORDING != logger_state))
        return;
    stored = *record;
    stored.timestamp_ms -= session_header.start_ms;
    next = (uint8)((localization_queue_head + 1u) &
                   (TELEMETRY_LOCALIZATION_QUEUE_CAPACITY - 1u));
    if (next == localization_queue_tail)
    {
        telemetry_increment_drop(&localization_queue_drop_count);
        return;
    }
    localization_queue[localization_queue_head] = stored;
    localization_queue_head = next;
}

void telemetry_record_localization_feature(
    const TelemetryLocalizationFeatureRecord *record)
{
    uint8 next;

    if ((NULL == record) || !logger_accepting ||
        (TELEMETRY_STATE_RECORDING != logger_state) ||
        (record->route_feature_occurrence >=
         TELEMETRY_LOCALIZATION_FEATURE_CAPACITY))
        return;
    next = (uint8)((localization_feature_queue_head + 1u) &
                   (TELEMETRY_LOCALIZATION_QUEUE_CAPACITY - 1u));
    if (next == localization_feature_queue_tail)
    {
        telemetry_increment_drop(&localization_queue_drop_count);
        return;
    }
    localization_feature_queue[localization_feature_queue_head] = *record;
    localization_feature_queue_head = next;
}

void telemetry_service(void)
{
    uint32 service_start_us;
    uint32 elapsed_us;
    uint8 phase_count;

    if (!logger_enabled)
    {
        return;
    }

    service_start_us = system_getval_us();
    for (phase_count = 0u;
         phase_count < TELEMETRY_SERVICE_PHASE_COUNT;
         phase_count++)
    {
        switch (telemetry_service_phase)
        {
            case 0u:
                telemetry_drain_control();
                break;
            case 1u:
                telemetry_drain_navigation();
                break;
            case 2u:
                telemetry_drain_vision();
                break;
            case 3u:
                telemetry_drain_imu();
                break;
            case 4u:
                telemetry_drain_phototube_scan();
                break;
            case 5u:
                telemetry_drain_events();
                break;
            default:
                telemetry_drain_localization();
                break;
        }
        telemetry_service_phase++;
        if (telemetry_service_phase >= TELEMETRY_SERVICE_PHASE_COUNT)
        {
            telemetry_service_phase = 0u;
        }
        if ((system_getval_us() - service_start_us) >=
            TELEMETRY_SERVICE_SOFT_BUDGET_US)
        {
            break;
        }
    }

    elapsed_us = system_getval_us() - service_start_us;
    telemetry_service_exec_time_us = elapsed_us;
    if (elapsed_us > telemetry_service_exec_max_us)
    {
        telemetry_service_exec_max_us = elapsed_us;
    }
    if ((elapsed_us > TELEMETRY_SERVICE_SOFT_BUDGET_US) &&
        (telemetry_service_overrun_count < 0xFFFFFFFFu))
    {
        telemetry_service_overrun_count++;
    }

    if ((TELEMETRY_STATE_FROZEN == logger_state) &&
        !logger_finalized && telemetry_queues_empty())
    {
        /* Feature records are drained after freeze.  Commit their final drop
         * count only after both localization queues are empty, in the same
         * finalization pass that makes the session header valid. */
        telemetry_emem_write(TELEMETRY_LOCALIZATION_FEATURE_OFFSET,
                             &localization_feature_header,
                             sizeof(localization_feature_header));
        if (!telemetry_write_phototube_scan_header())
        {
            return;
        }
        telemetry_write_drop_summaries();
        telemetry_write_runtime_ledger();
        telemetry_append_event_direct(TELEMETRY_EVENT_RUN_STOP,
                                      TELEMETRY_EVENT_SOURCE_PRESTART,
                                      session_header.stop_reason,
                                      session_header.control_count,
                                      session_header.navigation_count);
        session_header.state = TELEMETRY_STATE_FROZEN;
        if (telemetry_commit_session_header())
        {
            logger_finalized = 1u;
        }
    }
}

uint8 telemetry_request_export(void)
{
    if (!logger_enabled)
    {
        return 0u;
    }

    /* A stop request freezes the session before the 500 ms teardown delay
     * completes.  Accept the third key during that STOPPING window and let
     * telemetry_export_service() wait for finalization/idle before sending.
     * This prevents the operator from needing a fourth press merely because
     * the state machine has not finished its delay yet. */
    if (TELEMETRY_STATE_FROZEN == logger_state)
    {
        export_requested = 1u;
        return 1u;
    }

    if ((TELEMETRY_STATE_READY != logger_state) ||
        !logger_finalized || !prestart_is_idle())
    {
        return 0u;
    }
    export_requested = 1u;
    return 1u;
}

static uint32 telemetry_export_zero_run_length(uint32 offset, uint32 limit)
{
    uint32 length = 0u;

    while ((length < limit) &&
           ((offset + length) < TELEMETRY_EMEM_SIZE) &&
           (0u == telemetry_emem[offset + length]))
    {
        length++;
    }
    return length;
}

static uint32 telemetry_export_literal_length(uint32 offset)
{
    uint32 length = 0u;

    while ((length < TELEMETRY_EXPORT_CHUNK_BYTES) &&
           ((offset + length) < TELEMETRY_EMEM_SIZE))
    {
        uint32 zero_run = telemetry_export_zero_run_length(
            offset + length,
            TELEMETRY_EXPORT_ZERO_MIN_RUN);
        if (zero_run >= TELEMETRY_EXPORT_ZERO_MIN_RUN)
        {
            break;
        }
        length++;
    }
    return length;
}

static void telemetry_export_block(uint8 zero_run, uint32 length)
{
    uint16 block_header = (uint16)(length - 1u);
    uint8 encoded_header[2];

    if (zero_run)
    {
        block_header |= TELEMETRY_WIRE_BLOCK_ZERO_FLAG;
    }
    encoded_header[0] = (uint8)(block_header & 0xFFu);
    encoded_header[1] = (uint8)(block_header >> 8u);
    uart_write_buffer(DEBUG_UART_INDEX, encoded_header, sizeof(encoded_header));
    if (!zero_run)
    {
        uart_write_buffer(
            DEBUG_UART_INDEX,
            (const uint8 *)(IFXEMEM_START_ADDR_CPU + export_payload_offset),
            length);
    }
    export_payload_offset += length;
}

void telemetry_export_service(void)
{
    if (!logger_enabled)
    {
        return;
    }

    if (TELEMETRY_STATE_EXPORTING != logger_state)
    {
        if (!export_requested || !logger_finalized || !prestart_is_idle())
        {
            return;
        }
        /* Finalization already computed the immutable frozen-image CRC.  A
         * second byte-wise scan here only delays the first wire byte; the PC
         * verifies the same CRC after reconstructing the image. */
        if (0u == session_header.payload_crc32)
        {
            export_requested = 0u;
            logger_state = TELEMETRY_STATE_FROZEN;
            return;
        }
        wire_prefix.magic[0] = TELEMETRY_WIRE_MAGIC_0;
        wire_prefix.magic[1] = TELEMETRY_WIRE_MAGIC_1;
        wire_prefix.magic[2] = TELEMETRY_WIRE_MAGIC_2;
        wire_prefix.magic[3] = TELEMETRY_WIRE_MAGIC_3;
        wire_prefix.protocol_version = TELEMETRY_PROTOCOL_VERSION;
        wire_prefix.flags = TELEMETRY_WIRE_FLAG_ZERO_RLE;
        wire_prefix.payload_size = TELEMETRY_EMEM_SIZE;
        wire_prefix.payload_crc32 = session_header.payload_crc32;
        export_prefix_offset = 0u;
        export_payload_offset = 0u;
        export_session_sequence = session_sequence;
        logger_state = TELEMETRY_STATE_EXPORTING;
    }

    if (!prestart_is_idle() ||
        (export_session_sequence != session_sequence) ||
        (export_session_sequence != session_header.session_sequence))
    {
        if (TELEMETRY_STATE_EXPORTING == logger_state)
        {
            export_requested = 0u;
            logger_state = TELEMETRY_STATE_FROZEN;
        }
        return;
    }

    if (export_prefix_offset < sizeof(wire_prefix))
    {
        uint32 remaining = sizeof(wire_prefix) - export_prefix_offset;
        uint32 size = (remaining > TELEMETRY_EXPORT_CHUNK_BYTES)
                          ? TELEMETRY_EXPORT_CHUNK_BYTES
                          : remaining;
        uart_write_buffer(DEBUG_UART_INDEX,
                          ((const uint8 *)&wire_prefix) + export_prefix_offset,
                          size);
        export_prefix_offset += size;
        return;
    }

    if (export_payload_offset < TELEMETRY_EMEM_SIZE)
    {
        uint32 remaining = TELEMETRY_EMEM_SIZE - export_payload_offset;
        uint32 zero_run = telemetry_export_zero_run_length(
            export_payload_offset,
            (remaining > TELEMETRY_WIRE_BLOCK_MAX_LENGTH) ?
                TELEMETRY_WIRE_BLOCK_MAX_LENGTH : remaining);

        if (zero_run >= TELEMETRY_EXPORT_ZERO_MIN_RUN)
        {
            telemetry_export_block(1u, zero_run);
        }
        else
        {
            uint32 literal_length = telemetry_export_literal_length(
                export_payload_offset);
            telemetry_export_block(0u, literal_length);
        }
        return;
    }

    export_requested = 0u;
    logger_state = TELEMETRY_STATE_FROZEN;
}

void telemetry_link_service(void)
{
    uint32 now_ms;
    uint16 flags;
    TelemetryLinkBeacon beacon;

    if (!prestart_is_idle() || (TELEMETRY_STATE_EXPORTING == logger_state))
    {
        return;
    }

    now_ms = vision_clock_ms;
    if ((uint32)(now_ms - telemetry_link_last_ms) < TELEMETRY_LINK_BEACON_INTERVAL_MS)
    {
        return;
    }
    telemetry_link_last_ms = now_ms;

    flags = TELEMETRY_LINK_FLAG_IDLE;
    if (logger_enabled)
    {
        flags |= TELEMETRY_LINK_FLAG_LOGGER_ENABLED;
    }
    if (logger_finalized)
    {
        flags |= TELEMETRY_LINK_FLAG_RUN_FINALIZED;
    }

    beacon.magic[0] = TELEMETRY_LINK_MAGIC_0;
    beacon.magic[1] = TELEMETRY_LINK_MAGIC_1;
    beacon.magic[2] = TELEMETRY_LINK_MAGIC_2;
    beacon.magic[3] = TELEMETRY_LINK_MAGIC_3;
    beacon.protocol_version = TELEMETRY_PROTOCOL_VERSION;
    beacon.flags = flags;
    beacon.payload_size = TELEMETRY_EMEM_SIZE;
    beacon.session_sequence = session_sequence;
    uart_write_buffer(DEBUG_UART_INDEX, (const uint8 *)&beacon, sizeof(beacon));
}

uint8 telemetry_is_capturing(void)
{
    return logger_accepting;
}

uint16 telemetry_relative_timestamp_ms(uint32 timestamp_ms)
{
    uint32 relative_ms = timestamp_ms - session_header.start_ms;

    return (uint16)((relative_ms > 0xFFFFu) ? 0xFFFFu : relative_ms);
}

TelemetryState telemetry_get_state(void)
{
    return logger_state;
}

#pragma section all restore
