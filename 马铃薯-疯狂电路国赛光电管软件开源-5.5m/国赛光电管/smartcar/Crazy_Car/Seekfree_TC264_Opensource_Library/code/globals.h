/**
 * @file globals.h
 * @brief 双核 IMU、光电观测和扫图队列的共享发布接口。
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

// globals.h —— CPU0/CPU1 跨核共享变量声明（定义在 globals.c）。
// 两核无函数调用，全靠共享内存里的全局变量互通。可调参数见 flash.h。

#ifndef CODE_GLOBALS_H_
#define CODE_GLOBALS_H_

#include "zf_common_headfile.h"
#include "PID.h"
#include "telemetry_format.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 控制运行变量（带 _debug 的仅供菜单/示波器观测，不参与控制）
extern float left_target_speed;        // 左轮 PI 输出
extern float right_target_speed;       // 右轮 PI 输出
extern float max_duty_limit;          // 输出软限幅
extern volatile float actual_l_debug; // 左轮最近一次速度估计
extern volatile float actual_r_debug; // 右轮最近一次速度估计
extern volatile float plan_v_debug_l; // 速度规划当前值
extern volatile float plan_v_debug_r; // 速度规划当前值
extern float speed;                   // 当前目标速度

extern float turn_control;      // 转向控制量

// PID 对象（速度 PI 增益在 control_init() 灌入）
extern PID speed_PID1; // 左轮速度 PI
extern PID speed_PID2; // 右轮速度 PI

// 决策与中间量：两核间核心消息位。volatile 因会被另一核/中断随时改写，
// 强制每次从内存重读，避免读到寄存器里的过期值。
extern volatile float total_dist;       // 本次发车累计里程(m)，motor_read_*_count() 内累加，仅菜单显示
extern volatile uint32 encoder_motion_seq; // 任一车轮出现非零编码器增量时递增，不会被左右/前后抵消
extern volatile uint32 time_cost;       // 最近一次16路光电完整扫描处理耗时(us)
extern volatile uint8 motor_fault_request; // 电机/编码器失控停车请求：1ms 速度环置位，Duty_10ms 消费

/* Runtime budget diagnostics.  All durations are microseconds; counters are
 * reset at the start of each run and copied to telemetry events at freeze. */
extern volatile uint32 control_exec_time_us;
extern volatile uint32 control_exec_max_us;
extern volatile uint32 control_exec_overrun_count;
extern volatile uint32 control_period_us;
extern volatile uint32 control_period_max_us;
extern volatile uint32 control_period_overrun_count;
extern volatile uint32 duty_10ms_exec_time_us;
extern volatile uint32 duty_10ms_exec_max_us;
extern volatile uint32 duty_10ms_overrun_count;
extern volatile uint32 nav_match_exec_time_us;
extern volatile uint32 nav_match_exec_max_us;
extern volatile uint32 nav_match_overrun_count;
extern volatile uint32 telemetry_service_exec_time_us;
extern volatile uint32 telemetry_service_exec_max_us;
extern volatile uint32 telemetry_service_overrun_count;
extern volatile uint32 phototube_scan_exec_max_us;
extern volatile uint32 phototube_scan_overrun_count;

/* CPU0 control-side diagnostics for the lock-free IMU snapshot reader.  A
 * failed read must not discard the encoder delta from the same control tick;
 * navigation keeps the last coherent sample and briefly extrapolates it. */
extern volatile uint32 imu_snapshot_read_failure_count;
extern volatile uint16 imu_snapshot_consecutive_failure_count;
extern volatile uint16 imu_snapshot_max_consecutive_failure_count;
extern volatile uint32 imu_snapshot_last_age_us;
extern volatile uint32 imu_snapshot_max_age_us;
/* CPU1 IMU ISR samples rejected because the bounded SPI transaction did not
 * complete.  A rejected frame never advances the published VQF snapshot. */
extern volatile uint32 imu_sample_failure_count;
extern volatile uint32 imu_sample_gap_count;
extern volatile uint32 imu_isr_exec_time_us;
extern volatile uint32 imu_isr_exec_max_us;
extern volatile uint32 imu_isr_overrun_count;

/* CPU1 owns the IMU/VQF state.  CPU0 consumes only this published, coherent
 * snapshot; it must never read the private IMU object directly. */
typedef struct
{
    uint32 publish_seq;
    uint32 sample_us;
    uint32 command_ack_seq;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float gyro_x_radps;
    float gyro_y_radps;
    float gyro_z_radps;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float heading_deg;
    float yaw_rate_radps;
    float pure_yaw_deg;
    float roll_delta_deg;
    float pitch_delta_deg;
    float quaternion[4];
    float bias_z_dps;
    float bias_sigma_dps;
    uint8 ready;
    uint8 zero_calibrated;
    uint8 zero_calibrating;
    uint8 rest_detected;
    uint16 startup_stationary_ms;
} ImuSnapshot;

#define IMU_SNAPSHOT_SLOT_COUNT (2u)

#if ((IMU_SNAPSHOT_SLOT_COUNT & (IMU_SNAPSHOT_SLOT_COUNT - 1u)) != 0u)
#error "IMU snapshot slot count must be a power of two"
#endif

typedef enum
{
    IMU_COMMAND_NONE = 0u,
    IMU_COMMAND_LAUNCH_REZERO = 1u << 0
} ImuCommandFlag;

extern volatile ImuSnapshot imu_snapshot_slots[IMU_SNAPSHOT_SLOT_COUNT];
extern volatile uint32 imu_snapshot_publish_seq;
/* Single-writer command mailbox: CPU0 publishes, CPU1 acknowledges. */
extern volatile uint32 imu_command_publish_seq;
extern volatile uint16 imu_command_flags;
extern volatile uint32 imu_command_ack_seq;
extern volatile uint8 imu_service_ready;

uint8 imu_snapshot_read(ImuSnapshot *snapshot);

/* CPU1-produced, CPU0-consumed compact IMU telemetry mailbox.  CPU1 owns
 * queue/head/drop/active, CPU0 owns tail and publishes the CPU1-clock session
 * epoch before opening the producer boundary. */
#define IMU_TELEMETRY_QUEUE_CAPACITY (64u)

#if ((IMU_TELEMETRY_QUEUE_CAPACITY & \
      (IMU_TELEMETRY_QUEUE_CAPACITY - 1u)) != 0u)
#error "IMU telemetry queue capacity must be a power of two"
#endif

extern volatile TelemetryImuRecord
    imu_telemetry_queue[IMU_TELEMETRY_QUEUE_CAPACITY];
extern volatile uint8 imu_telemetry_head;
extern volatile uint8 imu_telemetry_tail;
extern volatile uint8 imu_telemetry_producer_active;
extern volatile uint16 imu_telemetry_producer_drop_count;
extern volatile uint32 imu_telemetry_producer_epoch_seq;
extern volatile uint32 imu_telemetry_epoch_seq;
extern volatile uint32 imu_telemetry_session_start_us;
/* CPU0 opens/closes this producer gate; CPU1 reads it only at the IMU ISR
 * boundary.  It is shared state, so it must not be a telemetry.c private. */
extern volatile uint8 imu_telemetry_capture_enabled;

// 两核共用毫秒时钟与光电采集元数据，用于关联扫描帧和采集时刻姿态。
extern volatile uint32 vision_clock_ms;
extern volatile uint32 vision_capture_seq;
extern volatile uint32 vision_capture_ms;
extern volatile uint32 vision_processed_seq;
// CPU1 光电扫描统计：最后白点帧和统计帧均在载荷写完后发布。
extern volatile uint32 vision_line_last_white_frame_seq;
extern volatile uint32 vision_line_stats_seq;
extern volatile uint32 vision_line_stats_version;
extern volatile uint32 vision_line_stats_epoch;
extern volatile uint32 vision_line_last_white_epoch;

// CPU1 -> CPU0 线路观测队列。导航读取最新帧以保持最低延迟，扫图按发布顺序
// 消费所有仍在环形队列中的真实帧。CPU1 始终最后更新 vision_pose_publish_seq。
#define VISION_OBSERVATION_QUEUE_CAPACITY (16u)

/* The scan archive is fed by a separate queue.  A vision frame may contain
 * one line, several independent peaks, or a full lateral scan segment; none
 * of those raw hits may be discarded merely because the steering diagnostic
 * could not select one peak. */
#define VISION_SCAN_HIT_QUEUE_CAPACITY (64u)
#define VISION_SCAN_HIT_MAX_COUNT (8u)

/* Queue publication sequence 0 is the uninitialized marker.  The first
 * publication is 1; after UINT32_MAX the producer resumes at 2, skipping
 * both the marker and the already-used startup sequence.  Keep every
 * producer/consumer on these helpers: ordinary uint32 subtraction is off by
 * two at that boundary. */
static inline uint32 vision_queue_sequence_next(uint32 sequence)
{
    return (0xFFFFFFFFu == sequence) ? 2u : (sequence + 1u);
}

static inline uint32 vision_queue_sequence_distance(uint32 newer,
                                                    uint32 older)
{
    if (newer >= older)
    {
        return newer - older;
    }
    return (0xFFFFFFFFu - older) + (newer - 1u);
}

static inline uint32 vision_queue_sequence_rewind(uint32 sequence,
                                                  uint32 count)
{
    while (count > 0u)
    {
        sequence = (2u == sequence) ? 0xFFFFFFFFu : (sequence - 1u);
        count--;
    }
    return sequence;
}

#if ((VISION_SCAN_HIT_QUEUE_CAPACITY & \
      (VISION_SCAN_HIT_QUEUE_CAPACITY - 1u)) != 0u)
#error "vision scan hit queue capacity must be a power of two"
#endif

typedef enum
{
    VISION_SCAN_HIT_SINGLE = 1u << 0,
    VISION_SCAN_HIT_MULTIPLE = 1u << 1,
    VISION_SCAN_HIT_FULL_COVERAGE = 1u << 2,
    VISION_SCAN_HIT_EDGE = 1u << 3,
    /* 单个连续活动区保留下来的中心/两端几何采样。 */
    VISION_SCAN_HIT_RUN_SAMPLE = 1u << 4,
    /* A center sample is the only point that represents the route centerline.
     * Boundary samples describe the width of a wide run and must not be
     * counted as independent centerline evidence. */
    VISION_SCAN_HIT_CENTER = 1u << 5,
    VISION_SCAN_HIT_BOUNDARY = 1u << 6
} VisionScanHitFlag;

typedef struct
{
    float lateral_mm;       // 阵列前方100 mm处，右侧为正
    uint8 confidence;       // 帧内归一化权重；总和受该帧实际质量限制
    uint8 flags;            // VisionScanHitFlag
} VisionScanHit;

typedef struct
{
    uint32 publish_seq;
    uint32 frame_seq;
    uint32 capture_ms;
    uint32 publish_ms;
    uint32 capture_epoch;    // CPU0运行代次；按ADC扫描开始时刻锁存
    uint16 hit_count;
    uint16 source_frame_gap; // CPU1扫描帧号跳变
    uint16 flags;
    uint8 frame_quality;
    uint8 reserved;
    VisionScanHit hits[VISION_SCAN_HIT_MAX_COUNT];
} VisionScanHitPacket;

#if ((VISION_OBSERVATION_QUEUE_CAPACITY & \
      (VISION_OBSERVATION_QUEUE_CAPACITY - 1u)) != 0u)
#error "vision observation queue capacity must be a power of two"
#endif

// flags 位标志：用 1u<<n 让每种情况占独立比特，可按位或组合。
typedef enum
{
    VISION_OBS_FLAG_LATERAL_VALID = 1u << 0,
    VISION_OBS_FLAG_FIT_VALID = 1u << 1,
    VISION_OBS_FLAG_SIDE_OPEN = 1u << 2,
    VISION_OBS_FLAG_COVERAGE = 1u << 3,
    VISION_OBS_FLAG_EDGE_VALID = 1u << 4,
    // 检测到直行路口(矩形四边开口数≥3 且顶边有开口)：地图门控弧长重置的触发信号。
    // 每帧照常发布(不依赖横向拟合是否有效)，供 CPU0 消费。
    VISION_OBS_FLAG_CROSSING = 1u << 5,
    // 观测来自固定ADC中点分割后的真实16路光电阵列，不是旧中线拟合。
    // line_position_mm 表示轮轴前方98.5mm处、线路相对阵列中心的横向位置。
    VISION_OBS_FLAG_VIRTUAL_LINE = 1u << 6,
    // 16路所在近场带被线路区域整体覆盖。位置按阵列中心输出，但该帧
    // 不包含可用于 yaw bias 学习的横向变化信息。
    VISION_OBS_FLAG_FULL_COVERAGE = 1u << 7,
    // 当前帧存在候选峰，但无法从连续性、宽度和唯一性上确定哪一个是线路。
    // 该帧仍发布供诊断和保护使用，但不能生成光电目标 yaw。
    VISION_OBS_FLAG_SELECTION_AMBIGUOUS = 1u << 8
} VisionObservationFlag;

// 一帧视觉观测快照。字段顺序不能动——两核靠一模一样的内存布局传递它。
typedef struct
{
    uint32 publish_seq;             // 发布序号，用于前后校验
    uint32 frame_seq;               // 采集帧号
    uint32 capture_ms;              // 拍摄时刻毫秒时间戳
    uint32 capture_epoch;           // ADC扫描开始时CPU0发布的运行代次
    float capture_gyro_yaw_deg;     // 拍摄时刻陀螺仪偏航角(度)
    float line_position_px;         // 拟合中线横向位置(像素)
    float line_position_mm;         // 虚拟/真实光电阵列测得的前方线路位置(mm，右为正)
    float line_slope_px_per_row;    // 中线斜率(每行偏移像素)
    float fit_mse_px2;              // 拟合均方误差(像素²)，越小越好
    uint16 valid_rows;              // 参与拟合的有效行数
    uint16 candidate_rows;          // 候选行数
    uint16 row_span;                // 遗留覆盖跨度：旧视觉为行，单排阵列为首末活动通道间距(mm)
    uint16 flags;                   // VISION_OBS_FLAG_* 位标志组合
    uint8 white_percent;            // 遗留字段名：16路低电平线路活动占比(0..100)
    uint8 reserved[3];              // 保留旧字段偏移并保持4字节对齐
    uint16 active_mask;             // 16路固定阈值低电平线路二值位图
    uint8 peak_count;               // 连续活动候选峰数量
    uint8 threshold_gray;           // 固定12位ADC阈值除以16后的诊断值
} VisionPoseObservation;

extern volatile VisionPoseObservation
    vision_pose_slots[VISION_OBSERVATION_QUEUE_CAPACITY];
extern volatile uint32 vision_pose_publish_seq; // 最新发布序号，读时防撕裂
extern volatile uint8 vision_pose_publish_slot; // 最新发布序号对应的物理槽
// 读取最新一帧，供1ms导航链使用；中间帧允许被跳过，以保持控制延迟最低。
uint8 vision_observation_read_new(volatile uint32 *last_publish_seq,
                                  VisionPoseObservation *observation);

extern volatile VisionScanHitPacket
    vision_scan_hit_slots[VISION_SCAN_HIT_QUEUE_CAPACITY];
extern volatile uint32 vision_scan_hit_publish_seq;
extern volatile uint8 vision_scan_hit_publish_slot;
/* CPU0 publishes a monotonically changing capture epoch at every run-open and
 * run-close boundary.  CPU1 latches it before the first ADC channel and
 * acknowledges that exact value, so an in-flight frame cannot cross runs. */
extern volatile uint32 vision_scan_run_epoch;
extern volatile uint32 vision_scan_run_epoch_ack;
uint8 vision_scan_hit_read_next(volatile uint32 *last_publish_seq,
                                VisionScanHitPacket *packet);

/* CPU1扫描调度统计：一次调度延迟记为 source drop，期间跳过的1ms
 * 时隙记为 source frame skipped；真实ADC帧不会被静默丢弃。 */
extern volatile uint32 vision_capture_drop_count;
extern volatile uint32 vision_capture_skipped_count;
extern volatile uint32 vision_capture_adc_failure_count;
/* CPU1 phototube producer signals consumed by the CPU0 menu task. */
extern volatile uint32 phototube_menu_frame_seq;
extern volatile uint8 phototube_menu_ready_signal;

// 停车原因（调试用）：各触发点在停车前把原因码写进 g_stop_reason，菜单显示是哪条停的。
// 每次发车清 0。
typedef enum
{
    STOP_REASON_NONE = 0u,
    STOP_REASON_LINE_LOST = 2u,      // 16路光电沿程200mm持续全黑
    STOP_REASON_PATH_DONE = 3u,
    STOP_REASON_PLAN_MISMATCH = 4u,
    STOP_REASON_ROLL = 5u,
    STOP_REASON_PITCH = 6u,
    STOP_REASON_ASSISTANT = 7u,
    STOP_REASON_MANUAL = 8u,
    STOP_REASON_SPEED_TEST_DONE = 10u,
    STOP_REASON_MOTOR_FAULT = 11u,
    STOP_REASON_TRIGGER_TRAP = 12u,
    STOP_REASON_IMU_STALE = 13u
} StopReason;
extern volatile uint8 g_stop_reason;    // 最近一次停车原因码，见 STOP_REASON_*

#ifdef __cplusplus
}
#endif

#endif /* CODE_GLOBALS_H_ */
