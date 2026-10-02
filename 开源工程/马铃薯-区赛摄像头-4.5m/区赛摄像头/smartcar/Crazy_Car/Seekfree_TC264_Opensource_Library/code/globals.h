/**
 * @file globals.h
 * @brief 模块间及双核之间共享的数据、队列和快照接口。
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

#ifndef CODE_GLOBALS_H_
#define CODE_GLOBALS_H_

#include "zf_common_headfile.h"
#include "PID.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ==================== 控制运行变量 ==================== */
extern float left_TargetSpeed;        /* 左轮 PI 输出。 */
extern float right_TargetSpeed;       /* 右轮 PI 输出。 */
extern float max_duty_limit;          /* 输出软限幅。 */
extern volatile float actual_l_debug; /* 左轮最近一次速度估计。 */
extern volatile float actual_r_debug; /* 右轮最近一次速度估计。 */

extern float turn_control;      /* 转向控制量。 */

/* ==================== PID 对象 ==================== */
extern PID turn_PID;   /* 转向 PID。 */
extern PID speed_PID1; /* 左轮速度 PI。 */
extern PID speed_PID2; /* 右轮速度 PI。 */

/* ==================== 决策与中间量 ==================== */
extern volatile float total_dist;       /* 本次发车累计行驶距离，单位 m；motor_read_*_count() 内累加，仅用于菜单显示。 */
extern volatile uint32 time_cost;       /* 最近一帧图像处理耗时。 */
extern volatile uint8 image_stop_request; /* 图像异常停车请求。 */
extern volatile uint8 motor_fault_request; /* 电机/编码器失控停车请求：1ms 速度环置位，Duty_10ms 消费。 */

/* CPU0 产生的共享毫秒时钟与相机采集元数据。 */
extern volatile uint32 vision_clock_ms;
extern volatile uint32 vision_capture_seq;
extern volatile uint32 vision_processed_seq;

/* CPU1 -> CPU0 双槽视觉观测邮箱。vision_pose_publish_seq 最后写入，
 * CPU0 前后两次读取序号相同才接受槽内快照。 */
typedef enum
{
    VISION_OBS_FLAG_LATERAL_VALID = 1u << 0,
    VISION_OBS_FLAG_FIT_VALID = 1u << 1,
    VISION_OBS_FLAG_SIDE_OPEN = 1u << 2,
    VISION_OBS_FLAG_COVERAGE = 1u << 3,
    VISION_OBS_FLAG_EDGE_VALID = 1u << 4,
    /* 检测到直行路口(矩形四边开口数≥3 且顶边有开口):地图门控弧长重置的
     * 触发信号。每帧照常发布(不依赖横向拟合是否有效),供 CPU0 消费。 */
    VISION_OBS_FLAG_CROSSING = 1u << 5
} VisionObservationFlag;

typedef struct
{
    uint32 publish_seq;
    uint32 frame_seq;
    uint32 capture_ms;
    float capture_gyro_yaw_deg;
    float line_position_px;
    float line_slope_px_per_row;
    float fit_mse_px2;
    uint16 valid_rows;
    uint16 candidate_rows;
    uint16 row_span;
    uint16 flags;
} VisionPoseObservation;

extern volatile VisionPoseObservation vision_pose_slots[2];
extern volatile uint32 vision_pose_publish_seq;
/* 稳定读取一个尚未消费的新观测；双槽正被写入时会内部重试。 */
uint8 vision_observation_read_new(volatile uint32 *last_publish_seq,
                                  VisionPoseObservation *observation);

/* ==================== 摄像头区横向纠偏跨核通道 ====================
 * CPU1 每帧鲁棒拟合中线并给出有效性；侧开口只作诊断，三岔直行只要前向线
 * 通过完整质量门仍可有效。CPU0 仅在编辑器规划的直行摄像头区内消费，并按
 * 限速步长修正横向坐标；路口对表机制已撤销。 */
extern volatile float image_lat_offset_px; /* CPU1→CPU0：1/3 行中线偏差(像素,线偏图像右为正) */
extern volatile uint8 image_lat_valid;     /* CPU1→CPU0：横向测量本帧有效 */

/* CPU0→CPU1：当前回放点起前方 0.4m(摄像头视距35cm+余量,上限0.4m)内
 * 路线航向恒定(位于直行道)。直行三叉补线状态机的准入门：0 时立即放手，
 * 真弯道拐点不会被补线盖掉。非导航态恒为 1，便于台架上直接观察补线效果。 */
extern volatile uint8 nav_straight_ahead;

/* ==================== 停车原因（调试用） ====================
 * 每个停车触发点在停车前写入对应原因码，菜单显示出来一眼定位是哪条停的。
 * 发车时清 0。 */
typedef enum
{
    STOP_REASON_NONE = 0u,
    STOP_REASON_FRAME_WATCHDOG = 1u, /* 超过 100ms 没有新图像帧 */
    STOP_REASON_EXTREME_FRAME = 2u,  /* 连续全白或全黑 */
    STOP_REASON_PATH_DONE = 3u,
    STOP_REASON_ROLL = 5u,
    STOP_REASON_PITCH = 6u,
    STOP_REASON_ASSISTANT = 7u,
    STOP_REASON_MANUAL = 8u,
    STOP_REASON_RUN_TIMEOUT = 9u,
    STOP_REASON_MOTOR_FAULT = 11u
} StopReason;
extern volatile uint8 g_stop_reason;    /* 最近一次停车原因码，见上方 STOP_REASON_*。 */

#ifdef __cplusplus
}
#endif

#endif /* CODE_GLOBALS_H_ */
