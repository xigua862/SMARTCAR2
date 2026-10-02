/**
 * @file gyroscope.c
 * @brief CPU1 IMU 采样、VQF、上电零偏和发车姿态重锚。
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
 * @note CPU1 是 VQF 状态唯一写者，CPU0 只读完整快照。
 */

// gyroscope.c — LSM6DSV16X 姿态与航向。芯片 SFLP 关闭，使用软件 VQF。
// roll/pitch 供姿态保护；运行主航向使用 VQF 6D yaw，相对发车零点输出。
// 运行在 CPU1，由 IMU 数据就绪外部中断(INT2，实测约 480Hz)驱动采样。

#include "gyroscope.h"
#include "competition_config.h"
#include "telemetry.h"
#include "vqf_adapter.h"
#include <math.h>

#pragma section all "cpu1_dsram"

typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float roll;
    float pitch;
    float yaw;
    float quaternion[4];
} ImuWorkingState;

static ImuWorkingState imu_data;

static uint8 gyro_ready = 0u;   // IMU 初始化成功标志，未就绪时拒绝一切采样/校准

// dt 逐帧用微秒时钟实测，不假设固定 ODR——本模式实测并非名义 480Hz，
// 固定 dt 曾造成 yaw 系统性少计约 25%(4 连右簇 360° 只读出 270°)。名义值仅供首帧/野值兜底和零飘计数
#define GYRO_NOMINAL_PERIOD_S (1.0f / (float)GYRO_NOMINAL_RATE_HZ)
#define VQF_ACCEL_UPDATE_DIVIDER (4u)
#define STANDARD_GRAVITY_MPS2 (9.80665f)
// VQF先确认持续静止，再累计这些原始样本；编码器可见或引起IMU非稳态的运动会整窗重采。
#define GYRO_ZERO_SAMPLE_COUNT ((uint16)((GYRO_NOMINAL_RATE_HZ * GYRO_ZERO_CALIBRATION_MS) / 1000u))
#define GYRO_STARTUP_STATIONARY_MAX_US (65535000u)

static uint32 gyro_last_sample_us = 0u; // 上一帧微秒时间戳，算两帧真实间隔
static uint8 gyro_dt_primed = 0u;
static uint8 vqf_accel_update_counter = 0u;
static uint32 gyro_runtime_epoch_seq = 0u;
static uint8 gyro_consecutive_sample_failures = 0u;

static void gyro_record_isr_time(uint32 start_us)
{
    uint32 elapsed_us = system_getval_us() - start_us;
    imu_isr_exec_time_us = elapsed_us;
    if (elapsed_us > imu_isr_exec_max_us)
    {
        imu_isr_exec_max_us = elapsed_us;
    }
    if ((elapsed_us > GYRO_ISR_PERIOD_US) &&
        (imu_isr_overrun_count < 0xFFFFFFFFu))
    {
        imu_isr_overrun_count++;
    }
}

// 陀螺刻度因子：乘在角速度换算源头修正灵敏度乘性误差。标定：车平放原地精转一整圈读 Yaw=m，
// 新因子=旧因子×360/m。真实 MEMS 刻度误差仅 ±1~3%，结果必须落在 1.00±0.03；显著偏离说明另有
// 根因(dt 计时、混叠、量程)，禁止用本因子掩盖。历史教训：曾标出 1.2，实为固定 dt 少计 25% 的
// 双重补偿，dt 改实测后 1.2 变成纯放大器，是"提前触发"帮凶。标定时车必须放平(手持倾斜读数小于 360 是正确的倾斜补偿)。

static float gyro_zero_offset_x = 0.0f; // 三轴角速度零偏，校准算得后每帧扣掉
static float gyro_zero_offset_y = 0.0f;
static float gyro_zero_offset_z = 0.0f;
static volatile uint8 gyro_zero_start_requested = 0u; // 校准状态机标志(volatile：跨中断/主循环共享)
static volatile uint8 gyro_zero_calibrated = 0u;
static volatile uint8 gyro_zero_calibrating = 0u;
static volatile uint16 gyro_zero_sample_count = 0u;
static float gyro_zero_sum_x = 0.0f; // 采样期三轴累加和，除以帧数即零偏
static float gyro_zero_sum_y = 0.0f;
static float gyro_zero_sum_z = 0.0f;
static float gyro_static_corrected_radps[3] = {0.0f, 0.0f, 0.0f};
static uint32 gyro_zero_encoder_motion_seq = 0u;
static uint32 gyro_startup_stationary_us = 0u;

// 姿态保护相对零点：校准完成(车平放赛道)时记录当前倾角。板子倒装原始 roll 近 ±180°，保护看相对变化而非绝对值
static float attitude_reference_roll = 0.0f;
static float attitude_reference_pitch = 0.0f;
static uint8 attitude_reference_valid = 0u;

// 航向符号适配：当前地图/导航航向使用与路径表一致的符号。正方向由 IMU 安装方向决定。
// 台架判定：手拿车向右(俯视顺时针)转，调试页 Yaw 变大→1，变小→0。航向源换过时必须重新实测。
// 本板实测：右转时世界系 yaw 变小，因此地图航向取反。

static volatile float gyro_pure_yaw_deg = 0.0f; // 上电后连续累加的纯陀螺参考，软件归零和编码器逻辑均不写它
static volatile float vqf_yaw_reference_deg = 0.0f;
static float vqf_yaw_last = 0.0f;
static volatile uint8 vqf_heading_reanchor_pending = 0u;
static float vqf_bias_radps[3] = {0.0f, 0.0f, 0.0f};
static volatile float vqf_bias_sigma_radps = 0.0f;
static volatile uint8 attitude_latch_request = 0u; // 发车重锚请求：以当前姿态重锁 roll/pitch 保护零点
static uint32 gyro_last_command_seq = 0u;

static float gyro_normalize_angle_delta(float angle); // 前向声明
static void gyro_zero_calibration_start(void);

static void gyro_zero_calibration_reset_accumulator(void)
{
    gyro_zero_sum_x = 0.0f;
    gyro_zero_sum_y = 0.0f;
    gyro_zero_sum_z = 0.0f;
    gyro_zero_sample_count = 0u;
}

static void gyro_update_startup_stationary_time(float dt)
{
    float rate_squared;
    uint32 increment_us;

    if (!gyro_zero_calibrated || vqf_heading_reanchor_pending ||
        !vqf_adapter_rest_measurements_stable())
    {
        gyro_startup_stationary_us = 0u;
        return;
    }

    /* Use only the fixed startup offset here.  VQF's dynamic bias estimate
     * must not hide a real slow constant rotation from the launch gate. */
    rate_squared =
        gyro_static_corrected_radps[0] * gyro_static_corrected_radps[0] +
        gyro_static_corrected_radps[1] * gyro_static_corrected_radps[1] +
        gyro_static_corrected_radps[2] * gyro_static_corrected_radps[2];
    if (rate_squared > (GYRO_STARTUP_STATIONARY_RATE_RADPS *
                        GYRO_STARTUP_STATIONARY_RATE_RADPS))
    {
        gyro_startup_stationary_us = 0u;
        return;
    }

    increment_us = (uint32)(dt * 1000000.0f + 0.5f);
    if (gyro_startup_stationary_us >=
        (GYRO_STARTUP_STATIONARY_MAX_US - increment_us))
    {
        gyro_startup_stationary_us = GYRO_STARTUP_STATIONARY_MAX_US;
    }
    else
    {
        gyro_startup_stationary_us += increment_us;
    }
}

static void vqf_publish_orientation(void)
{
    float quaternion[4];
    float sinp;

    vqf_adapter_get_quaternion(quaternion);
    imu_data.roll = atan2f(2.0f * (quaternion[0] * quaternion[1] +
                                   quaternion[2] * quaternion[3]),
                           1.0f - 2.0f * (quaternion[1] * quaternion[1] +
                                          quaternion[2] * quaternion[2])) *
                    (180.0f / PI);
    sinp = 2.0f * (quaternion[0] * quaternion[2] -
                   quaternion[3] * quaternion[1]);
    if (sinp > 1.0f)
    {
        sinp = 1.0f;
    }
    if (sinp < -1.0f)
    {
        sinp = -1.0f;
    }
    imu_data.pitch = asinf(sinp) * (180.0f / PI);
    imu_data.yaw = atan2f(2.0f * (quaternion[0] * quaternion[3] +
                                  quaternion[1] * quaternion[2]),
                          1.0f - 2.0f * (quaternion[2] * quaternion[2] +
                                         quaternion[3] * quaternion[3])) *
                   (180.0f / PI);
    imu_data.quaternion[0] = quaternion[0];
    imu_data.quaternion[1] = quaternion[1];
    imu_data.quaternion[2] = quaternion[2];
    imu_data.quaternion[3] = quaternion[3];
}

// 采集并发布一帧 IMU 数据，同时完成 yaw 积分。数据就绪中断内调用，dt=本帧实测采样间隔(秒)。
// 流程：读寄存器→静态零偏→VQF 运动/静止零偏与姿态→发布航向和兼容累计量。
static uint8 imu_get_values(float dt)
{
    int16 acc_x;
    int16 acc_y;
    int16 acc_z;
    int16 gyro_x;
    int16 gyro_y;
    int16 gyro_z;
    float gyro_input_radps[3];
    float accel_mps2[3];
    float gyro_heading_delta_deg;

    // 从 IMU660RC 读一帧原始整数(陀螺 + 加速度)
    if (!imu660rc_get_gyro() || !imu660rc_get_acc())
    {
        return 0u;
    }

    acc_x = imu660rc_acc_x;
    acc_y = imu660rc_acc_y;
    acc_z = imu660rc_acc_z;
    gyro_x = imu660rc_gyro_x;
    gyro_y = imu660rc_gyro_y;
    gyro_z = imu660rc_gyro_z;

    // 上电静态均值先去掉大部分零偏；VQF 再估计温漂和剩余动态零偏。
    if (gyro_zero_calibrated)
    {
        gyro_input_radps[0] = imu660rc_gyro_transition((float)gyro_x - gyro_zero_offset_x) * (PI / 180.0f) * GYRO_SCALE;
        gyro_input_radps[1] = imu660rc_gyro_transition((float)gyro_y - gyro_zero_offset_y) * (PI / 180.0f) * GYRO_SCALE;
        gyro_input_radps[2] = imu660rc_gyro_transition((float)gyro_z - gyro_zero_offset_z) * (PI / 180.0f) * GYRO_SCALE;
    }
    else
    {
        gyro_input_radps[0] = imu660rc_gyro_transition(gyro_x) * (PI / 180.0f) * GYRO_SCALE;
        gyro_input_radps[1] = imu660rc_gyro_transition(gyro_y) * (PI / 180.0f) * GYRO_SCALE;
        gyro_input_radps[2] = imu660rc_gyro_transition(gyro_z) * (PI / 180.0f) * GYRO_SCALE;
    }
    gyro_static_corrected_radps[0] = gyro_input_radps[0];
    gyro_static_corrected_radps[1] = gyro_input_radps[1];
    gyro_static_corrected_radps[2] = gyro_input_radps[2];

    // 遥测保留原始 g；VQF 按接口要求接收 m/s^2，并用内部二阶滤波处理动态加速度。
    imu_data.acc_x = imu660rc_acc_transition(acc_x);
    imu_data.acc_y = imu660rc_acc_transition(acc_y);
    imu_data.acc_z = imu660rc_acc_transition(acc_z);
    accel_mps2[0] = imu_data.acc_x * STANDARD_GRAVITY_MPS2;
    accel_mps2[1] = imu_data.acc_y * STANDARD_GRAVITY_MPS2;
    accel_mps2[2] = imu_data.acc_z * STANDARD_GRAVITY_MPS2;

    vqf_adapter_update_gyro(gyro_input_radps, dt);
    if (0u == vqf_accel_update_counter)
    {
        vqf_adapter_update_accel(accel_mps2);
        vqf_bias_sigma_radps = vqf_adapter_get_bias(vqf_bias_radps);
    }
    vqf_accel_update_counter++;
    if (vqf_accel_update_counter >= VQF_ACCEL_UPDATE_DIVIDER)
    {
        vqf_accel_update_counter = 0u;
    }
    vqf_publish_orientation();

    // 对外角速度使用与 VQF 姿态相同的剩余零偏估计，角速度闭环与日志看到同一物理量。
    imu_data.gyro_x = gyro_input_radps[0] - vqf_bias_radps[0];
    imu_data.gyro_y = gyro_input_radps[1] - vqf_bias_radps[1];
    imu_data.gyro_z = gyro_input_radps[2] - vqf_bias_radps[2];

    // 发车重锚：车已摆正在赛道上，以当前姿态为 roll/pitch 保护零点
    if (attitude_latch_request)
    {
        attitude_reference_roll = imu_data.roll;
        attitude_reference_pitch = imu_data.pitch;
        attitude_reference_valid = 1u;
        attitude_latch_request = 0u;
    }

    /* VQF reset 后的第一帧会根据重力重新建立倒装安装姿态，欧拉 yaw
     * 可能随 roll/pitch 重建发生大跳变。这一帧只建立上电航向零点，
     * 不能当作车身真实转动。 */
    if (vqf_heading_reanchor_pending)
    {
        vqf_yaw_last = imu_data.yaw;
        vqf_yaw_reference_deg = imu_data.yaw;
        vqf_heading_reanchor_pending = 0u;
        gyro_heading_delta_deg = 0.0f;
    }
    // VQF 世界系航向增量统一为地图右转正。
    else
    {
#if VQF_YAW_RIGHT_POSITIVE
        gyro_heading_delta_deg = gyro_normalize_angle_delta(imu_data.yaw - vqf_yaw_last);
#else
        gyro_heading_delta_deg = -gyro_normalize_angle_delta(imu_data.yaw - vqf_yaw_last);
#endif
        vqf_yaw_last = imu_data.yaw;
    }

    // 纯陀螺累计只服务采集时刻诊断，不参与控制航向零点。
    gyro_pure_yaw_deg += gyro_heading_delta_deg;
    return 1u;
}

static float gyro_current_heading_deg(void)
{
    if (!gyro_zero_calibrated || vqf_heading_reanchor_pending)
    {
        return 0.0f;
    }
    return gyro_normalize_angle_delta(
        VQF_YAW_MAP_SIGN * (imu_data.yaw - vqf_yaw_reference_deg));
}

static void gyro_process_command(void)
{
    uint32 sequence = imu_command_publish_seq;
    uint16 flags;

    if ((0u == sequence) || (sequence == gyro_last_command_seq))
    {
        return;
    }
    __dsync();
    flags = imu_command_flags;
    __dsync();
    if (sequence != imu_command_publish_seq)
    {
        return;
    }

    /* A launch reference is valid only after the post-calibration VQF frame
     * has rebuilt orientation.  Until then the request remains pending and is
     * deliberately not acknowledged. */
    if ((flags & IMU_COMMAND_LAUNCH_REZERO) != 0u)
    {
        if (!gyro_zero_calibrated || vqf_heading_reanchor_pending)
        {
            return;
        }
        attitude_reference_roll = imu_data.roll;
        attitude_reference_pitch = imu_data.pitch;
        attitude_reference_valid = 1u;
        attitude_latch_request = 0u;
        vqf_yaw_reference_deg = imu_data.yaw;
    }
    gyro_last_command_seq = sequence;
}

static void gyro_publish_snapshot(uint32 sample_us)
{
    uint32 sequence = imu_snapshot_publish_seq + 1u;
    uint8 slot_index;
    uint8 quaternion_index;
    volatile ImuSnapshot *slot;

    if (0u == sequence)
    {
        sequence = 1u;
    }
    slot_index = (uint8)(sequence & (IMU_SNAPSHOT_SLOT_COUNT - 1u));
    slot = &imu_snapshot_slots[slot_index];
    slot->sample_us = sample_us;
    slot->command_ack_seq = gyro_last_command_seq;
    slot->acc_x_g = imu_data.acc_x;
    slot->acc_y_g = imu_data.acc_y;
    slot->acc_z_g = imu_data.acc_z;
    slot->gyro_x_radps = imu_data.gyro_x;
    slot->gyro_y_radps = imu_data.gyro_y;
    slot->gyro_z_radps = imu_data.gyro_z;
    slot->roll_deg = imu_data.roll;
    slot->pitch_deg = imu_data.pitch;
    slot->yaw_deg = imu_data.yaw;
    slot->heading_deg = gyro_current_heading_deg();
    slot->yaw_rate_radps = VQF_YAW_RATE_MAP_SIGN * imu_data.gyro_z;
    slot->pure_yaw_deg = gyro_pure_yaw_deg;
    slot->roll_delta_deg = attitude_reference_valid ?
        gyro_normalize_angle_delta(imu_data.roll - attitude_reference_roll) :
        0.0f;
    slot->pitch_delta_deg = attitude_reference_valid ?
        gyro_normalize_angle_delta(imu_data.pitch - attitude_reference_pitch) :
        0.0f;
    for (quaternion_index = 0u; quaternion_index < 4u; quaternion_index++)
    {
        slot->quaternion[quaternion_index] =
            imu_data.quaternion[quaternion_index];
    }
    slot->bias_z_dps = vqf_bias_radps[2] * (180.0f / PI);
    slot->bias_sigma_dps = vqf_bias_sigma_radps * (180.0f / PI);
    slot->ready = gyro_ready;
    slot->zero_calibrated = gyro_zero_calibrated;
    slot->zero_calibrating =
        (uint8)(gyro_zero_start_requested || gyro_zero_calibrating);
    slot->rest_detected = (uint8)(0 != vqf_adapter_rest_detected());
    slot->startup_stationary_ms =
        (uint16)(gyro_startup_stationary_us / 1000u);
    __dsync();
    slot->publish_seq = sequence;
    __dsync();
    imu_snapshot_publish_seq = sequence;
}

// CPU1光电任务直接读同核工作态；CPU0不能调用此接口。
float gyro_cpu1_get_pure_yaw_deg(void)
{
    return gyro_pure_yaw_deg;
}

// 以下CPU0访问器只读CPU1发布快照，不触碰VQF内部状态。
// VQF yaw 相对发车零点，统一到地图使用的航向符号。
float gyro_get_heading_deg(void)
{
    ImuSnapshot snapshot;
    return imu_snapshot_read(&snapshot) ? snapshot.heading_deg : 0.0f;
}

uint32 gyro_request_launch_rezero(void)
{
    uint32 sequence;
    uint32 interrupt_state;

    if (!imu_service_ready)
    {
        return 0u;
    }
    interrupt_state = interrupt_global_disable();
    sequence = imu_command_publish_seq + 1u;
    if (0u == sequence)
    {
        sequence = 1u;
    }
    imu_command_flags = IMU_COMMAND_LAUNCH_REZERO;
    __dsync();
    imu_command_publish_seq = sequence;
    __dsync();
    interrupt_global_enable(interrupt_state);
    return sequence;
}

uint8 gyro_command_acknowledged(uint32 command_seq)
{
    __dsync();
    return (uint8)((0u != command_seq) &&
                   (imu_command_ack_seq == command_seq));
}

uint8 gyro_cpu1_service_ready(void)
{
    uint8 ready;

    __dsync();
    ready = imu_service_ready;
    __dsync();
    return ready;
}

// 角度差限制到 [-180,180]，处理跨 ±180 环绕(如 +179→-178 实际只转 +3°，相减得 -357° 加 360 还原)
static float gyro_normalize_angle_delta(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

// 相对 roll(相对校准完成时的姿态零点，倒装安装角被自然扣除)。零点未建立时返回 0，保护不动作
float gyro_get_roll_delta(void)
{
    ImuSnapshot snapshot;
    return imu_snapshot_read(&snapshot) ? snapshot.roll_delta_deg : 0.0f;
}

// 相对 pitch，同 roll
float gyro_get_pitch_delta(void)
{
    ImuSnapshot snapshot;
    return imu_snapshot_read(&snapshot) ? snapshot.pitch_delta_deg : 0.0f;
}

// 在 IMU 外部中断中采一帧零飘样本，采够 GYRO_ZERO_SAMPLE_COUNT 后求均值即三轴零偏并请求航向归零。
// 原理：VQF确认静止后，连续窗口的原始均值就是零偏；可观测运动会使整窗重采。
static void gyro_zero_calibration_sample(void)
{
    uint32 encoder_sequence;
    uint32 encoder_sequence_after;

    if (!gyro_zero_calibrating)
    {
        return;
    }

    __dsync();
    encoder_sequence = encoder_motion_seq;
    __dsync();
    if (encoder_sequence != gyro_zero_encoder_motion_seq)
    {
        gyro_zero_encoder_motion_seq = encoder_sequence;
        gyro_zero_calibration_reset_accumulator();
        return;
    }

    /* VQF requires gyro and acceleration measurements to remain inside its
     * rest gates before any raw sample can influence the persistent offset.
     * A touch during accumulation invalidates the complete candidate window. */
    if (!vqf_adapter_rest_detected())
    {
        gyro_zero_calibration_reset_accumulator();
        return;
    }

    gyro_zero_sum_x += (float)imu660rc_gyro_x;
    gyro_zero_sum_y += (float)imu660rc_gyro_y;
    gyro_zero_sum_z += (float)imu660rc_gyro_z;
    gyro_zero_sample_count++;

    if (gyro_zero_sample_count >= GYRO_ZERO_SAMPLE_COUNT)
    {
        /* The encoder sequence can advance while this ISR is accumulating
         * the final sample.  Recheck it before committing the candidate so a
         * wheel movement in that narrow window invalidates the whole window
         * instead of being discovered only after calibration is marked done. */
        __dsync();
        encoder_sequence_after = encoder_motion_seq;
        __dsync();
        if (encoder_sequence_after != gyro_zero_encoder_motion_seq)
        {
            gyro_zero_encoder_motion_seq = encoder_sequence_after;
            gyro_zero_calibration_reset_accumulator();
            return;
        }
        // 采够帧：求均值即零偏，切到已校准并请求航向归零
        gyro_zero_offset_x = gyro_zero_sum_x / (float)gyro_zero_sample_count;
        gyro_zero_offset_y = gyro_zero_sum_y / (float)gyro_zero_sample_count;
        gyro_zero_offset_z = gyro_zero_sum_z / (float)gyro_zero_sample_count;
        gyro_zero_calibrating = 0u;
        gyro_zero_calibrated = 1u;
        // 静态零偏开始生效后重新建立 VQF，避免前一秒未扣偏置的数据残留在状态中。
        vqf_adapter_init();
        vqf_accel_update_counter = 0u;
        vqf_bias_radps[0] = 0.0f;
        vqf_bias_radps[1] = 0.0f;
        vqf_bias_radps[2] = 0.0f;
        vqf_bias_sigma_radps = 0.0f;
        vqf_yaw_last = 0.0f;
        vqf_yaw_reference_deg = 0.0f;
        gyro_pure_yaw_deg = 0.0f;
        gyro_startup_stationary_us = 0u;
        vqf_heading_reanchor_pending = 1u;
        // 下一采样帧用已校准数据重建姿态和航向参考。
        attitude_reference_valid = 0u;
        attitude_latch_request = 1u;
    }
}

// 在 IMU 外部中断中响应开始校准请求：按键/发车流程只置请求位，真正清累加器和进入采样状态在此执行
static void gyro_zero_calibration_begin(void)
{
    if (!gyro_zero_start_requested)
    {
        return;
    }

    gyro_zero_start_requested = 0u;

    // IMU 没就绪却要求校准：撤销请求并发布未就绪，不能从CPU1关闭全芯片PIT。
    if (!gyro_ready)
    {
        gyro_zero_calibrated = 0u;
        gyro_zero_calibrating = 0u;
        imu_service_ready = 0u;
        return;
    }

    // 清空累加器，先等待VQF确认静止，再进入有效样本窗口。
    gyro_zero_calibration_reset_accumulator();
    __dsync();
    gyro_zero_encoder_motion_seq = encoder_motion_seq;
    __dsync();
    gyro_zero_calibrated = 0u;
    gyro_zero_calibrating = 1u;
    gyro_startup_stationary_us = 0u;
    attitude_reference_valid = 0u;
}

// 清空应用层 IMU 缓存，只在初始化阶段调用
static void imu_data_clear(void)
{
    imu_data.acc_x = 0.0f;
    imu_data.acc_y = 0.0f;
    imu_data.acc_z = 0.0f;
    imu_data.gyro_x = 0.0f;
    imu_data.gyro_y = 0.0f;
    imu_data.gyro_z = 0.0f;
    imu_data.roll = 0.0f;
    imu_data.pitch = 0.0f;
    imu_data.yaw = 0.0f;
    imu_data.quaternion[0] = 0.0f;
    imu_data.quaternion[1] = 0.0f;
    imu_data.quaternion[2] = 0.0f;
    imu_data.quaternion[3] = 0.0f;
    gyro_static_corrected_radps[0] = 0.0f;
    gyro_static_corrected_radps[1] = 0.0f;
    gyro_static_corrected_radps[2] = 0.0f;
}

// 初始化 IMU：四元数/SFLP 禁用，INT2 配为陀螺数据就绪中断(480Hz)驱动采样。
// 就绪后立即请求一次非阻塞零飘校准，由外部中断在开机静置阶段采完；发车流程只归零航向不再重复校准
uint8 gyro_cpu1_init(void)
{
    uint8 init_state;

    imu_data_clear();
    gyro_zero_calibrated = 0u;
    gyro_zero_calibrating = 0u;
    gyro_zero_start_requested = 0u;
    gyro_zero_sample_count = 0u;
    gyro_zero_offset_x = 0.0f;
    gyro_zero_offset_y = 0.0f;
    gyro_zero_offset_z = 0.0f;
    gyro_zero_encoder_motion_seq = encoder_motion_seq;
    gyro_startup_stationary_us = 0u;
    gyro_ready = 0u;
    imu_service_ready = 0u;
    imu_snapshot_publish_seq = 0u;
    imu_sample_failure_count = 0u;
    imu_sample_gap_count = 0u;
    gyro_consecutive_sample_failures = 0u;
    imu_isr_exec_time_us = 0u;
    imu_isr_exec_max_us = 0u;
    imu_isr_overrun_count = 0u;
    imu_command_publish_seq = 0u;
    imu_command_ack_seq = 0u;
    imu_command_flags = IMU_COMMAND_NONE;
    gyro_last_command_seq = 0u;
    gyro_last_sample_us = 0u;
    gyro_dt_primed = 0u;
    gyro_runtime_epoch_seq = imu_telemetry_epoch_seq;
    gyro_pure_yaw_deg = 0.0f;
    vqf_yaw_last = 0.0f;
    vqf_yaw_reference_deg = 0.0f;
    vqf_heading_reanchor_pending = 0u;
    vqf_accel_update_counter = 0u;
    vqf_bias_radps[0] = 0.0f;
    vqf_bias_radps[1] = 0.0f;
    vqf_bias_radps[2] = 0.0f;
    vqf_bias_sigma_radps = 0.0f;
    vqf_adapter_init();

    // 关掉芯片内置 SFLP 四元数输出，本模块使用开源 VQF。
    init_state = imu660rc_init(IMU660RC_QUARTERNION_DISABLE);

    if (0u == init_state)
    {
        gyro_ready = 1u;
        // 唯一一次零偏校准放在风机启动前；VQF确认静止后才累计，碰车会清空窗口并延后完成。
        // 发车时不在风机振动工况下重采，避免振动混叠造成每次零偏不同。
        gyro_zero_calibration_start();
        gyro_publish_snapshot(system_getval_us());
        __dsync();
        imu_service_ready = 1u;
    }

    // CPU1外设故障不能调用zf_assert：断言会关闭全芯片PIT，连CPU0菜单和按键也会被冻结。
    // 保持service_ready=0即可让发车事务明确拒绝，同时CPU0界面和光电诊断继续工作。
    return init_state;
}

// CPU1 IMU中断：采样/VQF、消费CPU0命令、发布完整快照，最后才确认命令。
void gyro_cpu1_external_interrupt_update(void)
{
    uint32 now_us;
    uint32 isr_start_us = system_getval_us();
    float dt;

    if (gyro_runtime_epoch_seq != imu_telemetry_epoch_seq)
    {
        gyro_runtime_epoch_seq = imu_telemetry_epoch_seq;
        imu_sample_failure_count = 0u;
        imu_sample_gap_count = 0u;
        imu_isr_exec_time_us = 0u;
        imu_isr_exec_max_us = 0u;
        imu_isr_overrun_count = 0u;
    }

    // IMU 没就绪：校准状态全部打回失败并直接返回，不做任何积分
    if (!gyro_ready)
    {
        gyro_zero_calibrated = 0u;
        gyro_zero_calibrating = 0u;
        gyro_zero_start_requested = 0u;
        gyro_record_isr_time(isr_start_us);
        return;
    }

    // 逐帧实测采样间隔：真实 ODR 多少就积多少。首帧无基准或间隔野值(毛刺双沿/中断长时阻塞)时退回名义节拍
    now_us = system_getval_us();
    if (!gyro_dt_primed)
    {
        dt = GYRO_NOMINAL_PERIOD_S;
        gyro_dt_primed = 1u;
    }
    else
    {
        uint32 sample_interval_us = now_us - gyro_last_sample_us;
        if ((sample_interval_us > 5000u) &&
            (imu_sample_gap_count < 0xFFFFFFFFu))
        {
            imu_sample_gap_count++;
        }
        // 两帧微秒差 × 1e-6 = 秒；落在合理区间外视为野值退回名义周期
        dt = (float)sample_interval_us * 1.0e-6f;
        if ((dt < 0.0002f) || (dt > 0.02f))
        {
            dt = GYRO_NOMINAL_PERIOD_S;
        }
    }
    gyro_last_sample_us = now_us;

    if (!imu_get_values(dt))       // SPI失败时不发布伪造的新快照
    {
        if (imu_sample_failure_count < 0xFFFFFFFFu)
        {
            imu_sample_failure_count++;
        }
        if (gyro_consecutive_sample_failures < 0xFFu)
        {
            gyro_consecutive_sample_failures++;
        }
        if (gyro_consecutive_sample_failures >= GYRO_SERVICE_FAILURE_LIMIT)
        {
            /* service_ready is a live health indication, not only an init
             * latch.  A run may not launch against an IMU that stopped
             * returning samples after successful initialization. */
            imu_service_ready = 0u;
        }
        gyro_record_isr_time(isr_start_us);
        return;
    }
    gyro_consecutive_sample_failures = 0u;
    imu_service_ready = gyro_ready;
    gyro_zero_calibration_begin(); // 若有请求，进入校准采样状态
    gyro_zero_calibration_sample();// 校准中则采一帧零偏样本
    gyro_update_startup_stationary_time(dt);
    gyro_process_command();
    gyro_publish_snapshot(now_us);
    __dsync();
    imu_command_ack_seq = gyro_last_command_seq;
    telemetry_capture_imu_isr(imu_data.acc_x,
                              imu_data.acc_y,
                              imu_data.acc_z,
                              imu_data.gyro_x,
                              imu_data.gyro_y,
                              imu_data.gyro_z,
                              imu_data.quaternion[0],
                              imu_data.quaternion[1],
                              imu_data.quaternion[2],
                              imu_data.quaternion[3],
                              now_us);
    gyro_record_isr_time(isr_start_us);
}

// 请求开始零飘校准，只置位请求标志，真正开始采样由外部中断完成
static void gyro_zero_calibration_start(void)
{
    // IMU 没就绪：保持服务未就绪并拒绝校准，CPU0发车入口会显示错误。
    if (!gyro_ready)
    {
        gyro_zero_calibrated = 0u;
        gyro_zero_calibrating = 0u;
        gyro_zero_start_requested = 0u;
        imu_service_ready = 0u;
        return;
    }

    gyro_zero_calibration_reset_accumulator();
    __dsync();
    gyro_zero_encoder_motion_seq = encoder_motion_seq;
    __dsync();
    gyro_zero_calibrated = 0u;
    gyro_zero_calibrating = 0u;
    gyro_startup_stationary_us = 0u;
    gyro_zero_start_requested = 1u;
}

uint8 gyro_zero_calibration_done(void)
{
    ImuSnapshot snapshot;
    return imu_snapshot_read(&snapshot) ? snapshot.zero_calibrated : 0u;
}

// 是否正在等待开始或正在采样
uint8 gyro_zero_calibration_running(void)
{
    ImuSnapshot snapshot;
    return imu_snapshot_read(&snapshot) ? snapshot.zero_calibrating : 0u;
}

#pragma section all restore
