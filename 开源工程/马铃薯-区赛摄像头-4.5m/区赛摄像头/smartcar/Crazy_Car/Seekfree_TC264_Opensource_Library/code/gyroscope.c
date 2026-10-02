/**
 * @file gyroscope.c
 * @brief IMU 采样、航向积分、零偏校准和姿态保护数据。
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

/*
 * gyroscope.c
 *
 * 纯陀螺积分航向（恢复 264 原始方案，芯片四元数/SFLP 已关闭）。
 *
 * 数据链路：IMU 数据就绪外部中断（INT2，480Hz）驱动采样 → 去零飘 →
 * Mahony 姿态滤波（软件四元数，开源飞控标准）→ 得到世界系航向逐帧
 * 增量；独立纯陀螺参考连续累加该增量，yaw_total 则按运行模式生成。
 * 航向方案：编码器差速为主 + 打滑窗口陀螺兜底（GYRO_YAW_SOURCE_ENCODER_ONLY=1）。
 * 平时纯编码器（振动完全免疫、静止绝对不漂，实车完赛验证）；
 * 陀螺与编码器航向速率差持续超门限判定打滑，窗口内 yaw_total 切用
 * 陀螺增量（轮子在滑、转速差不代表车身旋转），窗口结束交还编码器。
 * 持续互补融合保留在代码里但实测不能完赛（见航向源选择宏注释），勿切回。
 * 零飘校准只在上电静置时做一次（风机未开、无振动，1 秒均值干净可信）。
 * 发车流程不再重新校准——风机工况下振动混叠的低频拍频在 1 秒均值里
 * 不收敛，校出的零偏每次发车都不一样，实测是"提前触发一次一个样"的
 * 来源之一；发车只归零航向并重锚姿态保护零点（gyro_request_launch_rezero）。
 */

#include "gyroscope.h"
#include "motor.h"
#include "control.h" /* DIST_PER_PULSE_MM：编码器差速航向换算 */
#include <math.h>

volatile IMU_param_t IMU_Data;

static uint8 gyro_ready = 0u;

/* 加速度低通系数（480Hz 采样下 α=0.05 ≈ 4Hz 截止）：负压风机振动很强，
 * 倾角必须在重滤后的加速度上算，否则 roll/pitch 全是毛刺。 */
#define GYRO_ACC_LPF_ALPHA (0.05f)

/* 采样节拍：IMU 数据就绪外部中断（INT2）按陀螺 ODR 触发。
 * dt 逐帧用微秒时钟实测，不假设固定 ODR——实测本模式下并非名义 480Hz，
 * 固定 dt 曾造成 yaw 系统性少计约 25%（4 连右簇 360° 只读出 270°）。
 * 名义值仅用于首帧/野值兜底和零飘采样计数。 */
#define GYRO_NOMINAL_RATE_HZ (480u)
#define GYRO_NOMINAL_PERIOD_S (1.0f / (float)GYRO_NOMINAL_RATE_HZ)
#define GYRO_ZERO_CALIBRATION_MS (1000u)
/* 零飘采样帧数按名义节拍折算；真实 ODR 偏低时实际时长会拉长到 ~1.3s。
 * 只在上电静置时采一次（安静工况），发车流程直接沿用。 */
#define GYRO_ZERO_SAMPLE_COUNT ((uint16)((GYRO_NOMINAL_RATE_HZ * GYRO_ZERO_CALIBRATION_MS) / 1000u))

/* dt 实测状态。 */
static uint32 gyro_last_sample_us = 0u;
static uint8 gyro_dt_primed = 0u;

/* 航向死区（rad/s，约 0.86°/s）：|gyro_z| 低于它的帧视为静止零飘，
 * 该帧的航向增量不进 yaw_total。原始 264 方案取值 0.015，
 * 实测静止零飘远低于车辆真实转动角速度。 */
#define GYRO_YAW_DEADZONE_RADPS (0.015f)

/* 编码器静止判定：死区门控只在车轮确实没动时才生效。
 * 行驶中风机/路面振动的 gyro_z 噪声远超死区，旧的逐帧死区在动态下
 * 随机丢增量，是非线性误差源；改为静止时才允许丢弃。
 * 窗口 ~200ms（名义 480Hz × 96 帧），窗口内左右轮累计脉冲变化绝对值
 * 之和 ≤ 阈值视为静止。上电初值=静止，与旧行为一致。
 * left/right_encoder_total_pulses 由 1ms 速度 PID 中断累加（motor.c），
 * 本中断优先级更高可抢占它，但 int32 对齐单字读取在 TriCore 上原子，
 * 读到新旧值都自洽。 */
#define GYRO_STATIONARY_WINDOW_FRAMES (96u)
#define GYRO_STATIONARY_PULSE_MAX (4)

static int32 gyro_window_left_base = 0;
static int32 gyro_window_right_base = 0;
static uint16 gyro_window_frames = 0u;
static uint8 gyro_encoders_stationary = 1u;

/* ===== 编码器主导的互补航向融合 =====
 * 差速里程计航向：Δψ = (左轮位移 − 右轮位移) / 轮距，右转（左轮外侧走得多）
 * 为正，与 yaw_total 约定一致；实测方向反了就把 GYRO_ENC_YAW_SIGN 改 -1。
 * 编码器数脉冲，对振动完全免疫（混叠进带内的振动分量滤波器无法区分，
 * 编码器根本看不见），纯编码器航向已实车验证可用。
 * 融合分工：yaw_total 每帧先加陀螺增量（高频细节：补编码器量化、打滑瞬态、
 * 倾斜补偿），再以短时间常数 τ=GYRO_ENC_FUSION_TAU_S 拉向编码器航向——
 * 超过 τ 时间尺度的一切误差（振动整流、零偏残差）都被编码器接管，
 * 稳态完全跟编码器（编码器主导）。
 * 轮子没滚（手转/悬空推车调试）时编码器无信息，融合停用、纯陀螺工作，
 * 360° 手测标定不受影响。
 * 轮距标定：临时把 GYRO_YAW_SOURCE_ENCODER_ONLY 置 1，原地转 360° 读 Y，
 * 新轮距 = 旧轮距 × Y/360；或直接卷尺量（±3mm 足够）。 */
#define GYRO_ENC_YAW_SIGN (1.0f)
#define GYRO_ENC_FUSION_TAU_S (0.15f)

/* 航向源选择：1 = 编码器为主 + 打滑窗口陀螺兜底（正式方案）。
 * 平时 yaw_total 只累加编码器增量（实车唯一完赛验证的振动免疫配置）；
 * 陀螺仅在检测到打滑的窗口内顶替（见下方打滑修正注释区）。
 * 持续互补融合（0）实测不能完赛，根因：融合高频通道（>1/2πτ ≈ 1Hz）
 * 把陀螺振动混叠噪声以 ±几度振荡放进航向，而转弯退出是"碰线即触发"
 * 的锁存判据，振荡探头就提前退出。勿切回 0。 */
#define GYRO_YAW_SOURCE_ENCODER_ONLY (1)

/* ===== 打滑修正（编码器模式下的陀螺兜底）=====
 * 高速过弯打滑时轮子转速差 ≠ 车身真实旋转（甩尾时车转得比轮差快，
 * 驱动轮空转时轮差虚高），编码器航向说谎；陀螺不打滑。
 * 施密特触发检测：|陀螺航向速率 − 编码器航向速率| 连续
 * GYRO_SLIP_CONFIRM_FRAMES 帧超过进入门限 → 判定打滑，窗口内 yaw_total
 * 改用陀螺增量；差值回落到退出门限以下并保持 GYRO_SLIP_RELEASE_FRAMES
 * 帧 → 交还编码器。防振动误触发的机理：进入门限（60°/s）远高于混叠
 * 噪声的持续同向水平——振动是来回振荡的，凑不出连续 5 帧同向 60°/s；
 * 打滑窗口通常只有零点几秒，窗口内混入的陀螺振荡误差有界且不留存。
 * 若上车发现直道/正常弯 Y 与 E 无故分叉（误触发），升高进入门限或
 * 加大确认帧数；若明显打滑没被修正，降低进入门限。 */
/* 打滑兜底总开关：0 = 停用（yaw_total 纯编码器，含手转/悬空也不用陀螺），
 * 1 = 启用施密特触发打滑窗口陀螺顶替。对比实验用。 */
#define GYRO_SLIP_CORRECTION_ENABLE (0)

#define GYRO_SLIP_ENTER_DPS (60.0f)
#define GYRO_SLIP_EXIT_DPS (25.0f)
#define GYRO_SLIP_CONFIRM_FRAMES (5u)
#define GYRO_SLIP_RELEASE_FRAMES (24u)

#if GYRO_SLIP_CORRECTION_ENABLE
static uint16 gyro_slip_confirm_count = 0u;
static uint16 gyro_slip_release_count = 0u;
static uint8 gyro_slip_active = 0u;
#endif

static volatile float yaw_enc_total = 0.0f;
static int32 gyro_enc_left_last = 0;
static int32 gyro_enc_right_last = 0;
static uint8 gyro_enc_primed = 0u;

/* 每个 IMU 采样帧调用一次：滚动窗口刷新编码器静止标志。 */
static void gyro_stationary_update(void)
{
    gyro_window_frames++;
    if (gyro_window_frames >= GYRO_STATIONARY_WINDOW_FRAMES)
    {
        int32 left_now = left_encoder_total_pulses;
        int32 right_now = right_encoder_total_pulses;
        int32 delta_left = left_now - gyro_window_left_base;
        int32 delta_right = right_now - gyro_window_right_base;

        if (delta_left < 0)
        {
            delta_left = -delta_left;
        }
        if (delta_right < 0)
        {
            delta_right = -delta_right;
        }
        gyro_encoders_stationary = (uint8)((delta_left + delta_right) <= GYRO_STATIONARY_PULSE_MAX);
        gyro_window_left_base = left_now;
        gyro_window_right_base = right_now;
        gyro_window_frames = 0u;
    }
}

/* 陀螺刻度因子：乘在角速度换算源头，修正灵敏度带来的乘性误差。
 * 标定方法：车平放地面，原地精确旋转一整圈回到原朝向，读 Yaw=m，
 * 则本因子 = 旧因子 × 360/m。真实 MEMS 刻度误差仅 ±1~3%，标定结果
 * 必须落在 1.00±0.03；显著偏离说明另有根因（dt 计时、混叠、量程配置），
 * 禁止用本因子掩盖。历史教训：曾标出 1.2，实为固定 dt 少计 25% 的
 * 双重补偿，dt 改实测后 1.2 变成纯放大器，是"提前触发"的帮凶之一。
 * 注意标定时车必须放平——手持带倾斜旋转时读数小于 360 是倾斜补偿的
 * 正确行为，不是误差。 */
#define GYRO_SCALE (1.0f)

static float gyro_zero_offset_x = 0.0f;
static float gyro_zero_offset_y = 0.0f;
static float gyro_zero_offset_z = 0.0f;
static volatile uint8 gyro_zero_start_requested = 0u;
static volatile uint8 gyro_zero_calibrated = 0u;
static volatile uint8 gyro_zero_calibrating = 0u;
static volatile uint16 gyro_zero_sample_count = 0u;
static float gyro_zero_sum_x = 0.0f;
static float gyro_zero_sum_y = 0.0f;
static float gyro_zero_sum_z = 0.0f;

/* 姿态保护的相对零点：校准完成（车平放在赛道上）时记录当前倾角。
 * 板子倒装原始 roll 接近 ±180°，保护必须看相对变化而不是绝对值。 */
static float attitude_reference_roll = 0.0f;
static float attitude_reference_pitch = 0.0f;
static uint8 attitude_reference_valid = 0u;

/* 航向符号适配：转弯状态机（image.c）约定"右转 yaw_total 增大"。
 * Mahony 航向角的正方向由 IMU 安装方向决定。台架判定：停车放图像调试页，
 * 手拿车向右（俯视顺时针）旋转——调试页 Yaw 变大 → 置 1；变小 → 置 0。
 * 注意：航向源换过（SFLP→纯积分→Mahony），此宏必须重新实测确认。
 * 本板实测（Mahony 源）：右转 Yaw 变小 → 置 0 取反累加。 */
#define YAW_TOTAL_RIGHT_POSITIVE (0u)

/* 运行控制航向（度）：当前正式配置由编码器主导。
 * 只由 IMU 外部中断写入；归零走请求标志，避免并发直接写。 */
volatile float yaw_total = 0.0f;
/* 上电后连续累加的纯陀螺参考；软件航向归零和编码器逻辑均不写它。 */
static volatile float gyro_pure_yaw_deg = 0.0f;
static volatile uint8 yaw_total_reset_request = 0u;
/* 发车重锚请求：以当前姿态重新锁定 roll/pitch 保护零点（IMU 中断内消费）。 */
static volatile uint8 attitude_latch_request = 0u;

/* ===== Mahony 姿态滤波（开源飞控标准，只服务 roll/pitch 保护）=====
 * 陀螺积分四元数 + 加速度重力方向 P(I) 反馈修正：
 * 动态由陀螺扛（急刹/过弯的动态加速度不污染倾角），
 * 静差由加速度计慢慢拉回（陀螺漂移不积累）。
 * yaw 无磁力计参考会缓慢漂移——本工程航向不用它（yaw_total 独立纯积分）。 */
#define MAHONY_TWO_KP (1.0f) /* 2×Kp：加速度修正强度，振动误修正明显就调小 */
#define MAHONY_TWO_KI (0.0f) /* 2×Ki：默认 0，防持续动态加速度把积分项拉偏 */
/* 加速度修正的机动闭嘴门：任一轴角速度超过该值（约 30°/s）视为激烈机动，
 * 修正关闭、姿态纯陀螺积分。连续多弯的持续向心加速度会把"重力方向"拉歪
 * 几十度，不闭嘴的话 pitch/roll 假角会一路涨过保护阈值（实测 R=6 误停）。 */
#define MAHONY_ACC_GATE_GYRO_RADPS (0.5f)

static float mahony_q0 = 1.0f;
static float mahony_q1 = 0.0f;
static float mahony_q2 = 0.0f;
static float mahony_q3 = 0.0f;
static float mahony_ifb_x = 0.0f;
static float mahony_ifb_y = 0.0f;
static float mahony_ifb_z = 0.0f;
static uint8 mahony_initialized = 0u;
/* 上一帧 Mahony 航向：yaw_total 累加其逐帧增量。 */
static float mahony_yaw_last = 0.0f;

/* 前向声明：IMU_GetValues 里解航向增量环绕要用。 */
static float gyro_normalize_angle_delta(float angle);

/* Mahony 单步更新：gx/gy/gz 单位 rad/s，ax/ay/az 单位 g（模长参与有效性判断），
 * dt 为本帧实测采样间隔（秒）。结果写入 IMU_Data.roll/pitch/yaw/quaternion（度）。 */
static void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float norm;
    float recip;
    float halfvx;
    float halfvy;
    float halfvz;
    float halfex;
    float halfey;
    float halfez;
    float qa;
    float qb;
    float qc;
    float sinp;

    norm = sqrtf(ax * ax + ay * ay + az * az);

    /* 首帧按加速度方向初始化四元数：板子倒装（roll≈180°），
     * 若从单位四元数靠反馈收敛要几十秒，直接对齐重力一步到位。 */
    if (!mahony_initialized)
    {
        if ((norm > 0.5f) && (norm < 1.5f))
        {
            float roll0 = atan2f(ay, az);
            float pitch0 = atan2f(-ax, sqrtf(ay * ay + az * az));
            float cr = cosf(roll0 * 0.5f);
            float sr = sinf(roll0 * 0.5f);
            float cp = cosf(pitch0 * 0.5f);
            float sp = sinf(pitch0 * 0.5f);

            mahony_q0 = cp * cr;
            mahony_q1 = cp * sr;
            mahony_q2 = sp * cr;
            mahony_q3 = -sp * sr;
            mahony_initialized = 1u;
        }
        return;
    }

    /* 重力修正双重门：① 模长贴近 1g（0.85~1.15，向心/刹车加速度会把模长
     * 拉离）；② 三轴角速度都小于机动门（弯中/打舵激烈时闭嘴）。
     * 门外的帧只积分陀螺——短时纯积分漂移可忽略，假重力却是致命的。 */
    if ((norm > 0.85f) && (norm < 1.15f) &&
        (fabsf(gx) < MAHONY_ACC_GATE_GYRO_RADPS) &&
        (fabsf(gy) < MAHONY_ACC_GATE_GYRO_RADPS) &&
        (fabsf(gz) < MAHONY_ACC_GATE_GYRO_RADPS))
    {
        recip = 1.0f / norm;
        ax *= recip;
        ay *= recip;
        az *= recip;

        /* 四元数推算的重力方向（半值）。 */
        halfvx = mahony_q1 * mahony_q3 - mahony_q0 * mahony_q2;
        halfvy = mahony_q0 * mahony_q1 + mahony_q2 * mahony_q3;
        halfvz = mahony_q0 * mahony_q0 - 0.5f + mahony_q3 * mahony_q3;

        /* 实测重力 × 推算重力 = 姿态误差（叉积）。 */
        halfex = ay * halfvz - az * halfvy;
        halfey = az * halfvx - ax * halfvz;
        halfez = ax * halfvy - ay * halfvx;

        if (MAHONY_TWO_KI > 0.0f)
        {
            mahony_ifb_x += MAHONY_TWO_KI * halfex * dt;
            mahony_ifb_y += MAHONY_TWO_KI * halfey * dt;
            mahony_ifb_z += MAHONY_TWO_KI * halfez * dt;
            gx += mahony_ifb_x;
            gy += mahony_ifb_y;
            gz += mahony_ifb_z;
        }

        gx += MAHONY_TWO_KP * halfex;
        gy += MAHONY_TWO_KP * halfey;
        gz += MAHONY_TWO_KP * halfez;
    }

    /* 陀螺积分四元数（实测 dt）。 */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = mahony_q0;
    qb = mahony_q1;
    qc = mahony_q2;
    mahony_q0 += (-qb * gx - qc * gy - mahony_q3 * gz);
    mahony_q1 += (qa * gx + qc * gz - mahony_q3 * gy);
    mahony_q2 += (qa * gy - qb * gz + mahony_q3 * gx);
    mahony_q3 += (qa * gz + qb * gy - qc * gx);

    norm = sqrtf(mahony_q0 * mahony_q0 + mahony_q1 * mahony_q1 + mahony_q2 * mahony_q2 + mahony_q3 * mahony_q3);
    if (norm > 0.0f)
    {
        recip = 1.0f / norm;
        mahony_q0 *= recip;
        mahony_q1 *= recip;
        mahony_q2 *= recip;
        mahony_q3 *= recip;
    }

    /* 四元数转欧拉角（度）。yaw 无磁参考仅供显示。 */
    IMU_Data.roll = atan2f(2.0f * (mahony_q0 * mahony_q1 + mahony_q2 * mahony_q3),
                           1.0f - 2.0f * (mahony_q1 * mahony_q1 + mahony_q2 * mahony_q2)) *
                    (180.0f / PI);
    sinp = 2.0f * (mahony_q0 * mahony_q2 - mahony_q3 * mahony_q1);
    if (sinp > 1.0f)
    {
        sinp = 1.0f;
    }
    if (sinp < -1.0f)
    {
        sinp = -1.0f;
    }
    IMU_Data.pitch = asinf(sinp) * (180.0f / PI);
    IMU_Data.yaw = atan2f(2.0f * (mahony_q0 * mahony_q3 + mahony_q1 * mahony_q2),
                          1.0f - 2.0f * (mahony_q2 * mahony_q2 + mahony_q3 * mahony_q3)) *
                   (180.0f / PI);
    IMU_Data.quaternion[0] = mahony_q0;
    IMU_Data.quaternion[1] = mahony_q1;
    IMU_Data.quaternion[2] = mahony_q2;
    IMU_Data.quaternion[3] = mahony_q3;
}

/* 采集并发布一帧 IMU 数据，同时完成 yaw 积分。
 * 在数据就绪中断内调用：主动读取陀螺仪/加速度计寄存器。
 * dt 为本帧实测采样间隔（秒），由 gyro_external_interrupt_update() 提供。
 */
void IMU_GetValues(float dt)
{
    int16 acc_x;
    int16 acc_y;
    int16 acc_z;
    int16 gyro_x;
    int16 gyro_y;
    int16 gyro_z;
    float gyro_heading_delta_deg;

    imu660rc_get_gyro();
    imu660rc_get_acc();

    acc_x = imu660rc_acc_x;
    acc_y = imu660rc_acc_y;
    acc_z = imu660rc_acc_z;
    gyro_x = imu660rc_gyro_x;
    gyro_y = imu660rc_gyro_y;
    gyro_z = imu660rc_gyro_z;

    if (gyro_zero_calibrated)
    {
        IMU_Data.gyro_x = imu660rc_gyro_transition((float)gyro_x - gyro_zero_offset_x) * (PI / 180.0f) * GYRO_SCALE;
        IMU_Data.gyro_y = imu660rc_gyro_transition((float)gyro_y - gyro_zero_offset_y) * (PI / 180.0f) * GYRO_SCALE;
        IMU_Data.gyro_z = imu660rc_gyro_transition((float)gyro_z - gyro_zero_offset_z) * (PI / 180.0f) * GYRO_SCALE;
    }
    else
    {
        IMU_Data.gyro_x = imu660rc_gyro_transition(gyro_x) * (PI / 180.0f) * GYRO_SCALE;
        IMU_Data.gyro_y = imu660rc_gyro_transition(gyro_y) * (PI / 180.0f) * GYRO_SCALE;
        IMU_Data.gyro_z = imu660rc_gyro_transition(gyro_z) * (PI / 180.0f) * GYRO_SCALE;
    }

    /* 加速度重低通（在 g 单位上滤波，IMU_Data.acc_* 兼作滤波器记忆）。 */
    IMU_Data.acc_x = imu660rc_acc_transition(acc_x) * GYRO_ACC_LPF_ALPHA + IMU_Data.acc_x * (1.0f - GYRO_ACC_LPF_ALPHA);
    IMU_Data.acc_y = imu660rc_acc_transition(acc_y) * GYRO_ACC_LPF_ALPHA + IMU_Data.acc_y * (1.0f - GYRO_ACC_LPF_ALPHA);
    IMU_Data.acc_z = imu660rc_acc_transition(acc_z) * GYRO_ACC_LPF_ALPHA + IMU_Data.acc_z * (1.0f - GYRO_ACC_LPF_ALPHA);

    /* Mahony 姿态更新（实测 dt）→ IMU_Data.roll/pitch/yaw/quaternion。 */
    mahony_update(IMU_Data.gyro_x, IMU_Data.gyro_y, IMU_Data.gyro_z,
                  IMU_Data.acc_x, IMU_Data.acc_y, IMU_Data.acc_z, dt);

    /* 发车重锚：此刻车已摆正在赛道上，以当前姿态为 roll/pitch 保护零点。 */
    if (attitude_latch_request)
    {
        attitude_reference_roll = IMU_Data.roll;
        attitude_reference_pitch = IMU_Data.pitch;
        attitude_reference_valid = 1u;
        attitude_latch_request = 0u;
    }

    /* 本帧陀螺航向增量（Mahony 世界系、倾斜补偿，右转为正）：
     * 编码器模式给打滑修正用，融合模式直接累加。 */
#if YAW_TOTAL_RIGHT_POSITIVE
    gyro_heading_delta_deg = gyro_normalize_angle_delta(IMU_Data.yaw - mahony_yaw_last);
#else
    /* 安装方向使航向"右转变小"，取反成"右转增大"约定。 */
    gyro_heading_delta_deg = -gyro_normalize_angle_delta(IMU_Data.yaw - mahony_yaw_last);
#endif
    mahony_yaw_last = IMU_Data.yaw;

    /* 标定参考必须与运行航向彻底解耦：同一份倾斜补偿陀螺增量每帧
     * 无条件进入独立累计量，后面的 yaw_total 模式和软件零点不影响它。 */
    gyro_pure_yaw_deg += gyro_heading_delta_deg;

    /* yaw_total 归零请求（发车/直行穿越校准），在累加之前消费。 */
    if (yaw_total_reset_request)
    {
        yaw_total = 0.0f;
        yaw_enc_total = 0.0f;
        yaw_total_reset_request = 0u;
    }
#if !GYRO_YAW_SOURCE_ENCODER_ONLY
    /* 融合模式：行驶中永不丢帧；只有车轮静止且 |gz| 低于死区才视为
     * 静止零飘丢弃增量。推车/手转调试时 |gz| 立即超死区，yaw 照常累加。 */
    else if (gyro_zero_calibrated &&
             (!gyro_encoders_stationary ||
              (IMU_Data.gyro_z >= GYRO_YAW_DEADZONE_RADPS) || (IMU_Data.gyro_z <= -GYRO_YAW_DEADZONE_RADPS)))
    {
        yaw_total += gyro_heading_delta_deg;
    }
#endif

    /* 编码器差速航向：每帧累加。
     * 编码器模式：编码器增量为主 + 打滑窗口内切陀螺增量（见打滑修正注释）；
     * 融合模式：进 yaw_enc_total，轮子滚动时以短 τ 把 yaw_total 拉向它。 */
    {
        int32 enc_left_now = left_encoder_total_pulses;
        int32 enc_right_now = right_encoder_total_pulses;
        float enc_delta_deg = 0.0f;
#if GYRO_YAW_SOURCE_ENCODER_ONLY && GYRO_SLIP_CORRECTION_ENABLE
        float rate_diff_dps;
#endif

        if (!gyro_enc_primed)
        {
            gyro_enc_primed = 1u;
        }
        else
        {
            float delta_pulses = (float)(enc_left_now - gyro_enc_left_last) -
                                 (float)(enc_right_now - gyro_enc_right_last);
            /* 航向刻度改为宏(control.h),与里程刻度解耦;90°标定页已删。 */
            enc_delta_deg = GYRO_ENC_YAW_SIGN * delta_pulses *
                            ENCODER_YAW_DEG_PER_DIFF_PULSE;
        }
        gyro_enc_left_last = enc_left_now;
        gyro_enc_right_last = enc_right_now;

#if GYRO_YAW_SOURCE_ENCODER_ONLY
#if GYRO_SLIP_CORRECTION_ENABLE
        /* 打滑检测：陀螺测车身真实旋转，编码器测轮子滚动，两者速率差
         * 持续超门限即打滑。校准完成前不检测（陀螺带零偏会误判）。 */
        rate_diff_dps = (gyro_heading_delta_deg - enc_delta_deg) / dt;
        if (rate_diff_dps < 0.0f)
        {
            rate_diff_dps = -rate_diff_dps;
        }

        if (!gyro_slip_active)
        {
            if (gyro_zero_calibrated && (rate_diff_dps > GYRO_SLIP_ENTER_DPS))
            {
                gyro_slip_confirm_count++;
                if (gyro_slip_confirm_count >= GYRO_SLIP_CONFIRM_FRAMES)
                {
                    gyro_slip_active = 1u;
                    gyro_slip_release_count = 0u;
                }
            }
            else
            {
                gyro_slip_confirm_count = 0u;
            }
        }
        else
        {
            if (rate_diff_dps < GYRO_SLIP_EXIT_DPS)
            {
                gyro_slip_release_count++;
                if (gyro_slip_release_count >= GYRO_SLIP_RELEASE_FRAMES)
                {
                    gyro_slip_active = 0u;
                    gyro_slip_confirm_count = 0u;
                }
            }
            else
            {
                gyro_slip_release_count = 0u;
            }
        }

        /* 编码器为主；打滑窗口内轮子转速差不代表车身旋转，改用陀螺增量。
         * 副作用（有益）：手转/悬空调试时速率差=纯陀螺速率，超门限即被判
         * "打滑"，yaw 跟陀螺走——手测 360° 依旧可用。
         * yaw_enc_total 独立累加纯编码器值供调试页 E 显示，
         * Y−E 的差就是打滑修正的累计量。 */
        yaw_total += gyro_slip_active ? gyro_heading_delta_deg : enc_delta_deg;
        yaw_enc_total += enc_delta_deg;
#else
        /* 打滑兜底停用（对比实验）：yaw_total 纯编码器，Y 与 E 恒等。
         * 注意此时手转/悬空推车不产生 yaw，轮子必须着地滚动。 */
        (void)gyro_heading_delta_deg;
        yaw_total += enc_delta_deg;
        yaw_enc_total = yaw_total;
#endif
#else
        yaw_enc_total += enc_delta_deg;

        if (gyro_encoders_stationary)
        {
            /* 轮子没滚（手转/悬空推车调试）：编码器无信息，跟随陀螺航向，
             * 绝不反向拉扯——保住 360° 手测标定和推车调试。 */
            yaw_enc_total = yaw_total;
        }
        else if (gyro_zero_calibrated)
        {
            /* 轮子在滚：编码器主导。τ 很短（0.15s），稳态完全跟编码器，
             * 陀螺只贡献 τ 尺度内的高频细节（编码器量化、打滑瞬态）。
             * 振动整流、零偏残差等一切慢误差都被编码器接管。 */
            yaw_total += (dt / GYRO_ENC_FUSION_TAU_S) * (yaw_enc_total - yaw_total);
        }
#endif
    }
}

/* 请求把连续航向 yaw_total 归零。
 * 只置请求位，真正清零由 IMU 中断完成，避免 CPU1 与中断并发写。 */
void gyro_request_yaw_total_reset(void)
{
    yaw_total_reset_request = 1u;
}

/* 32 位对齐 float 在 TC264 上按单字读取，不会读到半个数值。
 * 本变量只有 IMU 中断写入，调用方只能取得快照。 */
float gyro_get_pure_yaw_deg(void)
{
    return gyro_pure_yaw_deg;
}

/* 读取编码器差速航向（度，右转为正）。调试显示/轮距标定用。 */
float gyro_get_yaw_enc_total(void)
{
    return yaw_enc_total;
}

/* 发车瞬间调用：归零航向 + 以当前姿态重锚 roll/pitch 保护零点。
 * 零偏沿用上电静置校准的结果，发车不再重新校准（见文件头注释）。 */
void gyro_request_launch_rezero(void)
{
    attitude_latch_request = 1u;
    yaw_total_reset_request = 1u;
}

/* 将角度差限制在 [-180, 180]，处理跨 ±180 环绕。 */
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

/* 获取相对 roll（相对校准完成时的姿态零点，倒装安装角被自然扣除）。
 * 零点未建立（未校准过）时返回 0，保护不动作。 */
float gyro_get_roll_delta(void)
{
    if (!attitude_reference_valid)
    {
        return 0.0f;
    }
    return gyro_normalize_angle_delta(IMU_Data.roll - attitude_reference_roll);
}

/* 获取相对 pitch，同 roll。 */
float gyro_get_pitch_delta(void)
{
    if (!attitude_reference_valid)
    {
        return 0.0f;
    }
    return gyro_normalize_angle_delta(IMU_Data.pitch - attitude_reference_pitch);
}

/* 在 IMU 外部中断中采一帧零飘样本。
 * 采够 GYRO_ZERO_SAMPLE_COUNT 后计算三轴角速度偏置并请求航向归零。
 */
static void gyro_zero_calibration_sample(void)
{
    if (!gyro_zero_calibrating)
    {
        return;
    }

    gyro_zero_sum_x += (float)imu660rc_gyro_x;
    gyro_zero_sum_y += (float)imu660rc_gyro_y;
    gyro_zero_sum_z += (float)imu660rc_gyro_z;
    gyro_zero_sample_count++;

    if (gyro_zero_sample_count >= GYRO_ZERO_SAMPLE_COUNT)
    {
        gyro_zero_offset_x = gyro_zero_sum_x / (float)gyro_zero_sample_count;
        gyro_zero_offset_y = gyro_zero_sum_y / (float)gyro_zero_sample_count;
        gyro_zero_offset_z = gyro_zero_sum_z / (float)gyro_zero_sample_count;
        gyro_zero_calibrating = 0u;
        gyro_zero_calibrated = 1u;
        /* 校准完成：请求航向归零（下一采样帧生效），
         * 并以当前倾角为姿态保护零点（此刻车平放在赛道上）。 */
        gyro_request_yaw_total_reset();
        attitude_reference_roll = IMU_Data.roll;
        attitude_reference_pitch = IMU_Data.pitch;
        attitude_reference_valid = 1u;
    }
}

/* 在 IMU 外部中断中响应"开始校准"请求。
 * 按键/发车流程只置请求位，真正清累加器和进入采样状态在此执行。
 */
static void gyro_zero_calibration_begin(void)
{
    if (!gyro_zero_start_requested)
    {
        return;
    }

    gyro_zero_start_requested = 0u;

    if (!gyro_ready)
    {
        gyro_zero_calibrated = 0u;
        gyro_zero_calibrating = 0u;
        zf_assert(0u);
        return;
    }

    gyro_zero_sum_x = 0.0f;
    gyro_zero_sum_y = 0.0f;
    gyro_zero_sum_z = 0.0f;
    gyro_zero_sample_count = 0u;
    gyro_zero_calibrated = 0u;
    gyro_zero_calibrating = 1u;
    attitude_reference_valid = 0u;
}

/* 清空应用层 IMU 缓存。只在初始化阶段调用。 */
static void IMU_Data_Clear(void)
{
    IMU_Data.acc_x = 0.0f;
    IMU_Data.acc_y = 0.0f;
    IMU_Data.acc_z = 0.0f;
    IMU_Data.gyro_x = 0.0f;
    IMU_Data.gyro_y = 0.0f;
    IMU_Data.gyro_z = 0.0f;
    IMU_Data.roll = 0.0f;
    IMU_Data.pitch = 0.0f;
    IMU_Data.yaw = 0.0f;
    IMU_Data.quaternion[0] = 0.0f;
    IMU_Data.quaternion[1] = 0.0f;
    IMU_Data.quaternion[2] = 0.0f;
    IMU_Data.quaternion[3] = 0.0f;
}

/* 初始化 IMU。
 * 四元数/SFLP 输出禁用，INT2 配为陀螺数据就绪中断（480Hz）驱动采样；
 * 初始化失败直接断言。IMU 就绪后立即请求一次非阻塞零飘校准，
 * 由外部中断在开机静置阶段采完；发车流程只归零航向，不再重复校准。
 */
void gyro_init(void)
{
    uint8 init_state;

    IMU_Data_Clear();
    gyro_zero_calibrated = 0u;
    gyro_zero_calibrating = 0u;
    gyro_zero_start_requested = 0u;
    gyro_zero_sample_count = 0u;
    gyro_ready = 0u;
    gyro_pure_yaw_deg = 0.0f;

    init_state = imu660rc_init(IMU660RC_QUARTERNION_DISABLE);
    zf_assert(0u == init_state);

    if (0u == init_state)
    {
        gyro_ready = 1u;
        /* 上电即请求一次非阻塞零飘校准（中断内约 1 秒静默完成，不卡启动）。
         * 这是唯一一次零偏校准：风机未开、无振动，均值干净可信；
         * 发车流程不再重新校准（风机振动会污染均值），直接沿用本结果。
         * 上电后第 1 秒别晃车。 */
        gyro_zero_calibration_start();
    }
}

/* IMU 数据就绪外部中断的采样入口（isr.c 调用）。
 * 顺序：实测 dt → 采样发布 + yaw 积分 → 响应校准请求 → 校准采样。
 */
void gyro_external_interrupt_update(void)
{
    uint32 now_us;
    float dt;

    if (!gyro_ready)
    {
        gyro_zero_calibrated = 0u;
        gyro_zero_calibrating = 0u;
        gyro_zero_start_requested = 0u;
        return;
    }

    /* 逐帧实测采样间隔：真实 ODR 是多少就积多少。
     * 首帧无基准、间隔野值（毛刺双沿 / 中断被长时间阻塞）时退回名义节拍。 */
    now_us = system_getval_us();
    if (!gyro_dt_primed)
    {
        dt = GYRO_NOMINAL_PERIOD_S;
        gyro_dt_primed = 1u;
    }
    else
    {
        dt = (float)(uint32)(now_us - gyro_last_sample_us) * 1.0e-6f;
        if ((dt < 0.0002f) || (dt > 0.02f))
        {
            dt = GYRO_NOMINAL_PERIOD_S;
        }
    }
    gyro_last_sample_us = now_us;

    gyro_stationary_update();
    IMU_GetValues(dt);
    gyro_zero_calibration_begin();
    gyro_zero_calibration_sample();
}

/* 请求开始零飘校准。
 * 只置位请求标志，真正开始采样由下一次 IMU 数据就绪中断完成。
 */
void gyro_zero_calibration_start(void)
{
    if (!gyro_ready)
    {
        gyro_zero_calibrated = 0u;
        gyro_zero_calibrating = 0u;
        gyro_zero_start_requested = 0u;
        zf_assert(0u);
        return;
    }

    gyro_zero_sample_count = 0u;
    gyro_zero_calibrated = 0u;
    gyro_zero_calibrating = 0u;
    gyro_zero_start_requested = 1u;
}

/* 查询零飘是否完成。 */
uint8 gyro_zero_calibration_done(void)
{
    return gyro_zero_calibrated;
}

/* 查询零飘是否正在等待开始或正在采样。 */
uint8 gyro_zero_calibration_running(void)
{
    return (uint8)(gyro_zero_start_requested || gyro_zero_calibrating);
}
