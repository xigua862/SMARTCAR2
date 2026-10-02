/**
 * @file control.c
 * @brief 1 ms 轮速控制、横向执行、滑移阻尼和堵转保护。
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
 * @note CPU0 的 1 ms 控制中断调用，修改时需保持硬实时预算。
 */

// control.c —— 速度与转向控制：1ms 硬实时中断 pit_speed_pid()（CCU61_CH1）
// 控制链：编码器测速→离线位置速度/轮速参考→左右轮增量PI→限幅→FOC。

#include "control.h"
#include "motor.h"
#include "PID.h"
#include "globals.h"
#include "flash.h"
#include "gyroscope.h"
#include "navigation.h"
#include "prestart.h"
#include "telemetry.h"
#include "task.h"
#include <math.h>

// 文件作用域变量放进 cpu0_dsram（只有控制核 CPU0 访问）。这对 pragma 的开合范围绝对不能动。
#pragma section all "cpu0_dsram"

// 控制环当前工作模式
typedef enum
{
    CONTROL_MODE_IDLE = 0u,      // 空闲：不驱动，只在后台读里程
    CONTROL_MODE_NORMAL,         // 正常行驶：跑完整速度+转向控制链
} control_mode_enum;

// 安全相关阈值
typedef struct
{
    uint8 fault_duty_percent;        // 失控判定：占空比达到软限幅的百分之几算高负荷
    int16 fault_count_max;           // 失控判定：编码器计数在此范围内算“几乎不动”
    uint8 fault_speed_ratio_percent; // 失控判定：实际轮速低于目标轮速的百分比
    float fault_target_min_cmps;     // 失控判定：目标轮速低于此值时不参与判断
    float fault_min_actual_cmps;     // 失控判定：相对速度门的绝对下限
    uint16 fault_confirm_ms;         // 失控判定：连续满足多少毫秒才真正确认失控
    uint16 start_grace_ms;           // 发车后未建立轮速反馈前的宽限时间
} control_safety_config_struct;

// 软件堵转保护：高 PWM + 目标轮速较高 + 实际轮速严重不足，连续确认后请求急停。
static const control_safety_config_struct control_safety = {
    CONTROL_STALL_DUTY_PERCENT,
    CONTROL_STALL_ENCODER_COUNT_MAX,
    CONTROL_STALL_SPEED_RATIO_PERCENT,
    CONTROL_STALL_TARGET_MIN_CMPS,
    CONTROL_STALL_MIN_ACTUAL_CMPS,
    CONTROL_STALL_CONFIRM_MS,
    CONTROL_STALL_START_GRACE_MS,
};

static volatile control_mode_enum control_mode = CONTROL_MODE_IDLE;

// 纵向规划器遵循电子鼠常用的 time-optimal envelope：地图逐点表给出空间
// 上限，导航层按前方低速段/终点反推可达上限，本地状态再按最大加减速度
// 逐拍跟踪。这样定位重投影或RLE段切换不会给轮速环制造瞬时阶跃。
static float plan_speed_cmps = 0.0f;
static uint8 plan_speed_initialized = 0u;
static volatile float control_actual_speed_abs_cmps = 0.0f;
static float turn_command_last_cmps = 0.0f;
static float slip_guard_encoder_yaw_rate_degps = 0.0f;
static float slip_guard_gyro_yaw_rate_degps = 0.0f;
static uint8 slip_guard_filter_initialized = 0u;
static uint8 slip_guard_active = 0u;

// 电机/编码器失控保护：每侧独立累计“高 PWM、目标轮速较高、实际轮速严重不足”
// 的持续时间。只要任一侧确认，motor_fault_request 保持置位，下一拍先清 PWM，
// 后台任务再完成停车、日志冻结和故障原因提交。
static uint16 motor_fault_ms_l = 0u;    // 左轮已连续满足失控条件的等效毫秒数
static uint16 motor_fault_ms_r = 0u;    // 右轮已连续满足失控条件的等效毫秒数
static uint32 motor_fault_run_ms = 0u;  // 本轮速度环已运行的等效毫秒数
static uint8 motor_motion_seen_l = 0u;  // 左轮本轮是否已经出现过有效编码器运动
static uint8 motor_motion_seen_r = 0u;  // 右轮本轮是否已经出现过有效编码器运动
static uint8 control_limit_mask_last = TELEMETRY_CONTROL_LIMIT_NONE;
static uint32 control_last_start_us = 0u;

static float control_clamp(float value, float limit);

static float control_apply_slip_guard(float turn_cmps,
                                      int16 encoder_l,
                                      int16 encoder_r,
                                      float actual_body_cmps,
                                      float sample_period_s,
                                      uint8 *limit_mask)
{
    float encoder_yaw_rate_degps;
    float gyro_yaw_rate_degps;
    float alpha;
    float mismatch_degps;
    float mismatch_abs_degps;
    float correction_cmps;

    if (!nav_control_imu_is_valid() || (sample_period_s <= 0.0f))
    {
        slip_guard_filter_initialized = 0u;
        slip_guard_active = 0u;
        return turn_cmps;
    }

    encoder_yaw_rate_degps =
        ((float)encoder_l - (float)encoder_r) *
        ENCODER_YAW_DEG_PER_DIFF_PULSE / sample_period_s;
    gyro_yaw_rate_degps = nav_get_control_yaw_rate_degps();
    alpha = sample_period_s /
            (SLIP_GUARD_RATE_FILTER_TAU_S + sample_period_s);

    if (!slip_guard_filter_initialized)
    {
        slip_guard_encoder_yaw_rate_degps = encoder_yaw_rate_degps;
        slip_guard_gyro_yaw_rate_degps = gyro_yaw_rate_degps;
        slip_guard_filter_initialized = 1u;
    }
    else
    {
        slip_guard_encoder_yaw_rate_degps += alpha *
            (encoder_yaw_rate_degps - slip_guard_encoder_yaw_rate_degps);
        slip_guard_gyro_yaw_rate_degps += alpha *
            (gyro_yaw_rate_degps - slip_guard_gyro_yaw_rate_degps);
    }

    mismatch_degps = slip_guard_gyro_yaw_rate_degps -
                     slip_guard_encoder_yaw_rate_degps;
    mismatch_abs_degps = fabsf(mismatch_degps);
    if ((actual_body_cmps < SLIP_GUARD_MIN_SPEED_CMPS) ||
        (fabsf(slip_guard_gyro_yaw_rate_degps) <
         SLIP_GUARD_MIN_YAW_RATE_DEGPS))
    {
        slip_guard_active = 0u;
        return turn_cmps;
    }
    if (!slip_guard_active)
    {
        if (mismatch_abs_degps <= SLIP_GUARD_MISMATCH_DEADBAND_DEGPS)
        {
            return turn_cmps;
        }
        slip_guard_active = 1u;
    }
    else if (mismatch_abs_degps < SLIP_GUARD_MISMATCH_RELEASE_DEGPS)
    {
        slip_guard_active = 0u;
        return turn_cmps;
    }

    if (NULL != limit_mask)
    {
        *limit_mask |= TELEMETRY_CONTROL_LIMIT_SLIP_GUARD;
    }
    if (mismatch_abs_degps <= SLIP_GUARD_MISMATCH_DEADBAND_DEGPS)
    {
        return turn_cmps;
    }

    correction_cmps = SLIP_GUARD_GAIN_CMPS_PER_RADPS *
        (mismatch_abs_degps - SLIP_GUARD_MISMATCH_DEADBAND_DEGPS) *
        (3.14159265f / 180.0f);
    correction_cmps = control_clamp(correction_cmps,
                                    SLIP_GUARD_MAX_CORRECTION_CMPS);
    /* 本车控制符号中正turn产生负yaw；因此正yaw时增加正turn，负yaw时
     * 增加负turn，始终对车身真实横摆形成反向力矩。 */
    if (slip_guard_gyro_yaw_rate_degps < 0.0f)
    {
        correction_cmps = -correction_cmps;
    }
    return turn_cmps + correction_cmps;
}

static void control_reset_runtime_timing(void)
{
    control_last_start_us = 0u;
    control_exec_time_us = 0u;
    control_exec_max_us = 0u;
    control_exec_overrun_count = 0u;
    control_period_us = 0u;
    control_period_max_us = 0u;
    control_period_overrun_count = 0u;
    control_actual_speed_abs_cmps = 0.0f;
}

static float control_record_period(uint32 start_us)
{
    float sample_period_s = PID_SAMPLE_S;

    if (0u != control_last_start_us)
    {
        uint32 period_us = start_us - control_last_start_us;
        control_period_us = period_us;
        if (period_us > control_period_max_us)
        {
            control_period_max_us = period_us;
        }
        /* system_getval_us() is quantized; a 1,001 us interval is normal
         * timer jitter, not a lost 1 ms tick. */
        if ((period_us > (PID_SAMPLE_US + 50u)) &&
            (control_period_overrun_count < 0xFFFFFFFFu))
        {
            control_period_overrun_count++;
        }
        if (period_us >= 500u)
        {
            sample_period_s = (float)period_us * 1.0e-6f;
            if (sample_period_s > 0.020f)
            {
                sample_period_s = 0.020f;
            }
        }
    }
    control_last_start_us = start_us;
    return sample_period_s;
}

static void control_record_execution(uint32 start_us)
{
    uint32 elapsed_us = system_getval_us() - start_us;

    control_exec_time_us = elapsed_us;
    if (elapsed_us > control_exec_max_us)
    {
        control_exec_max_us = elapsed_us;
    }
    if ((elapsed_us > (PID_SAMPLE_US + 50u)) &&
        (control_exec_overrun_count < 0xFFFFFFFFu))
    {
        control_exec_overrun_count++;
    }
}

static void control_capture_limit_transition(uint8 current_mask)
{
    if (current_mask != control_limit_mask_last)
    {
        telemetry_capture_event_isr(TELEMETRY_EVENT_CONTROL_LIMIT,
                                    TELEMETRY_EVENT_SOURCE_CONTROL,
                                    current_mask,
                                    (int32)control_limit_mask_last,
                                    (int32)current_mask);
        control_limit_mask_last = current_mask;
    }
}

// 重置速度规划器；发车和所有停车路径都从静止边界重新开始。
static void speed_plan_reset(float speed_now)
{
    speed = speed_now;
    plan_speed_cmps = speed_now;
    plan_speed_initialized = 1u;
}

/* Track a spatial speed cap with a time-domain trapezoidal profile.  The
 * navigation cap already includes the distance needed to brake for the next
 * lower-speed segment; this second stage only enforces what the motors can
 * change in one control period.  A large encoder mismatch is reconciled once
 * so a localization jump cannot leave the planner far below the real body
 * speed for several hundred milliseconds. */
static float speed_profile_update(float cap_speed_cmps,
                                  float actual_body_cmps,
                                  float sample_period_s)
{
    float max_accel;
    float max_decel;
    float max_step;
    float delta;

    if (cap_speed_cmps < 0.0f)
    {
        cap_speed_cmps = 0.0f;
    }
    if (sample_period_s < 0.0005f)
    {
        sample_period_s = PID_SAMPLE_S;
    }
    else if (sample_period_s > 0.020f)
    {
        sample_period_s = 0.020f;
    }
    if (actual_body_cmps < 0.0f)
    {
        actual_body_cmps = 0.0f;
    }

    if (!plan_speed_initialized)
    {
        plan_speed_cmps = actual_body_cmps;
        plan_speed_initialized = 1u;
    }
    else if (actual_body_cmps > plan_speed_cmps +
             SPEED_PROFILE_RECONCILE_ERROR_CMPS)
    {
        /* Reconcile a large encoder jump without ever lifting the planner
         * above the navigation table's hard speed cap. */
        plan_speed_cmps = actual_body_cmps;
        if (plan_speed_cmps > cap_speed_cmps)
        {
            plan_speed_cmps = cap_speed_cmps;
        }
    }

    max_accel = nav_get_speed_profile_max_accel_cmps2();
    max_decel = nav_get_speed_profile_max_decel_cmps2();
    /* Older maps used 1e9 as an "unlimited" acceleration sentinel.  Keep
     * their protocol valid, but replace only that sentinel with the finite
     * actuator baseline; a finite map value remains authoritative. */
    if ((max_accel <= 0.0f) || (max_accel > 100000.0f))
    {
        max_accel = SPEED_PROFILE_MAX_ACCEL_CMPS2;
    }
    if ((max_decel <= 0.0f) || (max_decel > 100000.0f))
    {
        max_decel = SPEED_PROFILE_MAX_DECEL_CMPS2;
    }
    if (max_accel <= 0.0f)
    {
        max_accel = 1.0f;
    }
    if (max_decel <= 0.0f)
    {
        max_decel = 1.0f;
    }

    delta = cap_speed_cmps - plan_speed_cmps;
    max_step = ((delta >= 0.0f) ? max_accel : max_decel) *
               sample_period_s;
    if (delta > max_step)
    {
        delta = max_step;
    }
    else if (delta < -max_step)
    {
        delta = -max_step;
    }
    plan_speed_cmps += delta;
    if (plan_speed_cmps < 0.0f)
    {
        plan_speed_cmps = 0.0f;
    }
    if (plan_speed_cmps > cap_speed_cmps)
    {
        plan_speed_cmps = cap_speed_cmps;
    }
    return plan_speed_cmps;
}

// 清零速度 PI 及累计占空比：发车、标定切换、任何停车路径都要先清干净，避免旧积分带进新一轮
static void speed_pi_reset(void)
{
    left_target_speed = 0.0f;
    right_target_speed = 0.0f;
    pid_reset(&speed_PID1);
    pid_reset(&speed_PID2);
}

static float control_clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

/* One control tick is easier to audit when the data crossing each stage is
 * explicit.  These small records stay on the CPU0 ISR stack and contain no
 * cross-core state. */
typedef struct
{
    float target_l_cmps;
    float target_r_cmps;
    float tracking_turn_cmps;
    float yaw_rate_error_degps;
    uint8 limit_mask;
} control_wheel_reference_struct;

typedef struct
{
    float duty_l;
    float duty_r;
    uint8 limit_mask;
} control_motor_output_struct;

/* Stage 1: update odometry/diagnostics and build the two wheel references.
 * Map mode and the isolated straight speed test share the same output shape;
 * only their source of body speed and turn differs. */
static void control_build_wheel_reference(
    int16 raw_l,
    int16 raw_r,
    float actual_body_cmps,
    float sample_period_s,
    control_wheel_reference_struct *reference)
{
    reference->target_l_cmps = 0.0f;
    reference->target_r_cmps = 0.0f;
    reference->tracking_turn_cmps = 0.0f;
    reference->yaw_rate_error_degps = 0.0f;
    reference->limit_mask = TELEMETRY_CONTROL_LIMIT_NONE;

    /* The speed test keeps only raw odometry alive.  Normal navigation owns
     * the map update; these paths are deliberately mutually exclusive. */
    if (prestart_is_straight_speed_test())
    {
        nav_update_odometry_only(raw_l, raw_r, sample_period_s);
    }
    else
    {
        nav_update(raw_l, raw_r, sample_period_s);
    }
    task_capture_phototube_scan_pose_from_isr();

    turn_control = 0.0f;
    if (prestart_is_straight_speed_test())
    {
        float plan_v = prestart_get_speed_test_target_cmps();
        float turn_limit;

        reference->tracking_turn_cmps =
            nav_compute_straight_hold_turn_cmps();
        reference->yaw_rate_error_degps =
            nav_get_tracking_yaw_rate_error_degps();
        turn_control = reference->tracking_turn_cmps;
        turn_limit = plan_v * NAV_TURN_LIMIT_RATIO;
        if (turn_control > turn_limit)
        {
            turn_control = turn_limit;
            reference->limit_mask |= TELEMETRY_CONTROL_LIMIT_TURN;
        }
        else if (turn_control < -turn_limit)
        {
            turn_control = -turn_limit;
            reference->limit_mask |= TELEMETRY_CONTROL_LIMIT_TURN;
        }
        turn_command_last_cmps = turn_control;
        reference->target_l_cmps = plan_v - turn_control;
        reference->target_r_cmps = plan_v + turn_control;
        speed = plan_v;

    }
    else if ((NAV_STATE_NAVIGATING == nav_get_state()) ||
             (NAV_STATE_FINISHED == nav_get_state()))
    {
        float table_v = nav_get_target_speed_cmps();
        float plan_v;
        float profile_limit;
        float turn_limit;

        /* Normal routes keep their terminal speed through the endpoint so
         * braking does not make the car stop short.  After FINISHED, switch
         * the live wheel target to zero and let the PI loop actively brake.
         * Blind-box routes already brake early, but use the same finish phase
         * to remove any residual roll before the driver is disabled. */
        if (NAV_STATE_FINISHED == nav_get_state())
        {
            table_v = 0.0f;
        }

        if (table_v < 0.0f)
        {
            nav_fail_invalid_reference();
            plan_v = 0.0f;
            speed = 0.0f;
        }
        else
        {
            /* The exported table already contains the curvature cap and the
             * backward/forward acceleration envelope.  Keep the ISR path
             * O(1); the legacy look-ahead helper remains diagnostics-only. */
            profile_limit = table_v;
            plan_v = speed_profile_update(profile_limit,
                                          actual_body_cmps,
                                          sample_period_s);
            speed = plan_v;
        }

        reference->tracking_turn_cmps =
            nav_compute_tracking_turn_cmps(plan_v, actual_body_cmps);
        reference->yaw_rate_error_degps =
            nav_get_tracking_yaw_rate_error_degps();
        turn_control = control_apply_slip_guard(
            reference->tracking_turn_cmps,
            raw_l,
            raw_r,
            actual_body_cmps,
            sample_period_s,
            &reference->limit_mask);

        turn_limit = plan_v * NAV_TURN_LIMIT_RATIO;
        if (turn_control > turn_limit)
        {
            turn_control = turn_limit;
            reference->limit_mask |= TELEMETRY_CONTROL_LIMIT_TURN;
        }
        else if (turn_control < -turn_limit)
        {
            turn_control = -turn_limit;
            reference->limit_mask |= TELEMETRY_CONTROL_LIMIT_TURN;
        }

        turn_command_last_cmps = turn_control;
        reference->target_l_cmps = plan_v - turn_control;
        reference->target_r_cmps = plan_v + turn_control;
    }

    if ((NAV_STATE_NAVIGATING != nav_get_state()) &&
        !prestart_is_straight_speed_test())
    {
        turn_command_last_cmps = 0.0f;
    }

    /* Match the Git finish-version invariant: the wheel reference itself
     * never requests reverse.  The PI actuator state is intentionally kept
     * signed below so it may cross zero and apply reverse braking. */
    if (reference->target_l_cmps < 0.0f)
    {
        reference->target_l_cmps = 0.0f;
        reference->limit_mask |= TELEMETRY_CONTROL_LIMIT_TURN;
    }
    if (reference->target_r_cmps < 0.0f)
    {
        reference->target_r_cmps = 0.0f;
        reference->limit_mask |= TELEMETRY_CONTROL_LIMIT_TURN;
    }

    plan_v_debug_l = reference->target_l_cmps;
    plan_v_debug_r = reference->target_r_cmps;

}

/* Stage 2: the pre-race finish version's pure incremental PI speed loop.
 * The PI increment is accumulated directly into each wheel's PWM command;
 * there is no motor inverse model, feedforward term, or anti-reversal gate. */
static void control_apply_speed_loop(
    float actual_l,
    float actual_r,
    float soft_limit,
    const control_wheel_reference_struct *reference,
    control_motor_output_struct *output)
{
    float unclamped_pi_l;
    float unclamped_pi_r;

    output->limit_mask = reference->limit_mask;

    left_target_speed += pi_increment(&speed_PID1,
                                      actual_l,
                                      reference->target_l_cmps);
    right_target_speed += pi_increment(&speed_PID2,
                                       actual_r,
                                       reference->target_r_cmps);
    unclamped_pi_l = left_target_speed;
    unclamped_pi_r = right_target_speed;
    left_target_speed = control_clamp(left_target_speed, soft_limit);
    right_target_speed = control_clamp(right_target_speed, soft_limit);
    if (left_target_speed != unclamped_pi_l)
    {
        output->limit_mask |= TELEMETRY_CONTROL_LIMIT_DUTY_LEFT;
    }
    if (right_target_speed != unclamped_pi_r)
    {
        output->limit_mask |= TELEMETRY_CONTROL_LIMIT_DUTY_RIGHT;
    }

    output->duty_l = left_target_speed;
    output->duty_r = right_target_speed;
}

static uint16 control_period_ms(float sample_period_s)
{
    float period_ms = sample_period_s * 1000.0f;

    /* A bad/zero period must not make the protection timer stop advancing.
     * Large scheduling gaps count as elapsed wall time, but are bounded so a
     * single abnormal sample cannot overflow the small millisecond counters. */
    if (!(period_ms >= 1.0f))
    {
        return 1u;
    }
    if (period_ms > 50.0f)
    {
        return 50u;
    }
    return (uint16)(period_ms + 0.5f);
}

static void control_update_fault_counter(float final_duty,
                                         float target_speed,
                                         float actual_speed,
                                         int16 raw_count,
                                         float soft_limit,
                                         uint16 elapsed_ms,
                                         uint8 armed,
                                         uint16 *fault_ms)
{
    float fault_duty = soft_limit *
        (float)control_safety.fault_duty_percent / 100.0f;
    float target_abs = fabsf(target_speed);
    float actual_abs = fabsf(actual_speed);
    float low_speed_limit = target_abs *
        (float)control_safety.fault_speed_ratio_percent / 100.0f;
    uint8 near_limit;
    uint8 target_running;
    uint8 no_encoder_motion;
    uint8 low_feedback;
    uint8 wrong_direction;
    uint8 stall_candidate;
    uint16 decay_ms;

    if (low_speed_limit < control_safety.fault_min_actual_cmps)
    {
        low_speed_limit = control_safety.fault_min_actual_cmps;
    }

    near_limit = (uint8)((fault_duty > 0.0f) &&
                         ((final_duty >= fault_duty) ||
                          (final_duty <= -fault_duty)));
    target_running = (uint8)(target_abs >=
                             control_safety.fault_target_min_cmps);
    no_encoder_motion = (uint8)((raw_count <=
                                 control_safety.fault_count_max) &&
                                (raw_count >=
                                 -control_safety.fault_count_max));
    low_feedback = (uint8)(no_encoder_motion ||
                           (actual_abs <= low_speed_limit));
    /* A single tiny reverse encoder sample is not enough evidence: require
     * a real reverse-speed magnitude, then let the confirmation timer reject
     * brief sign glitches. */
    wrong_direction = (uint8)(target_running &&
                              (actual_abs >=
                               control_safety.fault_min_actual_cmps) &&
                              (target_speed * actual_speed < 0.0f));
    stall_candidate = (uint8)(armed && near_limit && target_running &&
                              (low_feedback || wrong_direction));

    if (stall_candidate)
    {
        if (*fault_ms < control_safety.fault_confirm_ms)
        {
            uint32 next_fault_ms = (uint32)(*fault_ms) + elapsed_ms;
            *fault_ms = (next_fault_ms >= control_safety.fault_confirm_ms) ?
                control_safety.fault_confirm_ms : (uint16)next_fault_ms;
        }
        if (*fault_ms >= control_safety.fault_confirm_ms)
        {
            motor_fault_request = 1u;
        }
    }
    else
    {
        decay_ms = (elapsed_ms > 1u) ? (uint16)(elapsed_ms * 2u) : 2u;
        *fault_ms = (*fault_ms > decay_ms) ?
            (uint16)(*fault_ms - decay_ms) : 0u;
    }
}

// 初始化控制模块：读 Flash 参数配置速度 PI，对齐编码器基准，配好 1ms PIT 但先关（发车才开）
void control_init(void)
{
    control_reset_runtime_timing();
    pi_init(&speed_PID1, g_flash_parameters.speed_KP, g_flash_parameters.speed_KI,
            g_flash_parameters.speed_IncrementLimit);
    pi_init(&speed_PID2, g_flash_parameters.speed_KP, g_flash_parameters.speed_KI,
            g_flash_parameters.speed_IncrementLimit);

    // 同步自由运行编码器的软件基准
    (void)motor_read_left_count();
    (void)motor_read_right_count();

    pit_us_init(CCU61_CH1, PID_SAMPLE_US);
    pit_disable(CCU61_CH1);
}

// 开启速度环：清 PI/转向/速度参考状态→读一次编码器清残留→复位里程和请求→切 NORMAL、开 1ms 中断。
// 正式跑图目标由离线位置速度轨迹直接给出，不在车端再次缓慢规划。
void control_enable(void)
{
    control_reset_runtime_timing();
    speed_pi_reset();
    control_reset_turn_state();
    speed_plan_reset(0.0f);
    // 再读一次编码器，避免旧计数直接带入控制
    (void)motor_read_left_count();
    (void)motor_read_right_count();
    // 每次发车重新累计里程显示；放在残留计数清理之后，起点干净的 0
    total_dist = 0.0f;
    // 清上一轮停车原因，让本次运行结束后显示的是本轮的停车原因
    g_stop_reason = STOP_REASON_NONE;
    // 清失控保护计数和残留请求，避免上一轮故障状态把新一轮发车瞬间停掉
    motor_fault_ms_l = 0u;
    motor_fault_ms_r = 0u;
    motor_fault_run_ms = 0u;
    motor_motion_seen_l = 0u;
    motor_motion_seen_r = 0u;
    control_limit_mask_last = TELEMETRY_CONTROL_LIMIT_NONE;
    motor_fault_request = 0u;
    // 启动 1ms 控制中断。先置标志再开中断，保证空闲里程更新先让位
    control_mode = CONTROL_MODE_NORMAL;
    pit_enable(CCU61_CH1);
}

// 重置转向相关状态。发车航向重锚由prestart事务统一请求并等待CPU1确认。
void control_reset_turn_state(void)
{
    turn_control = 0.0f;
    turn_command_last_cmps = 0.0f;
    slip_guard_encoder_yaw_rate_degps = 0.0f;
    slip_guard_gyro_yaw_rate_degps = 0.0f;
    slip_guard_filter_initialized = 0u;
    slip_guard_active = 0u;
}

// 完整关闭控制输出：先关左右轮驱动，再关负压 PWM
void control_disable(void)
{
    control_disable_drive_only();
    bldc_stop();
}

// 只关速度环和左右轮输出，不关负压。
// 顺序：先关 1ms 中断再切 IDLE，保证不会出现“中断和后台任务同时读编码器”的两条路径并存。
void control_disable_drive_only(void)
{
    // 先关中断再清标志，保证不会出现两条路径同时读编码器
    pit_disable(CCU61_CH1);
    control_mode = CONTROL_MODE_IDLE;

    motor_stop();
    motor_disable();

    speed_pi_reset();
    speed_plan_reset(0.0f);
}
// 停车状态下定期读自由运行编码器，由 10ms 后台任务调用。IDLE 时本函数读，NORMAL/CALIBRATION 时 1ms 速度环读。
// 读左右两轮非原子：若读完左轮、没读右轮时发车切换插进来，1ms 中断也去读编码器会“两边各读走一半计数”，
// 里程/航向就乱了。所以所有权检查+两轮读取整个包在关中断临界区里。
void control_odometer_idle_update(void)
{
    uint32 interrupt_state = interrupt_global_disable();
    int16 raw_l;
    int16 raw_r;

    if (CONTROL_MODE_IDLE != control_mode)
    {
        interrupt_global_enable(interrupt_state);
        return;
    }

    // 所有权检查和两轮读取必须原子完成，避免发车/标定切换在两轮之间插入
    raw_l = motor_read_left_count();
    raw_r = motor_read_right_count();
    interrupt_global_enable(interrupt_state);

    // 推车调试:未发车时惯导照常推进(10ms 一步,推车速度下每步毫米级),
    // 区切换/弯口武装/对表全套照常,TFT 实时可看。发车后本函数让位、
    // 1ms 速度环接管 nav_update,推车态自然退出。
    // 会话在进入光电调试页时由 nav_start() 开启(key.c)。
    nav_update(raw_l, raw_r, 0.010f);
}

// 速度控制主循环，由 cc61_pit_ch1_isr 以 1ms 周期调用。
// 每拍固定为：采样→生成左右轮参考→纯增量PI→输出/遥测/保护。
void pit_speed_pid(void)
{
    control_mode_enum active_mode = control_mode;
    control_wheel_reference_struct reference;
    control_motor_output_struct output;
    uint32 control_start_us = system_getval_us();
    float sample_period_s = control_record_period(control_start_us);
    int16 raw_l;
    int16 raw_r;
    float actual_l;
    float actual_r;
    float actual_body_cmps;
    float soft_limit;
    uint16 elapsed_ms;
    uint32 next_run_ms;

    if (CONTROL_MODE_IDLE == active_mode)
    {
        motor_stop();
        motor_disable();
        control_record_execution(control_start_us);
        return;
    }

    /* Keep the request sticky until the safety task claims STOPPING.  This
     * makes the output cut happen in the 1 ms producer even if the 5/10 ms
     * background task is busy with map matching. */
    if (motor_fault_request)
    {
        motor_stop();
        motor_disable();
        control_record_execution(control_start_us);
        return;
    }

    raw_l = motor_read_left_count();
    raw_r = motor_read_right_count();
    actual_l = (float)raw_l * DIST_PER_PULSE_MM /
               sample_period_s / 10.0f;
    actual_r = (float)raw_r * DIST_PER_PULSE_MM /
               sample_period_s / 10.0f;
    actual_body_cmps = 0.5f * (actual_l + actual_r);
    if (actual_body_cmps < 0.0f)
    {
        actual_body_cmps = 0.0f;
    }
    control_actual_speed_abs_cmps = fabsf(actual_l);
    if (fabsf(actual_r) > control_actual_speed_abs_cmps)
    {
        control_actual_speed_abs_cmps = fabsf(actual_r);
    }
    actual_l_debug = actual_l;
    actual_r_debug = actual_r;
    soft_limit = max_duty_limit;
    elapsed_ms = control_period_ms(sample_period_s);
    if (motor_fault_run_ms < control_safety.start_grace_ms)
    {
        next_run_ms = motor_fault_run_ms + elapsed_ms;
        motor_fault_run_ms = (next_run_ms >= control_safety.start_grace_ms) ?
            control_safety.start_grace_ms : next_run_ms;
    }
    if (0 != raw_l)
    {
        motor_motion_seen_l = 1u;
    }
    if (0 != raw_r)
    {
        motor_motion_seen_r = 1u;
    }

    control_build_wheel_reference(raw_l,
                                  raw_r,
                                  actual_body_cmps,
                                  sample_period_s,
                                  &reference);
    control_apply_speed_loop(actual_l,
                             actual_r,
                             soft_limit,
                             &reference,
                             &output);

    telemetry_capture_control_isr(raw_l,
                                  raw_r,
                                  reference.target_l_cmps,
                                  reference.target_r_cmps,
                                  0.0f,
                                  0.0f,
                                  (int16)output.duty_l,
                                  (int16)output.duty_r,
                                  reference.yaw_rate_error_degps,
                                  reference.tracking_turn_cmps);
    control_capture_limit_transition(output.limit_mask);

    /* A high-load command whose measured wheel speed collapses relative to
     * its target is handled as a sticky request.  During the intentional
     * post-end active brake, the target is zero and the counters are reset so
     * reverse braking is not mistaken for a wheel stall. */
    if (NAV_STATE_FINISHED == nav_get_state())
    {
        motor_fault_ms_l = 0u;
        motor_fault_ms_r = 0u;
    }
    else
    {
        control_update_fault_counter(output.duty_l,
                                     reference.target_l_cmps,
                                     actual_l,
                                     raw_l,
                                     soft_limit,
                                     elapsed_ms,
                                     (uint8)(motor_motion_seen_l ||
                                              (motor_fault_run_ms >=
                                               control_safety.start_grace_ms)),
                                     &motor_fault_ms_l);
        control_update_fault_counter(output.duty_r,
                                     reference.target_r_cmps,
                                     actual_r,
                                     raw_r,
                                     soft_limit,
                                     elapsed_ms,
                                     (uint8)(motor_motion_seen_r ||
                                              (motor_fault_run_ms >=
                                               control_safety.start_grace_ms)),
                                     &motor_fault_ms_r);
    }

    /* The requested output is still recorded above for diagnosis, but once a
     * wheel confirms a stall the physical output is cleared in this same 1 ms
     * tick.  The next tick exits at the sticky-request guard above. */
    if (motor_fault_request)
    {
        motor_stop();
        motor_disable();
    }
    else
    {
        motor_duty(PWM_L, DIR_L, (int16)output.duty_l);
        motor_duty(PWM_R, DIR_R, (int16)output.duty_r);
    }
    control_record_execution(control_start_us);
}

float control_get_actual_speed_abs_cmps(void)
{
    return control_actual_speed_abs_cmps;
}
#pragma section all restore
