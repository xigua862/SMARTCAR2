/**
 * @file prestart.c
 * @brief 负压预热、VQF 发车重锚、运行和停车事务。
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

/*
 * prestart.c —— 发车/台架测试状态机实现。
 * 实车流程：负压预热 → 负压稳定 → 自动正式运行 → 停车；台架模式跳过执行器。
 * 先预热负压：吸风机从静止到运行档要时间，先低档转 2s、再切运行档等 500ms 稳定，吸力上来再发车，
 * 车不易打滑飞出赛道。陀螺零偏沿用上电静置校准（风机没开、车不动、读数最干净）——发车时开着
 * 风机再标定会把振动误当零偏、每次发车都不一样，已废弃；发车只归零航向并以当前姿态重锚 roll/pitch。
 */

#include "prestart.h"
#include "encoder_test.h"

#include "control.h"
#include "flash.h"
#include "globals.h"
#include "gyroscope.h"
#include "key.h"
#include "motor.h"
#include "navigation.h"
#include "phototube_scan.h"
#include "task.h"
#include "telemetry.h"
#include "telemetry_format.h"

#if ENCODER_TEST_MODE_ENABLE && \
    (PRESTART_BENCH_TEST_MODE || PRESTART_STRAIGHT_SPEED_TEST_MODE)
#error "ENCODER_TEST_MODE_ENABLE cannot be combined with bench/straight speed test mode"
#endif

// 发车状态值会写入遥测，保留既有数值；历史值 2 不再对应运行状态。
typedef enum
{
    PRESTART_IDLE = 0u,         // 空闲：负压和控制都关闭
    PRESTART_WARMUP = 1u,       // 第一次按键后：保持预热档负压
    PRESTART_SUCTION_SETTLING = 3u, // 等待负压和振动稳定
    PRESTART_RUNNING = 4u,      // 开启速度环和控制保护，车正式运行
    PRESTART_STOPPING = 5u,     // 停车：先停驱动，负压延时保持后再关闭
} prestart_stage_enum;

// 当前阶段。volatile：会被 5ms 中断修改，主循环读取时不能被缓存。
static volatile prestart_stage_enum prestart_stage = PRESTART_IDLE;
/* The timer ISR posts the launch transition; the CPU0 main loop commits it. */
static volatile uint8 prestart_launch_pending = 0u;
static volatile uint8 prestart_launch_committing = 0u;
static volatile uint32 prestart_launch_command_seq = 0u;
// 进入当前阶段后累计毫秒数，每 5ms 节拍加 5，只用于预热、稳定和停车阶段等待。
static volatile uint16 prestart_elapsed_ms = 0u;
static volatile uint8 path_brake_pending = 0u;
static volatile uint8 path_brake_ready = 0u;
static volatile uint16 path_brake_stable_ms = 0u;
// 正式运行时的负压档位集中在 competition_config.h。

static uint8 prestart_reason_is_protection(uint8 stop_reason)
{
    switch (stop_reason)
    {
    case STOP_REASON_LINE_LOST:
    case STOP_REASON_PLAN_MISMATCH:
    case STOP_REASON_ROLL:
    case STOP_REASON_PITCH:
    case STOP_REASON_ASSISTANT:
    case STOP_REASON_MOTOR_FAULT:
    case STOP_REASON_TRIGGER_TRAP:
    case STOP_REASON_IMU_STALE:
        return 1u;
    default:
        return 0u;
    }
}

// 清空状态机内部计时和状态，不直接操作电机或负压输出。
static void prestart_reset_state(void)
{
    prestart_elapsed_ms = 0u;
    path_brake_pending = 0u;
    path_brake_ready = 0u;
    path_brake_stable_ms = 0u;
    prestart_stage = PRESTART_IDLE;
    prestart_launch_pending = 0u;
    prestart_launch_committing = 0u;
    prestart_launch_command_seq = 0u;
}

// 运行/手动标定统一使用的负压档位，保证运行档只有一个定义处。
uint8 prestart_running_suction_percent(void)
{
    return (uint8)PRESTART_RUNNING_SUCTION_PERCENT;
}

uint8 prestart_is_straight_speed_test(void)
{
#if PRESTART_STRAIGHT_SPEED_TEST_MODE
    return 1u;
#else
    return 0u;
#endif
}

#if PRESTART_STRAIGHT_SPEED_TEST_MODE && \
    (PRESTART_STRAIGHT_SPEED_TEST_PROFILE == 1u)
/* Evaluate the analytical test trajectory without modifying the 5ms state
 * machine.  The motor reference below intentionally reads this at t+lead:
 * the physical wheel response then lands on the nominal trajectory at t. */
static float prestart_speed_test_profile_at_elapsed(uint32 elapsed_ms)
{
    uint32 elapsed_after_prestep;
    uint32 elapsed_after_accel;
    uint32 elapsed_after_hold;

    if (elapsed_ms < PRESTART_STRAIGHT_SPEED_TEST_PRESTEP_MS)
    {
        return 0.0f;
    }

    elapsed_after_prestep = elapsed_ms - PRESTART_STRAIGHT_SPEED_TEST_PRESTEP_MS;
    if (elapsed_after_prestep < PRESTART_STRAIGHT_SPEED_TEST_RAMP_ACCEL_MS)
    {
        return PRESTART_STRAIGHT_SPEED_TEST_TARGET_CMPS *
            (float)elapsed_after_prestep /
            (float)PRESTART_STRAIGHT_SPEED_TEST_RAMP_ACCEL_MS;
    }

    elapsed_after_accel = elapsed_after_prestep -
        PRESTART_STRAIGHT_SPEED_TEST_RAMP_ACCEL_MS;
    if (elapsed_after_accel < PRESTART_STRAIGHT_SPEED_TEST_HOLD_MS)
    {
        return PRESTART_STRAIGHT_SPEED_TEST_TARGET_CMPS;
    }

    elapsed_after_hold = elapsed_after_accel - PRESTART_STRAIGHT_SPEED_TEST_HOLD_MS;
    if (elapsed_after_hold < PRESTART_STRAIGHT_SPEED_TEST_RAMP_DECEL_MS)
    {
        return PRESTART_STRAIGHT_SPEED_TEST_TARGET_CMPS *
            (1.0f - (float)elapsed_after_hold /
             (float)PRESTART_STRAIGHT_SPEED_TEST_RAMP_DECEL_MS);
    }
    return 0.0f;
}
#endif

#if PRESTART_STRAIGHT_SPEED_TEST_MODE && \
    (PRESTART_STRAIGHT_SPEED_TEST_PROFILE == 2u)
/* The operating-point test deliberately contains hard speed steps.  It
 * starts from a moderate base speed, so the first step measures the speed
 * loop rather than the motor's zero-speed breakaway or a PWM rail hit. */
static float prestart_speed_test_operating_point_at_elapsed(uint32 elapsed_ms)
{
    uint32 elapsed_after_prestep;
    uint32 elapsed_after_ramp;
    uint32 elapsed_after_settle;
    uint32 elapsed_after_high;

    if (elapsed_ms < PRESTART_STRAIGHT_SPEED_TEST_PRESTEP_MS)
    {
        return 0.0f;
    }
    elapsed_after_prestep = elapsed_ms -
        PRESTART_STRAIGHT_SPEED_TEST_PRESTEP_MS;
    if (elapsed_after_prestep < PRESTART_STRAIGHT_SPEED_TEST_RAMP_TO_BASE_MS)
    {
        return PRESTART_STRAIGHT_SPEED_TEST_BASE_CMPS *
            (float)elapsed_after_prestep /
            (float)PRESTART_STRAIGHT_SPEED_TEST_RAMP_TO_BASE_MS;
    }
    elapsed_after_ramp = elapsed_after_prestep -
        PRESTART_STRAIGHT_SPEED_TEST_RAMP_TO_BASE_MS;
    if (elapsed_after_ramp < PRESTART_STRAIGHT_SPEED_TEST_BASE_SETTLE_MS)
    {
        return PRESTART_STRAIGHT_SPEED_TEST_BASE_CMPS;
    }
    elapsed_after_settle = elapsed_after_ramp -
        PRESTART_STRAIGHT_SPEED_TEST_BASE_SETTLE_MS;
    if (elapsed_after_settle < PRESTART_STRAIGHT_SPEED_TEST_HIGH_HOLD_MS)
    {
        return PRESTART_STRAIGHT_SPEED_TEST_STEP_HIGH_CMPS;
    }
    elapsed_after_high = elapsed_after_settle -
        PRESTART_STRAIGHT_SPEED_TEST_HIGH_HOLD_MS;
    if (elapsed_after_high < PRESTART_STRAIGHT_SPEED_TEST_LOW_HOLD_MS)
    {
        return PRESTART_STRAIGHT_SPEED_TEST_BASE_CMPS;
    }
    return 0.0f;
}
#endif

#if PRESTART_STRAIGHT_SPEED_TEST_MODE && \
    (PRESTART_STRAIGHT_SPEED_TEST_PROFILE == 3u)
/* Persistently exciting speed steps around a 4 m/s operating point expose
 * the motor time constant and delay without spending the seven-metre runway
 * on a long steady hold.  Both wheels receive the same body-speed sequence;
 * the straight-hold controller remains the only differential command. */
static float prestart_speed_test_identification_at_elapsed(uint32 elapsed_ms)
{
    static const float segment_targets_cmps[
        PRESTART_STRAIGHT_SPEED_TEST_ID_SEGMENT_COUNT] =
    {
        440.0f, 400.0f, 480.0f, 400.0f,
        460.0f, 400.0f, 480.0f, 400.0f
    };
    uint32 elapsed_after_prestep;
    uint32 elapsed_after_settle;
    uint32 segment_index;

    if (elapsed_ms < PRESTART_STRAIGHT_SPEED_TEST_PRESTEP_MS)
    {
        return 0.0f;
    }
    elapsed_after_prestep = elapsed_ms -
        PRESTART_STRAIGHT_SPEED_TEST_PRESTEP_MS;
    /* PROFILE=3 intentionally starts with a hard 0 -> 400 cm/s step. */
    if (elapsed_after_prestep <
        PRESTART_STRAIGHT_SPEED_TEST_ID_BASE_SETTLE_MS)
    {
        return PRESTART_STRAIGHT_SPEED_TEST_ID_BASE_CMPS;
    }
    elapsed_after_settle = elapsed_after_prestep -
        PRESTART_STRAIGHT_SPEED_TEST_ID_BASE_SETTLE_MS;
    segment_index = elapsed_after_settle /
        PRESTART_STRAIGHT_SPEED_TEST_ID_SEGMENT_MS;
    if (segment_index < PRESTART_STRAIGHT_SPEED_TEST_ID_SEGMENT_COUNT)
    {
        return segment_targets_cmps[segment_index];
    }
    return 0.0f;
}
#endif

/* 速度测试按已测执行延迟提前目标；日志中的目标就是实际送给纯增量PI的目标，
 * 不是仅供显示的未补偿曲线。正式跑图使用独立的离线位置速度轨迹。 */
float prestart_get_speed_test_target_cmps(void)
{
#if PRESTART_STRAIGHT_SPEED_TEST_MODE
#if PRESTART_STRAIGHT_SPEED_TEST_PROFILE == 1u
    if (PRESTART_RUNNING != prestart_stage)
    {
        return 0.0f;
    }
    return prestart_speed_test_profile_at_elapsed(
        (uint32)prestart_elapsed_ms + MOTOR_REFERENCE_LEAD_TIME_MS);
#elif PRESTART_STRAIGHT_SPEED_TEST_PROFILE == 2u
    if (PRESTART_RUNNING != prestart_stage)
    {
        return 0.0f;
    }
    return prestart_speed_test_operating_point_at_elapsed(
        (uint32)prestart_elapsed_ms + MOTOR_REFERENCE_LEAD_TIME_MS);
#elif PRESTART_STRAIGHT_SPEED_TEST_PROFILE == 3u
    if (PRESTART_RUNNING != prestart_stage)
    {
        return 0.0f;
    }
    return prestart_speed_test_identification_at_elapsed(
        (uint32)prestart_elapsed_ms + MOTOR_REFERENCE_LEAD_TIME_MS);
#else
    if ((PRESTART_RUNNING == prestart_stage) &&
        (prestart_elapsed_ms < PRESTART_STRAIGHT_SPEED_TEST_PRESTEP_MS))
    {
        return 0.0f;
    }
    if ((PRESTART_RUNNING == prestart_stage) &&
        (prestart_elapsed_ms < (PRESTART_STRAIGHT_SPEED_TEST_PRESTEP_MS +
                                PRESTART_STRAIGHT_SPEED_TEST_HOLD_MS)))
    {
        return PRESTART_STRAIGHT_SPEED_TEST_TARGET_CMPS;
    }
    return 0.0f;
#endif
#else
    return 0.0f;
#endif
}

uint8 prestart_speed_test_should_stop(void)
{
#if PRESTART_STRAIGHT_SPEED_TEST_MODE
    /* Bound the commanded run to six metres and leave nominal runway for
     * residual coasting.  total_dist is reset by control_enable() at the same
     * launch transaction. */
    if ((PRESTART_RUNNING == prestart_stage) &&
        (total_dist >= PRESTART_STRAIGHT_SPEED_TEST_STOP_DISTANCE_M))
    {
        return 1u;
    }
#if PRESTART_STRAIGHT_SPEED_TEST_PROFILE == 1u
    if ((PRESTART_RUNNING == prestart_stage) &&
        (prestart_elapsed_ms >= (PRESTART_STRAIGHT_SPEED_TEST_PRESTEP_MS +
                                 PRESTART_STRAIGHT_SPEED_TEST_RAMP_ACCEL_MS +
                                 PRESTART_STRAIGHT_SPEED_TEST_HOLD_MS +
                                 PRESTART_STRAIGHT_SPEED_TEST_RAMP_DECEL_MS +
                                 PRESTART_STRAIGHT_SPEED_TEST_POSTSTEP_MS)))
    {
        return 1u;
    }
#elif PRESTART_STRAIGHT_SPEED_TEST_PROFILE == 2u
    if ((PRESTART_RUNNING == prestart_stage) &&
        (prestart_elapsed_ms >=
         (PRESTART_STRAIGHT_SPEED_TEST_PRESTEP_MS +
          PRESTART_STRAIGHT_SPEED_TEST_RAMP_TO_BASE_MS +
          PRESTART_STRAIGHT_SPEED_TEST_BASE_SETTLE_MS +
          PRESTART_STRAIGHT_SPEED_TEST_HIGH_HOLD_MS +
          PRESTART_STRAIGHT_SPEED_TEST_LOW_HOLD_MS +
          PRESTART_STRAIGHT_SPEED_TEST_POSTSTEP_MS)))
    {
        return 1u;
    }
#elif PRESTART_STRAIGHT_SPEED_TEST_PROFILE == 3u
    if ((PRESTART_RUNNING == prestart_stage) &&
        (prestart_elapsed_ms >=
         (PRESTART_STRAIGHT_SPEED_TEST_PRESTEP_MS +
          PRESTART_STRAIGHT_SPEED_TEST_ID_BASE_SETTLE_MS +
          PRESTART_STRAIGHT_SPEED_TEST_ID_SEGMENT_MS *
              PRESTART_STRAIGHT_SPEED_TEST_ID_SEGMENT_COUNT +
          PRESTART_STRAIGHT_SPEED_TEST_ID_POSTSTEP_MS)))
    {
        return 1u;
    }
#else
    if ((PRESTART_RUNNING == prestart_stage) &&
        (prestart_elapsed_ms >= (PRESTART_STRAIGHT_SPEED_TEST_PRESTEP_MS +
                                 PRESTART_STRAIGHT_SPEED_TEST_HOLD_MS +
                                 PRESTART_STRAIGHT_SPEED_TEST_POSTSTEP_MS)))
    {
        return 1u;
    }
#endif
#endif
    return 0u;
}

// 建立发车状态机用的 5ms 定时器（CCU60_CH1）。先建后关，发车时（prestart_begin）才真正打开。
void prestart_init(void)
{
    pit_ms_init(CCU60_CH1, 5);
    pit_disable(CCU60_CH1);
}

// 第一次发车按键入口：实车模式开启负压预热；台架模式只启动静止测试计时。
uint8 prestart_begin(void)
{
    // 只有 IMU 服务正常且零偏校准完成后，手动按键才允许启动驱动板和负压。
    if (!prestart_is_launch_ready())
    {
        return 0u;
    }
#if (!PRESTART_BENCH_TEST_MODE && !PRESTART_STRAIGHT_SPEED_TEST_MODE)
    /* Reject an invalid/partially exported route before enabling the motor
     * driver or suction.  nav_start() keeps its existing reset semantics;
     * this read-only preflight prevents a bad map from briefly entering the
     * running control path. */
    if (!nav_ready_for_start())
    {
        return 0u;
    }
#endif
    /* Do not enter a running state with zero/erased speed PI parameters.  A
     * failed Flash load is a visible launch refusal; it must not silently
     * become an uncontrolled drive attempt. */
    if (!flash_parameters_valid(&g_flash_parameters))
    {
        return 0u;
    }
    prestart_reset_state();
#if PRESTART_BENCH_TEST_MODE
    /* Bench mode must not generate even the ESC's 1 ms idle pulse. */
    bldc_stop();
    prestart_stage = PRESTART_SUCTION_SETTLING;
    /* No suction settling is needed on the bench; commit the static session
     * on the next CPU0 main-loop pass instead of waiting 500 ms. */
    prestart_launch_pending = 1u;
#elif ENCODER_TEST_MODE_ENABLE
    /* Map encoder test drives the real speed loop with the wheels lifted.
     * Skip suction and the two-second warmup; keep the normal launch
     * transaction so telemetry and the IMU command boundary stay identical. */
    motor_enable();
    bldc_stop();
    phototube_scan_stop();
    prestart_stage = PRESTART_SUCTION_SETTLING;
    prestart_launch_pending = 1u;
#else
    motor_enable();                        // 发车前按高1.5ms、低10us、再高的时序复位FOC驱动板
    bldc_set_percent(PRESTART_WARMUP_SUCTION_PERCENT); // 低档负压预热
    prestart_stage = PRESTART_WARMUP;
#endif
    pit_enable(CCU60_CH1);                 // 打开 5ms 定时中断，让状态机跑起来
    return 1u;
}

/*
 * 请求停车：普通停车只停驱动，负压由 STOPPING 阶段延时关闭（减小突然失去吸附的点头风险）。
 * 电机堵转保护属于紧急停车：驱动清零后立即关闭负压，先解除可能把车轮压死的吸附力。
 * 注意：不要在这里中止上电零飘校准——它是唯一一次校准，打断后 done 永不置位，
 * 发车条件将永久无法满足（直到重新上电）。
 */
void prestart_stop(void)
{
    uint32 interrupt_state;
    prestart_stage_enum stopping_from;
    NavSystemState final_nav_state;
    uint8 stop_reason;

    /* Claim STOPPING before any teardown so the main loop and 5 ms ISR cannot
     * enter a second stop path or replace the original stop reason. */
    interrupt_state = interrupt_global_disable();
    stopping_from = prestart_stage;
    if (PRESTART_STOPPING == stopping_from)
    {
        interrupt_global_enable(interrupt_state);
        return;                            // 已在停车流程，避免重复触发
    }
    prestart_stage = PRESTART_STOPPING;
    prestart_launch_pending = 0u;
    prestart_launch_committing = 0u;
    prestart_launch_command_seq = 0u;
    prestart_elapsed_ms = 0u;
    is_car_running = 0u;
    stop_reason = g_stop_reason;
    /* Stop the 1 ms producer and publish the scan close epoch before CPU0
     * interrupts are restored.  CPU1 remains independent, but every frame is
     * now unambiguously before or after this boundary. */
    control_disable_drive_only();
    if (STOP_REASON_MOTOR_FAULT == stop_reason)
    {
        /* Do not wait for the normal 500 ms STOPPING settle period when the
         * suspected cause is a wheel held down by the suction fan. */
        bldc_stop();
    }
    task_close_phototube_scan_ingress();
    final_nav_state = nav_get_state();
    interrupt_global_enable(interrupt_state);

    // 先停 1ms 驱动生产者，保证最后一个已完成的控制周期也能进入队列，
    // 在冻结日志前按停止边界有界收尾扫图队列。
    task_finalize_phototube_scan();
    nav_scan_finalize_pending_transaction();
    if (prestart_reason_is_protection(stop_reason))
    {
        telemetry_capture_event_isr(TELEMETRY_EVENT_PROTECTION,
                                    TELEMETRY_EVENT_SOURCE_SAFETY,
                                    stop_reason,
                                    (int32)stopping_from,
                                    (int32)final_nav_state);
    }
    telemetry_capture_event_isr(TELEMETRY_EVENT_PRESTART_STATE,
                                TELEMETRY_EVENT_SOURCE_PRESTART,
                                (uint16)PRESTART_STOPPING,
                                (int32)stopping_from,
                                (int32)PRESTART_STOPPING);
    telemetry_freeze(stop_reason,
                     (uint8)PRESTART_STOPPING,
                     (uint8)final_nav_state);
    phototube_scan_stop();
    nav_stop();                            // 任何停车原因都同步结束惯导回放，可重入
    pit_enable(CCU60_CH1);                 // 确保 5ms 节拍开着，才能走完延时关负压
}

uint8 prestart_is_warming_up(void)
{
    return (uint8)(PRESTART_WARMUP == prestart_stage);
}

// 车是否真正进入运行期。出界停车、姿态保护等运行期保护只应在 RUNNING 后生效，
// 不能在 WARMUP / SUCTION_SETTLING 阶段误关负压。
uint8 prestart_is_running(void)
{
    return (uint8)(PRESTART_RUNNING == prestart_stage);
}

void prestart_begin_path_brake(void)
{
    if ((PRESTART_RUNNING != prestart_stage) || path_brake_pending)
    {
        return;
    }

    path_brake_pending = 1u;
    path_brake_ready = 0u;
    path_brake_stable_ms = 0u;
    prestart_elapsed_ms = 0u;
    /* Navigation is already at its terminal sample; no more phototube
     * blackout decisions should interrupt the controlled braking phase. */
    phototube_scan_stop();
}

uint8 prestart_path_brake_ready(void)
{
    return path_brake_ready;
}

// 只有 IDLE 表示负压、发车计时和速度环都未占用车辆。
uint8 prestart_is_idle(void)
{
    return (uint8)(PRESTART_IDLE == prestart_stage);
}

uint8 prestart_is_launch_ready(void)
{
    ImuSnapshot snapshot;

    if (PRESTART_IDLE != prestart_stage ||
        !gyro_cpu1_service_ready() ||
        !imu_snapshot_read(&snapshot) ||
        !snapshot.ready ||
        !snapshot.zero_calibrated ||
        snapshot.zero_calibrating)
    {
        return 0u;
    }

    /* Motion at the instant of the button press is intentionally allowed.
     * The zero-bias calibration itself was already guarded by the encoder
     * motion check; launch must not reject a valid button press just because
     * the car is being positioned on the start line. */
    return 1u;
}

// 5ms 发车状态机，由 CCU60_CH1 中断调用：预热计时、负压稳定等待、发车切换、负压延时关闭。
void prestart_tick_5ms(void)
{
    // IDLE：状态机没启动，直接返回不空转
    if (PRESTART_IDLE == prestart_stage)
    {
        return;
    }

    // STOPPING：已停驱动，等 500ms 让负压多吸一会儿，时间到再彻底关控制/负压并复位回 IDLE
    if (PRESTART_STOPPING == prestart_stage)
    {
        prestart_elapsed_ms += 5u;
        if (prestart_elapsed_ms >= PRESTART_STOP_SUCTION_HOLD_MS)
        {
            control_disable();                     // 彻底关控制（含负压关闭）
            pit_disable(CCU60_CH1);
            prestart_reset_state();
        }
        return;
    }

    if ((PRESTART_RUNNING == prestart_stage) && path_brake_pending)
    {
        prestart_elapsed_ms += 5u;
        if (control_get_actual_speed_abs_cmps() <=
            PRESTART_PATH_BRAKE_SPEED_THRESHOLD_CMPS)
        {
            path_brake_stable_ms += 5u;
        }
        else
        {
            path_brake_stable_ms = 0u;
        }
        if ((path_brake_stable_ms >= PRESTART_PATH_BRAKE_STABLE_MS) ||
            (prestart_elapsed_ms >= PRESTART_PATH_BRAKE_TIMEOUT_MS))
        {
            path_brake_ready = 1u;
        }
        return;
    }

    if (prestart_launch_pending || prestart_launch_committing)
    {
        return;
    }

    /* Static bench sessions are stopped by the second KEY_1 press, not by the
     * real-car 12 s course timeout. */
#if PRESTART_BENCH_TEST_MODE
    if (PRESTART_RUNNING == prestart_stage)
    {
        return;
    }
#endif

    // 以下阶段（WARMUP / SUCTION_SETTLING / RUNNING）随时间推进，统一先加 5ms
    prestart_elapsed_ms += 5u;

    // WARMUP：低档负压预热，满 2 秒切运行档并进入负压稳定
    if (PRESTART_WARMUP == prestart_stage)
    {
        if (prestart_elapsed_ms >= PRESTART_WARMUP_MS)
        {
            prestart_elapsed_ms = 0u;
            prestart_stage = PRESTART_SUCTION_SETTLING;
#if PRESTART_BENCH_TEST_MODE
            bldc_stop();
#else
            bldc_set_percent(prestart_running_suction_percent());  // 吸风机切到运行档
#endif
        }
        return;
    }

    // SUCTION_SETTLING：吸风机运行档，等 500ms 让吸力/振动稳定后发车
    if (PRESTART_SUCTION_SETTLING == prestart_stage)
    {
        /*
         * 负压运行档稳定 500ms 后直接发车：归零航向 + 重锚姿态零点 + 开速度环。
         * prestart_begin() 已要求CPU1静止零偏校准完成；此处只处理发车重锚，
         * 不在风机振动工况下重复估计零偏。
         * launch_rezero 锁存当前 VQF yaw 作为发车航向零点，并重锚姿态保护零点。
        */
        if ((prestart_elapsed_ms >= PRESTART_SUCTION_SETTLING_MS) &&
            !prestart_launch_pending)
        {
            /* Do not perform EMEM writes or enable the 1 ms control ISR from
             * this timer ISR.  prestart_service() commits the complete launch
             * transaction in the CPU0 main loop. */
            prestart_launch_pending = 1u;
            prestart_elapsed_ms = 0u;
        }
        return;
    }

}

void prestart_service(void)
{
    uint32 interrupt_state;
    prestart_stage_enum previous_stage;

    if (!prestart_launch_pending && !prestart_launch_committing)
    {
        return;
    }

    /* CPU1 applies the launch reference on an actual IMU/VQF frame.  Do not
     * enable control until that frame has been published and acknowledged. */
    if (prestart_launch_committing &&
        !gyro_command_acknowledged(prestart_launch_command_seq))
    {
        return;
    }

    if (!prestart_launch_committing)
    {
        interrupt_state = interrupt_global_disable();
        if (!prestart_launch_pending ||
            (PRESTART_SUCTION_SETTLING != prestart_stage))
        {
            interrupt_global_enable(interrupt_state);
            return;
        }
        prestart_launch_pending = 0u;
        prestart_launch_committing = 1u;
        prestart_elapsed_ms = 0u;
        interrupt_global_enable(interrupt_state);

        prestart_launch_command_seq = gyro_request_launch_rezero();
        if (0u == prestart_launch_command_seq)
        {
            /* CPU1 service is not ready; leave the launch transaction armed
             * and retry on a later main-loop pass. */
            interrupt_state = interrupt_global_disable();
            prestart_launch_committing = 0u;
            prestart_launch_pending = 1u;
            prestart_launch_command_seq = 0u;
            interrupt_global_enable(interrupt_state);
            return;
        }

        telemetry_begin_run(nav_get_source_route_id(),
                            nav_get_total_points(),
                            nav_get_route_hash());
        return;
    }

    previous_stage = prestart_stage;
    interrupt_state = interrupt_global_disable();
    if (prestart_launch_committing &&
        (PRESTART_SUCTION_SETTLING == prestart_stage))
    {
        /* Keep the whole final transition atomic: no control ISR can observe
         * a RUNNING stage before the transition is complete. */
#if PRESTART_BENCH_TEST_MODE
        /* Passive navigation remains available for encoder/IMU telemetry, but
         * the wheel driver and 1 ms speed loop stay disabled on the bench. */
        control_disable_drive_only();
        nav_start();
#elif PRESTART_STRAIGHT_SPEED_TEST_MODE
        /* Real motor test: keep navigation idle so the map curvature cannot
         * inject a turn command into the straight speed-loop run.  Reset the
         * odometry session once, then leave navigation idle; scan service
         * still consumes the standalone odometry-only updates. */
        control_enable();
        nav_start();
        nav_stop();
#elif ENCODER_TEST_MODE_ENABLE
        /* Real map-control test: same navigation and 1 ms speed PI, but no
         * suction/phototube side path because the wheels are lifted. */
        control_enable();
        nav_start();
#else
        control_enable();
        nav_start();
#endif
        prestart_stage = PRESTART_RUNNING;
#if (!PRESTART_BENCH_TEST_MODE && !PRESTART_STRAIGHT_SPEED_TEST_MODE && \
    !ENCODER_TEST_MODE_ENABLE && \
    PHOTOTUBE_SCAN_AUTO_RUN)
        // 正常地图运行时启动连续扫图定位；定位器不直接生成转向量。
        phototube_scan_start();
        task_prepare_phototube_scan_ingress();
        task_seed_phototube_scan_pose_history();
#endif
        telemetry_activate_run();
        prestart_launch_committing = 0u;
        prestart_launch_command_seq = 0u;
        /* The logger only starts accepting producers at the same atomic
         * boundary as control/nav.  Emit RUN_START after activation so the
         * event cannot be dropped by the closed-session guard. */
        telemetry_capture_event_isr(TELEMETRY_EVENT_RUN_START,
                                    TELEMETRY_EVENT_SOURCE_PRESTART,
                                    nav_get_source_route_id(),
                                    (int32)nav_get_total_points(),
                                    (int32)nav_get_route_hash());
        telemetry_capture_event_isr(TELEMETRY_EVENT_PRESTART_STATE,
                                    TELEMETRY_EVENT_SOURCE_PRESTART,
                                    (uint16)PRESTART_RUNNING,
                                    (int32)previous_stage,
                                    (int32)PRESTART_RUNNING);
    }
    else
    {
        prestart_launch_committing = 0u;
        prestart_launch_command_seq = 0u;
    }
    interrupt_global_enable(interrupt_state);
}
