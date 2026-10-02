/**
 * @file control.c
 * @brief 1 ms 速度环、地图转向执行和电机保护。
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
 *
 * @note CPU0 的 1 ms 控制中断调用，修改时需保持硬实时预算。
 */

#include "control.h"
#include "motor.h"
#include "PID.h"
#include "globals.h"
#include "flash.h"
#include <stdlib.h>
#include "gyroscope.h"
#include "image.h"
#include "navigation.h"

#pragma section all "cpu0_dsram"

typedef enum
{
    CONTROL_MODE_IDLE = 0u,
    CONTROL_MODE_NORMAL,
} control_mode_enum;

typedef struct
{
    uint8 fault_duty_percent;
    int16 fault_count_max;
    uint16 fault_confirm_ms;
} control_safety_config_struct;

/* 电机/编码器反馈异常判定参数。 */
static const control_safety_config_struct control_safety = {
    70u,
    1,
    100u,
};

/* 转向量最多等于当前基础速度，保证左右轮目标速度均不小于零。 */
#define TURN_CONTROL_LIMIT_RATIO (1.0f)

static volatile control_mode_enum control_mode = CONTROL_MODE_IDLE;

static float plan_speed = 0.0f;
static float plan_start = 0.0f;
static float plan_target = 0.0f;
static uint32 plan_tick = 0u;
static uint32 plan_total_tick = 1u;

/* 电机/编码器失控保护：占空比被推到接近软限幅、编码器却几乎无脉冲，说明
 * 速度反馈链路断了（编码器线松/插头脱落）或车轮堵转——增量 PI 失去反馈
 * 会把占空比一路加到满幅狂飙。连续满足 control_safety.fault_confirm_ms 才判
 * 失控，正常起步时占空比远未到阈值前编码器就已有脉冲，不会误触发。 */
static uint16 motor_fault_ms_l = 0u;
static uint16 motor_fault_ms_r = 0u;

/* 重置速度规划器的内部状态。
 * speed_now 既作为当前速度，也作为新的起点和目标，避免启动瞬间产生阶跃。
 */
static void speed_plan_reset(float speed_now)
{
    plan_speed = speed_now;
    plan_start = speed_now;
    plan_target = speed_now;
    plan_tick = 0u;
    plan_total_tick = 1u;
}

/* 清零速度 PI 及其累计占空比，供正常发车和所有停车路径复用。 */
static void speed_pi_reset(void)
{
    left_TargetSpeed = 0.0f;
    right_TargetSpeed = 0.0f;
    PID_Reset(&speed_PID1);
    PID_Reset(&speed_PID2);
}

/* 初始化控制模块：读取 Flash 参数、配置 PID，并准备 1ms 速度环 PIT。 */
void control_init(void)
{
    PI_Init(&speed_PID1, g_flash_parameters.speed_KP, g_flash_parameters.speed_KI,
            g_flash_parameters.speed_IncrementLimit);
    PI_Init(&speed_PID2, g_flash_parameters.speed_KP, g_flash_parameters.speed_KI,
            g_flash_parameters.speed_IncrementLimit);

    PID_Init_Basic(&turn_PID, g_flash_parameters.turn_KP, 0.0f, g_flash_parameters.turn_KD);
    turn_PID.KP2 = g_flash_parameters.turn_KP2;
    turn_PID.GKD = g_flash_parameters.turn_GKD;

    /* 同步自由运行编码器的软件基准。 */
    (void)motor_read_left_count();
    (void)motor_read_right_count();

    pit_us_init(CCU61_CH1, PID_SAMPLE_US);
    pit_disable(CCU61_CH1);
}

/* 设置目标速度和过渡时间。
 * time_ms 为 0 时立即切到目标速度；非 0 时由 speed_plan_update() 做平滑过渡。
 */
void control_set_speed(float target_speed, uint32 time_ms)
{
    plan_start = plan_speed;
    plan_target = target_speed;
    plan_tick = 0u;

    if (time_ms == 0u)
    {
        plan_speed = target_speed;
        plan_start = target_speed;
        plan_total_tick = 1u;
        plan_tick = 1u;
        return;
    }

    plan_total_tick = (time_ms * 1000u) / PID_SAMPLE_US;
    if (plan_total_tick == 0u)
    {
        plan_total_tick = 1u;
    }
}

/* 更新一次速度规划输出。
 * 使用三次平滑曲线 3s^2-2s^3，减少目标速度变化时的突变。
 */
static float speed_plan_update(void)
{
    if (plan_tick >= plan_total_tick)
    {
        plan_speed = plan_target;
        return plan_speed;
    }

    plan_tick++;

    float s = (float)plan_tick / (float)plan_total_tick;
    if (s > 1.0f)
    {
        s = 1.0f;
    }

    float h = 3.0f * s * s - 2.0f * s * s * s;

    plan_speed = plan_start + (plan_target - plan_start) * h;

    return plan_speed;
}

/* 开启速度环。
 * 发车状态机完成负压稳定后调用，负责清旧输出、清编码器残留并启动 1ms PIT。
 */
void control_enable(void)
{
    /* 清零输出并复位 PI 积分状态。 */
    speed_pi_reset();
    control_reset_turn_state();
    speed_plan_reset(0.0f);
    /* 再读一次编码器，避免旧计数直接带入控制。 */
    (void)motor_read_left_count();
    (void)motor_read_right_count();
    /* 每次发车重新累计里程显示；放在残留计数清理之后，起点干净的 0。 */
    total_dist = 0.0f;
    /* 清上一轮停车原因，让本次运行结束后显示的是本轮的停车原因。 */
    g_stop_reason = STOP_REASON_NONE;
    /* 清失控保护计数和残留请求，避免上一轮的故障状态把新一轮发车瞬间停掉。 */
    motor_fault_ms_l = 0u;
    motor_fault_ms_r = 0u;
    motor_fault_request = 0u;
    /* 同理清掉预发车阶段可能挂起的图像停车请求：CPU1 的计数重置要等下一帧
     * 才生效，这 10ms 窗口里旧请求足以把刚发的车停掉。 */
    image_stop_request = 0u;

    /* 启动 1ms 控制中断。先置标志再开中断，保证空闲里程更新先让位。 */
    control_mode = CONTROL_MODE_NORMAL;
    pit_enable(CCU61_CH1);
    control_set_speed(350.0f, 320);
}

/* 重置转向相关状态。
 * 发车、停车或图像状态重置时调用，避免上一轮弯道状态影响下一次运行。
 */
void control_reset_turn_state(void)
{
    PID_Reset(&turn_PID);
    turn_control = 0.0f;
    gyro_request_yaw_total_reset();
    image_request_vision_state_reset();
}

/* 完整关闭控制输出。
 * 会先关闭左右轮驱动，再关闭负压 PWM，一般用于最终停车或复位状态。
 */
void control_disable(void)
{
    control_disable_drive_only();
    pwm_set_duty(BLDC_CHANNEL, 0);
}

/* 只关闭速度环和左右轮输出，不关闭负压。
 * 保护停车时负压需要延后关闭，所以使用这个接口先停驱动。
 */
void control_disable_drive_only(void)
{
    /* 关闭控制中断。先关中断再清标志，保证不会出现两条路径同时读编码器。 */
    pit_disable(CCU61_CH1);
    control_mode = CONTROL_MODE_IDLE;

    /* 停止电机输出。 */
    motor_stop();

    /* 清零当前目标输出。 */
    speed_pi_reset();
    speed_plan_reset(0.0f);
}
/* 停车状态下定期读取自由运行编码器，由 10ms 后台任务调用。
 * IDLE 时由本函数读取，NORMAL 时由 1ms 速度环读取。
 * 所有权检查和两轮读取放在短临界区，模式切换不能插到两轮之间。 */
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

    /* 检查所有权和读取两轮必须原子完成，避免发车切换在两轮之间插入。 */
    raw_l = motor_read_left_count();
    raw_r = motor_read_right_count();
    interrupt_global_enable(interrupt_state);

    /* 推车调试:未发车时惯导照常推进(10ms 一步,推车速度下每步毫米级),
     * 区切换/弯口武装/对表全套照常,TFT 实时可看。发车后本函数让位、
     * 1ms 速度环接管 nav_update,推车态自然退出。
     * 会话在进图像调试页时由 nav_start() 开启(key.c)。 */
    nav_update(raw_l, raw_r);
}

/* 速度控制主循环，由 cc61_pit_ch1_isr 以 1ms 周期调用。
 *
 * 流程：
 *   1. 读取左右编码器计数并换算线速度。
 *   2. 计算转向控制量。
 *   3. 更新速度规划目标。
 *   4. 组合左右轮目标速度。
 *   5. 通过速度 PI 累加得到输出占空比。
 *   6. 软限幅后写入 FOC。
 */
void pit_speed_pid(void)
{
    control_mode_enum active_mode = control_mode;
    float target_l;
    float target_r;
    float soft_limit = max_duty_limit;

    if (CONTROL_MODE_IDLE == active_mode)
    {
        motor_stop();
        return;
    }

    /* (1) 读取当前编码器反馈。 */
    int16 raw_l = motor_read_left_count();
    int16 raw_r = motor_read_right_count();

    float actual_l = (float)raw_l * CMPS_PER_COUNT;
    float actual_r = (float)raw_r * CMPS_PER_COUNT;
    actual_l_debug = actual_l;
    actual_r_debug = actual_r;

    /* 惯导回放:按本周期里程推进目标航向(非导航态内部直接返回)。 */
    nav_update(raw_l, raw_r);

    /* (2) 计算转向控制量并使用普通速度规划。 */
    float turn_diff = 0.0f;
    if (NAV_STATE_NAVIGATING == nav_get_state())
    {
        turn_diff = nav_get_correction() * NAV_CORRECTION_TO_DIFF;
    }
    turn_control = PID_Position(&turn_PID, turn_diff, 0.0f,
                                (0.0f == turn_diff) ? 0.0f : IMU_Data.gyro_z);

    {
        float plan_v = speed_plan_update();
        float turn_limit = plan_v * TURN_CONTROL_LIMIT_RATIO;

        if (turn_limit < 0.0f)
        {
            turn_limit = -turn_limit;
        }
        if (turn_control > turn_limit)
        {
            turn_control = turn_limit;
        }
        else if (turn_control < -turn_limit)
        {
            turn_control = -turn_limit;
        }
        target_l = plan_v - turn_control;
        target_r = plan_v + turn_control;
    }

    /* (5) 通过速度 PI 累加得到占空比。 */
    left_TargetSpeed += PI_Increment(&speed_PID1, actual_l, target_l);
    right_TargetSpeed += PI_Increment(&speed_PID2, actual_r, target_r);

    /* (6) 软限幅。 */
    if (left_TargetSpeed > soft_limit)
        left_TargetSpeed = soft_limit;
    if (left_TargetSpeed < -soft_limit)
        left_TargetSpeed = -soft_limit;
    if (right_TargetSpeed > soft_limit)
        right_TargetSpeed = soft_limit;
    if (right_TargetSpeed < -soft_limit)
        right_TargetSpeed = -soft_limit;

    /* 写入 FOC，内部会处理方向和占空比。 */
    motor_duty(PWM_L, DIR_L, (int16)left_TargetSpeed);
    motor_duty(PWM_R, DIR_R, (int16)right_TargetSpeed);

    /* (7) 电机/编码器失控保护：任一轮"近满幅占空比 + 编码器无脉冲"持续
     * 达到配置确认时间就置失控标志。正常行驶由 Duty_10ms 消费停车。 */
    {
        float fault_duty = soft_limit * (float)control_safety.fault_duty_percent / 100.0f;

        /* 漏斗式计数：故障帧 +1、正常帧 -2（不清零）。编码器线"松而未断"时
         * 接触时断时续，偶发的正常毫秒不会抹掉已积累的故障证据，
         * 只要多数时间无反馈照样能确认；真正常行驶时计数快速回落到 0。 */
        if (((left_TargetSpeed >= fault_duty) || (left_TargetSpeed <= -fault_duty)) &&
            (raw_l <= control_safety.fault_count_max) && (raw_l >= -control_safety.fault_count_max))
        {
            if (motor_fault_ms_l < control_safety.fault_confirm_ms)
            {
                motor_fault_ms_l++;
            }
            if (motor_fault_ms_l >= control_safety.fault_confirm_ms)
            {
                motor_fault_request = 1u;
            }
        }
        else
        {
            motor_fault_ms_l = (motor_fault_ms_l > 2u) ? (uint16)(motor_fault_ms_l - 2u) : 0u;
        }

        if (((right_TargetSpeed >= fault_duty) || (right_TargetSpeed <= -fault_duty)) &&
            (raw_r <= control_safety.fault_count_max) && (raw_r >= -control_safety.fault_count_max))
        {
            if (motor_fault_ms_r < control_safety.fault_confirm_ms)
            {
                motor_fault_ms_r++;
            }
            if (motor_fault_ms_r >= control_safety.fault_confirm_ms)
            {
                motor_fault_request = 1u;
            }
        }
        else
        {
            motor_fault_ms_r = (motor_fault_ms_r > 2u) ? (uint16)(motor_fault_ms_r - 2u) : 0u;
        }
    }

}

#pragma section all restore
