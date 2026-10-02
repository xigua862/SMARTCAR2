/**
 * @file PID.c
 * @brief 速度 PI 与转向控制器的基础计算。
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

/*********************************************************************************************************************
 * PID 控制与速度决策函数。
 *********************************************************************************************************************/
#include "PID.h"
#include <math.h>

/* 初始化基础 PID 参数。 */
void PID_Init_Basic(PID *pid, float kp, float ki, float kd)
{
    pid->KP = kp;
    pid->KI = ki;
    pid->KD = kd;
    pid->KP2 = 0.0f;
    pid->GKD = 0.0f;

    pid->Error = 0.0f;
    pid->LastError = 0.0f;
    pid->IncrementLimit = 0.0f;
}

/* 初始化增量式 PI 参数。 */
void PI_Init(PID *pi, float kp, float ki, float incrementLimit)
{
    pi->KP = kp;
    pi->KI = ki;
    pi->KD = 0.0f;
    pi->KP2 = 0.0f;
    pi->GKD = 0.0f;

    pi->IncrementLimit = fabsf(incrementLimit);

    pi->Error = 0.0f;
    pi->LastError = 0.0f;
}

/* 增量式 PI，误差定义为 target - actual，返回本周期应叠加的控制量。 */
float PI_Increment(PID *pi, float actual, float target)
{
    pi->Error = target - actual;
    /* 增量式 PI 输出的是本周期应该叠加到占空比上的变化量。 */
    float p_term = pi->KP * (pi->Error - pi->LastError);
    float i_term = pi->KI * pi->Error;

    float increment = p_term + i_term;

    if (increment > pi->IncrementLimit)
        increment = pi->IncrementLimit;
    if (increment < -pi->IncrementLimit)
        increment = -pi->IncrementLimit;

    pi->LastError = pi->Error;

    return increment;
}

/* 位置式 PID，附加陀螺仪补偿项，用于转向环。 */
float PID_Position(PID *pid, float actual, float target, float gyro_value)
{
    float error = target - actual;
    float error_delta = error - pid->LastError;

    /* KP2 是误差平方项，GKD 用角速度做阻尼，抑制转向过冲。 */
    float output = error * pid->KP + error * fabsf(error) * pid->KP2 + error_delta * pid->KD + gyro_value * pid->GKD;

    pid->LastError = error;
    pid->Error = error;

    return output;
}

/* 清零 PID 内部状态。 */
void PID_Reset(PID *pid)
{
    pid->Error = 0.0f;
    pid->LastError = 0.0f;
}
