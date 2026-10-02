/**
 * @file PID.c
 * @brief 左右轮增量 PI 的基础计算。
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

// PID 控制函数。当前只保留增量式 PI 速度环。
// 注：老的视觉定速函数（calculate_speed/speed_decision/fangcha_speed）和
// 实车恒速 5m/s 的旧速度阶跃接口均已删除。

#include "PID.h"
#include <math.h>

// 初始化与复位

// 初始化增量式 PI(速度环用)：只填 KP/KI，限幅取绝对值防外部传负数把逻辑搞反。
void pi_init(PID *pi, float kp, float ki, float incrementLimit)
{
    pi->KP = kp;
    pi->KI = ki;

    pi->IncrementLimit = fabsf(incrementLimit);

    pi->Error = 0.0f;
    pi->LastError = 0.0f;
}

// PID 计算

// 增量式 PI(速度环)：误差=target-actual，输出本周期应叠加到占空比的增量(已增量限幅)。
// 比例项看误差比上一拍变了多少，积分项看当前误差多大。
float pi_increment(PID *pi, float actual, float target)
{
    pi->Error = target - actual;
    float p_term = pi->KP * (pi->Error - pi->LastError); // 比例项：误差这一拍的变化
    float i_term = pi->KI * pi->Error;                   // 积分项：当前误差大小

    float increment = p_term + i_term;

    // 增量限幅：单拍变化别太猛
    if (increment > pi->IncrementLimit)
        increment = pi->IncrementLimit;
    if (increment < -pi->IncrementLimit)
        increment = -pi->IncrementLimit;

    pi->LastError = pi->Error; // 记住本拍误差供下一拍算比例项

    return increment;
}

// 清零 PI 内部状态(不动 KP/KI)。重启控制/切换赛道模式时调用，避免残留误差影响。
void pid_reset(PID *pid)
{
    pid->Error = 0.0f;
    pid->LastError = 0.0f;
}
