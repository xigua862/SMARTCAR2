/**
 * @file PID.h
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

// PID 控制模块头文件：PID 参数结构与控制函数接口。
// CPU0 在 1ms 速度环调增量式 PI；横向转向由单一目标 yaw 控制器完成。

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 速度 PI 状态结构。
typedef struct
{
    float KP;  // 比例系数
    float KI;  // 积分系数，消除稳态误差

    // 误差与状态记录
    float Error;      // 当前误差
    float LastError;  // 上一次误差

    float IncrementLimit; // 增量限幅：防单拍输出变化太猛
} PID;

// 初始化与复位
void pi_init(PID *pi, float kp, float ki, float incrementLimit);
void pid_reset(PID *pid);

// PID 计算
// 增量式 PI，用于速度环。输出是"这一拍占空比该加/减多少"，天然带记忆、抗积分饱和
float pi_increment(PID *pi, float actual, float target);

#ifdef __cplusplus
}
#endif

#endif /* CODE_PID_H_ */
