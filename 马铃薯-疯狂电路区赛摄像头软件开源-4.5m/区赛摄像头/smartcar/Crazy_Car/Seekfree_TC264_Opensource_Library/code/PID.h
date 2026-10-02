/**
 * @file PID.h
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
 * PID 控制模块头文件。
 * 定义 PID 参数结构以及当前控制链路使用的计算接口。
 *********************************************************************************************************************/
#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* 通用 PID 状态结构。
 * 速度环和转向环都复用这组字段，未使用的项保持 0。
 */
typedef struct
{
    // PID 参数
    float KP;  // 比例系数
    float KI;  // 积分系数
    float KD;  // 微分系数
    float KP2; // 非线性平方项比例系数
    float GKD; // 陀螺仪微分系数

    // 误差与状态记录
    float Error;      // 当前误差
    float LastError;  // 上一次误差
    float IncrementLimit; // 每周期增量限幅
} PID;

// ==================== 初始化与复位 ====================
void PID_Init_Basic(PID *pid, float kp, float ki, float kd);
void PI_Init(PID *pi, float kp, float ki, float incrementLimit);
void PID_Reset(PID *pid);

// ==================== PID 计算 ====================
// 增量式 PI，用于速度环
float PI_Increment(PID *pi, float actual, float target);

// 位置式 PID，带陀螺仪补偿，用于转向环
float PID_Position(PID *pid, float actual, float target, float gyro_value);

#ifdef __cplusplus
}
#endif

#endif /* CODE_PID_H_ */
