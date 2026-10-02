/**
 * @file control.h
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

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* 速度 PI 固定采样周期，避免积分项随调用周期漂移。 */
#define PID_SAMPLE_S (0.001f) /* 1 ms */
#define PID_SAMPLE_US (1000u) /* 1 ms */

/* ==================== 线速度换算参数 ==================== */
/* 物理参数化（2026-07-12 用户实测）：胎径 24.00mm（卷尺）。
 * 减速比 3.896 由推车里程反推：f254535 实测 27.11@4.4 时里程误差
 * 0.04%，等价换算 4.4×24/27.11=3.896；若拿到真实齿数请直接替换。
 * 里程刻度只取决于 轮径/减速比 之比——24.00/3.896 与推车实测一致；
 * 24 配 4.4 会短 11.5%，26.00 配 4.4 实测短 4.2%（010bae0），勿用。
 * 换胎后推车 2m 复测一次即可终审。
 * 调节范围：22.00~26.00（±8%，覆盖胎磨损/气压/换胎；
 * 推车修正后落到范围外，说明错的是减速比或编码器线数，不是轮径）。
 * 23.02 = 完赛版本里程刻度的换算基线（完赛 26.00@4.4 × 3.896/4.4），
 * 2026-07-12 换回完赛胎后以此起步，推车 2m 后按比例微调。 */
#define WHEEL_DIAMETER_MM (23.02f)
#define ENCODER_PPR (1024u)
#define GEAR_RATIO (3.896f)

/* 每个脉冲对应的轮端距离，单位 mm。 */
#define DIST_PER_PULSE_MM (3.14159265f * WHEEL_DIAMETER_MM / (GEAR_RATIO * (float)ENCODER_PPR))

/* ==================== 航向刻度（与里程刻度解耦） ====================
 * 编码器差速航向：每 1 个左右轮脉冲差对应的航向角(度)。
 * 初值 = 每脉冲距离 0.018899mm ÷ 轮距 125mm(卷尺) × 57.2958 ≈ 0.00866。
 * 本宏是航向刻度的唯一来源（原顺时针 90° 编码器标定页已删），
 * 有意不用 DIST_PER_PULSE_MM 推导——保证两个旋钮完全解耦：
 *   ① 地图/里程整体缩放（推车 2m 读数偏小→调大） → 只调 WHEEL_DIAMETER_MM
 *   ② 弯角不对 → 只调本宏：
 *      直角走成锐角（实际转过头）→ 调大；走成钝角（转不够）→ 调小；
 *      按比例修：新值 = 旧值 × 实际转角/期望转角。
 * 调节范围：0.0075~0.0100（对应等效轮距 144~108mm，含滑移等效余量；
 *   90° 实测修出来落到范围外，先查转向方向符号 GYRO_ENC_YAW_SIGN、
 *   编码器线数和减速比，不要硬填）。
 * 0.00799 = 完赛版本航向刻度的换算基线（完赛 26.00@4.4 / 轮距130），
 * 2026-07-12 换回完赛胎后以此起步，90° 实测后按比例微调。 */
#define ENCODER_YAW_DEG_PER_DIFF_PULSE (0.00799f)

/* 每个控制周期的编码器计数换算为 cm/s。 */
#define CMPS_PER_COUNT (DIST_PER_PULSE_MM / PID_SAMPLE_S / 10.0f)

/* 初始化控制模块和 1ms 控制中断。 */
void control_init(void);

/* 开启控制环输出。 */
void control_enable(void);

/* 关闭控制环输出并清零目标。 */
void control_disable(void);

/* 只关闭速度环和左右轮输出，不关闭负压。 */
void control_disable_drive_only(void);

/* 清开跑前残留的转向输入和转向 PID 状态。 */
void control_reset_turn_state(void);

/* 1ms 周期控制入口。 */
void pit_speed_pid(void);
/* 设置目标速度，并按 time_ms 做平滑过渡。 */
void control_set_speed(float target_speed, uint32 time_ms);
/* 停车状态下的里程更新，由 10ms 后台任务调用；速度环运行时内部直接返回。 */
void control_odometer_idle_update(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_CONTROL_H_ */
