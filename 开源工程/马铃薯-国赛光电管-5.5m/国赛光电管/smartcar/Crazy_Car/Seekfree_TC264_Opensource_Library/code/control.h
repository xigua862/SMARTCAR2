/**
 * @file control.h
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

// control.h —— 底盘控制核心（CPU0）：1ms 中断生成左右轮参考并执行纯增量PI

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"
#include "competition_config.h"
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

// 控制器名义采样周期。实际控制拍会用 PIT 时间戳修正速度和规划
// （速度 PI 的离散增益仍以该名义周期标定）。
#define PID_SAMPLE_S (0.001f) // 1 ms
#define PID_SAMPLE_US (1000u) // 1 ms

/* Stall-protection thresholds are centralized in competition_config.h. */

// 实测物理胎径为23.5mm、齿比为4.1、左右轮接地中心距为119mm。编码器
// 距离换算继续使用“等效滚动直径”吸收传动与滚动误差：原4.4齿比配置下
// 车端显示210mm而尺子实测200mm；换成真实4.1齿比后保持相同每脉冲标定：
// 旧配置按齿比换算得到过23.8071mm；本次先将等效D设为23.5mm，验证负压
// 加载后的滚动直径是否导致里程提前。换胎或负压工况变化后仍只改等效D：
// D_new = D_old * 实际距离 / 车端显示距离。

// 每个脉冲对应的轮端距离(mm) = π×胎径 / (减速比 × 每圈脉冲数)
#define DIST_PER_PULSE_MM (3.14159265f * WHEEL_DIAMETER_MM / (GEAR_RATIO * (float)ENCODER_PPR))

// 航向刻度（与里程刻度解耦）：每 1 个左右轮脉冲差对应的航向角(度)。
// 初值 = 每脉冲距离 0.018899mm ÷ 轮距 125mm(卷尺) × 57.2958 ≈ 0.00866。
// 本宏是航向刻度的唯一来源（原顺时针 90° 编码器标定页已删），
// 有意不用 DIST_PER_PULSE_MM 推导——保证两个旋钮完全解耦：
//   ① 地图/里程整体缩放（推车 2m 读数偏小→调大） → 只调 WHEEL_DIAMETER_MM
//   ② 弯角不对 → 只调本宏：
//      直角走成锐角（实际转过头）→ 调大；走成钝角（转不够）→ 调小；
//      按比例修：新值 = 旧值 × 实际转角/期望转角。
// 调节范围：0.0075~0.0100（对应等效轮距 144~108mm，含滑移等效余量；
//   90° 实测修出来落到范围外，先查转向方向符号 GYRO_ENC_YAW_SIGN、
//   编码器线数和减速比，不要硬填）。
// 0.00799 = 完赛版本航向刻度基线（完赛 26.00@4.4 / 轮距130）。
// 2026-07-20 定稿 0.008285 = 里程刻度 0.018800 × 57.2958 ÷ 轮距 130(卷尺):
// 用两个更准的量(推车刻度 ±0.2%、卷尺轮距 ±0.4%)反推,等效轮距恰=130.0。
// 旁证:原地转 360° 三次测量(344/367/357°)各自反推 0.008362/0.008203/0.008272,
// 平均 0.008279,与本值差 0.07%,在单圈 ±1% 对位噪声内一致。
// 复核用连转 3 圈取 读数/3,判据 360±3°;单圈读数是随机行走,勿逐圈追。

// 每控制周期编码器计数换算 cm/s：计数 × 本宏 = 本周期线速度(cm/s)
#define CMPS_PER_COUNT (DIST_PER_PULSE_MM / PID_SAMPLE_S / 10.0f)

/* 直线速度测试按已测得的约22ms执行链提前下发目标。正式跑图的纵向
 * 规划使用地图速度上限、前方制动可达性和本地加减速度跟踪三层约束。 */
/* PI 允许在实际轮速明显高于目标时输出反向制动力，保持预赛完赛版执行顺序。 */

/* 车端在线速度规划的执行器约束。地图导出的逐点表只提供空间速度上限；
 * 本地规划器再用这些约束把上限变成可执行的时间轨迹。数值是候选基线，
 * 需要用实际轮速/PWM日志单独辨识，不通过横向误差反推。 */

/* 编码器给出轮端无滑移横摆率，VQF给出车身真实横摆率。高速急弯中两者
 * 经同一低通后仍明显分叉，说明轮胎纵/横向滑移或车身受冲击；此时只增加
 * 反向横摆阻尼，不改地图速度，也不削弱已经在反打恢复的控制量。 */

void control_init(void);
void control_enable(void);
void control_disable(void);
// 只关闭速度环和左右轮输出，不关闭负压
void control_disable_drive_only(void);
// 清开跑前残留的转向输入和 VQF 航向控制状态
void control_reset_turn_state(void);
// 1ms 周期控制入口
void pit_speed_pid(void);
// 停车状态下的里程更新，由 10ms 后台任务调用；速度环运行时内部直接返回
void control_odometer_idle_update(void);
// 末端主动制动使用的轮端绝对速度估计(cm/s)，由1ms速度环发布。
float control_get_actual_speed_abs_cmps(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_CONTROL_H_ */
