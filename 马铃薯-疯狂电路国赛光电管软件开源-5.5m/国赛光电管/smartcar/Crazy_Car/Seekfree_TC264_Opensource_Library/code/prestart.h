/**
 * @file prestart.h
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
 * prestart.h —— 发车（起跑）状态机对外接口。
 * 发车前负压预热、负压稳定发车；陀螺零偏沿用上电静置校准，发车流程不再重新校准。
 */

#ifndef CODE_PRESTART_H_
#define CODE_PRESTART_H_

#include "zf_common_headfile.h"
#include "competition_config.h"

/*
 * 台架/EMEM 测试模式：按 KEY_1 后只建立静止遥测会话，不启动负压、轮毂
 * 驱动或 1 ms 速度环。再次按 KEY_1 停止并冻结，再按一次 KEY_1 导出镜像。
 * 实车发车使用 0；需要做静止 EMEM 测试时再临时改为 1。
 */

/* 实车直线速度测试：使用真实电机和负压，但不启动赛道地图转向。
 * 仅用于直道速度环辨识；测试完成后必须恢复为 0，正常扫图/比赛不能开启。 */
/*
 * 0 = 全速硬阶跃（只适合低目标速度）；
 * 1 = 原来的 0->6m/s 已知斜率曲线，主要辨识 kA；
 * 2 = 工作点响应测试：先到300cm/s，再做300<->450cm/s两次硬阶跃。
 * 3 = 在线辨识测试：起步先对400cm/s做硬阶跃，再执行4.0/4.4/4.8/4.6m/s伪随机短阶跃，
 *     用于拟合左右轮的静态模型、时间常数和执行延迟。
 */
/* PROFILE=3 在线辨识：所有有效阶跃均在4m/s以上，避免起步摩擦污染模型。 */
/* PROFILE=1 使用已知斜率的加速/减速曲线，避免0->6m/s瞬时目标阶跃。 */
/* 先保持静止，给编码器和遥测留出稳态基线。 */
/* PROFILE=1 的目标速度保持时间。 */
/* 目标回到0后等待500ms让减速波形落地，再请求停车。 */
/* PROFILE=1 的加速/减速时长。 */
/* 作为异常兜底，避免测试车因编码器/目标异常一直运行。 */

/* Route-end braking: keep the drive loop alive until both wheels are nearly
 * stopped, so the final stop does not become a PWM cut followed by coasting. */

// 发车/台架流程控制（由按键/菜单调用）
void prestart_init(void);               // 建立发车状态机用的 5ms 定时器（CCU60_CH1，先建后关）
uint8 prestart_begin(void);             // 实车：启动预热；台架：启动静止 EMEM 会话；IMU校准未完成返回0
void prestart_stop(void);               // 请求停车：先停驱动，负压延后关闭
void prestart_begin_path_brake(void);   // 路径末点已通过，进入主动制动阶段
uint8 prestart_path_brake_ready(void);  // 主动制动已降到停止阈值，可断驱动

// 时钟节拍与中断回调
void prestart_tick_5ms(void);           // 5ms 状态机节拍，由 CCU60_CH1 中断调用
void prestart_service(void);            // CPU0 主循环提交发车切换（不在 5ms ISR 写 EMEM）

// 状态查询（供菜单/保护逻辑判断当前阶段）
uint8 prestart_is_launch_ready(void);      // IMU服务正常且零偏校准完成时允许手动发车
uint8 prestart_is_idle(void);              // 是否完全空闲；只有此时允许开始手动编码器标定
uint8 prestart_is_warming_up(void);        // 是否仍在第一次按键后的预热阶段
uint8 prestart_is_running(void);           // 是否已真正进入运行期
uint8 prestart_running_suction_percent(void); // 运行和手动标定共用的负压档位
uint8 prestart_is_straight_speed_test(void);  // 当前是否为实车直线速度环测试模式
float prestart_get_speed_test_target_cmps(void); // 阶跃测试当前目标（0→目标速度→0）
uint8 prestart_speed_test_should_stop(void); // 加速/保持/减速波形完成后请求停车

#endif /* CODE_PRESTART_H_ */
