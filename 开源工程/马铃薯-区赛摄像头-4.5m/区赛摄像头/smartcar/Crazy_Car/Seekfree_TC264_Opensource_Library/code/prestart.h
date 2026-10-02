/**
 * @file prestart.h
 * @brief 负压预热、发车切换、运行计时和停车状态机。
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

/*
 * prestart.h
 *
 * 发车前负压预热、二次确认、负压稳定发车状态机。
 * 陀螺零偏沿用上电静置校准，发车流程不再重新校准。
 */

#ifndef CODE_PRESTART_H_
#define CODE_PRESTART_H_

#include "zf_common_headfile.h"

/* 第一次按键：启动负压预热和发车前状态机。 */
void prestart_begin(void);
/* 第二次按键：确认发车，负压稳定 500ms 后直接发车（零偏用上电校准）。 */
uint8 prestart_confirm_launch(void);
/* 请求停车，先停驱动，负压延后关闭。 */
void prestart_stop(void);
/* 5ms 状态机节拍，由 CCU60_CH1 中断调用。 */
void prestart_tick_5ms(void);
/* IMU 外部中断更新后的回调；发车已不依赖校准，保留为空实现兼容调用点。 */
void prestart_gyro_update(void);
/* 查询发车状态机是否完全空闲；标定页和 WiFi 调试据此判断能否接管。 */
uint8 prestart_is_idle(void);
/* 查询是否仍处于第一次按键后的预热阶段。 */
uint8 prestart_is_warming_up(void);
/* 查询是否已经预热完成并等待第二次确认。 */
uint8 prestart_is_waiting_confirm(void);
/* 查询车是否已经真正进入运行期。 */
uint8 prestart_is_running(void);
/* 正常运行和手动编码器标定共用的负压档位。 */
uint8 prestart_running_suction_percent(void);

#endif /* CODE_PRESTART_H_ */
