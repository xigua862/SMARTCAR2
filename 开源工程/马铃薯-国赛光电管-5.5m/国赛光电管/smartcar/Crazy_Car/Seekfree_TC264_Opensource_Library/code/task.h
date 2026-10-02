/**
 * @file task.h
 * @brief CPU0 后台调度、扫图入口、地图匹配和安全裁决。
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

// task.h —— CPU0（控制核）软定时任务调度框架：1ms PIT 节拍按分频派发到 Duty_Xms() 子任务

#ifndef CODE_TASK_H_
#define CODE_TASK_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 调度状态：各 cnt_Xms 在 1ms PIT 中断里自增 1，主循环到点派发对应 Duty_Xms() 后清零
typedef struct
{
    int16 cnt_1ms;    // 到 1 派发 Duty_1ms 并清零
    int16 cnt_2ms;    // 到 2 派发 Duty_2ms 并清零
    int16 cnt_4ms;    // 到 4 派发 Duty_4ms 并清零
    int16 cnt_5ms;    // 到 5 派发 Duty_5ms 并清零
    int16 cnt_10ms;   // 到 10 派发 Duty_10ms 并清零
    int16 cnt_20ms;   // 到 20 派发 Duty_20ms 并清零
    int16 cnt_50ms;   // 到 50 派发 Duty_50ms 并清零
    int16 cnt_100ms;  // 到 100 派发 Duty_100ms 并清零
    int16 cnt_200ms;  // 到 200 派发 Duty_200ms 并清零
    int16 cnt_500ms;  // 到 500 派发 Duty_500ms 并清零
    int16 cnt_1000ms; // 到 1000 派发 Duty_1000ms 并清零
} loop_t;

// 全局调度状态实例，定义在 task.c
extern volatile loop_t loop;

// 初始化 1ms 节拍源（CCU60_CH0），在 core0_main() 中调用一次
void task_init(void);
// 节拍推进：在 CCU60_CH0 PIT 中断里每 1ms 调用一次
void task_loop_tick(void);
// 分发主循环：在 CPU0 主循环里每圈调用一次，根据计数器派发到 Duty_Xms()
void task_main_loop(void);

/* The timestamped scan history contains raw odometry only.  Map localization
 * never rewrites it. */
void task_seed_phototube_scan_pose_history(void);
void task_capture_phototube_scan_pose_from_isr(void);
/* Publish a new capture epoch and bind the CPU0 staging consumer to the source
 * sequence immediately before that boundary.  CPU1 in-flight frames retain
 * their previous epoch and cannot enter the new raw-odom run. */
void task_prepare_phototube_scan_ingress(void);
/* Publish the distinct close epoch while the CPU0 stop transaction still has
 * interrupts masked.  Finalization later waits for CPU1 and drains data. */
void task_close_phototube_scan_ingress(void);
/* Pull the latest CPU1 line observation for diagnostics outside the 1 ms
 * control ISR.  It never contributes a steering command. */
void task_update_phototube_line_diagnostics(void);
/* Stop-boundary drain: close the capture epoch, wait boundedly for CPU1 to
 * acknowledge it, then consume the exact final publish cutoff. */
void task_finalize_phototube_scan(void);

// 分频子任务：由 1ms 节拍派生的各周期任务，主循环到点调用一次。空实现的档位为预留位。
void Duty_1ms(void);
void Duty_2ms(void);
void Duty_4ms(void);
void Duty_5ms(void);    // 5ms 保护裁决、按键扫描和菜单输入
void Duty_10ms(void);   // 参数/空闲里程、保护复查和10ms点云匹配
void Duty_20ms(void);   // 预留
void Duty_50ms(void);
void Duty_100ms(void);  // 预留低频监控
void Duty_200ms(void);
void Duty_500ms(void);
void Duty_1000ms(void); // 参数自动保存等

// 蜂鸣器接口（当前工程预留）
void buzzer_init(void);
void buzzer_beep(uint16 ms);
#ifdef __cplusplus
}
#endif

#endif /* CODE_TASK_H_ */
