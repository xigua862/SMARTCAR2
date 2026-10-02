/**
 * @file task.h
 * @brief CPU0 软件任务调度、诊断和运行保护。
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

#ifndef CODE_TASK_H_
#define CODE_TASK_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ==================== 调度状态结构体 ====================
 * 用于记录 1ms 节拍和各档位的待执行计数。
 * 每个 cnt_Xms 字段在 1ms PIT 中断里自增 1，主循环消费后清零。
 */
typedef struct
{
    uint8 check_flag; /* 节拍看门狗：每 1ms 置 1，主循环跑完一圈清 0。用于检测超时 */
    int16 err_flag;   /* 主循环超时计数：1ms 中断时若 check_flag 仍 >= 1，此处 +1 */
    int16 cnt_1ms;    /* 1ms   档累积计数 —— 到 1 就派发 Duty_1ms    并清零 */
    int16 cnt_2ms;    /* 2ms   档累积计数 —— 到 2 就派发 Duty_2ms    并清零 */
    int16 cnt_4ms;    /* 4ms   档累积计数 —— 到 4 就派发 Duty_4ms    并清零 */
    int16 cnt_5ms;    /* 5ms   档累积计数 —— 到 5 就派发 Duty_5ms    并清零 */
    int16 cnt_10ms;   /* 10ms  档累积计数 —— 到 10 就派发 Duty_10ms   并清零 */
    int16 cnt_20ms;   /* 20ms  档累积计数 —— 到 20 就派发 Duty_20ms   并清零 */
    int16 cnt_50ms;   /* 50ms  档累积计数 —— 到 50 就派发 Duty_50ms   并清零 */
    int16 cnt_100ms;  /* 100ms 档累积计数 —— 到 100 就派发 Duty_100ms  并清零 */
    int16 cnt_200ms;  /* 200ms 档累积计数 —— 到 200 就派发 Duty_200ms  并清零 */
    int16 cnt_500ms;  /* 500ms 档累积计数 —— 到 500 就派发 Duty_500ms  并清零 */
    int16 cnt_1000ms; /* 1s    档累积计数 —— 到 1000 就派发 Duty_1000ms 并清零 */
} loop_t;

/* 全局调度状态实例，定义在 task.c 中，通过调试器可观察各 cnt_Xms 推进情况 */
extern loop_t loop;

/* 初始化 1ms 节拍源（CCU60_CH0），在 core0_main() 中调用一次 */
void task_init(void);

/* 节拍推进：在 CCU60_CH0 PIT 中断里每 1ms 调用一次 */
void task_loop_tick(void);

/* 分发主循环：在 CPU0 主循环里每圈调用一次，根据计数器派发到 Duty_Xms() */
void task_main_loop(void);

/* ==================== 分频子任务 ====================
 * 5ms 处理按键和菜单输入，10ms 解析逐飞助手参数并发送示波器数据。
 * 20ms 预留，100ms 预留给低频监控任务。
 */
void Duty_1ms(void);    /* 1ms    周期任务 */
void Duty_2ms(void);    /* 2ms    周期任务 */
void Duty_4ms(void);    /* 4ms    周期任务 */
void Duty_5ms(void);    /* 5ms    按键扫描和菜单输入 */
void Duty_10ms(void);   /* 10ms   助手参数解析和示波器发送 */
void Duty_20ms(void);   /* 20ms   预留 */
void Duty_50ms(void);   /* 50ms   周期任务 */
void Duty_100ms(void);  /* 100ms  预留低频监控 */
void Duty_200ms(void);  /* 200ms  周期任务 */
void Duty_500ms(void);  /* 500ms  周期任务 */
void Duty_1000ms(void); /* 1000ms 周期任务（参数自动保存等） */
#ifdef __cplusplus
}
#endif

#endif /* CODE_TASK_H_ */
