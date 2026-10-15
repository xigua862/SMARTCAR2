/**
  ******************************************************************************
  * @file    launch.h
  * @brief   发车读秒时序器（**纯逻辑，无硬件依赖，可在 PC 上单测**）
  *
  * 需求（用户 2026-10-15）：PC13 长按 200ms 发车 → 倒数 3 秒，这 3 秒里
  * 蜂鸣 **三声**，节奏 = **短 2 声 + 长 1 声** → 然后正式开始循迹。
  *
  * 本模块只回答一个问题："从发车那一刻算起过了 N 毫秒，这一拍该做什么？"
  * 它不碰蜂鸣器、不碰状态机（那是 main.c 的活），所以能拿一串假时间戳在电脑上跑完。
  *
  * 本文件不许 #include 任何 stm32/HAL/驱动头文件 —— 硬纪律（保证 PC 可单测）。
  ******************************************************************************
  */
#ifndef __APP_LAUNCH_H
#define __APP_LAUNCH_H

#include <stdint.h>

/* 读秒总时长：3 秒（进 RUN 的时刻）*/
#define LAUNCH_TOTAL_MS   3000u
/* 这 3 秒里响几声：短 短 长 */
#define LAUNCH_BEEP_N     3u

typedef enum {
    LAUNCH_EV_NONE = 0,     /* 这一拍没到点，什么都不用做 */
    LAUNCH_EV_BEEP_SHORT,   /* 该短鸣一声 */
    LAUNCH_EV_BEEP_LONG,    /* 该长鸣一声（第 3 声 = 松手那声）*/
    LAUNCH_EV_DONE          /* 读秒完了：该切 RUN 起跑（★只报一次）*/
} launch_ev_t;

typedef struct {
    uint8_t  active;   /* 1 = 正在读秒 */
    uint8_t  step;     /* 已经排到第几声（0 ~ LAUNCH_BEEP_N）*/
    uint32_t t0_ms;    /* 发车那一刻的时间戳 */
} launch_t;

/** @brief 开始读秒（把当前时刻当 t0）*/
void launch_begin(launch_t *l, uint32_t now_ms);

/** @brief 取消读秒（比如读秒期间按了 PB0）；之后 launch_step 只返回 NONE */
void launch_abort(launch_t *l);

/** @brief 是否正在读秒（1 = 是）*/
uint8_t launch_active(const launch_t *l);

/** @brief 已经读了多久（毫秒）；没在读秒时返回 0 */
uint32_t launch_elapsed_ms(const launch_t *l, uint32_t now_ms);

/**
  * @brief  推进一拍：返回这一拍该做的事
  * @note   每拍最多报一件事（真卡了拍也不会漏掉 DONE —— 到点优先判完事）
  */
launch_ev_t launch_step(launch_t *l, uint32_t now_ms);

#endif /* __APP_LAUNCH_H */
