/**
 * @file cpu1_main.c
 * @brief CPU1 VQF 与 16 路光电扫描入口。
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

// CPU1 主程序 —— IMU/VQF 服务与16路模拟光电阵列扫描核。

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "code/gyroscope.h"
#include "code/phototube_array.h"
#include "code/globals.h"                // CPU1→CPU0 光电状态信号
#include "zf_common_headfile.h"          // 逐飞开源库统一头文件

// 把本文件变量钉在 CPU1 自己的 RAM（与 CPU0 的 cpu0_dsram 相对），直到末尾 restore。范围不可改。
#pragma section all "cpu1_dsram"

// core1_main —— CPU1（光电扫描核）入口，上电由启动代码自动调用。永不返回。
void core1_main(void)
{
    disable_Watchdog();             // 关 CPU1 看门狗，避免后台扫描被复位
    interrupt_global_enable(0);

    // 与 CPU0 启动同步，等两核都就绪再继续，避免抢跑。
    cpu_wait_event_ready();
    // IMU/VQF 是CPU1唯一写者；初始化失败时保持未就绪，仍继续提供光电和CPU0诊断界面。
    (void)gyro_cpu1_init();
    phototube_array_init();
    phototube_menu_ready_signal = 1u;

    while (TRUE)
    {
        // 以共享毫秒时钟触发1kHz完整的16通道MUX扫描。
        phototube_array_task();
    }
}

#pragma section all restore
