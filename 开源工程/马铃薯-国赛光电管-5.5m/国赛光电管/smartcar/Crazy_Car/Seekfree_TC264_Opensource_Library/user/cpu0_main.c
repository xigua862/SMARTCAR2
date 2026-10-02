/**
 * @file cpu0_main.c
 * @brief CPU0 初始化、控制后台任务和遥测服务入口。
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

// CPU0 主程序 —— 控制核入口：菜单/按键、Flash参数、发车状态机和日志。
// CPU1负责IMU/VQF与16路光电；CPU0只消费一致姿态快照、原始odom点云和诊断观测。

#include "zf_common_headfile.h"     // 逐飞开源库统一头文件
#include "code/flash.h"             // CarParameters 参数的 Flash 读写
#include "code/key.h"               // TFT 菜单与按键
#include "code/adc_key.h"           // ADC 摇杆按键
#include "code/prestart.h"          // 发车状态机
#include "code/task.h"              // 软件任务调度器
#include "code/control.h"           // 速度/转向控制
#include "code/motor.h"             // 左右轮电机与编码器
#include "code/gyroscope.h"         // LSM6DSV16XTR 六轴 IMU
#include "code/globals.h"           // CPU0/CPU1 共享全局变量
#include "code/navigation.h"        // 路径导航
#include "code/phototube_scan.h"    // 位姿补偿扫图
#include "code/telemetry.h"         // 运行遥测：CPU0 写 EMEM，停车后按键导出

// 把本文件变量钉在 CPU0 自己的 RAM（DSRAM），直到文件末尾 restore。放对核才快、才不冲突，pragma 范围不能动。
#pragma section all "cpu0_dsram"

// core0_main —— CPU0（控制核）入口，上电由启动代码自动调用。永不返回（while(TRUE) 死循环）。
// 各步先后顺序有讲究（先时钟、再调试串口、再读 Flash 参数……），顺序改了可能读到未初始化的参数或脏数据。
int core0_main(void)
{
    clock_init();                   // 时钟必须最先配好，后面所有外设都依赖它
    /* clock_init() re-enables IRQs; keep startup atomic while PIT/ERU
     * peripherals are created and the optional EMEM logger is checked. */
    interrupt_global_disable();
    debug_init();                   // EMEM 可能因芯片型号/ECC配置被禁用，先保证失败可诊断

    // 先加载存档参数，再初始化依赖参数的控制/菜单模块。
    flash_parameter_init();
    adc_key_init();                 // ADC 摇杆按键
    key_init(5);                    // 物理按键（库函数，5ms 消抖）
    prestart_init();                // 发车状态机 5ms 定时器（先建后关）
    menu_platform_init();           // TFT 显示 + 菜单
    task_init();
    motor_init();
    control_init();
    nav_init();
    phototube_scan_init();
    telemetry_init();               // 显示/调试和外设就绪后再初始化 EMEM，避免上电黑屏
    // 开机即落在线路传感器调试页(menu_init 里 current_menu 直接设死,早于本行,
    // 从不经过 key.c 里"按确认键进入"的那条分支)——开机就地开一个推车
    // 调试会话,手推车即可在该页看阵列位置/有效性/累计纠偏,不用先进主菜单绕一圈。
    // key.c 里发车页的 nav_start() 仅覆盖"退出再进"重新清零的场景。
    nav_start();

    // 所有外设初始化完成后再开全局中断。
    interrupt_global_enable(0);
    // 与 CPU1 握手：确认两核都就绪后再一起进主循环，避免某核抢跑。
    cpu_wait_event_ready();

    // CPU1 在此屏障后初始化 IMU/VQF 和光电。不要在 CPU0 永久等待 IMU 就绪，
    // 否则 IMU 初始化失败时菜单和按键也会一起锁死；发车事务仍会拒绝未就绪的 IMU。
    while (TRUE)
    {
        /* Export is allowed only after the vehicle is fully idle.  Keep it
         * on a dedicated path so the 1 kHz phototube display and scan
         * background work cannot insert long gaps between UART blocks. */
        if (TELEMETRY_STATE_EXPORTING == telemetry_get_state())
        {
            telemetry_export_service();
            continue;
        }
        prestart_service();       // 提交发车切换；EMEM 会话初始化不在 5ms ISR 中执行
        // 后台任务在主循环调度，硬实时速度环在 PIT 中断执行。
        task_main_loop();
        telemetry_service();        // 限量把 DSRAM staging 写入 EMEM
        telemetry_export_service(); // 完全空闲时启动导出；后续主循环走上面的专用快路径
        telemetry_link_service();   // 非导出空闲期发送低频 TLD1 信标，供上位机自动识别串口
    }
}

#pragma section all restore
