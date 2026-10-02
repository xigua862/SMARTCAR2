/**
 * @file cpu0_main.c
 * @brief CPU0 外设初始化、控制任务和主循环入口。
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
 * CPU0 主程序
 *********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "code/flash.h"
#include "code/key.h"
#include "code/task.h"
#include "code/control.h"
#include "code/motor.h"
#include "code/gyroscope.h"
#include "code/image.h"
#include "code/globals.h"
#include "code/navigation.h"
#include "code/calibration.h"

#pragma section all "cpu0_dsram"

/* CPU0 主入口。
 * CPU0 负责控制、菜单、按键、Flash 参数、IMU 和发车状态机。
 */
int core0_main(void)
{
    clock_init();
    debug_init();
    /* 先加载参数，再初始化依赖参数的控制和菜单模块。 */
    flash_parameter_init();
    /* 标定记录独立于完赛参数，必须在 IMU/菜单初始化前恢复。 */
    calibration_init();
    /* 图传初始化暂由当前工程菜单/助手链路管理。 */
    menu_platform_init();
    task_init();
    motor_init();
    control_init();
    gyro_init();
    nav_init();
    /* 开机即落在图像调试页(menu_init 里 current_menu 直接设死,早于本行,
     * 从不经过 key.c 里"按确认键进入"的那条分支)——开机就地开一个推车
     * 调试会话,手推车即可在该页看 P/V/A(横向纠偏),不用先进主菜单绕一圈。
     * key.c 里发车页的 nav_start() 仅覆盖"退出再进"重新清零的场景。 */
    nav_start();
    /* 8701 使能 */
    gpio_high(MOTOR_ENABLE_PIN);
    /* 所有外设初始化完成后再打开全局中断。 */
    interrupt_global_enable(0);
    cpu_wait_event_ready();

    while (TRUE)
    {
        /* 后台任务调度在主循环执行，硬实时速度环在 PIT 中断中执行。 */
        task_main_loop();
    }
}

#pragma section all restore
