/**
 * @file cpu1_main.c
 * @brief CPU1 摄像头采集与图像处理入口。
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
 * @note CPU1 专用入口；图像发布后由 CPU0 只读消费。
 */

/*********************************************************************************************************************
 * CPU1 主程序
 *********************************************************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "code/camera_pipeline.h"
#include "code/key.h"
#include "zf_common_headfile.h"

#pragma section all "cpu1_dsram"

/* CPU1 主入口。
 * CPU1 负责相机初始化、图像帧复制和视觉处理，处理结果通过全局变量给 CPU0 控制环使用。
 */
void core1_main(void)
{
    disable_Watchdog();
    interrupt_global_enable(0);

    cpu_wait_event_ready();
    camera_pipeline_init();
    menu_set_camera_ready(camera_pipeline_is_ready());

    while (TRUE)
    {
        /* 相机任务检测新帧并执行图像处理；没有新帧时只做轻量等待。 */
        camera_pipeline_task();
    }
}

#pragma section all restore
