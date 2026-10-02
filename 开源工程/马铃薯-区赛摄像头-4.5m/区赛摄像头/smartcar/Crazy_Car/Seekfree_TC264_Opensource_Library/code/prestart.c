/**
 * @file prestart.c
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
 * prestart.c
 *
 * 发车前状态机：第一次按键开启低档负压预热，预热完成后等待第二次确认。
 * 第二次按键切到发车档，等待 500ms 稳定后直接发车。
 * 陀螺仪零偏沿用上电静置校准（安静工况，均值可信）——风机开着校准
 * 会把振动拍频烙进零偏、每次发车都不一样，已废弃；发车只归零航向
 * 并以当前姿态重锚 roll/pitch 保护零点。
 */

#include "prestart.h"

#include "control.h"
#include "globals.h"
#include "gyroscope.h"
#include "key.h"
#include "motor.h"
#include "navigation.h"

typedef enum
{
    PRESTART_IDLE = 0u,          /* 空闲：负压和控制都关闭。 */
    PRESTART_WARMUP,            /* 第一次按键后：保持预热档负压。 */
    PRESTART_WAIT_CONFIRM,      /* 预热完成：保持负压，等待第二次按键。 */
    PRESTART_SUCTION_SETTLING,  /* 第二次按键后：等待负压和振动状态稳定。 */
    PRESTART_RUNNING,           /* 开启速度环和控制保护，车正式运行。 */
    PRESTART_STOPPING,          /* 停车：先停驱动，负压延时保持后再关闭。 */
} prestart_stage_enum;

static volatile prestart_stage_enum prestart_stage = PRESTART_IDLE;
static uint16 prestart_elapsed_ms = 0u;
static const uint8 running_suction_percent = 100u;

/* 清空发车状态机内部计时和状态。
 * 只负责状态变量，不直接操作电机或负压输出。
 */
static void prestart_reset_state(void)
{
    prestart_elapsed_ms = 0u;
    prestart_stage = PRESTART_IDLE;
}

uint8 prestart_running_suction_percent(void)
{
    return running_suction_percent;
}

/* 第一次发车按键入口：开启预热档负压和 5ms 状态机。 */
void prestart_begin(void)
{
    prestart_reset_state();
    BLDC_set_percent(10u);
    prestart_stage = PRESTART_WARMUP;
    pit_enable(CCU60_CH1);
}

/* 第二次确认发车入口。
 * 只有预热完成并处于 WAIT_CONFIRM 时才有效，随后进入负压稳定等待阶段。
 */
uint8 prestart_confirm_launch(void)
{
    /* 只有预热结束后才能二次确认发车，避免第一次按键直接进入校准和控制。 */
    if (PRESTART_WAIT_CONFIRM != prestart_stage)
    {
        return 0u;
    }

    prestart_elapsed_ms = 0u;
    prestart_stage = PRESTART_SUCTION_SETTLING;
    BLDC_set_percent(prestart_running_suction_percent());

    return 1u;
}

/* 请求停车。
 * 只停驱动，负压由 STOPPING 阶段延时关闭。
 * 注意：不要在这里中止上电零飘校准——它是唯一一次校准，
 * 打断后 done 永远不置位，发车条件将永久无法满足（直到重新上电）。
 */
void prestart_stop(void)
{
    if (PRESTART_STOPPING == prestart_stage)
    {
        return;
    }

    /*
     * 停车时先关电机驱动，不马上关负压。
     * 负压最后关闭可以减小车辆突然失去吸附后的点头风险。
     */
    /* 任何停车原因都同步结束惯导回放,可重入。 */
    nav_stop();
    control_disable_drive_only();
    is_car_running = 0u;
    prestart_elapsed_ms = 0u;
    prestart_stage = PRESTART_STOPPING;
    pit_enable(CCU60_CH1);
}

/* 查询是否仍在第一次按键后的预热阶段。
 * 菜单按键逻辑用它避免用户在预热未完成时误触发发车。
 */
uint8 prestart_is_warming_up(void)
{
    return (uint8)(PRESTART_WARMUP == prestart_stage);
}

/* 查询是否可以接受第二次确认发车。
 * 只有这个状态下 prestart_confirm_launch() 会返回成功。
 */
uint8 prestart_is_waiting_confirm(void)
{
    return (uint8)(PRESTART_WAIT_CONFIRM == prestart_stage);
}

/* 查询车是否已经真正进入运行期。
 * 运行期保护只在这个状态后生效，避免预热和负压稳定阶段误停车。
 */
uint8 prestart_is_running(void)
{
    /*
     * 这个状态用于区分“流程已启动”和“车真正运行”。
     * 出界停车、姿态保护等运行期保护只应该在 RUNNING 后生效，
     * 不能在 WARMUP / WAIT_CONFIRM 阶段误关负压。
     */
    return (uint8)(PRESTART_RUNNING == prestart_stage);
}

/* IMU 外部中断后的发车状态钩子。
 * 发车不再依赖风机工况校准（负压稳定即发车，见 prestart_tick_5ms），
 * 本函数保留为空实现以兼容中断侧调用点。
 */
void prestart_gyro_update(void)
{
}

/* 只有 IDLE 表示负压、发车计时和正常速度环都未占用车辆。 */
uint8 prestart_is_idle(void)
{
    return (uint8)(PRESTART_IDLE == prestart_stage);
}

/* 5ms 发车状态机。
 * 由 CCU60_CH1 中断调用，负责预热计时、负压稳定等待、发车切换和负压延时关闭。
 */
void prestart_tick_5ms(void)
{
    if (PRESTART_IDLE == prestart_stage)
    {
        return;
    }

    if (PRESTART_STOPPING == prestart_stage)
    {
        prestart_elapsed_ms += 5u;
        if (prestart_elapsed_ms >= 500u)
        {
            control_disable();
            pit_disable(CCU60_CH1);
            prestart_reset_state();
        }
        return;
    }

    if (PRESTART_WAIT_CONFIRM == prestart_stage)
    {
        /* 这里必须停住计时并保持负压，直到用户第二次按键确认发车。 */
        return;
    }

    prestart_elapsed_ms += 5u;

    if (PRESTART_WARMUP == prestart_stage)
    {
        if (prestart_elapsed_ms >= 750u)
        {
            prestart_stage = PRESTART_WAIT_CONFIRM;
        }
        return;
    }

    if (PRESTART_SUCTION_SETTLING == prestart_stage)
    {
        /* 负压开满 500ms 直接发车：归零航向 + 重锚姿态零点 + 开速度环。
         * 陀螺零飘校准在开机时就做完了(gyro_init→gyro_zero_calibration_start,
         * 上电约1秒非阻塞完成,风机未开均值干净)——发车流程走到这里离上电
         * 早已远超1秒,校准必然已完成,故不再等它(用户定,2026-07-13)。
         * 编码器航向每帧独立累加、不受校准状态影响;launch_rezero 只锁存
         * 当前姿态+归零航向,也不依赖校准。 */
        if (prestart_elapsed_ms >= 500u)
        {
            gyro_request_launch_rezero();
            prestart_elapsed_ms = 0u;
            prestart_stage = PRESTART_RUNNING;
            control_enable();
            /* 发车航向已请求归零,紧随其后开始惯导回放,
             * 发车朝向即 nav_path.h 数据的 0 度基准。 */
            nav_start();
        }
        return;
    }

    if ((PRESTART_RUNNING == prestart_stage) && (11000u == prestart_elapsed_ms))
    {
        /* 发车流程超时兜底：先关驱动，负压继续保持到 FINISH。 */
        control_disable_drive_only();
    }

    if ((PRESTART_RUNNING == prestart_stage) && (prestart_elapsed_ms >= 12000u))
    {
        g_stop_reason = STOP_REASON_RUN_TIMEOUT;
        prestart_stop();
    }
}
