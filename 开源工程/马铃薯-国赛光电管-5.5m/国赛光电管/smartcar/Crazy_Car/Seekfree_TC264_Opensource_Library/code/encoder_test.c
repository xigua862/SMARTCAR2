/**
 * @file encoder_test.c
 * @brief 编码器和地图控制链的台架诊断模式。
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

// encoder_test.c - 按地图运行的编码器/速度环诊断模式

#include "encoder_test.h"
#include "globals.h"
#include "key.h"
#include "prestart.h"
#include "telemetry.h"

#if ENCODER_TEST_MODE_ENABLE

uint8 encoder_test_handle_key(uint8 key)
{
    if ((KEY_CONFIRM != key) && (KEY_CONFIRM_LONG != key))
    {
        return 0u;
    }

    if (0u == is_car_running)
    {
        /* Keep the existing frozen-run -> export one-key workflow, but do
         * not print anything on the debug UART. */
        if (KEY_CONFIRM_LONG == key)
        {
            return 1u;
        }
        if (TELEMETRY_STATE_FROZEN == telemetry_get_state())
        {
            (void)telemetry_request_export();
            return 1u;
        }
        if (TELEMETRY_STATE_EXPORTING == telemetry_get_state())
        {
            return 1u;
        }
        if (!prestart_is_idle())
        {
            return 1u;
        }
        if (prestart_begin())
        {
            is_car_running = 1u;
        }
        return 1u;
    }

    g_stop_reason = STOP_REASON_MANUAL;
    prestart_stop();
    return 1u;
}

void encoder_test_tick_10ms(void)
{
    /* Intentionally empty.  The normal 1 ms control/navigation producers
     * already store encoder deltas, L/R targets, L/R actual speed, PWM,
     * progress and encoder-fitted raw yaw in EMEM.  No debug-UART output or
     * second encoder reader is allowed in this mode. */
}

#else

uint8 encoder_test_handle_key(uint8 key)
{
    (void)key;
    return 0u;
}

void encoder_test_tick_10ms(void)
{
    // 默认关闭时保持空实现，比赛固件不增加运行开销。
}

#endif
