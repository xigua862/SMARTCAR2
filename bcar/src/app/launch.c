/**
  ******************************************************************************
  * @file    launch.c
  * @brief   发车读秒时序器实现（纯逻辑）
  ******************************************************************************
  */
#include "launch.h"

/* 三声的时间点（ms，从发车那一刻算起）：0 = 短、1000 = 短、2000 = 长；
 * 3000（= LAUNCH_TOTAL_MS）到点起跑。
 * 单独列成表是为了"读秒快慢"只改这一处，不用碰逻辑。 */
static const uint32_t LAUNCH_BEEP_AT_MS[LAUNCH_BEEP_N] = {0u, 1000u, 2000u};

void launch_begin(launch_t *l, uint32_t now_ms)
{
    if (l == 0) {
        return;
    }
    l->active = 1u;
    l->step   = 0u;
    l->t0_ms  = now_ms;
}

void launch_abort(launch_t *l)
{
    if (l == 0) {
        return;
    }
    l->active = 0u;
    l->step   = 0u;
}

uint8_t launch_active(const launch_t *l)
{
    return (l != 0 && l->active != 0u) ? 1u : 0u;
}

uint32_t launch_elapsed_ms(const launch_t *l, uint32_t now_ms)
{
    if (l == 0 || l->active == 0u) {
        return 0u;
    }
    /* 无符号相减：时间戳回绕（49.7 天）也照样算得对 */
    return (uint32_t)(now_ms - l->t0_ms);
}

launch_ev_t launch_step(launch_t *l, uint32_t now_ms)
{
    uint32_t el;

    if (l == 0 || l->active == 0u) {
        return LAUNCH_EV_NONE;
    }

    el = (uint32_t)(now_ms - l->t0_ms);

    /* ★先判"读秒完事"：主循环万一被什么卡住漏了几拍，也绝不会漏掉起跑
     * （宁可少响一声，也不能到点了还杵在原地不动）。 */
    if (el >= LAUNCH_TOTAL_MS) {
        l->active = 0u;
        l->step   = 0u;
        return LAUNCH_EV_DONE;
    }

    if (l->step < LAUNCH_BEEP_N && el >= LAUNCH_BEEP_AT_MS[l->step]) {
        l->step++;
        /* 最后一声是长鸣，前两声短鸣 */
        return (l->step >= LAUNCH_BEEP_N) ? LAUNCH_EV_BEEP_LONG : LAUNCH_EV_BEEP_SHORT;
    }

    return LAUNCH_EV_NONE;
}
