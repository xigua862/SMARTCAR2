/**
  ******************************************************************************
  * @file    zebra.c
  * @brief   斑马线（S/G 地标）检测实现（纯逻辑）
  ******************************************************************************
  */
#include "zebra.h"

void zebra_init(zebra_t *z)
{
    if (z == 0) {
        return;
    }
    z->inside    = 0u;
    z->on_ticks  = 0u;
    z->off_ticks = 0u;
}

/* 统计"强白路"，返回路数和强白区的 min/max 位置 */
static uint8_t strong_stats(const line_pos_t *lp, float *lo, float *hi)
{
    const line_pos_cfg_t *cfg = line_pos_cfg();
    uint8_t i;
    uint8_t n = 0u;
    float   mn = 0.0f, mx = 0.0f;

    if (cfg == 0) {
        return 0u;
    }
    for (i = 0u; i < LINE_CH; i++) {
        if (lp->p[i] >= ZEBRA_P_STRONG) {
            if (n == 0u || cfg->pos_mm[i] < mn) { mn = cfg->pos_mm[i]; }
            if (n == 0u || cfg->pos_mm[i] > mx) { mx = cfg->pos_mm[i]; }
            n++;
        }
    }
    if (lo != 0) { *lo = mn; }
    if (hi != 0) { *hi = mx; }
    return n;
}

uint8_t zebra_step(zebra_t *z, const line_pos_t *lp)
{
    uint8_t n;
    float   lo = 0.0f, hi = 0.0f;
    uint8_t wide = 0u;

    if (z == 0 || lp == 0) {
        return 0u;
    }

    n = strong_stats(lp, &lo, &hi);

    /* 强白够多、而且**铺得比一条线宽得多**、总白量也够 ⇒ 判定"压在斑马线上" */
    if (n >= ZEBRA_MIN_STRONG_CH
        && (hi - lo) >= ZEBRA_MIN_SPAN_MM
        && lp->sum_p >= ZEBRA_MIN_SUM_P) {
        wide = 1u;
    }

    if (wide) {
        z->off_ticks = 0u;
        if (z->on_ticks < 0xFFFFu) {
            z->on_ticks++;
        }
        if (z->inside == 0u && z->on_ticks >= (uint16_t)ZEBRA_ON_TICKS) {
            z->inside = 1u;      /* 报一次事件；离开后才允许再报 */
            return 1u;
        }
        return 0u;
    }

    /* 不宽：可能只是普通白线，或者已经开过去了 */
    z->on_ticks = 0u;
    if (z->inside != 0u) {
        if (z->off_ticks < 0xFFFFu) {
            z->off_ticks++;
        }
        if (z->off_ticks >= (uint16_t)ZEBRA_OFF_TICKS) {
            z->inside    = 0u;
            z->off_ticks = 0u;
        }
    }
    return 0u;
}

uint8_t zebra_inside(const zebra_t *z)
{
    return (z == 0) ? 0u : z->inside;
}
