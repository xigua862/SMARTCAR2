/**
  ******************************************************************************
  * @file    lapstat.c
  * @brief   单圈统计累加器实现（纯逻辑）
  ******************************************************************************
  */
#include "lapstat.h"

#ifndef LAPSTAT_FABS
#define LAPSTAT_FABS(x)  (((x) < 0.0f) ? -(x) : (x))
#endif

void lapstat_reset(lapstat_t *s)
{
    if (s == 0) {
        return;
    }
    p2_init(&s->err_abs_p95, 0.95f);
    s->ticks       = 0u;
    s->t_ms        = 0u;
    s->s_mm        = 0.0f;
    s->v_max       = 0.0f;
    s->v_sum       = 0.0f;
    s->abs_err_sum = 0.0f;
    s->lost_ticks  = 0u;
}

void lapstat_push(lapstat_t *s, float err_mm, uint8_t visible,
                  float v_mm_s, float ds_mm, uint32_t dt_ms)
{
    if (s == 0) {
        return;
    }

    s->ticks++;
    s->t_ms += dt_ms;
    s->s_mm += ds_mm;
    s->v_sum += v_mm_s;

    if (v_mm_s > s->v_max) {
        s->v_max = v_mm_s;
    }

    if (visible) {
        /* 只看"看见线"时的偏差：丢线那几拍的 err 恒为 0，混进来会把统计压好看，
         * 而那恰恰是最危险的时候 —— 这种自欺欺人的指标要不得。 */
        float a = LAPSTAT_FABS(err_mm);
        s->abs_err_sum += a;
        p2_push(&s->err_abs_p95, a);
    } else {
        s->lost_ticks++;
    }
}

uint8_t lapstat_finish(const lapstat_t *s, lap_record_t *out)
{
    uint32_t visible_ticks;

    if (s == 0 || out == 0) {
        return 0u;
    }
    if (s->ticks < LAPSTAT_MIN_TICKS) {
        return 0u;      /* 太短，不算一圈 */
    }

    visible_ticks = s->ticks - (uint32_t)s->lost_ticks;

    out->ticks          = s->ticks;
    out->t_ms           = s->t_ms;
    out->s_mm           = s->s_mm;
    out->v_max_mm_s     = s->v_max;
    out->v_avg_mm_s     = (s->t_ms > 0u)
                          ? (s->s_mm / ((float)s->t_ms / 1000.0f))
                          : 0.0f;
    out->abs_err_mean_mm = (visible_ticks > 0u)
                          ? (s->abs_err_sum / (float)visible_ticks)
                          : 0.0f;
    out->abs_err_p95_mm = p2_value(&s->err_abs_p95);
    out->lost_ticks     = s->lost_ticks;
    return 1u;
}
