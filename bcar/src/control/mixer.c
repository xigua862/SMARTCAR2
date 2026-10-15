/**
  ******************************************************************************
  * @file    mixer.c
  * @brief   差速混合实现（纯逻辑）
  ******************************************************************************
  */
#include "mixer.h"

#ifndef MIXER_ABS
#define MIXER_ABS(x)  (((x) < 0.0f) ? -(x) : (x))
#endif

#define MIXER_MAX  1000.0f

static mix_wheel_t to_wheel(float v, uint16_t min_mag)
{
    mix_wheel_t w;
    float a = MIXER_ABS(v);

    if (a < (float)min_mag) {
        /* 低于消隐下限：给 0（不是给一点点），否则会出现"指令有、轮不转"的假象 */
        w.mag = 0u;
        w.dir = MIX_DIR_FWD;
        return w;
    }
    if (a > MIXER_MAX) {
        a = MIXER_MAX;
    }
    w.mag = (uint16_t)(a + 0.5f);
    w.dir = (v < 0.0f) ? MIX_DIR_REV : MIX_DIR_FWD;
    return w;
}

mix_out_t mixer_step(float base, float steer, uint16_t min_mag)
{
    mix_out_t out;
    float ul = base + steer;
    float ur = base - steer;

    /* ① 超量程时按比例整体缩放，保住差速比例 */
    {
        float peak = MIXER_ABS(ul);
        if (MIXER_ABS(ur) > peak) {
            peak = MIXER_ABS(ur);
        }
        if (peak > MIXER_MAX) {
            ul = ul * (MIXER_MAX / peak);
            ur = ur * (MIXER_MAX / peak);
        }
    }

    out.l = to_wheel(ul, min_mag);
    out.r = to_wheel(ur, min_mag);
    return out;
}
