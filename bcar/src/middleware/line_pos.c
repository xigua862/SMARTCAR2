/**
  ******************************************************************************
  * @file    line_pos.c
  * @brief   8 路灰度 → 线位置偏差 实现（纯逻辑）
  ******************************************************************************
  */
#include "line_pos.h"

static line_pos_cfg_t s_cfg;
static line_pos_t     s_out;

void line_pos_init(const line_pos_cfg_t *cfg)
{
    uint8_t i;

    if (cfg != 0) {
        s_cfg = *cfg;
    } else {
        /* 默认：没标定过的安全值 —— 位置按 6mm 间距排开、黑白基准给个典型值。
         * ⚠ 位置和基准都必须实测后再填，这里只是让程序不至于算炸。 */
        for (i = 0u; i < LINE_CH; i++) {
            s_cfg.pos_mm[i] = ((float)i - 3.5f) * 6.0f;   /* -21 ~ +21 mm */
            s_cfg.black[i]  = 200.0f;
            s_cfg.white[i]  = 3000.0f;
        }
        s_cfg.lost_sum_p = 0.4f;
    }
    for (i = 0u; i < LINE_CH; i++) {
        s_out.p[i] = 0.0f;
    }
    s_out.err_mm  = 0.0f;
    s_out.sum_p   = 0.0f;
    s_out.visible = 0u;
}

const line_pos_cfg_t *line_pos_cfg(void)
{
    return &s_cfg;
}


line_pos_cfg_t *line_pos_cfg_mut(void)
{
    return &s_cfg;
}

void line_pos_set_ref(uint8_t ch, float black, float white)
{
    if (ch < LINE_CH) {
        s_cfg.black[ch] = black;
        s_cfg.white[ch] = white;
    }
}

line_pos_t line_pos_step(const uint16_t *raw8)
{
    uint8_t i;
    float   sum_p = 0.0f;
    float   sum_wp = 0.0f;

    for (i = 0u; i < LINE_CH; i++) {
        float v = (raw8 != 0) ? (float)raw8[i] : 0.0f;
        float span = s_cfg.white[i] - s_cfg.black[i];
        float p;

        if (span > 0.0f) {
            p = (v - s_cfg.black[i]) / span;
        } else if (span < 0.0f) {
            /* 白比黑还暗（极性反了/接反）：照样算，只是取反 */
            p = (s_cfg.black[i] - v) / (-span);
        } else {
            p = 0.0f;   /* 黑白基准一样 = 这路没标定，别贡献 */
        }

        /* 钳到 [0,1]：超出范围的读数（比如被强光晃到）不该把质心拽飞 */
        if (p < 0.0f) {
            p = 0.0f;
        } else if (p > 1.0f) {
            p = 1.0f;
        }

        s_out.p[i] = p;
        sum_p  += p;
        sum_wp += p * s_cfg.pos_mm[i];
    }

    s_out.sum_p = sum_p;

    if (sum_p >= s_cfg.lost_sum_p && sum_p > 0.0f) {
        s_out.err_mm  = sum_wp / sum_p;
        s_out.visible = 1u;
    } else {
        /* 丢线：不许瞎给 0，把 visible 报出去让上层决定（减速/靠惯导撑过去）*/
        s_out.err_mm  = 0.0f;
        s_out.visible = 0u;
    }

    return s_out;
}
