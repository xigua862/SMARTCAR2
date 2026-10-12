/**
  ******************************************************************************
  * @file    heading.c
  * @brief   陀螺航向积分实现（纯逻辑）
  ******************************************************************************
  */
#include "heading.h"

static float    s_bias[3];
static float    s_dead_zone;
static float    s_yaw_deg;
static uint32_t s_samples;

void heading_init(float dead_zone_dps)
{
    s_bias[0] = s_bias[1] = s_bias[2] = 0.0f;
    s_dead_zone = (dead_zone_dps > 0.0f) ? dead_zone_dps : 0.0f;
    s_yaw_deg   = 0.0f;
    s_samples   = 0u;
}

void heading_reset(void)
{
    s_yaw_deg = 0.0f;
    s_samples = 0u;
}

void heading_set_bias(float gx, float gy, float gz)
{
    s_bias[0] = gx;
    s_bias[1] = gy;
    s_bias[2] = gz;
}

float heading_bias(uint8_t axis)
{
    return (axis < 3u) ? s_bias[axis] : 0.0f;
}

float heading_dead_zone(void)
{
    return s_dead_zone;
}

float heading_step(float gz_dps, float dt_s)
{
    float gz = gz_dps - s_bias[2];   /* 先减零偏 */
    float dth;

    /* 死区：小于这个就认为"没在转"，避免噪声被积分成漂移 */
    if (gz < 0.0f) {
        if (gz > -s_dead_zone) {
            gz = 0.0f;
        }
    } else {
        if (gz < s_dead_zone) {
            gz = 0.0f;
        }
    }

    dth = gz * dt_s;          /* 角速度 × 时间 = 角度 */
    s_yaw_deg += dth;
    s_samples++;
    return dth;
}

float heading_yaw_deg(void)
{
    return s_yaw_deg;
}

uint32_t heading_samples(void)
{
    return s_samples;
}

void heading_cal_begin(heading_cal_t *c)
{
    if (c == 0) {
        return;
    }
    c->gx_sum_f = 0.0f;
    c->gy_sum_f = 0.0f;
    c->gz_sum_f = 0.0f;
    c->n        = 0u;
}

void heading_cal_push(heading_cal_t *c, float gx, float gy, float gz)
{
    if (c == 0) {
        return;
    }
    c->gx_sum_f += gx;
    c->gy_sum_f += gy;
    c->gz_sum_f += gz;
    c->n++;
}

uint8_t heading_cal_finish(const heading_cal_t *c, float out[3], uint32_t min_samples)
{
    if (c == 0 || out == 0 || c->n == 0u || c->n < min_samples) {
        return 0u;
    }
    out[0] = c->gx_sum_f / (float)c->n;
    out[1] = c->gy_sum_f / (float)c->n;
    out[2] = c->gz_sum_f / (float)c->n;
    return 1u;
}
