/**
  ******************************************************************************
  * @file    straight.c
  * @brief   陀螺直行（航向保持）实现（纯逻辑）
  ******************************************************************************
  */
#include "straight.h"

void straight_init(straight_t *s, float kp, float kd, float steer_max, int8_t sign)
{
    if (s == 0) {
        return;
    }
    s->kp         = kp;
    s->kd         = kd;
    s->steer_max  = (steer_max > 0.0f) ? steer_max : 1000.0f;
    s->target_deg = 0.0f;
    s->err_prev   = 0.0f;
    s->sign       = (sign < 0) ? (int8_t)-1 : (int8_t)1;
    /* ★inited=0：第一拍只记误差、不算 D —— 和 angle_pd 同一个教训，
     * 否则进直行那一下 kd 会把"误差本身"当成阶跃，车会猛拐一下。 */
    s->inited     = 0u;
}

void straight_set_gains(straight_t *s, float kp, float kd)
{
    if (s == 0) {
        return;
    }
    s->kp = kp;
    s->kd = kd;
}

void straight_get(const straight_t *s, float *kp, float *kd, int8_t *sign, float *target)
{
    if (s == 0) {
        return;
    }
    if (kp != 0)     { *kp = s->kp; }
    if (kd != 0)     { *kd = s->kd; }
    if (sign != 0)   { *sign = s->sign; }
    if (target != 0) { *target = s->target_deg; }
}

void straight_set_sign(straight_t *s, int8_t sign)
{
    if (s == 0) {
        return;
    }
    s->sign = (sign < 0) ? (int8_t)-1 : (int8_t)1;
}

void straight_set_target(straight_t *s, float yaw_deg)
{
    if (s == 0) {
        return;
    }
    s->target_deg = yaw_deg;
    straight_reset(s);
}

float straight_target(const straight_t *s)
{
    return (s == 0) ? 0.0f : s->target_deg;
}

void straight_reset(straight_t *s)
{
    if (s == 0) {
        return;
    }
    s->err_prev = 0.0f;
    s->inited   = 0u;
}

float straight_step(straight_t *s, float yaw_deg, float dt_s)
{
    float err, d, steer;

    if (s == 0) {
        return 0.0f;
    }

    /* 目标 − 偏差方向：见头文件的符号口径。sign = -1 时整条反过来。 */
    err = (float)s->sign * (yaw_deg - s->target_deg);

    if (!s->inited) {
        s->err_prev = err;
        s->inited   = 1u;
    }
    d = (dt_s > 0.0f) ? ((err - s->err_prev) / dt_s) : 0.0f;   /* 度/秒 */

    steer = s->kp * err + s->kd * d;

    if (steer > s->steer_max) {
        steer = s->steer_max;
    } else if (steer < -s->steer_max) {
        steer = -s->steer_max;
    }

    s->err_prev = err;
    return steer;
}
