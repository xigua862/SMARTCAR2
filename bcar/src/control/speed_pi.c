/**
  ******************************************************************************
  * @file    speed_pi.c
  * @brief   速度环 PI 实现（纯逻辑）
  ******************************************************************************
  */
#include "speed_pi.h"

void speed_pi_init(speed_pi_t *s, float kp, float ki, float out_min, float out_max, float out_init)
{
    if (s == 0) {
        return;
    }
    s->kp      = kp;
    s->ki      = ki;
    s->out_min = out_min;
    s->out_max = (out_max > out_min) ? out_max : (out_min + 1.0f);
    s->integ   = 0.0f;
    s->out     = out_init;
    if (s->out < s->out_min) { s->out = s->out_min; }
    if (s->out > s->out_max) { s->out = s->out_max; }
}

void speed_pi_set(speed_pi_t *s, float kp, float ki)
{
    if (s == 0) {
        return;
    }
    s->kp = kp;
    s->ki = ki;
}

void speed_pi_reset(speed_pi_t *s, float out_init)
{
    if (s == 0) {
        return;
    }
    s->integ = 0.0f;
    s->out   = out_init;
    if (s->out < s->out_min) { s->out = s->out_min; }
    if (s->out > s->out_max) { s->out = s->out_max; }
}

float speed_pi_step(speed_pi_t *s, float v_target, float v_meas, float dt_s)
{
    float err;
    float p;
    float raw;

    if (s == 0) {
        return 0.0f;
    }
    if (dt_s <= 0.0f) {
        dt_s = 0.0f;
    }

    err = v_target - v_meas;
    p   = s->kp * err;
    raw = p + s->ki * s->integ;

    /* ---- 条件积分（抗饱和）：只有"还没顶到限"或者"误差能把输出拉回来"时才积分 ---- */
    if ((raw < s->out_max && raw > s->out_min)
        || (raw >= s->out_max && err < 0.0f)
        || (raw <= s->out_min && err > 0.0f)) {
        s->integ += err * dt_s;
    }

    raw = p + s->ki * s->integ;

    if (raw > s->out_max) { raw = s->out_max; }
    if (raw < s->out_min) { raw = s->out_min; }

    s->out = raw;
    return s->out;
}

void speed_pi_get(const speed_pi_t *s, float *kp, float *ki, float *out)
{
    if (s == 0) {
        return;
    }
    if (kp != 0)  { *kp = s->kp; }
    if (ki != 0)  { *ki = s->ki; }
    if (out != 0) { *out = s->out; }
}
