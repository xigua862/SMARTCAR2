/**
  ******************************************************************************
  * @file    angle_pd.c
  * @brief   转向 PD 实现（纯逻辑）
  ******************************************************************************
  */
#include "angle_pd.h"

void angle_pd_init(angle_pd_t *pd, float kp, float kd, float steer_max)
{
    if (pd == 0) {
        return;
    }
    pd->kp = kp;
    pd->kd = kd;
    pd->steer_max = (steer_max > 0.0f) ? steer_max : 1000.0f;
    pd->err_prev = 0.0f;
    /* ★inited=0，不是 1：第一拍只把当前偏差记为"上一次"，不算微分。
     * 否则第一拍会凭空产生 kd x (err-0) 的一次冲击（偏差 0 -> 实际值），
     * 现场把 kd 调大后，这一下会让车在进 RUN 的瞬间猛拐一下。
     * （这个 bug 是 Tools/test_control.c 的单测抓出来的） */
    pd->inited = 0u;
}

void angle_pd_set(angle_pd_t *pd, float kp, float kd)
{
    if (pd == 0) {
        return;
    }
    pd->kp = kp;
    pd->kd = kd;
}

void angle_pd_get(const angle_pd_t *pd, float *kp, float *kd, float *steer_max)
{
    if (pd == 0) {
        return;
    }
    if (kp != 0)        { *kp = pd->kp; }
    if (kd != 0)        { *kd = pd->kd; }
    if (steer_max != 0) { *steer_max = pd->steer_max; }
}

float angle_pd_step(angle_pd_t *pd, float err_mm)
{
    float steer;

    if (pd == 0) {
        return 0.0f;
    }
    if (!pd->inited) {
        pd->err_prev = err_mm;
        pd->inited = 1u;
    }

    steer = pd->kp * err_mm + pd->kd * (err_mm - pd->err_prev);

    /* 限幅：别让一次大偏差把转向量顶到天上 */
    if (steer > pd->steer_max) {
        steer = pd->steer_max;
    } else if (steer < -pd->steer_max) {
        steer = -pd->steer_max;
    }

    pd->err_prev = err_mm;
    return steer;
}

void angle_pd_reset(angle_pd_t *pd)
{
    if (pd == 0) {
        return;
    }
    pd->err_prev = 0.0f;
    pd->inited = 0u;
}
