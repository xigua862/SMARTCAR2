/**
  ******************************************************************************
  * @file    control.c
  * @brief   循迹控制环总装实现
  ******************************************************************************
  */
#include "control.h"
#include "bcar_config.h"
#include "angle_pd.h"
#include "mixer.h"
#include "motor.h"

static angle_pd_t s_pd;
static uint16_t   s_base = BCAR_CTRL_BASE_MAG;
static uint32_t   s_lost = 0u;
static uint8_t    s_running = 0u;

void control_init(void)
{
    angle_pd_init(&s_pd, BCAR_CTRL_KP, BCAR_CTRL_KD, BCAR_CTRL_STEER_MAX);
    s_base    = BCAR_CTRL_BASE_MAG;
    s_lost    = 0u;
    s_running = 0u;
}

void control_enter_run(void)
{
    angle_pd_init(&s_pd, s_pd.kp, s_pd.kd, s_pd.steer_max);   /* 清微分历史 */
    s_lost    = 0u;
    s_running = 1u;
}

void control_exit_run(void)
{
    s_running = 0u;
    s_lost    = 0u;
    motor_brake_all();
}

void control_set_base(uint16_t mag)
{
    s_base = (mag > 1000u) ? 1000u : mag;
}

uint16_t control_base(void)
{
    return s_base;
}

void control_set_pd(float kp, float kd)
{
    angle_pd_set(&s_pd, kp, kd);
}

void control_get_pd(float *kp, float *kd)
{
    angle_pd_get(&s_pd, kp, kd, 0);
}

uint32_t control_lost_ticks(void)
{
    return s_lost;
}

uint8_t control_lost_holding(void)
{
    return (uint8_t)((s_running && s_lost > 0u) ? 1u : 0u);
}

static void drive(float base, float steer)
{
    mix_out_t m = mixer_step(base, steer, BCAR_MOTION_MIN_MAG);

    motor_set(MOTOR_L, (m.l.dir == MIX_DIR_REV) ? MOTOR_REV : MOTOR_FWD, m.l.mag);
    motor_set(MOTOR_R, (m.r.dir == MIX_DIR_REV) ? MOTOR_REV : MOTOR_FWD, m.r.mag);
}

void control_step(float err_mm, uint8_t visible)
{
    if (!s_running) {
        return;
    }

    if (visible) {
        float steer;

        /* 丢线回来：先复位微分历史，避免拿"很久以前的偏差"算出一个巨大微分 */
        if (s_lost > 0u) {
            angle_pd_reset(&s_pd);
            s_lost = 0u;
        }
        steer = angle_pd_step(&s_pd, err_mm);
        drive((float)s_base, steer);
        return;
    }

    /* ---- 丢线 ---- */
    s_lost++;

    if (s_lost < (uint32_t)BCAR_CTRL_LOST_TICKS) {
        /* 容忍期内：保持"直着走"，别拿最后的转向量硬拐（那会直接冲出去）*/
        drive((float)s_base * 0.6f, 0.0f);
    } else {
        /* 撑不住了：停车。宁可停在赛道上等人捡，也别瞎冲。 */
        motor_brake_all();
    }
}
