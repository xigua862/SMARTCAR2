/**
  ******************************************************************************
  * @file    odom.c
  * @brief   里程与位姿解算实现（纯逻辑）
  ******************************************************************************
  */
#include "odom.h"
#include <math.h>

#ifndef ODOM_PI
#define ODOM_PI 3.14159265358979f
#endif

static odom_cfg_t   s_cfg;
static odom_state_t s_st;

static float deg2rad(float d)
{
    return d * (ODOM_PI / 180.0f);
}

static float rad2deg(float r)
{
    return r * (180.0f / ODOM_PI);
}

void odom_init(const odom_cfg_t *cfg)
{
    if (cfg != 0) {
        s_cfg = *cfg;
    } else {
        s_cfg.mm_per_count = 0.0f;
        s_cfg.track_mm     = 110.0f;   /* 实测轮距，见框架 §1 */
        s_cfg.dt_s         = 0.005f;   /* 5ms 拍 */
    }
    odom_reset();
}

void odom_reset(void)
{
    s_st.x_mm    = 0.0f;
    s_st.y_mm    = 0.0f;
    s_st.yaw_deg = 0.0f;
    s_st.s_mm    = 0.0f;
    s_st.v_mm_s  = 0.0f;
}

void odom_step(int16_t dL_counts, int16_t dR_counts,
               uint8_t has_gyro, float dtheta_deg)
{
    float ds_l = (float)dL_counts * s_cfg.mm_per_count;   /* 左轮这段走过的弧长 */
    float ds_r = (float)dR_counts * s_cfg.mm_per_count;   /* 右轮 */
    float ds   = (ds_l + ds_r) * 0.5f;                    /* 车体中心走的弧长 */
    float dth;                                            /* 这段的转角（弧度）*/
    float yaw_mid;

    if (has_gyro) {
        dth = deg2rad(dtheta_deg);
    } else if (s_cfg.track_mm > 0.0f) {
        /* 没有陀螺时的退路：左右轮弧长差 / 轮距 = 转角（防除零）*/
        dth = (ds_r - ds_l) / s_cfg.track_mm;
    } else {
        dth = 0.0f;
    }

    /* 中点法：用"转了一半"的航向去推这段弧长。
     * 走圆弧时比"用起始航向推整段"的系统误差小得多。 */
    yaw_mid = deg2rad(s_st.yaw_deg) + dth * 0.5f;

    s_st.x_mm += ds * cosf(yaw_mid);
    s_st.y_mm += ds * sinf(yaw_mid);
    s_st.yaw_deg += has_gyro ? dtheta_deg : rad2deg(dth);
    s_st.s_mm += ds;
    s_st.v_mm_s = (s_cfg.dt_s > 0.0f) ? (ds / s_cfg.dt_s) : 0.0f;
}

odom_state_t odom_get(void)
{
    return s_st;
}
