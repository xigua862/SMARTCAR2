/**
  ******************************************************************************
  * @file    control.c
  * @brief   循迹控制环总装实现（循迹 + 陀螺直行 两种模式）
  ******************************************************************************
  */
#include "control.h"
#include "bcar_config.h"
#include "angle_pd.h"
#include "straight.h"
#include "speed_pi.h"
#include "mixer.h"
#include "motor.h"

static angle_pd_t s_pd;
static straight_t s_str;
static uint16_t   s_base = BCAR_CTRL_BASE_MAG;
static uint32_t   s_lost = 0u;        /* 已经连续多少拍看不到线 */
static uint8_t    s_running = 0u;
static ctrl_mode_t s_mode = CTRL_MODE_LINE;
static uint32_t   s_kick_ticks = 0u;  /* >0 = 起步斜坡中（还剩几拍）*/
static int8_t     s_line_sign = BCAR_CTRL_LINE_SIGN;   /* 线环方向：err 反了就翻 */
/* 速度环（见 speed_pi.h）：把"摩擦"当扰动自动补偿掉 */
static speed_pi_t s_vel;
static float      s_v_target = BCAR_V_TARGET_MS;
static uint8_t    s_vel_on   = (uint8_t)BCAR_VEL_ON;
static uint8_t    s_str_on   = (uint8_t)BCAR_STR_ON;   /* 陀螺直行开关（用户要求先关掉）*/
static float      s_steer_prev = 0.0f;                 /* 上一次给出去的转向量（做变化率限幅用）*/
static uint32_t   s_vramp    = 0u;    /* 起步时目标速度的斜坡计数 */
#define CTRL_VRAMP_TICKS  ((uint32_t)(BCAR_V_RAMP_MS / BCAR_TICK_MS))
static uint8_t    s_vis_run = 0u;     /* 连续多少拍"看得见线"（去抖用）*/
static uint8_t    s_invis_run = 0u;   /* 连续多少拍"看不见线"*/
/* 模式切换去抖：visible 在阈值附近抖动时，不许每拍都切模式 —— 那会让转向量在两个环之间跳，
 * 现场表现就是"瞎甩头"。 */

/* 读秒那 3 秒的节拍是 5ms，直行环的 D 项要用秒 */
#define CTRL_DT_S  ((float)BCAR_TICK_MS / 1000.0f)
/* 起步斜坡时长（拍） */
#define CTRL_KICK_TICKS  ((uint32_t)(BCAR_CTRL_KICK_MS / BCAR_TICK_MS))
/* 模式切换去抖：连续这么多拍才切（15ms / 40ms），防止 visible 抖动导致来回切 */
#define CTRL_VIS_TICKS    3u
#define CTRL_INVIS_TICKS  8u

/**
  * @brief 这一拍该用多大速度基准：起步那一下是"斜坡软起"，之后回 BASE
  *
  * 为什么是斜坡而不是"踹一脚"（原来 60ms 满力阶跃）：
  *   这台车**后驱 + 没风机（负压坏了）**，阶跃那一脚会把车头抬起来（用户报"翘头严重"），
  *   车头一抬灰度就看不见线。斜坡让扭矩慢慢加到"刚好破静摩擦"，加速摊在几百毫秒里，
  *   抬头的冲量小得多。（实测：停→动要 60~80%，但转起来后 30% 就能维持 ⇒ 斜坡顶端
  *   只需要短暂到 800，之后立刻回 BASE 巡航。）
  *
  * ★只在这里递减计数 —— 刹车那一支不会调它，所以"刹住不动"时不会白消耗，
  *   等真要走的时候才起坡。
  */
static float base_effective(float v_mm_s)
{
    /* ---- 速度环优先：自动补偿摩擦，目标速度带 600ms 斜坡（软起）---- */
    if (s_vel_on != 0u) {
        float vt = s_v_target;
        if (s_vramp < CTRL_VRAMP_TICKS) {
            s_vramp++;
            vt = s_v_target * (float)s_vramp / (float)CTRL_VRAMP_TICKS;
        }
        return speed_pi_step(&s_vel, vt, v_mm_s, CTRL_DT_S);
    }

    /* ---- 速度环关掉时：退回"起步斜坡 + BASE"的老路子 ---- */
    if (s_kick_ticks > 0u) {
        uint32_t done = CTRL_KICK_TICKS - s_kick_ticks;          /* 已经过了几拍 */
        float    f    = (float)done / (float)CTRL_KICK_TICKS;    /* 0 → 1 */
        float    base = (float)s_base
                      + ((float)BCAR_CTRL_KICK_MAG - (float)s_base) * f;

        s_kick_ticks--;
        if (s_kick_ticks == 0u) {
            return (float)s_base;                                /* 斜坡走完 = 回巡航 */
        }
        return base;
    }
    return (float)s_base;
}

void control_init(void)
{
    angle_pd_init(&s_pd, BCAR_CTRL_KP, BCAR_CTRL_KD, BCAR_CTRL_STEER_MAX);
    straight_init(&s_str, BCAR_STR_KP, BCAR_STR_KD, BCAR_STR_STEER_MAX, BCAR_STR_SIGN);
    speed_pi_init(&s_vel, BCAR_VEL_KP, BCAR_VEL_KI,
                  (float)BCAR_VEL_MIN_MAG, (float)BCAR_VEL_MAX_MAG, (float)BCAR_CTRL_BASE_MAG);
    s_base    = BCAR_CTRL_BASE_MAG;
    s_lost    = 0u;
    s_running = 0u;
    s_mode    = CTRL_MODE_LINE;
    s_kick_ticks = 0u;
    s_vis_run = 0u;
    s_invis_run = 0u;
}

void control_enter_run(float yaw_deg)
{
    angle_pd_init(&s_pd, s_pd.kp, s_pd.kd, s_pd.steer_max);   /* 清微分历史 */
    /* ★进 RUN 先进"直行"：发车点在线上、但万一没压准 / 线上有反光，
     * 车也能朝 S 直着出去，看到线立刻交回循迹环（见 control_step）。 */
    if (s_str_on != 0u) {
        straight_set_target(&s_str, yaw_deg);                 /* 目标航向 = 发车方向 */
        s_mode = CTRL_MODE_STRAIGHT;
    } else {
        s_mode = CTRL_MODE_LINE;      /* ★纯循迹模式：发车就直接交给灰度环，不做航向保持 */
    }
    s_lost    = 0u;
    s_running = 1u;
    s_kick_ticks = CTRL_KICK_TICKS;                           /* 起步那一脚（速度环关时用）*/
    speed_pi_reset(&s_vel, (float)BCAR_VEL_MIN_MAG);           /* 速度环：积分清零、从下限起步 */
    s_vramp = 0u;                                             /* 目标速度斜坡从头来（软起）*/
}

void control_exit_run(void)
{
    s_running = 0u;
    s_lost    = 0u;
    s_mode    = CTRL_MODE_LINE;
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

void control_set_str_on(uint8_t on)
{
    s_str_on = (on != 0u) ? 1u : 0u;
    if (s_str_on == 0u) {
        s_mode = CTRL_MODE_LINE;
    }
}

uint8_t control_str_on(void)
{
    return s_str_on;
}

void control_set_speed(float v_mm_s)
{
    if (v_mm_s <= 0.0f) {
        s_vel_on = 0u;                      /* SPD 0 = 关掉速度环 */
        return;
    }
    s_v_target = v_mm_s;
    s_vel_on   = 1u;
}

float control_v_target(void)
{
    return s_v_target;
}

float control_vel_out(void)
{
    return (s_vel_on != 0u) ? s_vel.out : 0.0f;
}

uint8_t control_vel_on(void)
{
    return s_vel_on;
}

void control_set_pd(float kp, float kd)
{
    angle_pd_set(&s_pd, kp, kd);
}

void control_set_line_sign(int8_t sign)
{
    s_line_sign = (sign < 0) ? (int8_t)-1 : (int8_t)1;
}

int8_t control_line_sign(void)
{
    return s_line_sign;
}

void control_get_pd(float *kp, float *kd)
{
    angle_pd_get(&s_pd, kp, kd, 0);
}

void control_set_straight(float kp, float kd)
{
    straight_set_gains(&s_str, kp, kd);
}

void control_get_straight(float *kp, float *kd, int8_t *sign, float *target)
{
    straight_get(&s_str, kp, kd, sign, target);
}

void control_set_straight_sign(int8_t sign)
{
    straight_set_sign(&s_str, sign);
}

ctrl_mode_t control_mode(void)
{
    return s_mode;
}

const char *control_mode_name(void)
{
    return (s_mode == CTRL_MODE_STRAIGHT) ? "STRAIGHT" : "LINE";
}

uint32_t control_lost_ticks(void)
{
    return s_lost;
}

uint32_t control_lost_ms(void)
{
    return s_lost * (uint32_t)BCAR_TICK_MS;
}

uint8_t control_lost_holding(void)
{
    /* "还在撑"= 丢线了但没超时，车还在用陀螺直行找线 */
    return (uint8_t)((s_running && s_lost > 0u
                      && s_lost <= (uint32_t)(BCAR_STR_LOST_MS / BCAR_TICK_MS)) ? 1u : 0u);
}

/**
  * @brief 转向量变化率限幅：这一拍最多让转向量变 BCAR_CTRL_SLEW
  * @note  治"猛甩头"：偏差一抖就打到满舵会让车甩出去、车头一偏就再也看不见线。
  *        限了变化率之后：拐弯照样拐得动（0→满舵只要 75ms），但不会瞬间甩到底。
  */
static float steer_slew(float steer)
{
    float d = steer - s_steer_prev;

    if (d > BCAR_CTRL_SLEW) {
        steer = s_steer_prev + BCAR_CTRL_SLEW;
    } else if (d < -BCAR_CTRL_SLEW) {
        steer = s_steer_prev - BCAR_CTRL_SLEW;
    }
    s_steer_prev = steer;
    return steer;
}

static void drive(float base, float steer)
{
    steer = steer_slew(steer);

    mix_out_t m = mixer_step(base, steer, BCAR_MOTION_MIN_MAG);

    motor_set(MOTOR_L, (m.l.dir == MIX_DIR_REV) ? MOTOR_REV : MOTOR_FWD, m.l.mag);
    motor_set(MOTOR_R, (m.r.dir == MIX_DIR_REV) ? MOTOR_REV : MOTOR_FWD, m.r.mag);
}

void control_step(float err_mm, uint8_t visible, float yaw_deg, float v_mm_s)
{
    if (!s_running) {
        return;
    }

    if (visible) {
        s_invis_run = 0u;
        if (s_vis_run < 255u) { s_vis_run++; }
        /* ---- 看得到线：循迹环接管 ---- */
        if (s_mode != CTRL_MODE_LINE && s_vis_run >= CTRL_VIS_TICKS) {
            /* 刚从直行切过来：清一下 PD 历史，免得拿"很久以前的偏差"算出一个巨大微分 */
            angle_pd_reset(&s_pd);
            s_mode = CTRL_MODE_LINE;
        }
        s_lost = 0u;
        /* err 乘上方向开关：翻它就能把"朝远离线拐"改成"朝线拐" */
        drive(base_effective(v_mm_s), angle_pd_step(&s_pd, err_mm * (float)s_line_sign));
        return;
    }

    /* ---- 看不到线：陀螺直行找线 ---- */
    s_lost++;
    s_vis_run = 0u;
    if (s_invis_run < 255u) { s_invis_run++; }

    if (s_str_on != 0u) {
        /* 陀螺直行：保持"丢线那一刻"的航向直着找线 */
        if (s_mode != CTRL_MODE_STRAIGHT && s_invis_run >= CTRL_INVIS_TICKS) {
            straight_set_target(&s_str, yaw_deg);
            s_mode = CTRL_MODE_STRAIGHT;
        }
    } else {
        s_mode = CTRL_MODE_LINE;      /* 纯循迹模式：不做航向保持 */
    }

    if (s_lost <= (uint32_t)(BCAR_STR_LOST_MS / BCAR_TICK_MS)) {
        /* 容忍期内：不刹车，直着走一段找线。
         * ★陀螺开着时用航向保持纠偏；关掉时转向量给 0（就是"两轮同速直着走"）。 */
        float steer = (s_str_on != 0u)
                      ? straight_step(&s_str, yaw_deg, CTRL_DT_S)
                      : 0.0f;
        drive(base_effective(v_mm_s) * BCAR_STR_LOST_BASE_P, steer);
    } else {
        /* 撑不住了：停车。宁可停在赛道上等人捡，也别瞎冲。
         * ★刹停之后把"起步那一脚"重新装上 —— 再起步时还得踹一脚才动得了。 */
        motor_brake_all();
        s_steer_prev = 0.0f;      /* 停在原地：转向量归零，下次起步从 0 限幅 */
        s_kick_ticks = CTRL_KICK_TICKS;
    }
}
