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
#include "mixer.h"
#include "motor.h"

static angle_pd_t s_pd;
static straight_t s_str;
static uint16_t   s_base = BCAR_CTRL_BASE_MAG;
static uint32_t   s_lost = 0u;        /* 已经连续多少拍看不到线 */
static uint8_t    s_running = 0u;
static ctrl_mode_t s_mode = CTRL_MODE_LINE;
static uint32_t   s_kick_ticks = 0u;  /* >0 = 起步助推中（还剩几拍）*/

/* 读秒那 3 秒的节拍是 5ms，直行环的 D 项要用秒 */
#define CTRL_DT_S  ((float)BCAR_TICK_MS / 1000.0f)
/* 起步助推时长（拍） */
#define CTRL_KICK_TICKS  ((uint32_t)(BCAR_CTRL_KICK_MS / BCAR_TICK_MS))

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
static float base_effective(void)
{
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
    s_base    = BCAR_CTRL_BASE_MAG;
    s_lost    = 0u;
    s_running = 0u;
    s_mode    = CTRL_MODE_LINE;
    s_kick_ticks = 0u;
}

void control_enter_run(float yaw_deg)
{
    angle_pd_init(&s_pd, s_pd.kp, s_pd.kd, s_pd.steer_max);   /* 清微分历史 */
    /* ★进 RUN 先进"直行"：发车点在线上、但万一没压准 / 线上有反光，
     * 车也能朝 S 直着出去，看到线立刻交回循迹环（见 control_step）。 */
    straight_set_target(&s_str, yaw_deg);                     /* 目标航向 = 发车方向 */
    s_mode    = CTRL_MODE_STRAIGHT;
    s_lost    = 0u;
    s_running = 1u;
    s_kick_ticks = CTRL_KICK_TICKS;                           /* 起步那一脚 */
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

void control_set_pd(float kp, float kd)
{
    angle_pd_set(&s_pd, kp, kd);
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

static void drive(float base, float steer)
{
    mix_out_t m = mixer_step(base, steer, BCAR_MOTION_MIN_MAG);

    motor_set(MOTOR_L, (m.l.dir == MIX_DIR_REV) ? MOTOR_REV : MOTOR_FWD, m.l.mag);
    motor_set(MOTOR_R, (m.r.dir == MIX_DIR_REV) ? MOTOR_REV : MOTOR_FWD, m.r.mag);
}

void control_step(float err_mm, uint8_t visible, float yaw_deg)
{
    if (!s_running) {
        return;
    }

    if (visible) {
        /* ---- 看得到线：循迹环接管 ---- */
        if (s_mode != CTRL_MODE_LINE) {
            /* 刚从直行切过来：清一下 PD 历史，免得拿"很久以前的偏差"算出一个巨大微分 */
            angle_pd_reset(&s_pd);
            s_mode = CTRL_MODE_LINE;
        }
        s_lost = 0u;
        drive(base_effective(), angle_pd_step(&s_pd, err_mm));
        return;
    }

    /* ---- 看不到线：陀螺直行找线 ---- */
    s_lost++;

    if (s_mode != CTRL_MODE_STRAIGHT) {
        /* 切的这一刻把"当前航向"定成目标 —— 也就是"照丢线那一瞬间的朝向直着走" */
        straight_set_target(&s_str, yaw_deg);
        s_mode = CTRL_MODE_STRAIGHT;
    }

    if (s_lost <= (uint32_t)(BCAR_STR_LOST_MS / BCAR_TICK_MS)) {
        /* 容忍期内：**不刹车**，保持航向直着走找线（速度打折，找线别冲）*/
        drive(base_effective() * BCAR_STR_LOST_BASE_P, straight_step(&s_str, yaw_deg, CTRL_DT_S));
    } else {
        /* 撑不住了：停车。宁可停在赛道上等人捡，也别瞎冲。
         * ★刹停之后把"起步那一脚"重新装上 —— 再起步时还得踹一脚才动得了。 */
        motor_brake_all();
        s_kick_ticks = CTRL_KICK_TICKS;
    }
}
