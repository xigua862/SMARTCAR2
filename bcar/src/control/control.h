/**
  ******************************************************************************
  * @file    control.h
  * @brief   循迹控制环总装（**循迹 PD** 与 **陀螺直行** 两种模式 + 差速混合 + 安全保护）
  *
  * 两种模式怎么切（都在 RUN 里）：
  *   - 看得到线 → **LINE**：灰度偏差走转向 PD（原来那套）。
  *   - 看不到线 → **STRAIGHT**：用陀螺保持"丢线那一刻的航向"直着走找线，
  *     速度打折；找 `BCAR_STR_LOST_MS` 还没找到才刹车（安全兜底）。
  *   - 进 RUN 的第一个模式也是 **STRAIGHT**：发车点朝 S 直着出去，
  *     看到线立刻交给循迹环 —— 这样"看得到线"才能"会走"，起步不会因为没压准线就发呆。
  ******************************************************************************
  */
#ifndef __CTRL_CONTROL_H
#define __CTRL_CONTROL_H

#include <stdint.h>

typedef enum {
    CTRL_MODE_LINE = 0,   /* 灰度循迹（转向 PD）*/
    CTRL_MODE_STRAIGHT    /* 陀螺直行（航向保持）*/
} ctrl_mode_t;

void control_init(void);

/**
  * @brief 进入 RUN
  * @param yaw_deg 当前航向 —— 会被当成"起步直行"的目标航向
  */
void control_enter_run(float yaw_deg);

/** @brief 退出 RUN：立刻刹车 */
void control_exit_run(void);

/**
  * @brief  5ms 拍调用（只在 RUN 状态由 main 调用）
  * @param  err_mm  灰度给的线偏差（正 = 线在右侧）
  * @param  visible 1 = 看到线
  * @param  yaw_deg 当前航向（逆时针为正）
  */
void control_step(float err_mm, uint8_t visible, float yaw_deg, float v_mm_s);

/** @brief 现场改速度基准（速度环关掉时才用得上）*/
void control_set_base(uint16_t mag);
uint16_t control_base(void);

/** @brief 速度环：设目标速度（mm/s）。传 0 = 关掉速度环、退回手动 BASE */
void control_set_speed(float v_mm_s);
float control_v_target(void);
/** @brief 陀螺直行开关：1 = 开（找不到线时用陀螺保持航向）；0 = 关（纯循迹）*/
void control_set_str_on(uint8_t on);
uint8_t control_str_on(void);

/** @brief 最后一次真正下发的转向量（诊断：看它有没有一直被顶在限幅上）*/
float control_steer_last(void);

/** @brief 速度环当前输出（诊断/串口看）*/
float control_vel_out(void);
/** @brief 速度环是否开着 */
uint8_t control_vel_on(void);

/** @brief 现场改循迹环增益 */
void control_set_pd(float kp, float kd);
void control_get_pd(float *kp, float *kd);

/** @brief 现场翻"循迹环方向"（±1）—— 车朝远离线的方向拐时用 */
void control_set_line_sign(int8_t sign);
int8_t control_line_sign(void);

/** @brief 现场改直行环增益（kp [mag/度]、kd [mag/(度/秒)]）*/
void control_set_straight(float kp, float kd);
void control_get_straight(float *kp, float *kd, int8_t *sign, float *target);

/** @brief 现场翻直行环方向（±1）—— 台上试出"车往反方向打"时用 */
void control_set_straight_sign(int8_t sign);

/** @brief 当前模式（诊断/串口看）*/
ctrl_mode_t control_mode(void);
const char *control_mode_name(void);

/** @brief 丢线持续了多少拍 / 多少毫秒（诊断）*/
uint32_t control_lost_ticks(void);
uint32_t control_lost_ms(void);

/** @brief 是否处于"丢线但还在撑（直行找线）"的状态 */
uint8_t control_lost_holding(void);

#endif /* __CTRL_CONTROL_H */
