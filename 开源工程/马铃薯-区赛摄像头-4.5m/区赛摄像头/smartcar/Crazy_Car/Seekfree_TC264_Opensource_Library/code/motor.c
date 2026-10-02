/**
 * @file motor.c
 * @brief 电机 PWM、方向、编码器和负压风机的底层封装。
 * @author 高冷马铃薯
 *
 * @par 所属项目
 * 2026 智能车区赛（摄像头）
 *
 * @par WegooAI 中转站
 * 本项目用户代码 99% 通过作者自建的 WegooAI 中转站，
 * 使用 Claude Code、Codex 等 AI 编程工具完成。
 * WegooAI 由作者高冷马铃薯搭建并维护，
 * 方便大家接入 AI 编程工具编写和修改代码，欢迎使用 :)
 * 官网：https://ai.wegoo.site/
 *
 * @par 联系作者
 * Bilibili：https://space.bilibili.com/513729639
 * QQ：1153626206
 * Email：moonserver@foxmail.com
 */

/*
 * motor.c
 *
 * FOC 驱动板接口：
 *   输出：左右轮速度 PWM、方向 GPIO、电调 PWM。
 *   输入：左右轮 GPT12 编码器。
 */

#include "motor.h"
#include "control.h" /* DIST_PER_PULSE_MM */
#include "globals.h" /* total_dist */

volatile int32 left_encoder_total_pulses = 0;
volatile int32 right_encoder_total_pulses = 0;

/* 单个编码器脉冲折算的整车里程（左右轮各占一半），单位 m。 */
static const float distance_m_per_wheel_pulse = 0.5f * DIST_PER_PULSE_MM * 0.001f;

/* GPT12 保持自由运行。软件保存上次读数，用 16 位环形差分取得本周期增量。 */
static int16 encoder_last_left = 0;
static int16 encoder_last_right = 0;

/* 初始化车辆执行机构相关外设。
 * 左右轮使用 FOC 驱动板的 PWM+DIR 接口，负压电调用单独一路 50Hz PWM。
 */
void motor_init(void)
{
    gpio_init(MOTOR_ENABLE_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    /* 速度 PWM：频率 MOTOR_PWM_FREQ，初始占空比 0。 */
    pwm_init(PWM_L, MOTOR_PWM_FREQ, 0);
    pwm_init(PWM_R, MOTOR_PWM_FREQ, 0);

    /* 方向 GPIO：推挽输出，初值 DIR_FORWARD。 */
    gpio_init(DIR_L, GPO, DIR_FORWARD, GPO_PUSH_PULL);
    gpio_init(DIR_R, GPO, DIR_FORWARD, GPO_PUSH_PULL);

    /* 编码器：GPT12 脉冲+方向模式。 */
    encoder_dir_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
    encoder_dir_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);
    encoder_last_left = encoder_get_count(TIM4_ENCODER);
    encoder_last_right = encoder_get_count(TIM6_ENCODER);

    /* 电调 PWM：初始占空比 0。 */
    pwm_init(BLDC_CHANNEL, BLDC_PWM_FREQ, 0);
}

/* 输出单个 FOC 通道的目标占空比。
 * duty 的符号决定方向，绝对值决定 PWM 占空比；超过 PWM_DUTY_MAX 会被限幅。
 */
void motor_duty(pwm_channel_enum pwm_ch, gpio_pin_enum dir_pin, int16 duty)
{
    //    const uint32 pwm_input_min = 1000u;
    //     const uint32 pwm_input_max = 9000u;
    uint32 abs_duty;
    if (duty >= 0)
    {
        gpio_set_level(dir_pin, DIR_REVERSE);
        abs_duty = (uint32)duty;
    }
    else
    {
        gpio_set_level(dir_pin, DIR_FORWARD);
        abs_duty = (uint32)(-duty);
    }
    if (abs_duty > PWM_DUTY_MAX)
    {
        abs_duty = PWM_DUTY_MAX;
    }
    //    if (0u != abs_duty)//死区
    //     {
    //         abs_duty = pwm_input_min + (abs_duty * (pwm_input_max - pwm_input_min)) / PWM_DUTY_MAX;
    //     }
    pwm_set_duty(pwm_ch, abs_duty);
}

/* 关闭左右轮 PWM。
 * 这里只清 PWM，不改方向 GPIO，避免停车瞬间方向电平抖动。
 */
void motor_stop(void)
{
    pwm_set_duty(PWM_L, 0);
    pwm_set_duty(PWM_R, 0);
    /* 方向电平不动，保持最后一次设定 */
}

/* 读取逻辑左轮编码器增量，硬件计数器不清零。
 * 先转成 uint16 做模 65536 的减法，再转回 int16；因此正转、反转以及
 * 32767/-32768 回绕都能保持正确符号。相邻两次读取不得相差 32768 脉冲。
 */
int16 motor_read_left_count(void)
{
    int16 now = encoder_get_count(TIM4_ENCODER);
    int16 delta = (int16)(uint16)((uint16)now - (uint16)encoder_last_left);

    encoder_last_left = now;
    left_encoder_total_pulses += delta;
    /* 里程显示与脉冲累计同源同处：任何调用方读编码器都会自动走表。 */
    total_dist += (float)delta * distance_m_per_wheel_pulse;
    return delta;
}

/* 读取逻辑右轮编码器增量，硬件计数器不清零。
 * 右侧安装方向相反，最终取反，使两轮向前时在控制层都返回正数。
 */
int16 motor_read_right_count(void)
{
    int16 now = encoder_get_count(TIM6_ENCODER);
    int16 delta = (int16)(uint16)((uint16)now - (uint16)encoder_last_right);
    int16 logical_delta = (int16)(-delta);

    encoder_last_right = now;
    right_encoder_total_pulses += logical_delta;
    /* 与左轮一致使用取反后的符号，前进为正。 */
    total_dist += (float)logical_delta * distance_m_per_wheel_pulse;
    return logical_delta;
}

/* 直接设置负压电调 PWM duty。
 * 这是底层接口，业务代码用 BLDC_set_percent() 选档、BLDC_stop() 关闭。
 */
void BLDC_set_duty(uint16 duty)
{
    if (duty > PWM_DUTY_MAX)
    {
        duty = PWM_DUTY_MAX;
    }
    pwm_set_duty(BLDC_CHANNEL, (uint32)duty);
}

void BLDC_stop(void)
{
    pwm_set_duty(BLDC_CHANNEL, 0u);
}

/* 按百分比设置负压电调。
 * 0% 映射到 1ms，100% 映射到 2ms，适配 50Hz 舵机/电调输入语义。
 */
void BLDC_set_percent(uint8 percent)
{
    uint32 duty;

    if (percent > 100u)
    {
        percent = 100u;
    }

    /*
     * 50Hz PWM 周期为 20ms。
     * pwm_set_duty 使用 PWM_DUTY_MAX 作为满占空比，因此 1ms=5%，2ms=10%。
     */
    duty = (PWM_DUTY_MAX / 20u) + ((uint32)percent * (PWM_DUTY_MAX / 20u)) / 100u;
    BLDC_set_duty((uint16)duty);
}
