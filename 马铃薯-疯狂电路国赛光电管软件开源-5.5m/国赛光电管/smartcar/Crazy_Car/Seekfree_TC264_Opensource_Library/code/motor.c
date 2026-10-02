/**
 * @file motor.c
 * @brief 电机 PWM、方向、编码器和负压风机的底层封装。
 * @author 高冷马铃薯
 *
 * @par 所属项目
 * 2026 智能车国赛（光电管）
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

// motor.c —— 左右轮 FOC 驱动、编码器读取、负压电调(BLDC) 实现。
// 电机引脚、duty方向和编码器符号必须作为一组配置，保证上层正速度对应车辆前进。

#include "motor.h"
#include "control.h" /* DIST_PER_PULSE_MM */
#include "globals.h" /* total_dist */

// 单脉冲折算整车里程(m)：一脉冲=DIST_PER_PULSE_MM 毫米×0.001 换米；再×0.5 因左右轮各
// 贡献一半，避免左右读数相加把里程算成 2 倍。
static const float distance_m_per_wheel_pulse = 0.5f * DIST_PER_PULSE_MM * 0.001f;

// GPT12 自由运行，软件存上次读数用 16 位环形差分取本周期增量。
static int16 encoder_last_left = 0;
static int16 encoder_last_right = 0;

// 初始化车辆执行机构：左右轮 FOC 的 PWM+DIR，负压电调单独一路 50Hz PWM。上电调用一次。
void motor_init(void)
{
    // FOC 驱动板总使能：推挽输出，初始低电平
    gpio_init(MOTOR_ENABLE_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    // 速度 PWM：初始占空比 0
    pwm_init(PWM_L, MOTOR_PWM_FREQ, 0);
    pwm_init(PWM_R, MOTOR_PWM_FREQ, 0);

    // 方向 GPIO：推挽输出，初值 DIR_FORWARD
    gpio_init(DIR_L, GPO, DIR_FORWARD, GPO_PUSH_PULL);
    gpio_init(DIR_R, GPO, DIR_FORWARD, GPO_PUSH_PULL);

    // 编码器 GPT12 脉冲+方向模式。初始化后立刻读一次存作差分基准，避免首次读取把上电初值当增量。
    encoder_dir_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
    encoder_dir_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);
    encoder_last_left = encoder_get_count(TIM4_ENCODER);
    encoder_last_right = encoder_get_count(TIM6_ENCODER);

    // 电调 PWM：初始占空比 0
    pwm_init(BLDC_CHANNEL, BLDC_PWM_FREQ, 0);
}

// 使能 8701 FOC 驱动板：执行一次高-低-高复位时序。
// motor_init() 只把使能脚建成低电平；发车入口才调用本函数，避免上电即给驱动板放行。
void motor_enable(void)
{
    gpio_high(MOTOR_ENABLE_PIN);
    system_delay_us(1500u);
    gpio_low(MOTOR_ENABLE_PIN);
    system_delay_us(10u);
    gpio_high(MOTOR_ENABLE_PIN);
}

// 关闭 8701 FOC 驱动板总使能。调用方应先清 PWM，再拉低使能脚。
void motor_disable(void)
{
    gpio_low(MOTOR_ENABLE_PIN);
}

// 输出单个 FOC 通道：duty 符号决定方向，绝对值决定占空比，超 PWM_DUTY_MAX 限幅。
void motor_duty(pwm_channel_enum pwm_ch, gpio_pin_enum dir_pin, int16 duty)
{
    //    const uint32 pwm_input_min = 1000u;
    //     const uint32 pwm_input_max = 9000u;
    uint32 abs_duty; // duty 取绝对值后写进 PWM
    if (duty >= 0)
    {
        gpio_set_level(dir_pin, DIR_FORWARD);
        abs_duty = (uint32)duty;
    }
    else
    {
        gpio_set_level(dir_pin, DIR_REVERSE);
        abs_duty = (uint32)(-duty);
    }
    // 限幅：占空比不能超过硬件最大值，否则 PWM 溢出
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

// 关闭左右轮 PWM。只清 PWM 不改方向 GPIO，避免停车瞬间方向电平抖动。
void motor_stop(void)
{
    pwm_set_duty(PWM_L, 0);
    pwm_set_duty(PWM_R, 0);
    // 方向电平不动，保持最后一次设定
}

// 读逻辑左轮编码器增量(前进为正)，硬件计数器不清零。用 uint16 差分再转 int16，正反转及
// 32767/-32768 回绕都保持正确符号；相邻两次读取不得相差 32768 脉冲。左轮硬件方向取反。
// 副作用：更新 encoder_last_left 与 total_dist。
int16 motor_read_left_count(void)
{
    int16 now = encoder_get_count(TIM4_ENCODER);
    int16 delta = (int16)(uint16)((uint16)now - (uint16)encoder_last_left); // 无符号差分再转有符号，处理 16 位回绕
    int16 logical_delta = (int16)(-delta); // 左轮硬件方向与逻辑前进相反，取反后前进为正

    encoder_last_left = now; // 作为下次差分基准
    if (0 != logical_delta)
    {
        encoder_motion_seq++;
    }
    total_dist += (float)logical_delta * distance_m_per_wheel_pulse; // 里程与脉冲累计同源，读编码器即走表
    return logical_delta;
}

// 读逻辑右轮编码器增量，硬件计数器不清零。右轮硬件增量直接作为逻辑前进正方向。
// 副作用：更新 encoder_last_right 与 total_dist。
int16 motor_read_right_count(void)
{
    int16 now = encoder_get_count(TIM6_ENCODER);
    int16 delta = (int16)(uint16)((uint16)now - (uint16)encoder_last_right); // 环形差分得硬件方向增量

    encoder_last_right = now; // 作为下次差分基准
    if (0 != delta)
    {
        encoder_motion_seq++;
    }
    total_dist += (float)delta * distance_m_per_wheel_pulse; // 与左轮一致用逻辑前进符号
    return delta;
}

// 负压电调(BLDC)

// 直接设置电调 PWM duty(底层接口)，超 PWM_DUTY_MAX 限幅。业务用 bldc_set_percent()/bldc_stop()。
void bldc_set_duty(uint16 duty)
{
    if (duty > PWM_DUTY_MAX)
    {
        duty = PWM_DUTY_MAX;
    }
    pwm_set_duty(BLDC_CHANNEL, (uint32)duty);
}

// 完全关闭负压 PWM(占空比清零)。不同于 bldc_set_percent(0)，后者仍输出 1ms 最低档脉冲。
void bldc_stop(void)
{
    pwm_set_duty(BLDC_CHANNEL, 0u);
}

// 按百分比设置负压电调：0%=1ms，100%=2ms，适配 50Hz 输入语义。percent 超 100 夹到 100。
void bldc_set_percent(uint8 percent)
{
    uint32 duty;

    if (percent > 100u)
    {
        percent = 100u;
    }

    // 50Hz 周期 20ms，PWM_DUTY_MAX 为满占空比，故 1ms=5%、2ms=10%。
    // 基准 1ms(=PWM_DUTY_MAX/20) 再按百分比在 1ms~2ms 间线性插值。
    duty = (PWM_DUTY_MAX / 20u) + ((uint32)percent * (PWM_DUTY_MAX / 20u)) / 100u;
    bldc_set_duty((uint16)duty);
}
