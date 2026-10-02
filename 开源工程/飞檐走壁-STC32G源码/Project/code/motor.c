/*****************************************************************************
* 文件名称: motor.c
* 作者:     get_lucky()
* 日期:     2026.8.31
* 功能:     直流电机驱动：PWM占空比与方向控制、电机初始化
*****************************************************************************/

#include "my_headfile.h"

/* 电机 PWM 占空比上限(对应 PWM_DUTY_MAX) */
#define MOTOR_PWM_MAX       (10000)     // 最大占空比

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机 PWM 与方向控制
// 参数说明     motor_L    左轮控制量(有符号, 正转/反转)
// 参数说明     motor_R    右轮控制量(有符号, 正转/反转)
// 返回参数     void
// 备注         控制量限幅在 ±MOTOR_PWM_MAX；符号决定方向引脚电平，绝对值写入 PWM 占空比
//-------------------------------------------------------------------------------------------------------------------
void motor_control(int16 motor_L,int16 motor_R) {

    if (motor_L > MOTOR_PWM_MAX) {
        motor_L = MOTOR_PWM_MAX;
    }
    if (motor_L < -MOTOR_PWM_MAX) {
        motor_L = -MOTOR_PWM_MAX;
    }
    if (motor_R > MOTOR_PWM_MAX) {
        motor_R = MOTOR_PWM_MAX;
    }
    if (motor_R < -MOTOR_PWM_MAX) {
        motor_R = -MOTOR_PWM_MAX;
    }

    if(0 <= motor_L) {
        gpio_set_level(DIR_L, 0);
        pwm_set_duty(PWM_L,motor_L);
    }
    else {
        gpio_set_level(DIR_L, 1);
        pwm_set_duty(PWM_L, -motor_L);
    }

    if(0 <= motor_R) {
        gpio_set_level(DIR_R, 0);
        pwm_set_duty(PWM_R, motor_R);
    }
    else {
        gpio_set_level(DIR_R, 1);
        pwm_set_duty(PWM_R, -motor_R);
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机初始化
// 参数说明     无
// 返回参数     void
// 备注         方向引脚配置为推挽输出(默认高)，PWM 频率 17KHz，初始占空比 0
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
	gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

}