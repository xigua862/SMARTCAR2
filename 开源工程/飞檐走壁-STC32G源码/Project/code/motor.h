/*****************************************************************************
* 文件名称: motor.h
* 作者:     get_lucky()
* 日期:     2026.8.31
* 功能:     电机模块头文件：电机引脚宏与函数声明
*****************************************************************************/

#ifndef MOTOR_H
#define MOTOR_H

#define DIR_R               (IO_P64 )
#define PWM_R               (PWMA_CH4P_P66)

#define DIR_L               (IO_P60 )
#define PWM_L               (PWMA_CH2P_P62)


void motor_control(int16 motor_L,int16 motor_R);   // 电机PWM与方向控制
void motor_init(void);                             // 电机初始化

#endif