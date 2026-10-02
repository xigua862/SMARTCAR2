/*****************************************************************************
* 文件名称: pid.h
* 作者:     get_lucky()
* 日期:     2026.8.31
* 功能:     PID模块头文件：PID结构体、全局参数与函数声明
*****************************************************************************/

#ifndef PID_H
#define PID_H

typedef enum
{
    gyro  = 0,
    angle = 1,
    speed = 2,
}pid_part;

typedef struct
{

    float kp;
    float kp2;
    float ki;
    float kd;
    float kd2;

    float e;
    float e_1;
    float e_2;

    float integrator;//误差累计
    float i_max;//误差累计限幅
    float i_min;//误差累计限幅

    float out;
    float last_out;

    pid_part mode;

}pid_param;
extern uint16 edata count;
extern pid_param gyro_pid;
extern pid_param angle_pid;
extern pid_param speed_pid;
extern pid_param speed_pid_L;
extern pid_param speed_pid_R;

extern int16 edata car_speed;

extern float edata angle_kp;
extern float edata angle_kd;

extern float edata angle_rg1_kp;
extern float edata angle_rg1_kd;

extern uint16 edata power;

extern int16 edata speed_err_L,speed_err_R;

void PID_init_angle(pid_param *angle_pid);       // 角度环PID初始化
void PID_init_gyro(pid_param *gyro_pid);         // 角速度环PID初始化
void PID_init_speed(pid_param *speed_pid);       // 速度环PID初始化(通用)
void PID_init_speed_L(pid_param *speed_pid_L);   // 左轮速度环PID初始化
void PID_init_speed_R(pid_param *speed_pid_R);   // 右轮速度环PID初始化
void PID_PlaceCtrl(float error,float last_link,pid_param *pid);  // 位置式PID控制器
void PID_IncrementCtrl(int16 error, pid_param *pid);                    // 增量式PID控制器
void speed_control(pid_param *pid_L, pid_param *pid_R, int16 desired_speed);   // 差速转向速度控制

#endif