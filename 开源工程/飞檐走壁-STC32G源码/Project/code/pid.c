/*****************************************************************************
* 文件名称: pid.c
* 作者:     get_lucky()
* 日期:     2026.8.31
* 功能:     PID控制器：角度环/角速度环位置式PID、速度环增量式PID、差速转向控制
*****************************************************************************/

#include "my_headfile.h"

/* 控制器输出限幅常量 */
#define ANGLE_OUT_LIMIT      (5000)      // 角度环输出限幅
#define GYRO_OUT_LIMIT       (600)       // 角速度环输出限幅
#define SPEED_OUT_LIMIT      (10000)     // 速度环输出限幅
uint16 edata count = 0;
pid_param gyro_pid;
pid_param angle_pid;
pid_param speed_pid;
pid_param speed_pid_L;
pid_param speed_pid_R;

int16 edata car_speed = 650;
float edata angle_kp = 55;
float edata angle_kd = 50;

float edata angle_rg1_kp = 70;
float edata angle_rg1_kd = 50;

uint16 edata power = 888;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度环 PID 初始化
// 参数说明     angle_pid    角度环 PID 结构体指针
// 返回参数     void
// 备注         位置式PID；kp/kd 由菜单参数 angle_kp/angle_kd 在控制循环中实时刷新
//-------------------------------------------------------------------------------------------------------------------
void PID_init_angle(pid_param *angle_pid) {
    angle_pid->kp=55;//40
    angle_pid->kp2=0;//0.1  3.3
    angle_pid->ki=0;
    angle_pid->kd=50;//10
    angle_pid->kd2=0;//0.2

//	angle_pid->kp=3;//40
//    angle_pid->kp2=0;//0.1  3.3
//    angle_pid->ki=0;
//    angle_pid->kd=3;//10
//    angle_pid->kd2=0;//0.2

    angle_pid->e=0;
    angle_pid->e_1=0;
    angle_pid->e_2=0;

    angle_pid->integrator=0;
    angle_pid->i_max=0;
    angle_pid->i_min=0;

    angle_pid->out=0;

    angle_pid->mode=angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角速度环 PID 初始化
// 参数说明     gyro_pid    角速度环 PID 结构体指针
// 返回参数     void
// 备注         内环控制器，对 z 轴角速度闭环，输出作为差速转向量
//-------------------------------------------------------------------------------------------------------------------
void PID_init_gyro(pid_param *gyro_pid) {
    gyro_pid->kp=1.3;//1.3 12 18
    gyro_pid->kp2=0;
    gyro_pid->ki=0.0;//0.1
    gyro_pid->kd=0.1;//0.4 10
    gyro_pid->kd2=0;
	
//	gyro_pid->kp=1;//1.3 12 18
//    gyro_pid->kp2=0;
//    gyro_pid->ki=0.0;//0.1
//    gyro_pid->kd=0.4;//0.4 10
//    gyro_pid->kd2=0;

    gyro_pid->e=0;
    gyro_pid->e_1=0;
    gyro_pid->e_2=0;

    gyro_pid->integrator=0;
    gyro_pid->i_max=30;//30
    gyro_pid->i_min=-30;//30

    gyro_pid->out=0;

    gyro_pid->mode=gyro;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度环 PID 初始化(通用)
// 参数说明     speed_pid   速度环 PID 结构体指针
// 返回参数     void
// 备注         工程实际分别使用 PID_init_speed_L/R 初始化左右轮
//-------------------------------------------------------------------------------------------------------------------
void PID_init_speed(pid_param *speed_pid)
{
    speed_pid->kp=50;  //2.2  7.2  20  140  40
    speed_pid->ki=10;  //0.6   3.2   5   20   10
    speed_pid->kd=0;  //0.1   0.2        0

    speed_pid->e=0;
    speed_pid->e_1=0;
    speed_pid->e_2=0;

    speed_pid->integrator=0;
    speed_pid->i_max=0;
    speed_pid->i_min=0;

    speed_pid->out=0;

    speed_pid->mode=speed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     左轮速度环 PID 初始化
// 参数说明     speed_pid_L   左轮速度环 PID 结构体指针
// 返回参数     void
// 备注         增量式PID，输出为左轮 PWM 占空比
//-------------------------------------------------------------------------------------------------------------------
void PID_init_speed_L(pid_param *speed_pid_L)
{
    speed_pid_L->kp=10;
    speed_pid_L->ki=1.2;
    speed_pid_L->kd=0.00;

    speed_pid_L->e=0;
    speed_pid_L->e_1=0;
    speed_pid_L->e_2=0;

    speed_pid_L->integrator=0;
    speed_pid_L->i_max=0;
    speed_pid_L->i_min=0;

    speed_pid_L->out=0;

    speed_pid_L->mode=speed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     右轮速度环 PID 初始化
// 参数说明     speed_pid_R   右轮速度环 PID 结构体指针
// 返回参数     void
// 备注         增量式PID，输出为右轮 PWM 占空比
//-------------------------------------------------------------------------------------------------------------------
void PID_init_speed_R(pid_param *speed_pid_R)
{
    speed_pid_R->kp=10;
    speed_pid_R->ki=1.2;
    speed_pid_R->kd=0.00;

    speed_pid_R->e=0;
    speed_pid_R->e_1=0;
    speed_pid_R->e_2=0;

    speed_pid_R->integrator=0;
    speed_pid_R->i_max=0;
    speed_pid_R->i_min=0;

    speed_pid_R->out=0;

    speed_pid_R->mode=speed;
}
int16 edata speed_err_L,speed_err_R;
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     差速转向速度控制
// 参数说明     pid_L           左轮速度环结构体指针
// 参数说明     pid_R           右轮速度环结构体指针
// 参数说明     desired_speed   目标车速(编码器计数/周期)
// 返回参数     void
// 备注         根据角速度环输出(gyro_pid.out)对左右轮差速；左转内1.2/外0.9，右转内0.9/外1.2；
//             四路电感同时失信号或到达第9环终点时停车
//-------------------------------------------------------------------------------------------------------------------
void speed_control(pid_param *pid_L, pid_param *pid_R, int16 desired_speed)
{

	if(ERR >= 0) // 左转
	{
		speed_err_L = (desired_speed - gyro_pid.out * 1.2) - encoder_data_dir_L;
		speed_err_R = (desired_speed + gyro_pid.out * 0.9) - encoder_data_dir_R;

	}
	if(ERR < 0) // 右转
	{
		speed_err_L = (desired_speed - gyro_pid.out * 0.9) - encoder_data_dir_L;
		speed_err_R = (desired_speed + gyro_pid.out * 1.2) - encoder_data_dir_R;

	}
	//内1.3  外0.8
		

	PID_IncrementCtrl(speed_err_L, pid_L);
	PID_IncrementCtrl(speed_err_R, pid_R);


	if((LX < 5 && LY < 5 && RX < 5 && RY < 5))//|| (ring_number == 2 && ring_flag == 0)
	{
		while(1)
		{
				motor_control(0,0);
				pwm_set_duty(PWMB_CH4_P77, 500);	
		}
	}
	else if(ring_number == 9 && motor_distance >= 50000)
	{
		while(1)
		{
				motor_control(0,0);
				pwm_set_duty(PWMB_CH4_P77, 800);	
		}
	}
	if(flyboard_flag == 2)// || ring_number == 2
	{
					pid_L->out = 0;pid_R->out = 0;

	}
//	else if(flyboard_flag == 1)
//	{
//		pid_L->out = 0;pid_R->out = 0;
//	}
		
	motor_control(pid_L->out,pid_R->out);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     位置式 PID 控制器
// 参数说明     error        本次误差
// 参数说明     last_link    上一级控制器输出(参考量)
// 参数说明     pid          PID 结构体指针
// 返回参数     void
// 备注         角度环误差直通；角速度环误差按 0.7/0.3 加权融合；含积分限幅与输出限幅
//-------------------------------------------------------------------------------------------------------------------
void PID_PlaceCtrl(float error,float last_link,pid_param *pid)
{
//    const float weights[4] = {0.8f, 0.1f, 0.05f, 0.05f};

    if(pid->mode==angle)
    {
        pid->e=error;
    }
    else
    {
        pid->e=0.7*error + 0.3*last_link;
    }
    pid->integrator += pid->e;

    if(pid->integrator > pid->i_max){pid->integrator = pid->i_max;}
    if(pid->integrator < pid->i_min){pid->integrator = pid->i_min;}

    if(pid->mode==angle)
        pid->out =(pid->kp * pid->e + pid->kp2 * pid->e * abs(pid->e) + pid->ki * pid->integrator + pid->kd * (pid->e - pid->e_1));
    else
        pid->out =(pid->kp * pid->e + pid->ki * pid->integrator + pid->kd * (pid->e - pid->e_1) );

    if(pid->mode==gyro)
    {

        if(pid->out > GYRO_OUT_LIMIT){pid->out = GYRO_OUT_LIMIT;}
        if(pid->out < -GYRO_OUT_LIMIT){pid->out = -GYRO_OUT_LIMIT;}

    }
    else if(pid->mode==angle)
    {
        if(pid->out > ANGLE_OUT_LIMIT){pid->out = ANGLE_OUT_LIMIT;}
        if(pid->out < -ANGLE_OUT_LIMIT){pid->out = -ANGLE_OUT_LIMIT;}

    }

    pid->e_1=pid->e;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     增量式 PID 控制器
// 参数说明     error    本次误差
// 参数说明     pid      PID 结构体指针
// 返回参数     void
// 备注         输出在上一拍基础上累加增量，并限幅在 [0, SPEED_OUT_LIMIT]
//-------------------------------------------------------------------------------------------------------------------
void PID_IncrementCtrl(int16 error, pid_param *pid)
{

	pid->e=(float)error;

    pid->out += ( pid->kp * (pid->e - pid->e_1) ) + (pid->ki * pid->e) + ( pid->kd * ( pid->e - 2*pid->e_1 + pid->e_2 ) );

	if(pid->out > SPEED_OUT_LIMIT){pid->out = SPEED_OUT_LIMIT;}
	if(pid->out< 0){pid->out = 0;}

    pid->e_2=pid->e_1;
    pid->e_1=pid->e;
	
}