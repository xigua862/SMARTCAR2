/*****************************************************************************
* 文件名称: adc.c
* 作者:     get_lucky()
* 日期:     2026.8.31
* 功能:     电磁传感器采集与循迹算法：四路电感ADC采集、差和比误差计算、圆环与飞坡状态机
*****************************************************************************/

#include "my_headfile.h"

/* 圆环状态机状态定义 */
#define RING_NONE        (0)     // 无环(正常循迹)
#define RING_ENTER       (1)     // 入环
#define RING_INSIDE      (2)     // 环内行驶
#define RING_EXIT        (3)     // 出环
#define RING_LEAVE       (4)     // 离环(等待复位)
/* 飞坡状态机状态定义 */
#define FLY_GROUND       (0)     // 平地正常行驶
#define FLY_LAUNCH       (1)     // 检测到起飞
#define FLY_AIR          (2)     // 腾空阶段
#define FLY_LAND         (3)     // 落地滑行
/* 传感器标定与状态判定阈值 */
#define ADC_SCALE_XL     (925)   // 左横向电感满量程标定值
#define ADC_SCALE_YL     (925)   // 左纵向电感满量程标定值
#define ADC_SCALE_YR     (928)   // 右纵向电感满量程标定值
#define ADC_SCALE_XR     (926)   // 右横向电感满量程标定值
#define RING_YL_TH       (400)   // 入环判定: 纵向电感原始值阈值
#define FLY_ADC_X_TH     (255)   // 飞坡判定: 横向电感原始值阈值
#define FLY_ADC_Y_TH     (140)   // 飞坡判定: 纵向电感原始值阈值
#define FLY_GYRO_PITCH   (-290)  // 飞坡判定: 俯仰角速度阈值
#define RING_ANGLE_IN    (60)    // 出环判定角度(度)
#define RING_ANGLE_OUT   (340)   // 离环判定角度(度)
#define RING_LEAVE_DIS   (15000) // 离环复位距离
#define FLY_END_DIS      (50000) // 飞坡结束/终点距离
#define ERR_RING_LIMIT   (10)    // 环内误差限幅

float edata adc_x_l;//左横向
float edata adc_y_l;//左纵向
float edata adc_y_r;//右纵向
float edata adc_x_r;//右横向
float edata LX;//左横向
float edata LY;//左纵向
float edata RY;//右纵向
float edata RX;//右横向

uint8 edata ring_flag = 0;//圆环
uint8 edata ring_number = 0;
uint16 edata ring_tim = 0;

uint8 edata flyboard_flag = FLY_GROUND;//飞坡板
uint16 edata flyboard_tim = 0;//飞坡板停留时间
uint16 edata fly_tim = 30;
uint16 edata fly_dis = 2600;

uint16 edata in_ring_dis_d = 8000;//入环距离
uint16 edata in_ring_dis_x = 5000;//入环距离

uint16 edata in_ring_threshold = 1300;//入环阈值
uint16 edata out_ring_tim = 500;//退环阈值
float edata Kb = 1.2;//竖电感比例
float edata r_offset_d = 35;//入环偏差
float edata r_offset_x = 50;//入环偏差

float edata sub, add;
float edata ERR = 0;
float edata ERR_Last = 0;
int16 edata left_out = 0, right_out = 0;

uint8 count_over = 0;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电磁循迹模块初始化
// 参数说明     无
// 返回参数     uint8    固定返回 1
// 备注         初始化四路电感 ADC(10位) 及环/飞坡状态指示灯 P02、P03
//-------------------------------------------------------------------------------------------------------------------
uint8 electro_magnetic_init(void)
{
		adc_init(ADC_CHANNEL1, ADC_10BIT);
    adc_init(ADC_CHANNEL2, ADC_10BIT);
    adc_init(ADC_CHANNEL3, ADC_10BIT);
    adc_init(ADC_CHANNEL4, ADC_10BIT);
    gpio_init(IO_P02, GPO, 1, GPO_PUSH_PULL);  // LED P02 ring state
    gpio_init(IO_P03, GPO, 1, GPO_PUSH_PULL);  // LED P03 flyboard state

		return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电磁传感器采集与循迹误差计算
// 参数说明     无
// 返回参数     void
// 备注         2ms 周期由定时器中断调用；完成四路电感均值滤波采集、归一化、
//             差和比误差计算(ERR)、飞坡与圆环状态机更新、状态指示灯刷新
//-------------------------------------------------------------------------------------------------------------------
void electro_magnetic_collect(void)
{

	float const A = 1.0;
	float const B = Kb;//1.15
	float const C = 1.0;//0.8

	adc_x_l = adc_mean_filter_convert(ADC_CHANNEL1, 5);
	adc_y_l = adc_mean_filter_convert(ADC_CHANNEL2, 5);
	adc_y_r = adc_mean_filter_convert(ADC_CHANNEL3, 5);
	adc_x_r = adc_mean_filter_convert(ADC_CHANNEL4, 5);

	LX = adc_x_l / ADC_SCALE_XL * 100.0f;
	LY = adc_y_l / ADC_SCALE_YL * 100.0f;
	RY = adc_y_r / ADC_SCALE_YR * 100.0f;
	RX = adc_x_r / ADC_SCALE_XR * 100.0f;

	LX = func_limit_ab(LX, 0, 100);
	LY = func_limit_ab(LY, 0, 100);
	RY = func_limit_ab(RY, 0, 100);
	RX = func_limit_ab(RX, 0, 100);

	sub = (A * (LX - RX) + B * (LY - RY) ) * 100;
	add = A * (LX + RX) + C * abs(LY - RY) + 1;

	ERR = sub / add;//差和比计算误差
	flyboard();
	ring_judge();

	if((ring_flag == RING_ENTER || ring_flag == RING_INSIDE || ring_flag == RING_EXIT))
			gpio_set_level(IO_P02, 0);
	else gpio_set_level(IO_P02, 1);

	if((flyboard_flag == FLY_LAUNCH || flyboard_flag == FLY_AIR || flyboard_flag == FLY_LAND))//
			gpio_set_level(IO_P03, 0);
	else gpio_set_level(IO_P03, 1);

	if(ring_flag == RING_INSIDE && (ring_number == 2 || ring_number == 6 || ring_number == 10))ERR = -r_offset_x;//右环 小环
	else if(ring_flag == RING_INSIDE && (ring_number == 3 || ring_number == 7 || ring_number == 11))ERR = r_offset_d;//左环 大环


	if(ring_flag == RING_LEAVE 
		&& (ring_number == 3 || ring_number == 7 || ring_number == 11))//限制右环偏差
	{
		if(ERR < -ERR_RING_LIMIT)ERR = -ERR_RING_LIMIT;//右环
	}	
	else if(ring_flag == RING_LEAVE 
		&& (ring_number == 2 || ring_number == 6 || ring_number == 10))//限制左环偏差
	{
		if(ERR > ERR_RING_LIMIT)ERR = ERR_RING_LIMIT;//右环
	}	

  ERR_Last = ERR;

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     圆环状态机
// 参数说明     无
// 返回参数     void
// 备注         状态: NONE->ENTER->INSIDE->EXIT->LEAVE->NONE；
//             小环(2/6/10)按距离入环，大环(3/7/11)按距离入环，
//             环内按积分角度判定出环/离环，过环段(1/4/5/8/9/12)按时间计数复位
//-------------------------------------------------------------------------------------------------------------------
void ring_judge(void)
{
	if((adc_x_l + adc_x_r) >= in_ring_threshold && (adc_y_l + adc_y_r) >= RING_YL_TH
		&& ring_flag == RING_NONE && flyboard_flag == FLY_GROUND)
	{
		ring_flag = RING_ENTER;//入环
		ring_number ++;
		count_over = 0;//计数结束请0
		z_integral_flag = 1;
	}
	else if(motor_distance >= in_ring_dis_x && ring_flag == RING_ENTER && 
		 ((ring_number==2)||(ring_number==6)||(ring_number==10)) )//入环点参数，需进入菜单修改
	{
		ring_flag = RING_INSIDE;//环内行驶
		z_integral_flag = 1;
	}
	else if(motor_distance >= in_ring_dis_d && ring_flag == RING_ENTER && 
		  ((ring_number==3)||(ring_number==7)||(ring_number==11)))//入环点参数，需进入菜单修改
	{
		ring_flag = RING_INSIDE;//环内行驶
		z_integral_flag = 1;
	}	
	else if(ring_flag == RING_INSIDE && (angle_integral_Z > RING_ANGLE_IN || angle_integral_Z < -RING_ANGLE_IN))
	{
		ring_flag = RING_EXIT;//出环
		z_integral_flag = 1;
	}
	else if(ring_flag == RING_EXIT && (angle_integral_Z > RING_ANGLE_OUT || angle_integral_Z < -RING_ANGLE_OUT))//转到340度到达
	{
		ring_flag = RING_LEAVE;//离环
		z_integral_flag = 0;
		angle_integral_Y = 0;
	}
	else if(ring_flag == RING_LEAVE && motor_distance > RING_LEAVE_DIS)//离环偏远距离准备复位
	{
		ring_flag = RING_NONE;
	}
	if( ((ring_number==1)||(ring_number==4)||(ring_number==5)||(ring_number==8)||(ring_number==9)||(ring_number==12))
		&& ring_tim >= out_ring_tim)
	{
		count_over = 1;
		ring_flag = RING_NONE;
		z_integral_flag = 0;
		angle_integral_Z = 0;
	  angle_integral_Y = 0;
	}
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     飞坡状态机
// 参数说明     无
// 返回参数     void
// 备注         状态: GROUND->LAUNCH->AIR->LAND->GROUND；
//             电感信号骤降且俯仰角速度超阈值判定起飞，按距离/时间推进状态
//-------------------------------------------------------------------------------------------------------------------
void flyboard(void)
{
	if(flyboard_flag == FLY_GROUND
		&& adc_x_l < FLY_ADC_X_TH && adc_x_r < FLY_ADC_X_TH && adc_y_l < FLY_ADC_Y_TH && adc_y_r < FLY_ADC_Y_TH && IMU_Data.gyro_y <= FLY_GYRO_PITCH)
	{
		flyboard_flag = FLY_LAUNCH;
	}
	else if(flyboard_flag == FLY_LAUNCH && motor_distance >= fly_dis)
	{
		flyboard_flag = FLY_AIR;
	}
	else if(flyboard_flag == FLY_AIR && flyboard_tim >= fly_tim)
	{
		flyboard_flag = FLY_LAND;
	} 
	else if(flyboard_flag == FLY_LAND && motor_distance >= FLY_END_DIS)
	{
		flyboard_flag = FLY_GROUND;
		angle_integral_Y = 0;
	}
}
