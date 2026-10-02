/*********************************************************************************************************************
* STC32G Opensourec Library 即（STC32G 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是STC 开源库的一部分
*
* STC32G 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2024-08-01        大W            first version
********************************************************************************************************************/

#include "my_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主函数
// 参数说明     无
// 返回参数     void
// 备注         初始化时钟/调试/菜单/陀螺仪/电磁/EEPROM 参数/PID/PWM/定时器/电机/编码器；
//             菜单等待发车(go_flag)，发车后按 2ms 节拍执行角度环-角速度环-速度环级联控制
//-------------------------------------------------------------------------------------------------------------------
void main()
{
  clock_init(SYSTEM_CLOCK_30M);
	debug_init();

	menu_init();
	Init_ICM42688();          //陀螺仪初始化
	gyro_offset_init();       //零漂初始化
	electro_magnetic_init();
	
	eeprom_init();		//EEPROM初始化
	sys_param_load();	//上电恢复参数(校验失败自动用默认值)
	
	#if Bluetooth_or_LED
	ble6a20_init();
	seekfree_assistant_init();
	seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_BLE6A20);
	//需要传输四个通道数据
	seekfree_assistant_oscilloscope_data.channel_num = 2;
	#else
	gpio_init(IO_P02, GPO, 1, GPO_PUSH_PULL);  // LED P02 ring state
	gpio_init(IO_P03, GPO, 1, GPO_PUSH_PULL);  // LED P03 flyboard state
	#endif
	
	PID_init_angle(&angle_pid);              //角度环pid初始化
	PID_init_gyro(&gyro_pid);             	 //角速度环pid初始化
	PID_init_speed_L(&speed_pid_L);					 //左电机速度环pid初始化
	PID_init_speed_R(&speed_pid_R);					 //右电机速度环pid初始化
	
	pwm_init(PWMB_CH4_P77, 50, 500);				 //负压引脚初始化
	
	pit_ms_init(TIM1_PIT, 2);								 //控制中断

	motor_init();														 //电机初始化
	encoder_init();												
	
	while(go_flag == 0)Menu();							 //菜单显示

	ips114_clear(RGB565_WHITE);							 //清屏准备开跑
	
	pwm_set_duty(PWMB_CH4_P77, power);
	
	system_delay_ms(300);
	
	gogogo_flag = 1;
	
	while(1)
	{
			if(xunji_flag == 1)
			{
				if(ring_flag == 2)
				{
					angle_pid.kp = angle_rg1_kp;
					angle_pid.kd = angle_rg1_kd;
				}
				else
				{
					angle_pid.kp = angle_kp;
					angle_pid.kd = angle_kd;
				}
				PID_PlaceCtrl(ERR,0,&angle_pid); 
				PID_PlaceCtrl(IMU_Data.gyro_z,angle_pid.out,&gyro_pid); 
				speed_control(&speed_pid_L, &speed_pid_R, car_speed);

				xunji_flag = 0;
			}

	#if Bluetooth_or_LED

		seekfree_assistant_oscilloscope_data.dat[0] = ring_number;
		seekfree_assistant_oscilloscope_data.dat[1] = car_speed;
	
    // 通过串口发送虚拟示波器数据
		seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

    #endif
	}
}
