/*****************************************************************************
* 文件名称: gyro.c
* 作者:     get_lucky()
* 日期:     2026.8.31
* 功能:     陀螺仪数据处理：角速度采集、零漂校准、角度积分
*****************************************************************************/

#include "my_headfile.h"

IMU_param edata IMU_Data;

char edata z_integral_flag = 0;//陀螺仪积分标志位
char edata y_integral_flag = 1;//陀螺仪积分标志位
float edata angle_integral_Z = 0;//z轴角度
float edata angle_integral_Y = 0;//y轴角度

#define alpha          0.6f
#define alpha1         0.75f

/* 陀螺仪处理常量 */
#define GYRO_SAMPLE_NUM     (300)       // 零漂校准采样次数
#define GYRO_INTEGRAL_DT    (0.002f)    // 角度积分周期(对应2ms控制周期)
#define ANGLE_RANGE_DEG     (360.0f)    // 角度积分范围(度)
#define ZERO_DRIFT_TOLER    (4.0f)      // 零漂容差(度/秒)
#define GYRO_DEAD_ZONE      (0.5f)      // 积分死区(度/秒)
#define OFFSET_SETTLE_MS    (200)       // 零漂校准前稳定等待时间(ms)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪角速度采集与去零漂
// 参数说明     无
// 返回参数     void
// 备注         读取 ICM42688 原始角速度并减去零漂；z 轴用于转向环，y 轴按硬件安装方向映射；
//             随后调用角度积分
//-------------------------------------------------------------------------------------------------------------------
void gyro_readvalue(void)
{

    Get_Gyro_ICM42688();
    IMU_Data.gyro_z=(float)(icm42688_gyro_z - IMU_Data.z_offset);
	  IMU_Data.gyro_y=(float)(icm42688_gyro_x - IMU_Data.x_offset);//陀螺仪硬件方向问题，可自行调整
    gyro_integral();//陀螺仪积分

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪角度积分(角速度积分为角度)
// 参数说明     无
// 返回参数     void
// 备注         按 2ms 控制周期积分；小于死区的角速度不积分(滤波)；
//             角度超过 ±360° 清零归位；z/y 轴积分由 z_integral_flag/y_integral_flag 控制
//-------------------------------------------------------------------------------------------------------------------
void gyro_integral(void) {
    if(z_integral_flag == 1) 
    {
        if(IMU_Data.gyro_z < GYRO_DEAD_ZONE && IMU_Data.gyro_z > -GYRO_DEAD_ZONE) {
                angle_integral_Z += 0;//滤波过程
        }
        else {
            if(angle_integral_Z > ANGLE_RANGE_DEG || angle_integral_Z < -ANGLE_RANGE_DEG)//规划为360度制
            {
                angle_integral_Z = 0;
            }
            else {
                angle_integral_Z += (IMU_Data.gyro_z * GYRO_INTEGRAL_DT);
            }
        }
    }
    else
    {
        angle_integral_Z = 0;
    }
		
		
		if(y_integral_flag == 1) 
    {
        if(IMU_Data.gyro_y < GYRO_DEAD_ZONE && IMU_Data.gyro_y > -GYRO_DEAD_ZONE) {
                angle_integral_Y += 0;//滤波过程
        }
        else {
            if(angle_integral_Y > ANGLE_RANGE_DEG || angle_integral_Y < -ANGLE_RANGE_DEG)//规划为360度制
            {
                angle_integral_Y=0;
            }
            else {
                angle_integral_Y -= (IMU_Data.gyro_y * GYRO_INTEGRAL_DT);
            }
        }
    }
    else
    {
        angle_integral_Y = 0;
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     陀螺仪零漂校准
// 参数说明     无
// 返回参数     void
// 备注         静止采样 GYRO_SAMPLE_NUM 次取平均作为零漂；
//             若 z 轴零漂超出容差则持续提示等待重启
//-------------------------------------------------------------------------------------------------------------------
void gyro_offset_init(void) {
	int16 i;
	
    IMU_Data.x_offset = 0;                          //参数初始化
    IMU_Data.y_offset = 0;
    IMU_Data.z_offset = 0;

    system_delay_ms(OFFSET_SETTLE_MS);                     //防止抖动出错


    for (i = 0; i < GYRO_SAMPLE_NUM; i++)
    {
        Get_Gyro_ICM42688();                        //采集角速度
        IMU_Data.x_offset += icm42688_gyro_x;       //自加求平均
        IMU_Data.y_offset += icm42688_gyro_y;
        IMU_Data.z_offset += icm42688_gyro_z;
       system_delay_ms(1);   // 最大 1Khz
    }
    IMU_Data.x_offset /= GYRO_SAMPLE_NUM;
    IMU_Data.y_offset /= GYRO_SAMPLE_NUM;
    IMU_Data.z_offset /= GYRO_SAMPLE_NUM;

    while(IMU_Data.z_offset > ZERO_DRIFT_TOLER || IMU_Data.z_offset < -ZERO_DRIFT_TOLER)//采集已经无效，报警提示重启
    {
        ips114_show_string(30,4*16,"offset_z:");ips114_show_float(30+9*8,4*16,IMU_Data.z_offset,4,3);
    }
}
