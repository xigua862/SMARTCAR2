/*****************************************************************************
* 文件名称: gyro.h
* 作者:     get_lucky()
* 日期:     2026.8.31
* 功能:     陀螺仪模块头文件：IMU数据结构体与函数声明
*****************************************************************************/

#ifndef GYRO_H
#define GYRO_H

typedef struct{
    float acc_x;   //x轴加速度
    float acc_y;   //y轴加速度
    float acc_z;   //z轴加速度

    float gyro_x;  //x轴角速度
    float gyro_y;  //y轴角速度
    float gyro_z;  //z轴角速度

    float x_offset;   //零飘参数X
    float y_offset;   //零飘参数Y
    float z_offset;   //零飘参数Z
}IMU_param;
extern IMU_param edata  IMU_Data;

extern char edata z_integral_flag;//陀螺仪积分标志位
extern char edata y_integral_flag;//陀螺仪积分标志位
extern float edata angle_integral_Z;//陀螺仪积分值
extern float edata angle_integral_Y;//陀螺仪积分值


void gyro_readvalue(void);     // 陀螺仪角速度采集与去零漂
void gyro_integral(void);      // 陀螺仪角度积分
void gyro_offset_init(void);   // 陀螺仪零漂校准

#endif