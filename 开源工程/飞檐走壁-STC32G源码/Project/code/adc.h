/*****************************************************************************
* 文件名称: adc.h
* 作者:     get_lucky()
* 日期:     2026.8.31
* 功能:     电磁循迹模块头文件：ADC通道宏、全局变量与函数声明
*****************************************************************************/

#ifndef ADC_H
#define ADC_H

#define Bluetooth_or_LED     0

#define ADC_CHANNEL1            (ADC_CH8_P00)
#define ADC_CHANNEL2            (ADC_CH13_P05)
#define ADC_CHANNEL3            (ADC_CH14_P06)
#define ADC_CHANNEL4            (ADC_CH9_P01)

extern float edata adc_x_l;
extern float edata adc_y_l;
extern float edata adc_y_r;
extern float edata adc_x_r;

extern float edata LX;
extern float edata LY;
extern float edata RY;
extern float edata RX;

extern uint8 edata ring_flag;
extern uint8 edata ring_number;
extern uint16 edata ring_tim;

extern uint8 edata flyboard_flag;
extern uint16 edata flyboard_tim;

extern uint16 edata fly_tim;
extern uint16 edata fly_dis;

extern uint16 edata in_ring_dis_d;//入环距离
extern uint16 edata in_ring_dis_x;//入环距离
extern uint16 edata in_ring_threshold;//入环阈值
extern uint16 edata out_ring_tim;//退环时间

extern float edata Kb;

extern float edata r_offset_d;//入环偏差
extern float edata r_offset_x;//入环偏差

extern float edata sub, add ;
extern float edata ERR;
extern float edata ERR_Last;
extern int16 edata left_out, right_out;

extern uint8 edata count_over;

uint8 electro_magnetic_init(void);        // 电磁循迹模块初始化
void electro_magnetic_collect(void);      // 电磁采集与误差计算(2ms周期调用)
void ring_judge(void);                    // 圆环状态机
void flyboard(void);                      // 飞坡状态机

#endif
