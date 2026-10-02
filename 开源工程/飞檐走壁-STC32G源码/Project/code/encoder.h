/*****************************************************************************
* 文件名称: encoder.h
* 作者:     get_lucky()
* 日期:     2026.8.31
* 功能:     编码器模块头文件：通道/引脚宏与函数声明
*****************************************************************************/

#ifndef ENCODER_H
#define ENCODER_H

#define ENCODER_DIR_1                 	(TIM0_ENCOEDER)                         // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER1
#define ENCODER_DIR_DIR_1              	(IO_P35)            				 	// DIR 对应的引脚
#define ENCODER_DIR_PULSE_1            	(TIM0_ENCOEDER_P34)            			// PULSE 对应的引脚

#define ENCODER_DIR_2                 	(TIM3_ENCOEDER)                         // 带方向编码器对应使用的编码器接口 这里使用QTIMER1的ENCOEDER2
#define ENCODER_DIR_DIR_2           	(IO_P53)             					// DIR 对应的引脚
#define ENCODER_DIR_PULSE_2       		(TIM3_ENCOEDER_P04)            			// PULSE 对应的引脚

extern int16 edata encoder_data_dir_L;
extern int16 edata encoder_data_dir_R;

extern uint32 edata motor_distance;

void encoder_init(void);       // 编码器初始化
void get_encoder_vle(void);    // 编码器测速与里程累计

#endif
