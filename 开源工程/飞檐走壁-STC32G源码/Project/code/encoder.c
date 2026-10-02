/*****************************************************************************
* 文件名称: encoder.c
* 作者:     get_lucky()
* 日期:     2026.8.31
* 功能:     编码器测速与里程累计
*****************************************************************************/

#include "my_headfile.h"

int16 edata encoder_data_dir_L = 0;
int16 edata encoder_data_dir_R = 0;
uint32 edata motor_distance = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器测速与里程累计
// 参数说明     无
// 返回参数     void
// 备注         读取并清零两路编码器计数，换算为左右轮速度；
//             仅在圆环(入环/离环)与飞坡(起飞/落地)阶段累计里程 motor_distance
//-------------------------------------------------------------------------------------------------------------------
void get_encoder_vle(void)
{
    encoder_data_dir_R = encoder_get_count(ENCODER_DIR_1);                  // 获取编码器计数
    encoder_data_dir_L = -encoder_get_count(ENCODER_DIR_2);              	// 获取编码器计数

    encoder_clear_count(ENCODER_DIR_1);                                		// 清空编码器计数
    encoder_clear_count(ENCODER_DIR_2);                             		// 清空编码器计数

	if(ring_flag == 1 || ring_flag == 4
		|| flyboard_flag == 1 || flyboard_flag == 3) //
	{motor_distance += (encoder_data_dir_L + encoder_data_dir_R) >> 1; }
	
	else motor_distance = 0;
	
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器初始化
// 参数说明     无
// 返回参数     void
// 备注         两路带方向增量编码器：TIM0(脉冲P34/方向P35)、TIM3(脉冲P04/方向P53)
//-------------------------------------------------------------------------------------------------------------------
void encoder_init(void)
{
	
    encoder_dir_init(ENCODER_DIR_1, ENCODER_DIR_DIR_1, ENCODER_DIR_PULSE_1);   	// 初始化编码器模块与引脚 带方向增量编码器模式
    encoder_dir_init(ENCODER_DIR_2, ENCODER_DIR_DIR_2, ENCODER_DIR_PULSE_2);    // 初始化编码器模块与引脚 带方向增量编码器模式

}