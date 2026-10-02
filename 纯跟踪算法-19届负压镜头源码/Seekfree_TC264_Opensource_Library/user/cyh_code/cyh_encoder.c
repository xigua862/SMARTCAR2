
#include "zf_common_headfile.h"
#include "cyh_headfile.h"

// **************************** 代码区域 ****************************

int speedl,speedr;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     带方向编码器初始化
// 参数说明
// 返回参数
// 使用示例     encoder_int();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

void encoder_int(void)
{
    encoder_dir_init(TIM2_ENCODER,TIM2_ENCODER_CH1_P33_7,TIM2_ENCODER_CH2_P33_6);
    encoder_dir_init(TIM6_ENCODER,TIM6_ENCODER_CH1_P20_3,TIM6_ENCODER_CH2_P20_0);



    speedl=0;
    speedr=0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器数据读取
// 参数说明
// 返回参数
// 使用示例     encoder_int();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------


void get_encoder(void)
{

    speedl=encoder_get_count(TIM2_ENCODER);
    speedr=-encoder_get_count(TIM6_ENCODER);
    encoder_clear_count(TIM2_ENCODER);
    encoder_clear_count(TIM6_ENCODER);

//    printf("%d,%d\n",speedl,speedr);

}
