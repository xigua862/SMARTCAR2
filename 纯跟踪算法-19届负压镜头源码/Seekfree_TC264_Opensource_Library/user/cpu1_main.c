/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
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
* 文件名称          cpu1_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "cyh_headfile.h"
#pragma section all "cpu1_dsram"


// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// **************************** 代码区域 ****************************

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
        if(mt9v03x_finish_flag)
        {
            memcpy(image_use[0], mt9v03x_image[0], MT9V03X_H*MT9V03X_W);
            memcpy(image_see[0], mt9v03x_image[0], MT9V03X_H*MT9V03X_W);
            mt9v03x_finish_flag = 0;
        }

        if(tmel==1)
        {
            over=0;
            over=1;
            get_encoder();
            system_start();
//            daj=otsuThreshold(mt9v03x_image[0]);

            get_highest();
            //**********************************斑马线检测
            if_banmaxian();
            //**********************************边线爬取
            search_bianxian();

            //**********************************坡道检测
            if_po();

            //**********************************路障检测
            if_luzhang();
            //**********************************圆环检测
            if_yuanhuan();
            //**********************************十字检测
            if_shizi();

            if (shizi!=0||yuanhuan!=0||qipao!=0||luzhang!=0)//(ang_numR>0&&ang_numR<10)||(ang_numL>0&&ang_numL<10)(ifzhi_L!=0)||(ifzhi_R!=0)(ang_numR>0&&ang_numR<10)||(ang_numL>0&&ang_numL<10)
            {
                gpio_init(P33_10, GPO, 1, GPO_PUSH_PULL);
            }
            else
            {
                gpio_init(P33_10, GPO, 0, GPO_PUSH_PULL);
            }

            //**********************************边线采集中线
            if (Lout_count<50)Lmidnum=Lout_count;
            else Lmidnum=49;
            if (Rout_count<50)Rmidnum=Rout_count;
            else Rmidnum=49;
            get_mid_L(Luse_edge,Lmidnum,Lin_MID,&Lmidnum,2);
            get_mid_R(Ruse_edge,Rmidnum,Rin_MID,&Rmidnum,2);
            //**********************************中线滤波
            blur_points(L_edge,Lout_edge,L_count,3);
            blur_points(R_edge,Rout_edge,R_count,3);
            //**********************************重采样中线
            if (Lmidnum*2<49)Lmidnum2=Lmidnum*2;
            else Lmidnum2=50;
            if (Rmidnum*2<49)Rmidnum2=Rmidnum*2;
            else Rmidnum2=50;
            resample_points2(Lin_MID,Lmidnum,Lout_MID,&Lmidnum2,4);
            resample_points2(Rin_MID,Rmidnum,Rout_MID,&Rmidnum2,4);

            LC=Lout_count;
            RC=Rout_count;
            LMC=Lmidnum2;
            RMC=Rmidnum2;
            LG=ang_numL;
            RG=ang_numR;

            tmel=0;

            xunxain_celue();

            //**********************************

            jiansu();
            get_error();
            madasudu();
            duoji_out();


            if (fache==2)
            {
                motor_out();
                if (speedtobel==0)
                {
                    pwm_set_duty (ATOM2_CH5_P33_13,0);
                    pwm_set_duty (ATOM1_CH5_P32_4,0);
                    if (yuanhuan!=0)
                    {
                        pwm_set_duty (ATOM1_CH5_P32_4,5800/*+duty*30*/);
                        pwm_set_duty (ATOM2_CH5_P33_13,5800/*+duty*30*/);
                    }
                    if (banmacishu>=2)
                    {
                        pwm_set_duty (ATOM1_CH5_P32_4,5800/*+duty*30*/);
                        pwm_set_duty (ATOM2_CH5_P33_13,5800/*+duty*30*/);
                    }
                }
                else
                {
                    if (duty>0)
                    {
                        pwm_set_duty (ATOM1_CH5_P32_4,5600+duty_chasu*5/**/);
                        pwm_set_duty (ATOM2_CH5_P33_13,5600+duty_chasu*0/*+duty*30*/);
                    }
                    else
                    {
                        pwm_set_duty (ATOM1_CH5_P32_4,5600-duty_chasu*0/*-duty*30*/);
                        pwm_set_duty (ATOM2_CH5_P33_13,5600-duty_chasu*5/**/);
                    }
                }
            }
            else if (fache==1)
            {
                gpio_set_level(P21_2,0);
                pwm_set_duty(ATOM1_CH1_P21_3,0);

                gpio_set_level(P21_5,0);
                pwm_set_duty(ATOM1_CH2_P21_4,0);


                pwm_set_duty (ATOM2_CH5_P33_13,4000);
                pwm_set_duty (ATOM1_CH5_P32_4,4000);
            }
            else if (fache==0)
            {
                gpio_set_level(P21_2,0);
                pwm_set_duty(ATOM1_CH1_P21_3,0);

                gpio_set_level(P21_5,0);
                pwm_set_duty(ATOM1_CH2_P21_4,0);

                pwm_set_duty (ATOM2_CH5_P33_13,0);
                pwm_set_duty (ATOM1_CH5_P32_4,0);
            }

            sjjg=system_getval_us();


        }

        // 此处编写需要循环执行的代码
    }
}
#pragma section all restore
// **************************** 代码区域 ****************************
void shuju(void)
{
    seekfree_assistant_oscilloscope_data.data[0] = speedtobel;
    seekfree_assistant_oscilloscope_data.data[1] = speedtober;
    seekfree_assistant_oscilloscope_data.data[2] = (float)speedl;
    seekfree_assistant_oscilloscope_data.data[3] = (float)speedr;
    seekfree_assistant_oscilloscope_data.data[4] = (float)pwmr;
    seekfree_assistant_oscilloscope_data.data[5] = (float)pwml;
    seekfree_assistant_oscilloscope_data.data[6] = (float)duty;
    seekfree_assistant_oscilloscope_data.data[7] = (float)podao;
    seekfree_assistant_oscilloscope_data.channel_num = 8;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    seekfree_assistant_data_analysis();
}
