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
* 文件名称          cpu0_main
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
#pragma section all "cpu0_dsram"

// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

#define WIFI_SSID_TEST          "MIFI_D7DEF0"
#define WIFI_PASSWORD_TEST      "1234567890"  // 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL



// **************************** 代码区域 ****************************
uint8 geshi[]="channels: ";
uint8 fasong[6]="00000,";
uint8 fasongwei[7]="00000\n";

int core0_main(void)
{
 //   emembasic_init();
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等

    if (tuchuan==1)
    {
        wifi_int();
    }

    all_int();

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        //----------------------------------------------------
        if (tme==1&&tmel==0)
        {
            tme=0;
            key_scanner();
            get_bianma();
            show_information();
            daj=otsuThreshold(image_see[0]);


            if (tuchuan==1)
            {
                // 这里仅写入4个通道数据
                seekfree_assistant_oscilloscope_data.data[0] = (float)speedl;
                seekfree_assistant_oscilloscope_data.data[1] = speedtobel;
                seekfree_assistant_oscilloscope_data.data[2] = (float)pwml;
                seekfree_assistant_oscilloscope_data.data[3] = (float)speedr;
                seekfree_assistant_oscilloscope_data.data[4] = speedtober;
                seekfree_assistant_oscilloscope_data.data[5] = (float)pwmr;

            //        detector_oscilloscope_data.data[4] = 10;
            //        detector_oscilloscope_data.data[5] = 100;
            //        detector_oscilloscope_data.data[6] = 1000;
            //        detector_oscilloscope_data.data[7] = 10000;

                // 设置本次需要发送几个通道的数据
                seekfree_assistant_oscilloscope_data.channel_num = 6;

                // 这里进发送了4个通道的数据，最大支持8通道
                seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
            }


        }

        //----------------------------------------------------
        // 此处编写需要循环执行的代码
    }
}

#pragma section all restore


// **************************** 代码区域 ****************************

void fa(int data,int wei)
{


    if (wei==0)
    {
        if (data>=0)
        {
           fasong[0]='+';
           fasong[1]=(int)data/1000+'0';
           fasong[2]=(int)data/100%10+'0';
           fasong[3]=(int)data/10%10+'0';
           fasong[4]=(int)data%10+'0';
        }
        else
        {
           fasong[0]='-';
           fasong[1]=(int)-data/1000+'0';
           fasong[2]=(int)-data/100%10+'0';
           fasong[3]=(int)-data/10%10+'0';
           fasong[4]=(int)-data%10+'0';
        }

        wireless_uart_send_buffer(fasong, sizeof(fasong));
    }
    else
    {
        if (data>=0)
        {
            fasongwei[0]='+';
            fasongwei[1]=(int)data/1000+'0';
            fasongwei[2]=(int)data/100%10+'0';
            fasongwei[3]=(int)data/10%10+'0';
            fasongwei[4]=(int)data%10+'0';
        }
        else
        {
            fasongwei[0]='-';
            fasongwei[1]=(int)-data/1000+'0';
            fasongwei[2]=(int)-data/100%10+'0';
            fasongwei[3]=(int)-data/10%10+'0';
            fasongwei[4]=(int)-data%10+'0';
        }
        wireless_uart_send_buffer(fasongwei, sizeof(fasongwei));
    }
}

void wifi_int(void)
{

    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // 初始化失败 等待 100ms
    }

    printf("\r\n module version:%s",wifi_spi_version);                          // 模块固件版本
    printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // 模块 MAC 信息
    printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // 模块 IP 地址


    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接
    if(1 != WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                              // 指定使用TCP方式通讯
            WIFI_SPI_TARGET_IP,                                                 // 指定远端的IP地址，填写上位机的IP地址
            WIFI_SPI_TARGET_PORT,                                               // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_SPI_LOCAL_PORT))                                               // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
        }
    }


    // 逐飞助手初始化 数据传输使用高速WIFI SPI
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
}


