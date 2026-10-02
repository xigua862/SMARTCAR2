
#include "zf_common_headfile.h"
#include "cyh_headfile.h"

// **************************** 代码区域 ****************************

int sjjg=0,daj=0;
int tme=0,tmel=0;
int over=0;
int motortest=0;

void all_int(void)
{
    //屏幕初始化
    ips200_init(IPS200_TYPE_SPI);
    //ips200_init(IPS200_TYPE_PARALLEL8);


    //按鈕初始化
    anniu_init();

    gpio_init(P33_10, GPO, 0, GPO_PUSH_PULL);

    //电机初始化
    motor_int();

    //摄像头初始化
    mt9v03x_init ();
    mt9v03x_set_exposure_time(100);

    //编码器初始化
    encoder_int();

    //舵机初始化
    duoji_int();


    ips200_clear();

    //按键初始化
    key_init(10);

    //中断初始化
    pit_ms_init(CCU60_CH0,5);
    pit_ms_init(CCU61_CH0,5);


    //无线串口初始化
//    wireless_uart_init();

    //tof初始化
    dl1b_init();

}
