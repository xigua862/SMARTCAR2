
#include "zf_common_headfile.h"
#include "cyh_headfile.h"

// **************************** 代码区域 ****************************
int pwml,pwmr;


//速度环
int pofmada;
int chesu;
float speedtobel,speedtober;
float dl,dr;

int pwmrpl;
/* 原装电机
float KP_L = 75, KI_L = 12, KD_L = 50;//7.2v：80,25，30
float KP_R = 75, KI_R = 12, KD_R = 50;//200：190，30，10
*/

/*7.4v
float KP_L = 50, KI_L = 45, KD_L = 40;//7.2v：80,25，30
float KP_R = 50, KI_R = 45, KD_R = 40;//200：190，30，10
*/

/*小日子电机
float KP_L = 45, KI_L = 15, KD_L = 0;//7.2v：30,25，20
float KP_R = 45, KI_R = 15, KD_R = 0;//12v：190，30，10
*/

/*M车电机
float KP_L = 120, KI_L = 8, KD_L = 4;//7.2v：30,25，20
float KP_R = 120, KI_R = 8, KD_R = 4;//12v：190，30，10
*/

/*龙邱电机*/
float KP_L = 45, KI_L = 5, KD_L = 0;//7.2v：30,25，20
float KP_R = 45, KI_R = 5, KD_R = 0;//12v：190，30，10

float L_err[3] = {0, 0, 0};
float R_err[3] = {0, 0, 0};
float L_sum = 0;
float R_sum = 0;
float PIDL;
float PIDR;


//累加值限幅
#define A (4000)
//占空比限幅
#define C (9000)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机初始化
// 参数说明
// 返回参数
// 使用示例     encoder_int();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

void motor_int(void)
{
    pwm_init(ATOM1_CH1_P21_3,17000,0);    //左轮
    pwm_init(ATOM1_CH2_P21_4,17000,0);    //右轮


    gpio_init(P21_2, GPO, 0, GPO_PUSH_PULL);
    gpio_init(P21_5, GPO, 0, GPO_PUSH_PULL);

         pwml=2200;
         pwmr=2200;
}


/*******************************************************************************
 * 函 数 名       :madasudu()
 * 函数功能        :占空比计算
 * 输    入       : 无
 * 输    出        : 无
 *******************************************************************************/
//封装马达转速调节 (现在转速，期望转速)
void madasudu(void)
{

//    C  -----------------------------------------------------------------------
    if (banmacishu>=2)
    {
        chesu=0;
    }
    else if (Lmidnum>=3||Rmidnum>=3)
    {
        if (jian==0)
        {
            chesu=qiwangchesu;
        }
        else if (lw<=12&&luzhang==0&&(yuanhuan==0||yuanhuan==3))
        {
            chesu=qiwangchesu+jiasu*(20-lw);
        }
        else
        {
            chesu=qiwangchesu;
        }
    }
    else
    {
        chesu=0;
    }

    if (podao==1)
    {
        chesu=200;
    }

    if (duty>0)
    {
       speedtobel=chesu*(float)(1-0*(float)(duty_chasu)/500);
       speedtober=chesu*(float)(1+0*(float)(duty_chasu)/500);
    }
    else
    {
        speedtobel=chesu*(float)(1-0*(float)(duty_chasu)/500);
        speedtober=chesu*(float)(1+0*(float)(duty_chasu)/500);
    }

//    ------------------------------------------------------------------------------



//    测试速度环------------------------------------------------------------------------
    /*
    if (motortest<=200)
     {
         speedtobel=(200-(float)(motortest-200)*(float)(motortest-200)/400);
         speedtober=0;
     }
     else if (motortest<=400)
     {
         speedtobel=(200-(float)(motortest-200)*(float)(motortest-200)/400);
         speedtober=0;
     }
    if (motortest>=400)
    {
        motortest=0;
    }
*/

//    ------------------------------------------------------------------------------



    L_err[2] = L_err[1];
    L_err[1] = L_err[0];
    L_err[0] = ((speedtobel - speedl)-dl);
    L_sum = speedtobel - speedl;
    dl=speedtobel - speedl;

    R_err[2] = R_err[1];
    R_err[1] = R_err[0];
    R_err[0] = ((speedtober - speedr)-dr);
    R_sum = speedtober - speedr;
    dr=speedtober - speedr;

    //左右轮pid
    PIDL=KP_L * L_err[0] + KI_L * L_sum + KD_L * (L_err[0] - L_err[1]);
    PIDR=KP_R * R_err[0] + KI_R * R_sum + KD_R * (R_err[0] - R_err[1]);


    //左右轮单次限幅
    if (PIDL > A) // L_sum>1000 L_sum=1000
        PIDL = A;
    if (PIDL < -A)
        PIDL = -A;
    //左轮限幅
    if (PIDR > A) // L_sum>1000 L_sum=1000
        PIDR = A;
    if (PIDR < -A)
        PIDR = -A;


    //C----------------------------------------------------------------------


    if (speedl>100)
    {
        pwml+=PIDL;
    }
    else
    {
        pwml+=PIDL;
    }

    if (speedr>100)
    {
        pwmr+=PIDR;
    }
    else
    {
        pwmr+=PIDR;
    }


//--------------------------------------------------------------------------

  if (pwml<-C)
    {
      pwml=-C;
    }
    else if (pwml>C)
    {
        pwml=C;
    }

    if (pwmr<-C)
    {
        pwmr=-C;
    }
    else if (pwmr>C)
    {
        pwmr=C;
    }


    pwmrpl+=(PIDL+PIDR)/2;
    if (pwmrpl<-C)
    {
        pwmrpl=-C;
    }
    else if (pwmrpl>C)
    {
        pwmrpl=C;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     马达占空比输出
// 参数说明
// 返回参数
// 使用示例     motor_out();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------


void motor_out(void)
{


    //左轮
        if(pwml<=-10000)       pwml=-10000;
        if(pwml>=10000)        pwml=10000;
//        pwml=2000;
//        pwmr=2000;

/*
        if (pwml>=0)
        {
            gpio_set_level(P21_2,0);
            pwm_set_duty(ATOM1_CH1_P21_3,pwml);
        }
        else if (pwml<0)
        {
            gpio_set_level(P21_2,1);
            pwm_set_duty(ATOM1_CH1_P21_3,-pwml);
        }*/

        //右轮
        if(pwmr<=-10000)   pwmr=-10000;
        if(pwmr>=10000)    pwmr=10000;
        /*
        if (pwmr>=0)
        {
            gpio_set_level(P21_5,0);
            pwm_set_duty(ATOM1_CH2_P21_4,pwmr);
        }
        else if (pwmr<0)
        {
            gpio_set_level(P21_5,1);
            pwm_set_duty(ATOM1_CH2_P21_4,-pwmr);
        }*/

        if(pwmrpl<=-10000)   pwmrpl=-10000;
        if(pwmrpl>=10000)    pwmrpl=10000;

        int pwmrpll=pwmrpl-duty_chasu*0;
        int pwmrplr=pwmrpl+duty_chasu*0;

        if(pwmrpll<=-10000)   pwmrpll=-10000;
        if(pwmrpll>=10000)    pwmrpll=10000;


        if(pwmrplr<=-10000)   pwmrplr=-10000;
        if(pwmrplr>=10000)    pwmrplr=10000;

        if (pwmrpll>=0)
        {
            gpio_set_level(P21_2,0);
            pwm_set_duty(ATOM1_CH1_P21_3,pwmrpll);
        }
        else if (pwmrpll<0)
        {
            gpio_set_level(P21_2,1);
            pwm_set_duty(ATOM1_CH1_P21_3,-pwmrpll);
        }

        if (pwmrplr>=0)
        {
            gpio_set_level(P21_5,0);
            pwm_set_duty(ATOM1_CH2_P21_4,pwmrplr);
        }
        else if (pwmrplr<0)
        {
            gpio_set_level(P21_5,1);
            pwm_set_duty(ATOM1_CH2_P21_4,-pwmrplr);
        }


        /*测试
        gpio_set_level(P21_2,0);
        pwm_set_duty(ATOM1_CH1_P21_3,2000);

        gpio_set_level(P21_5,0);
        pwm_set_duty(ATOM1_CH2_P21_4,2000);
        */
}


