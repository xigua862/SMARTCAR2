#include "zf_common_headfile.h"
#include "cyh_headfile.h"

//*******************************按钮所需变量
int bianmaanniu,bianmaanniul,anniu;
int Lft,Rit;
int yemian,yemianl;

//*******************************菜单所需变量
int jishu,jishul;
int hang;
int facheyanshi;
int duojizhongzhi;

int LC=0,RC=0,LMC=0,RMC=0,LG=0,RG=0;

//********************************调参参数
int fache;
int qiwangchesu;
float P_fx,D_fx,jiasu;
int dj1,dj2,kfg0,kfg1,yumao;

float rood_wid;


// **************************** 代码区域 ****************************
void anniu_init(void)
{
    //按鈕初始化
    gpio_init(P33_12, GPI, 1, GPI_FLOATING_IN);
    gpio_init(P33_8, GPI, 1, GPI_FLOATING_IN);
    gpio_init(P33_11, GPI, 1, GPI_FLOATING_IN);
    encoder_dir_init(TIM5_ENCODER,TIM5_ENCODER_CH1_P21_7,TIM5_ENCODER_CH2_P21_6);
    bianmaanniu=0;
    anniu=0;
    Lft=0;
    Rit=0;
    yemian=0;
    yemianl=0;

    jishu=0;
    jishul=0;
    hang=0;

    //**********************所需参数
    qiwangchesu=250;//235
    P_fx=1.85;//1.56
    D_fx=15;//22.6

    facheyanshi=0;
    fache=0;

    dj1=55;
    dj2=180;

    kfg0=10;
    kfg1=15;

    yumao=18;

    jiasu=0;//5

    duojizhongzhi=760;
    rood_wid=45;
}

void get_bianma(void)
{
    yemianl=yemian;
    bianmaanniul=bianmaanniu;
    jishul=jishu;

    bianmaanniu+=encoder_get_count(TIM5_ENCODER);
    encoder_clear_count(TIM5_ENCODER);


    if (Lft==1&&1-gpio_get_level(P33_11)==0)
    {
        yemian--;
    }
    if (Rit==1&&1-gpio_get_level(P33_8)==0)
    {
        yemian++;
    }

    if (anniu==1&&1-gpio_get_level(P33_12)==0)
    {
        jishu++;
    }

    anniu=1-gpio_get_level(P33_12);
    Lft=1-gpio_get_level(P33_11);
    Rit=1-gpio_get_level(P33_8);

    if (yemian<0)
    {
        yemian=3;
    }
    if (yemian>3)
    {
        yemian=0;
    }

    if (jishu<0)
    {
        jishu=1;
    }
    if (jishu>1)
    {
        jishu=0;
    }

    if (bianmaanniu<0)
    {
        bianmaanniu=8;
    }

    if (bianmaanniu>8)
    {
        bianmaanniu=0;
    }
}



void show_information(void)
{
    if (yemianl!=yemian)
    {
        jishu=0;
        bianmaanniu=0;
        ips200_clear();
    }
        //************************************第一页显示赛道信息
        //************************************第一页显示赛道信息
        //************************************第一页显示赛道信息
        //************************************第一页显示赛道信息
        //************************************第一页显示赛道信息
        //************************************第一页显示赛道信息
    if (yemian==0)
    {
//            nitoushi();
        ips200_show_string(0,145,"LC:");
        ips200_show_int(40,145,LC,2);
        ips200_show_string(65,145,"RC:");
        ips200_show_int(105,145,RC,2);
        //显示边线中线拐点
        show_bianxian(170);



            ips200_show_gray_image(0,0, image_see[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
            ips200_show_gray_image(0, 170, show[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);

            //显示边线中线拐点
            show_bianxian(170);

            ips200_show_string(230,300,"1");



            ips200_show_string(190,0,"stl:");
            ips200_show_int(190,15,speedtobel,3);
            ips200_show_string(190,30,"str:");
            ips200_show_int(190,45,speedtober,3);
            ips200_show_string(190,60,"sl:");
            ips200_show_int(190,75,speedl,3);
            ips200_show_string(190,90,"sr:");
            ips200_show_int(190,105,speedr,5);

            ips200_show_string(65,125,"mm:");
            ips200_show_int(105,125,dl1b_distance_mm,5);
            ips200_show_string(0,125,"dir:");
            ips200_show_int(40,125,xunxian_dir,2);

            ips200_show_string(190,170,"podao:");
            ips200_show_int(190,190,podao,3);

            ips200_show_string(190,210,"LG:");
            ips200_show_int(190,230,LG,3);

            ips200_show_string(190,250,"RG:");
            ips200_show_int(190,270,RG,3);

            /*
            ips200_show_string(0,125,"duty:");
            ips200_show_int(40,125,duty,2);*/

            ips200_show_string(150,125,"sjjg");
            ips200_show_int(185,125,sjjg,5);
            ips200_show_string(160,145,"daj");
            ips200_show_int(190,145,daj,5);

    }
    //************************************第二页显示菜单信息
    //************************************第二页显示菜单信息
    //************************************第二页显示菜单信息
    //************************************第二页显示菜单信息
    //************************************第二页显示菜单信息
    //************************************第二页显示菜单信息
    //************************************第二页显示菜单信息
    //************************************第二页显示菜单信息
    else if (yemian==1)
    {
        ips200_show_string(230,300,"2");
        //********************************************显示第一行信息


        if (jishu==0)
        {
            //二值化图像显示
            nitoushi();
            ips200_show_gray_image(0, 200, show[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
            //显示中线边线拐点
            show_bianxian(190);
        }




        ips200_show_string(75,0,"bmcs:");
        ips200_show_int(135,0,banmacishu,2);
        ips200_show_string(0,0,"daj:");
        ips200_show_int(50,0,daj-kfg0,2);

        ips200_show_string(165,0,"dy:");
        ips200_show_int(190,0,duty,5);

        //*********************************************显示箭头
        if (yemianl!=1)
        {
            ips200_show_string(90,20,"        ");
            ips200_show_string(90,20,"<---");
        }
        if (jishu==0&&bianmaanniul!=bianmaanniu)
        {
            ips200_show_string(90,bianmaanniul*20+20,"        ");
            ips200_show_string(90,bianmaanniu*20+20,"<---");
        }

        if (jishu==1&&jishul==0)
        {
            hang=bianmaanniu+1;
            ips200_show_string(90,bianmaanniu*20+20,"         ");
            ips200_show_string(90,bianmaanniu*20+20,"<---<---");
        }

        if (jishu==0&&jishul==1)
        {
            ips200_show_string(90,hang*20,"        ");
            ips200_show_string(90,hang*20,"<---");
            bianmaanniu=hang-1;
            bianmaanniul=hang-1;
        }

        //****************************************************调参
        if (jishu==1)
        {
            if (hang==1)
            {
                if (bianmaanniu-bianmaanniul==1||bianmaanniu-bianmaanniul==-1)
                qiwangchesu+=bianmaanniu-bianmaanniul;
            }
            if (hang==2)
            {
                if (bianmaanniu-bianmaanniul==1||bianmaanniu-bianmaanniul==-1)
                    P_fx+=(float)(bianmaanniu-bianmaanniul)/10;
            }
            if (hang==3)
            {
                if (bianmaanniu-bianmaanniul==1||bianmaanniu-bianmaanniul==-1)
                    D_fx+=(float)(bianmaanniu-bianmaanniul)/10;
            }
            if (hang==4)
            {
                if (bianmaanniu-bianmaanniul==1||bianmaanniu-bianmaanniul==-1)
                    dj1+=bianmaanniu-bianmaanniul;
            }
            if (hang==5)
            {
                if (bianmaanniu-bianmaanniul==1||bianmaanniu-bianmaanniul==-1)
                    dj2+=bianmaanniu-bianmaanniul;
            }
            if (hang==6)
            {
                if (bianmaanniu-bianmaanniul==1||bianmaanniu-bianmaanniul==-1)
                    duojizhongzhi+=bianmaanniu-bianmaanniul;
            }
            if (hang==7)
            {
                if (bianmaanniu-bianmaanniul==1||bianmaanniu-bianmaanniul==-1)
                    rood_wid+=bianmaanniu-bianmaanniul;
            }
            if (hang==8)
            {
                if (bianmaanniu-bianmaanniul==1||bianmaanniu-bianmaanniul==-1)
                    yumao+=bianmaanniu-bianmaanniul;
            }
            if (hang==9)
            {
                if (bianmaanniu-bianmaanniul==1||bianmaanniu-bianmaanniul==-1)
                    jiasu+=(float)(bianmaanniu-bianmaanniul)/10;
            }
        }

        //**************************************************显示参数
        ips200_show_string(0,20,"chesu:");
        ips200_show_int(50,20,qiwangchesu,3);
        ips200_show_string(0,40,"P_fx:");
        ips200_show_float(50,40,P_fx,2,1);
        ips200_show_string(0,60,"D_fx:");
        ips200_show_float(50,60,D_fx,2,1);
        ips200_show_string(0,80,"dj1:");
        ips200_show_int(50,80,dj1,2);
        ips200_show_string(0,100,"dj2:");
        ips200_show_int(50,100,dj2,2);
        ips200_show_string(0,120,"djzz");
        ips200_show_int(50,120,duojizhongzhi,3);
        ips200_show_string(0,140,"rwid:");
        ips200_show_int(50,140,rood_wid,2);
        ips200_show_string(0,160,"yumao:");
        ips200_show_int(50,160,yumao,2);
        ips200_show_string(0,180,"jiasu:");
        ips200_show_float(50,180,jiasu,2,1);
    }
    //************************************第三页显示发车信息
    //************************************第三页显示发车信息
    //************************************第三页显示发车信息
    //************************************第三页显示发车信息
    //************************************第三页显示发车信息
    //************************************第三页显示发车信息
    else if(yemian==2)
    {
        if (fache==0)
        {
            ips200_show_string(230,300,"3");

            //二值化图像显示
            nitoushi();
            ips200_show_gray_image(0, 190, show[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
            //显示中线边线拐点
            show_bianxian(190);

            ips200_show_string(0,0,"shizi:");
            ips200_show_int(50,0,shizi,2);
            ips200_show_string(75,0,"huan:");
            ips200_show_int(120,0,yuanhuan,2);
            ips200_show_string(150,0,"duty:");
            ips200_show_int(190,0,duty,5);

            ips200_show_string(0,20,"fache:");
            ips200_show_int(50,20,fache,2);
            ips200_show_string(75,20,"ifzhiL:");
            ips200_show_int(120,20,ifzhi_L,2);
            ips200_show_string(150,20,"ifzhiR:");
            ips200_show_int(190,20,ifzhi_R,2);

            ips200_show_string(0,40,"daj:");
            ips200_show_int(50,40,daj,3);
            ips200_show_string(75,40,"qipao:");
            ips200_show_int(120,40,qipao,3);
            ips200_show_string(150,40,"bmc:");
            ips200_show_int(190,40,banmacishu,2);
        }
        else
        {
            ips200_show_string(0,20,"fache:");
            ips200_show_int(50,20,fache,2);
        }


        if (jishu!=jishul)
        {
            facheyanshi=0;
            fache=1;

            qipao=0;
            banmacishu=0;

            yuanhuan=0;
            shizi=0;
            luzhang=0;
        }

        //************************************发车延时
        if (fache==1&&facheyanshi<1000)
        {
            facheyanshi++;
        }
        if (facheyanshi>=1000)
        {
            fache=2;
            yemian=3;
            jishu=0;
            bianmaanniu=0;
            ips200_clear();
        }
    }
    else if (yemian==3)
    {
        if (jishu!=jishul)
        {
            facheyanshi=0;
            fache=0;
            jishu=0;
        }
    }
}


void show_bianxian(int h)
{
    //****************边线显示

    for (int i=0;i<LC;i++)
    {
        uint16 x=(uint16)Luse_edge[i][0];
        uint16 y=(uint16)Luse_edge[i][1]+h;
        if (x>0&&x<188&&y>h&&y<h+120)
        {
            ips200_draw_point((uint16)Luse_edge[i][0], (uint16)Luse_edge[i][1]+h, (uint16)RGB565_GREEN);
        }
    }
    for (int i=0;i<RC;i++)
    {
        uint16 x=(uint16)Ruse_edge[i][0];
        uint16 y=(uint16)Ruse_edge[i][1]+h;
        if (x>0&&x<188&&y>h&&y<h+120)
        {
            ips200_draw_point((uint16)Ruse_edge[i][0], (uint16)Ruse_edge[i][1]+h, (uint16)RGB565_BLUE);
        }
    }
/*
    for (int i=0;i<L_count;i++)
    {
        uint16 x=(uint16)L_edge[i][0];
        uint16 y=(uint16)L_edge[i][1]+170;
        if (x>0&&x<188&&y>170&&y<290)
        {
            ips200_draw_point((uint16)L_edge[i][0], (uint16)L_edge[i][1]+h, (uint16)RGB565_GREEN);
        }
    }
    for (int i=0;i<R_count;i++)
    {
        uint16 x=(uint16)R_edge[i][0];
        uint16 y=(uint16)R_edge[i][1]+170;
        if (x>0&&x<188&&y>170&&y<290)
        {
            ips200_draw_point((uint16)R_edge[i][0], (uint16)R_edge[i][1]+h, (uint16)RGB565_BLUE);
        }
    }*/


    //******************拐点显示
    if (LG!=0)
    {
        for (int i=-2;i<=2;i++)
        {
            for (int j=-2;j<=2;j++)
            {
                if (Luse_edge[ang_numL][0]+i>0&&Luse_edge[ang_numL][0]+i<188&&Luse_edge[ang_numL][1]+j>0&&Luse_edge[ang_numL][1]+j<120)
                {
                    ips200_draw_point((uint16)Luse_edge[ang_numL][0]+i, (uint16)Luse_edge[ang_numL][1]+h+j, (uint16)RGB565_RED);
                }
            }
        }
    }

    if (RG!=0)
    {
        for (int i=-2;i<=2;i++)
        {
            for (int j=-2;j<=2;j++)
            {
                if (Ruse_edge[ang_numR][0]+i>0&&Ruse_edge[ang_numR][0]+i<188&&Ruse_edge[ang_numR][1]+j>0&&Ruse_edge[ang_numR][1]+j<120)
                {
                    ips200_draw_point((uint16)Ruse_edge[ang_numR][0]+i, (uint16)Ruse_edge[ang_numR][1]+h+j, (uint16)RGB565_RED);
                }
            }
        }
    }

    //****************重采样后中线显示
    for (int i=0;i<LMC;i++)
    {
        uint16 x=(uint16)Lout_MID[i][0];
        uint16 y=(uint16)Lout_MID[i][1]+h;
        if (x>0&&x<188&&y>h&&y<h+120)
        {
            ips200_draw_point((uint16)Lout_MID[i][0], (uint16)Lout_MID[i][1]+h, (uint16)RGB565_BROWN);
        }

    }
    for (int i=0;i<RMC;i++)
    {
        uint16 x=(uint16)Rout_MID[i][0];
        uint16 y=(uint16)Rout_MID[i][1]+h;
        if (x>0&&x<188&&y>h&&y<h+120)
        {
            ips200_draw_point((uint16)Rout_MID[i][0], (uint16)Rout_MID[i][1]+h, (uint16)RGB565_PURPLE);
        }
    }
}
