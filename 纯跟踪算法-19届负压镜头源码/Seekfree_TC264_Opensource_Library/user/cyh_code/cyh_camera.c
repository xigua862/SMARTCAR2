
#include "zf_common_headfile.h"
#include "cyh_headfile.h"

// **************************** 代码区域 ****************************

float error=0;
float error_last=0;


uint8 last_threshold=0;
uint8 OTSU_Threshold=0;
float k=5;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逆透视图像
// 参数说明
// 返回参数
// 使用示例     transfer_pic();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

uint8 show[MT9V03X_H][MT9V03X_W];
uint8 image_use[MT9V03X_H][MT9V03X_W];
uint8 image_see[MT9V03X_H][MT9V03X_W];
void nitoushi(void)
{
    for(uint8 i=0;i<MT9V03X_H;i++)
    {
        for(uint8 j=0;j<MT9V03X_W;j++)
        {

            int x=0,y=0;
            x=(int)XShowRotimg(j,i);
            y=(int)YShowRotimg(j,i);

            if (x>=0&&x<MT9V03X_W&&y>=0&&y<MT9V03X_H)
            {

                if (image_use[y][x]>daj)
                {
                    show[i][j]=255;
                }
                else if(image_use[y][x]>daj&&j>=80)
                {
                    show[i][j]=255;
                }
                else
                {
                    show[i][j]=0;
                }
                //show[i][j]=image_use[y][x];
            }
/*
            if (image_use[i][j]>daj)
            {
                show[i][j]=255;
            }
            else
            {
                show[i][j]=0;
            }*/

        }
    }
}



void touxian(float line[][2],int *num)
{
    for (int i=0;i<*num;i++)
    {
        int x=0,y=0;
        x=Xrot_point(line[i][0],line[i][1]);
        y=Yrot_point(line[i][0],line[i][1]);
        if (x>=0&&x<MT9V03X_W&&y>=0&&y<MT9V03X_H)
        {
            line[i][0]=(float)x;
            line[i][1]=(float)y;
        }
        else
        {
            *num=i;
            break;
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     寻找最长白列
// 参数说明
// 返回参数
// 使用示例     get_highest();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

int lwline=74;
int lw=119;
int xxx=0;

void get_highest(void)
{
    lwline=74;
    lw=119;
    xxx=0;

    for (int i=64;i<124;i+=3)
    {
        if(image_use[80][i]>=daj)
        {
            for (int j=85;j>=0;j--)
            {
                if ((image_use[j][i]>=daj&&image_use[j-1][i]<daj&&image_use[j-2][i]<daj)||j<=5)
                {
                    if (j<lw)
                    {
                      lw=j;
                      lwline=i;
                    }
                    break;
                }
            }
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      快速大津
//  @return     uint8
//  @since      v1.1
//  Sample usage:   OTSU_Threshold = otsuThreshold(image_use_dvp[0]);//大津法阈值
//-------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *image)
{
    uint16 width=MT9V03X_W;
    uint16 height=MT9V03X_H-4;
    #define GrayScale 256
    int pixelCount[GrayScale] = {0};//每个灰度值所占像素个数
    float pixelPro[GrayScale] = {0};//每个灰度值所占总像素比例
    int i,j;
    int Sumpix = width * height/4;   //总像素点
    uint8 threshold = 0;

    uint8* data = image;  //指向像素数据的指针


    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将像素值作为计数数组的下标
          //   pixelCount[(int)image[i][j]]++;    若不用指针用这个
        }
    }
    float u = 0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / Sumpix;   //计算每个像素在整幅图像中的比例
        u += i * pixelPro[i];  //总平均灰度
    }


    float maxVariance=0.0;  //最大类间方差
    float w0 = 0, avgValue  = 0;  //w0 前景比例 ，avgValue 前景平均灰度
    for(int i = 0; i < 256; i++)     //每一次循环都是一次完整类间方差计算 (两个for叠加为1个)
    {
        w0 += pixelPro[i];  //假设当前灰度i为阈值, 0~i 灰度像素所占整幅图像的比例即前景比例
        avgValue  += i * pixelPro[i];

        float variance = pow((avgValue/w0 - u), 2) * w0 /(1 - w0);    //类间方差
        if(variance > maxVariance)
        {
            maxVariance = variance;
            threshold = (uint8)i;
        }
    }


    if(threshold>dj1 && threshold<dj2)
        last_threshold = threshold;
    else
        threshold = last_threshold;

    return threshold+kfg0;

}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     八邻域爬线
// 参数说明
// 返回参数
// 使用示例     get_border();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
//========================================================================================
int guize[8][2]={
/*
        {0,-2},
        {2,-2},
        {2,0},
        {2,2},
        {0,2},
        {-2,2},
        {-2,0},
        {-2,-2}
*/
        {0,-1},
        {1,-1},
        {1,0},
        {1,1},
        {0,1},
        {-1,1},
        {-1,0},
        {-1,-1}
                };

int hig=18;
uint8 cbh(int x,int y,int k)
{
    int i=0,j=0;
    i=x+guize[k][0];
    j=y+guize[k][1];

    if (i<=1||i>=MT9V03X_W-1||j>=MT9V03X_H-hig) return 1;
    else if (j<=1||i<=3||i>=MT9V03X_W-4)return 2;
    else if (image_use[j][i]>=daj+(float)kfg1*(float)(60.0-j)/60&&j<60) return 0;
    else if (image_use[j][i]>=daj&&j>=60) return 0;
    else return 1;
}

float L_edge[100][2];     //左边界结构体
float Lout_edge[100][2];     //左边界结构体
float Luse_edge[100][2];
float R_edge[100][2];    //右边界结构体
float Rout_edge[100][2];     //左边界结构体
float Ruse_edge[100][2];
int L_count=0, R_count = 0,Lout_count=0,Rout_count=0;                     //左右边点的个数
int L_amount = 100, R_amount = 100;  //左右边界搜点时最多允许的点

void get_BLY(float L_line[][2],int *Lnum,float R_line[][2],int *Rnum,int len)
{
    hig=len;
    int L_C=0,R_C=0;

    //爬初始点----------------------------------------------

    for (int16 i=(int16)lwline;i>0;i--)
    {
        if (cbh(i,MT9V03X_H-hig-1,6)==1&&cbh(i+1,MT9V03X_H-hig-1,6)==1&&cbh(i+2,MT9V03X_H-hig-1,6)==0)
        {
            L_line[0][0]=i+2;
            break;
        }
        else if (cbh(i+1,MT9V03X_H-hig-1,6)==2)
        {
            L_line[0][0]=i+2;
            break;
        }
    }

    for (int16 i=(int16)lwline;i<MT9V03X_W;i++)
    {
        if (cbh(i,MT9V03X_H-hig-1,2)==1&&cbh(i-1,MT9V03X_H-hig-1,2)==1&&cbh(i-2,MT9V03X_H-hig-1,2)==0)
        {
            R_line[0][0]=i-2;
            break;
        }
        else if (cbh(i-1,MT9V03X_H-hig-1,2)==2)
        {
            R_line[0][0]=i-2;
            break;
        }
    }

    L_line[0][1]=(float)MT9V03X_H-hig-1;
    R_line[0][1]=(float)MT9V03X_H-hig-1;


    L_C=1;
    R_C=1;

    int l_dirl=6,l_dirr=2;
    int dirl=6,dirr=2;
    //左边界------------------------------------------------------

    for (int i=1;i<L_amount;i++)
    {
        L_C++;
        int cishu=0;

        if (cbh(L_line[i-1][0],L_line[i-1][1],0)==2||cbh(L_line[i-1][0],L_line[i-1][1],1)==2||
                cbh(L_line[i-1][0],L_line[i-1][1],2)==2||cbh(L_line[i-1][0],L_line[i-1][1],3)==2||
                cbh(L_line[i-1][0],L_line[i-1][1],4)==2||cbh(L_line[i-1][0],L_line[i-1][1],5)==2||
                cbh(L_line[i-1][0],L_line[i-1][1],6)==2||cbh(L_line[i-1][0],L_line[i-1][1],7)==2) break;


        for (int j=0;j<8;j++)
        {
            if (cbh(L_line[i-1][0],L_line[i-1][1],(j+l_dirl+4)%8)==1&&
                    cbh(L_line[i-1][0],L_line[i-1][1],(j+l_dirl+4+1)%8)==0&&
                    cbh(L_line[i-1][0],L_line[i-1][1],(j+l_dirl+4+2)%8)==0)
            {
                dirl=(j+dirl+4+1)%8;
                break;
            }
            else
            {
                cishu++;
            }

        }
        l_dirl=dirl;

        if (cishu==8)
        {
            break;
        }
        else
        {
            L_line[i][0]=L_line[i-1][0]+guize[dirl][0];
            L_line[i][1]=L_line[i-1][1]+guize[dirl][1];
        }
    }

    //右边界------------------------------------------------------

    for (int i=1;i<R_amount;i++)
    {
        R_C++;
        int cishu=0;

        if (cbh(R_line[i-1][0],R_line[i-1][1],0)==2||cbh(R_line[i-1][0],R_line[i-1][1],1)==2||
                cbh(R_line[i-1][0],R_line[i-1][1],2)==2||cbh(R_line[i-1][0],R_line[i-1][1],3)==2||
                cbh(R_line[i-1][0],R_line[i-1][1],4)==2||cbh(R_line[i-1][0],R_line[i-1][1],5)==2||
                cbh(R_line[i-1][0],R_line[i-1][1],6)==2||cbh(R_line[i-1][0],R_line[i-1][1],7)==2) break;

        for (int j=0;j<8;j++)
        {
            if (cbh(R_line[i-1][0],R_line[i-1][1],(8-j+l_dirr+4)%8)==1&&
                    cbh(R_line[i-1][0],R_line[i-1][1],(8-j+l_dirr+4-1)%8)==0&&
                    cbh(R_line[i-1][0],R_line[i-1][1],(8-j+l_dirr+4-2)%8)==0)
            {
                dirr=(8-j+l_dirr+4-1)%8;
                break;
            }
            else
            {
                cishu++;
            }
        }
        l_dirr=dirr;

        if (cishu==8)
        {
            break;
        }
        else
        {
            R_line[i][0]=R_line[i-1][0]+guize[dirr][0];
            R_line[i][1]=R_line[i-1][1]+guize[dirr][1];
        }
    }

    *Lnum=L_C;
    *Rnum=R_C;
}

int clip(int x, int low, int up) {
    return x > up ? up : x < low ? low : x;
}

float fclip(float x, float low, float up) {
    return x > up ? up : x < low ? low : x;
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取中线
// 参数说明
// 返回参数
// 使用示例     get_mid(输入边线,输出边线,边线点数量);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float Lin_MID[50][2];     //左中线结构体
float Lout_MID[50][2];
float Rin_MID[50][2];     //左中线结构体
float Rout_MID[50][2];
int Lmidnum=0,Lmidnum2=0;
int Rmidnum=0,Rmidnum2=0;

void get_mid_L(float line[][2],int num,float mid[][2],int *num2,int approx_num)
{
    int mnum=1;
    float dist=rood_wid/2;
    mid[0][0]=MT9V03X_W/2-2;
    mid[0][1]=MT9V03X_H-2;
    for (int i = 0; i < num; i++)
    {
        float dx = line[clip(i + approx_num, 0, num - 1)][0] - line[clip(i - approx_num, 0, num - 1)][0];
        float dy = line[clip(i + approx_num, 0, num - 1)][1] - line[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        if (line[i][1] + dx * dist>MT9V03X_H-2) continue;
        else
        {
            mid[mnum][0] = line[i][0] - dy * dist;
            mid[mnum][1] = line[i][1] + dx * dist;
        }
        mnum++;
    }
    *num2=mnum;
}

void get_mid_R(float line[][2],int num,float mid[][2],int *num2,int approx_num)
{
    int mnum=1;
    float dist=rood_wid/2;
    mid[0][0]=MT9V03X_W/2-2;
    mid[0][1]=MT9V03X_H-2;
    for (int i = 0; i < num; i++)
    {
        float dx = line[clip(i + approx_num, 0, num - 1)][0] - line[clip(i - approx_num, 0, num - 1)][0];
        float dy = line[clip(i + approx_num, 0, num - 1)][1] - line[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        if (line[i][1] - dx * dist>MT9V03X_H-2) continue;
        else
        {
            mid[mnum][0] = line[i][0] + dy * dist;
            mid[mnum][1] = line[i][1] - dx * dist;
        }
        mnum++;
    }
    *num2=mnum;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     点集滤波
// 参数说明
// 返回参数
// 使用示例     blur_points(输入点集,输出点集,点集数量,滤波个数);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
// 点集三角滤波
void blur_points(float img_in[][2], float imag_out[][2], int num, int kernel)
{
    int half = kernel / 2;
    for (int i = 0; i < num; i++)
    {
        imag_out[i][0] = imag_out[i][1] = 0;
        for (int j = -half; j <= half; j++)
        {
            int fu=i+j;
            if (fu<0)fu=0;
            if (fu>num-1)fu=num-1;
            imag_out[i][0] += img_in[fu][0] * (half + 1 - abs(j));
            imag_out[i][1] += img_in[fu][1] * (half + 1 - abs(j));
        }
        imag_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
        imag_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     等距采样
// 参数说明
// 返回参数
// 使用示例     resample_points2(输入点集,输入点集数量,输出点集,输出点集数量,点间距);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

void resample_points2(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist)
{
    if (num1 < 0)
    {
        *num2 = 0;
        return;
    }
    pts_out[0][0] = pts_in[0][0];
    pts_out[0][1] = pts_in[0][1];
    int len = 1;
    for (int i = 0; i < num1 - 1 && len < *num2; i++) {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float x1 = pts_in[i + 1][0];
        float y1 = pts_in[i + 1][1];

        do {
            float x = pts_out[len - 1][0];
            float y = pts_out[len - 1][1];

            float dx0 = x0 - x;
            float dy0 = y0 - y;
            float dx1 = x1 - x;
            float dy1 = y1 - y;

            float dist0 = sqrt(dx0 * dx0 + dy0 * dy0);
            float dist1 = sqrt(dx1 * dx1 + dy1 * dy1);

            float r0 = (dist1 - dist) / (dist1 - dist0);
            float r1 = 1 - r0;

            if (r0 < 0 || r1 < 0) break;
            x0 = x0 * r0 + x1 * r1;
            y0 = y0 * r0 + y1 * r1;
            pts_out[len][0] = x0;
            pts_out[len][1] = y0;
            len++;
        } while (len < *num2);

    }
    *num2 = len;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     找拐点
// 参数说明
// 返回参数
// 使用示例     local_angle_points(输入点集,输入点集数量,输出角集);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist)
{
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            angle_out[i] = 0;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

// 角度变化率非极大抑制
void nms_angle(float angle_in[], int num, float angle_out[], int kernel)
{
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) {
            if (fabs(angle_in[clip(i + j, 0, num - 1)]) > fabs(angle_out[i])) {
                angle_out[i] = 0;
                break;
            }
        }
    }
}

float ang_L[50],ang_R[50];
int ang_numL=0,ang_numR=0;
int ifzhi_L=0,ifzhi_R=0;
float conmax=0;

void guai_mum(float Edge[][2], int num, float ang_out[],int *posi,int *if_zhi)
{
    conmax=0;
    *posi=0;
    *if_zhi=1;
    float ang_in[num];
    local_angle_points(Edge,num,ang_in,5);
    nms_angle(ang_in,num,ang_out,10);

        for (int i=0;i<num;i++)
        {
            if (ang_out[i]==0)continue;
            int im1 = clip(i - 5, 0, num - 1);
            int ip1 = clip(i + 5, 0, num - 1);
            float conf = fabs(ang_out[i]) - (fabs(ang_out[im1]) + fabs(ang_out[ip1])) / 2;
            if (conf>1.05&&conf<2)
            {
                *posi=i;
            }
            if (*posi>0)
            {
                break;
            }
        }

    /*最小二乘法判断直道*/
    *if_zhi=0;
    if (num>=25)
    {
        float x1=Edge[2][0];
        float y1=Edge[2][1];
        float x2=Edge[num-3][0];
        float y2=Edge[num-3][1];
        float A=1/(x1-x2);
        float B=1/(y2-y1);
        float C=y2/(y1-y2)-x2/(x1-x2);
        float D=0;

        for (int i=3;i<num-3;i++)
        {
            float x0=Edge[i][0];
            float y0=Edge[i][1];
            D+=fabs(A*x0+B*y0+C)*fabs(A*x0+B*y0+C)/(A*A+B*B)/(num-6);
        }

        conmax=D;

        if (D<=2.5)
        {
            *if_zhi=1;
        }
    }



}
/*
void zhi_if(float Edge[][2], int num,int *if)
{
    *posi=0;
    float ang_in[num];
    local_angle_points(Edge,num,ang_in,3);
    nms_angle(ang_in,num,ang_out,6);
    for (int i=0;i<num;i++)
    {
        if (ang_out[i]==0)continue;
        int im1 = clip(i - 3, 0, num - 1);
        int ip1 = clip(i + 3, 0, num - 1);
        float conf = fabs(ang_out[i]) - (fabs(ang_out[im1]) + fabs(ang_out[ip1])) / 2;
        if ((conf>1.05&&conf<2.22))
        {
            *posi=i;
            break;
        }
    }
}
*/


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取偏差值
// 参数说明
// 返回参数
// 使用示例     get_error();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

void get_error(void)
{
    int qianzhan=yumao;

    if (xunxian_dir==-1)
    {
        if (Lmidnum2>qianzhan)
        {
             float dx=Lout_MID[qianzhan][0]-MT9V03X_W/2+2,dy=MT9V03X_H+20-2-Lout_MID[qianzhan][1];
             error=1*atan2f(dx, dy) * 180 / PI;
        }
        else
        {
            float dx=Lout_MID[Lmidnum2-1][0]-MT9V03X_W/2+2,dy=MT9V03X_H+20-2-Lout_MID[Lmidnum2-1][1];
            error=1*atan2f(dx, dy) * 180 / PI;
        }
    }
    else if (xunxian_dir==1)
    {
        if (Rmidnum2>qianzhan)
        {
             float dx=Rout_MID[qianzhan][0]-MT9V03X_W/2+2,dy=MT9V03X_H+20-2-Rout_MID[qianzhan][1];
             error=1*atan2f(dx, dy) * 180 / PI;
        }
        else
        {
            float dx=Rout_MID[Rmidnum2-1][0]-MT9V03X_W/2+2,dy=MT9V03X_H+20-2-Rout_MID[Rmidnum2-1][1];
            error=1*atan2f(dx, dy) * 180 / PI;
        }
    }


    if (podao==1)
    {
        if (Lmidnum2>qianzhan&&Rmidnum2>qianzhan)
        {
            float dx=(Rout_MID[qianzhan-1][0]+Lout_MID[qianzhan-1][0])/2-MT9V03X_W/2+2,dy=MT9V03X_H+20-2-(Rout_MID[qianzhan-1][1]+Lout_MID[qianzhan-1][1])/2;
            error=1*atan2f(dx, dy) * 180 / PI;
        }
        else
        {
            if (Lmidnum2>Rmidnum2)
            {
                float dx=(Rout_MID[Rmidnum2-1][0]+Lout_MID[Rmidnum2-1][0])/2-MT9V03X_W/2+2,dy=MT9V03X_H+20-2-(Rout_MID[Rmidnum2-1][1]+Lout_MID[Rmidnum2-1][1])/2;
                error=1*atan2f(dx, dy) * 180 / PI;
            }
            else
            {
                float dx=(Rout_MID[Lmidnum2-1][0]+Lout_MID[Lmidnum2-1][0])/2-MT9V03X_W/2+2,dy=MT9V03X_H+20-2-(Rout_MID[Lmidnum2-1][1]+Lout_MID[Lmidnum2-1][1])/2;
                error=1*atan2f(dx, dy) * 180 / PI;
            }
        }
    }



    if ((xunxian_dir==-1&&Lout_count>4)||(xunxian_dir==1&&Rout_count>4))
    {

//        P_fx=Fuzzy_P(error,(error-error_last));
        duty= -P_fx*error-D_fx*(error-error_last);
        duty_chasu=-P_fx*error-5*(error-error_last);
    }

    error_last=error;

    if (duty<-70)
    {
        duty=-70;
    }
    if (duty>70)
    {
        duty=70;
    }

    if (duty_chasu<-70)
    {
        duty_chasu=-70;
    }
    if (duty_chasu>70)
    {
        duty_chasu=70;
    }

}


float Fuzzy_P(float E, float EC)
{
    /*输入量P语言值特征点*/
    float EFF[7] = {-45, -30, -15, 0, 15, 30, 45};    //Error范围

    /*输入量D语言值特征点*/
    float DFF[7] = {-1200, -800,-200 ,0 ,200 ,800 ,1200 };  // 误差变化率范围

    /*输出量U语言值特征点(根据赛道类型选择不同的输出值)*/
    float UFF[7] = {1.1, 1.2, 1.25, 1.3, 0.32, 0.35, 0.4}; // 1.15 2  2.2 2.4 2.6 2.8 3 4}
    // float UFF[7] = {2.0, 2.1, 2.2, 2.4, 2.8, 2.8, 3};
    int rule[7][7] = {
        {6, 5, 4, 3, 2, 1, 0},                           //规则表，不建议修改
        {5, 4, 3, 2, 1, 0, 1},
        {4, 3, 2, 1, 0, 1, 2},
        {3, 2, 1, 0, 1, 2, 3},
        {2, 1, 0, 1, 2, 3, 4},
        {1, 0, 1, 2, 3, 4, 5},
        {0, 1, 2, 3, 4, 5, 6},
    };

    float U = 0;                                 /*偏差,偏差微分以及输出值的精确量*/
    float PF[2] = {0}, DF[2] = {0}, UF[4] = {0}; //偏差,偏差微分以及输出值的隶属度

    int Pn = 0, Dn = 0, Un[4] = {0};
    float t1 = 0, t2 = 0, t3 = 0, t4 = 0, temp1 = 0, temp2 = 0;
    /*隶属度的确定*/
    /*根据PD的指定语言值获得有效隶属度*/
    if (E > EFF[0] && E < EFF[6])
    {
        if (E <= EFF[1])
        {
            Pn = -2;
            PF[0] = (EFF[1] - E) / (EFF[1] - EFF[0]);
        }
        else if (E <= EFF[2])
        {
            Pn = -1;
            PF[0] = (EFF[2] - E) / (EFF[2] - EFF[1]);
        }
        else if (E <= EFF[3])
        {
            Pn = 0;
            PF[0] = (EFF[3] - E) / (EFF[3] - EFF[2]);
        }
        else if (E <= EFF[4])
        {
            Pn = 1;
            PF[0] = (EFF[4] - E) / (EFF[4] - EFF[3]);
        }
        else if (E <= EFF[5])
        {
            Pn = 2;
            PF[0] = (EFF[5] - E) / (EFF[5] - EFF[4]);
        }
        else if (E <= EFF[6])
        {
            Pn = 3;
            PF[0] = (EFF[6] - E) / (EFF[6] - EFF[5]);
        }
    }

    else if (E <= EFF[0])
    {
        Pn = -2;
        PF[0] = 1;
    }
    else if (E >= EFF[6])
    {
        Pn = 3;
        PF[0] = 0;
    }

    PF[1] = 1 - PF[0];

    //判断D的隶属度
    if (EC > DFF[0] && EC < DFF[6])
    {
        if (EC <= DFF[1])
        {
            Dn = -2;
            DF[0] = (DFF[1] - EC) / (DFF[1] - DFF[0]);
        }
        else if (EC <= DFF[2])
        {
            Dn = -1;
            DF[0] = (DFF[2] - EC) / (DFF[2] - DFF[1]);
        }
        else if (EC <= DFF[3])
        {
            Dn = 0;
            DF[0] = (DFF[3] - EC) / (DFF[3] - DFF[2]);
        }
        else if (EC <= DFF[4])
        {
            Dn = 1;
            DF[0] = (DFF[4] - EC) / (DFF[4] - DFF[3]);
        }
        else if (EC <= DFF[5])
        {
            Dn = 2;
            DF[0] = (DFF[5] - EC) / (DFF[5] - DFF[4]);
        }
        else if (EC <= DFF[6])
        {
            Dn = 3;
            DF[0] = (DFF[6] - EC) / (DFF[6] - DFF[5]);
        }
    }
    //不在给定的区间内
    else if (EC <= DFF[0])
    {
        Dn = -2;
        DF[0] = 1;
    }
    else if (EC >= DFF[6])
    {
        Dn = 3;
        DF[0] = 0;
    }

    DF[1] = 1 - DF[0];

    /*使用误差范围优化后的规则表rule[7][7]*/
    /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
    /*一般都是四个规则有效*/
    Un[0] = rule[Pn + 2][Dn + 2];
    Un[1] = rule[Pn + 3][Dn + 2];
    Un[2] = rule[Pn + 2][Dn + 3];
    Un[3] = rule[Pn + 3][Dn + 3];

    if (PF[0] <= DF[0]) //求小
        UF[0] = PF[0];
    else
        UF[0] = DF[0];
    if (PF[1] <= DF[0])
        UF[1] = PF[1];
    else
        UF[1] = DF[0];
    if (PF[0] <= DF[1])
        UF[2] = PF[0];
    else
        UF[2] = DF[1];
    if (PF[1] <= DF[1])
        UF[3] = PF[1];
    else
        UF[3] = DF[1];
    /*同隶属函数输出语言值求大*/
    if (Un[0] == Un[1])
    {
        if (UF[0] > UF[1])
            UF[1] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[2])
    {
        if (UF[0] > UF[2])
            UF[2] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[3])
    {
        if (UF[0] > UF[3])
            UF[3] = 0;
        else
            UF[0] = 0;
    }
    if (Un[1] == Un[2])
    {
        if (UF[1] > UF[2])
            UF[2] = 0;
        else
            UF[1] = 0;
    }
    if (Un[1] == Un[3])
    {
        if (UF[1] > UF[3])
            UF[3] = 0;
        else
            UF[1] = 0;
    }
    if (Un[2] == Un[3])
    {
        if (UF[2] > UF[3])
            UF[3] = 0;
        else
            UF[2] = 0;
    }
    t1 = UF[0] * UFF[Un[0]];
    t2 = UF[1] * UFF[Un[1]];
    t3 = UF[2] * UFF[Un[2]];
    t4 = UF[3] * UFF[Un[3]];
    temp1 = t1 + t2 + t3 + t4;
    temp2 = UF[0] + UF[1] + UF[2] + UF[3]; //模糊量输出
    U = temp1 / temp2;
    return U;
}

