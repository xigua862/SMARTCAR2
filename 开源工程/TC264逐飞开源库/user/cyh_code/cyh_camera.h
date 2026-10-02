
#ifndef _cyh_camera_h
#define _cyh_camera_h

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "zf_common_headfile.h"
#include "cyh_headfile.h"

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------------Type Definitions------------------------------*/
/******************************************************************************/

void get_error(void);


void nitoushi(void);
extern uint8 show[MT9V03X_H][MT9V03X_W];
extern uint8 image_use[MT9V03X_H][MT9V03X_W];
extern uint8 image_see[MT9V03X_H][MT9V03X_W];



void get_mid_L(float line[][2],int num,float mid[][2],int *num2,int approx_num);
void get_mid_R(float line[][2],int num,float mid[][2],int *num2,int approx_num);
extern float Lin_MID[50][2];     //左中线结构体
extern float Lout_MID[50][2];
extern float Rin_MID[50][2];     //左中线结构体
extern float Rout_MID[50][2];
extern int Lmidnum,Lmidnum2;
extern int Rmidnum,Rmidnum2;

void get_highest(void);
extern int lwline,lw;

void blur_points(float img_in[][2], float imag_out[][2], int num, int kernel);
void resample_points2(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist);

extern int hig;

extern int L_count, R_count,L_amount, R_amount,Lout_count,Rout_count;  //左右边界搜点时最多允许的点;
extern float L_edge[100][2];     //左边界结构体
extern float Lout_edge[100][2],Luse_edge[100][2];
extern float R_edge[100][2];    //右边界结构体
extern float Rout_edge[100][2],Ruse_edge[100][2];

void touxian(float line[][2],int *num);


extern float ang_L[50],ang_R[50];
extern int ang_numL,ang_numR;
extern int ifzhi_L,ifzhi_R;
void guai_mum(float Edge[][2], int num, float ang_out[],int *posi,int *if_zhi);
extern float conmax;



uint8 cbh(int x,int y,int k);
void get_BLY(float L_line[][2],int *Lnum,float R_line[][2],int *Rnum,int len);



uint8 otsuThreshold(uint8 *image);
extern uint8 threshold;
extern uint8 OTSU_Threshold;
extern uint8 last_threshold;


extern float error;
extern float error_last;

float Fuzzy_P(float E, float EC);

#endif
