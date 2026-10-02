#ifndef _cyh_anniu_h
#define _cyh_anniu_h

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

void anniu_init(void);
void get_bianma(void);
void show_information(void);
void show_bianxian(int h);

extern int bianmaanniu,bianmaanniul,anniu;
extern int Lft,Rit,yemian,yemianl;

extern int qiwangchesu,fache;
extern float P_fx,D_fx,jiasu,rood_wid;
extern int dj1,dj2,kfg0,kfg1,yumao,duojizhongzhi;
extern int LC,RC,LMC,RMC,LG,RG;

#endif
