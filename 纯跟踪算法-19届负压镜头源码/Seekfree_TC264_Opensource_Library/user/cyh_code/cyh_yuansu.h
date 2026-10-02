
#ifndef _cyh_yuansu_h
#define _cyh_yuansu_h

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

extern int yuanhuan,dir_yuanhuan,jian,jianms;

extern int shizi;

extern int xunxian_dir;

extern int banmacishu;
extern int qipao;
extern int podao;

extern int luzhang;

void if_banmaxian(void);
void if_yuanhuan(void);
void if_shizi(void);
void search_bianxian(void);
void xunxain_celue(void);
void if_luzhang(void);
void jiansu(void);
void if_po(void);

#endif
