
#ifndef _cyh_headfile_h
#define _cyh_headfile_h

//===================================================cyh∂®“Â≤„===================================================

#define tuchuan 0

#include "cyh_encoder.h"
#include "cyh_duoji.h"
#include "cyh_camera.h"
#include "cyh_motor.h"
#include "Rot.h"
#include "cyh_yuansu.h"
#include "cyh_anniu.h"

void all_int(void);
void fa(int data,int wei);
void wifi_int(void);
void tuchuan_init(void);
extern int tme,tmel,sjjg,daj,over,motortest;

void shuju(void);

#endif
