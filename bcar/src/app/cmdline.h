/**
  ******************************************************************************
  * @file    cmdline.h
  * @brief   串口命令行（行式：收到 \n 或 \r 才算一条）
  *
  * 用途：阶段 2 起"人在电脑前调车"的主要手段 —— 不用重烧固件就能点动电机、
  *       切模式、看状态。上位机（调参迭代器）以后也走这条通路。
  ******************************************************************************
  */
#ifndef __APP_CMDLINE_H
#define __APP_CMDLINE_H

#include <stdint.h>

/** 心跳打印开关（1 = 每秒一行 hello，阶段 1 判据要看；调参时关掉免得刷屏）*/
extern volatile uint8_t g_hello_en;

void cmdline_init(void);

/** @brief 主循环调用：把串口收到的字节攒成行并执行 */
void cmdline_poll(void);

#endif /* __APP_CMDLINE_H */
