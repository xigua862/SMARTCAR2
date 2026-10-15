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

/** @brief 5ms 拍调用：处理"陀螺零偏标定"这类需要跨多拍采样的命令 */
void cmdline_tick_5ms(void);

/**
  * @brief  在发车读秒里自动标一次陀螺零偏（main 进 COUNTDOWN 时调）
  * @note   读秒那 3 秒车本来就刹着不动，正好采 1 秒静止样本把零偏标掉；
  *         采集期间若车在动，本次**放弃**标定并说明原因（见 bcar_config.h 的两个开关）。
  */
void cmdline_gyrozero_auto_begin(void);

#endif /* __APP_CMDLINE_H */
