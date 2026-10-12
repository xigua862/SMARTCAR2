/**
  ******************************************************************************
  * @file    bsp_afio.h
  * @brief   AFIO 复用功能配置（寄存器级）
  *
  * 为什么单开一个文件：CubeMX 不会替你「禁 JTAG 留 SWD」，也不会替你写
  * 重映射，而这两件事错了会表现成「程序在跑、外设全死」，极难查。
  * 所以集中在这里，并在上电时把 MAPR 打出来自证。
  ******************************************************************************
  */
#ifndef __BSP_AFIO_H
#define __BSP_AFIO_H

#include <stdint.h>

/**
  * @brief  一次性配好三件事：
  *         1) 关闭 JTAG、保留 SWD（PA13/PA14 留给下载器）
  *            —— 只有这样 PA15 / PB3 / PB4 才能当普通 IO（本板给了左编码器）
  *         2) USART1 重映射到 PB6(TX) / PB7(RX)
  *            —— 不重映射的话串口就是"哑巴"，发不出来也收不到
  *         3) 打开 AFIO 时钟（上面两件事的前提）
  * @note   必须在任何 GPIO/串口初始化之前调用。
  */
void bsp_afio_init(void);

/** @brief 读回 AFIO->MAPR 当前值（上电日志打印用，自证配置生效）*/
uint32_t bsp_afio_mapr(void);

/** @brief 把 MAPR 值按位解释成人话（返回静态字符串，给日志用）*/
const char *bsp_afio_mapr_explain(uint32_t mapr);

#endif /* __BSP_AFIO_H */
