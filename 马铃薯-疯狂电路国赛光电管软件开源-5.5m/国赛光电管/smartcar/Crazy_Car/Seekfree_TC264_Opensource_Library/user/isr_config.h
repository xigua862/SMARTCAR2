/*********************************************************************************************************************
 * TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 TC264 开源库的一部分
 *
 * TC264 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          isr_config
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.10.2
 * 适用平台          TC264D
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/

// isr_config.h —— 用 #define 定好每个中断的优先级 + 服务核，供 isr.c 的 IFX_INTERRUPT 使用。
// 改这里任何数字或服务核，都必须回 isr.c 一并核对。
//
// 优先级一览（数值唯一，勿改）：
//   255  ERU_CH0_CH4  IMU 数据就绪（最高）
//   254  ERU_CH3_CH7  预留外部中断
//    33  CCU6_1_CH1   1ms 速度环（硬实时）
//    32  CCU6_1_CH0   预留 PIT
//    31  CCU6_0_CH1   5ms 发车状态机
//    30  CCU6_0_CH0   1ms 软件任务节拍
//    21..10  UART3/2/1/0 的 ER/RX/TX
//    60  通用 DMA 驱动通道中断配置（当前应用未注册 DMA ISR）
//     7   ERU_CH1_CH5  预留（DMA 服务，范围 0-47）
//     6   ERU_CH2_CH6  预留（DMA 服务，范围 0-47）

#ifndef _isr_config_h
#define _isr_config_h

// 中断优先级不能重复。范围 1-255，0 表示关中断，255 最高。
// INT_SERVICE 决定服务核：IfxSrc_Tos_cpu0 / IfxSrc_Tos_cpu1 / IfxSrc_Tos_dma；用 dma 时优先级范围为 0-47。

// ---- PIT 定时器中断（产生 1ms/5ms 任务节拍）----
// CCU6_0 CH0 —— 1ms 软件任务节拍
#define CCU6_0_CH0_INT_SERVICE IfxSrc_Tos_cpu0
#define CCU6_0_CH0_ISR_PRIORITY 30

// CCU6_0 CH1 —— 5ms 发车状态机
#define CCU6_0_CH1_INT_SERVICE IfxSrc_Tos_cpu0
#define CCU6_0_CH1_ISR_PRIORITY 31

// CCU6_1 CH0 —— 预留 PIT
#define CCU6_1_CH0_INT_SERVICE IfxSrc_Tos_cpu0
#define CCU6_1_CH0_ISR_PRIORITY 32

// CCU6_1 CH1 —— 1ms 硬实时速度环
#define CCU6_1_CH1_INT_SERVICE IfxSrc_Tos_cpu0
#define CCU6_1_CH1_ISR_PRIORITY 33

// ---- GPIO/ERU 外部中断（引脚边沿触发）。每两个通道共用一个 ISR，进中断后靠标志位区分 ----
#define EXTI_CH0_CH4_INT_SERVICE IfxSrc_Tos_cpu1
#define EXTI_CH0_CH4_INT_PRIO 255

#define EXTI_CH1_CH5_INT_SERVICE IfxSrc_Tos_dma
#define EXTI_CH1_CH5_INT_PRIO 7                 // DMA 服务，范围 0-47

#define EXTI_CH2_CH6_INT_SERVICE IfxSrc_Tos_dma
#define EXTI_CH2_CH6_INT_PRIO 6                 // DMA 服务，范围 0-47

#define EXTI_CH3_CH7_INT_SERVICE IfxSrc_Tos_cpu0
#define EXTI_CH3_CH7_INT_PRIO 254

// ---- 通用 DMA 驱动配置 ----
// zf_driver_dma.c 始终参与 ADS 构建，即使当前应用不再使用相机 DMA，也必须提供这两个配置宏。
#define DMA_INT_SERVICE IfxSrc_Tos_cpu0
#define DMA_INT_PRIO 5                 // TC264源优先级范围0..47；通用DMA仅作低优先级后台中断

// ---- 串口中断，每口有 TX/RX/ER 三类。数值越大优先级越高（与常见单片机习惯相反）----
#define UART0_INT_SERVICE IfxSrc_Tos_cpu0
#define UART0_TX_INT_PRIO 11
#define UART0_RX_INT_PRIO 10
#define UART0_ER_INT_PRIO 12

#define UART1_INT_SERVICE IfxSrc_Tos_cpu0
#define UART1_TX_INT_PRIO 13
#define UART1_RX_INT_PRIO 14
#define UART1_ER_INT_PRIO 15

#define UART2_INT_SERVICE IfxSrc_Tos_cpu0
#define UART2_TX_INT_PRIO 16
#define UART2_RX_INT_PRIO 17
#define UART2_ER_INT_PRIO 18

#define UART3_INT_SERVICE IfxSrc_Tos_cpu0
#define UART3_TX_INT_PRIO 19
#define UART3_RX_INT_PRIO 20
#define UART3_ER_INT_PRIO 21

#endif
