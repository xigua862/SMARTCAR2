/**
  ******************************************************************************
  * @file    adc_ir.h
  * @brief   8 路灰度（红外）采集：ADC1 扫描 + DMA 循环搬运
  *
  * 接线（见 文档\引脚图-B车C8T6-20261002.html）：
  *   模块 A0~A7 → PA0~PA7（= ADC1_IN0~IN7）
  *   ⚠ 模块供 5V ⇒ 输出是 5V 电平 ⇒ 每路板上有 1k/1.5k 分压（→ 约 3.0V 满量程）
  *
  * 为什么用"扫描 + DMA"而不是一路一路问：
  *   8 次 12 位转换 ≈ 17µs，且通道间**零偏斜**（同时刻采样）；
  *   比"选通式"（400µs、有偏斜）快一个数量级 —— 这正是选这颗模块的理由。
  *   DMA 循环模式意味着 CPU 一个字节都不用搬，想读直接读数组。
  ******************************************************************************
  */
#ifndef __DRV_ADC_IR_H
#define __DRV_ADC_IR_H

#include <stdint.h>

#define ADC_IR_CH_COUNT  8u   /* 8 路，对应 PA0~PA7 */

/**
  * @brief  初始化 ADC1 8 通道扫描 + DMA1 通道1 循环搬运
  * @note   会做一次 ADC 自校准（不做的话读数有固定偏差）
  */
void adc_ir_init(void);

/** @brief 读某一路原始值（12 位，0~4095）。⚠ DMA 在后台更新，读到的是一组的快照 */
uint16_t adc_ir_raw(uint8_t ch);

/** @brief 一次拿到 8 个原始值的数组首地址（DMA 直接往里写）*/
const uint16_t *adc_ir_raw_all(void);

/** @brief DMA 已完成的"一轮 8 通道"计数（诊断：不动它也应该在涨）*/
uint32_t adc_ir_seq_count(void);

/** @brief DMA1_Channel1 中断入口调用（每搬完一轮 8 通道触发一次）*/
void adc_ir_dma_isr(void);

#endif /* __DRV_ADC_IR_H */
