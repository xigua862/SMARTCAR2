/**
  ******************************************************************************
  * @file    adc_ir.c
  * @brief   8 路灰度 ADC1 扫描 + DMA 循环 实现（寄存器级）
  ******************************************************************************
  */
#include "adc_ir.h"
#include "bsp_gpio.h"
#include "stm32f1xx.h"

/* 采样时间选 13.5 周期：分压源阻抗约 600Ω，很松裕。
 * 一轮耗时 ≈ 8 × (13.5 + 12.5) = 208 周期 ≈ 17µs @12MHz */

static volatile uint16_t s_raw[ADC_IR_CH_COUNT];
static volatile uint32_t s_seq = 0u;   /* DMA 每搬完一轮 8 通道 +1 */

/** @brief DMA1_Channel1 中断入口调用（每搬完一轮 8 通道触发一次）*/
void adc_ir_dma_isr(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CTCIF1;   /* 写 1 清标志 */
        s_seq++;
    }
}

void adc_ir_init(void)
{
    uint8_t  i;
    uint32_t smpr = 0u;

    /* 1) 时钟：ADC1 在 APB2、DMA1 在 AHB、GPIOA 在 APB2 */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
    (void)RCC->APB2ENR;
    (void)RCC->AHBENR;

    /* 2) ADC 时钟 = PCLK2 / 6 = 12MHz（F1 上限 14MHz，不能直接用 72MHz）*/
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;

    /* 3) PA0~PA7 配成模拟输入（模拟输入 = CNF=00、MODE=00，不要上下拉）*/
    for (i = 0u; i < 8u; i++) {
        GPIOA->CRL = (GPIOA->CRL & ~(0xFu << ((uint32_t)i * 4u)));   /* 全清零即模拟输入 */
    }

    /* 4) 自校准（不做的话每个通道都有固定偏移，标定白/黑时就歪了）*/
    ADC1->CR2 |= ADC_CR2_ADON;      /* 先上电 */
    {
        volatile uint32_t d;
        for (d = 0u; d < 2000u; d++) { }   /* 等 tSTAB（约 1µs，给足余量）*/
    }
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL) { }
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL) { }

    /* 5) 规则组：8 个通道，顺序 IN0..IN7。
     *    L[23:20] = 7 表示这一组有 8 次转换；通道号按 SQR3→SQR2→SQR1 依次填 */
    ADC1->SQR1 = (7u << 20);
    ADC1->SQR2 = 0u;
    ADC1->SQR3 = (0u << 0) | (1u << 5) | (2u << 10) | (3u << 15)
               | (4u << 20) | (5u << 25);
    ADC1->SQR2 = (6u << 0) | (7u << 5);

    /* 6) 采样时间：每路 13.5 周期（SMPR1 管通道 0~9，每路 3 位）*/
    smpr = 0u;
    for (i = 0u; i < 8u; i++) {
        smpr |= (2u << ((uint32_t)i * 3u));   /* 2 = 13.5 cycles */
    }
    ADC1->SMPR1 = smpr;
    ADC1->SMPR2 = 0u;

    /* 7) DMA1 通道1 ← ADC1：外设→内存、16 位、内存地址自增、循环模式 */
    DMA1_Channel1->CCR   = 0u;                       /* 先关，改配置 */
    DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR  = (uint32_t)s_raw;
    DMA1_Channel1->CNDTR = ADC_IR_CH_COUNT;
    DMA1_Channel1->CCR   = DMA_CCR_PL_1              /* 优先级高 */
                         | DMA_CCR_MSIZE_0           /* 内存 16 位 */
                         | DMA_CCR_PSIZE_0           /* 外设 16 位 */
                         | DMA_CCR_MINC              /* 内存地址自增 */
                         | DMA_CCR_CIRC              /* 循环：搬完自动回卷 */
                         | DMA_CCR_TCIE              /* 搬完一轮进中断（只用来计数，不搬数据）*/
                         | DMA_CCR_EN;
    NVIC_SetPriority(DMA1_Channel1_IRQn, 7u);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* 8) ADC 开扫描 + 开 DMA + 连续转换 + 软件触发 */
    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_CONT
               | ADC_CR2_EXTSEL          /* 111 = SWSTART */
               | ADC_CR2_EXTTRIG;
    ADC1->CR2 |= ADC_CR2_SWSTART;    /* 点火：之后 DMA 一直在后台滚 */
}

uint16_t adc_ir_raw(uint8_t ch)
{
    return (ch < ADC_IR_CH_COUNT) ? (uint16_t)s_raw[ch] : 0u;
}

const uint16_t *adc_ir_raw_all(void)
{
    return (const uint16_t *)s_raw;
}

uint32_t adc_ir_seq_count(void)
{
    return s_seq;
}
