/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   TIM2/TIM3 正交编码器计数（寄存器级）
  *
  * 为什么手写而不走 CubeMX：这两个定时器要**部分重映射**（TIM2→PA15/PB3、
  *   TIM3→PB4/PB5），而重映射本来就是 bsp_afio.c 的职责；手写在这里意图最清楚，
  *   也不会被 CubeMX 重新生成时改回去。参数在 bcar_config.h。
  ******************************************************************************
  */
#include "encoder.h"
#include "bcar_config.h"
#include "bsp_gpio.h"
#include "stm32f1xx.h"

#define ENC_ARR  65535u   /* 16 位满量程，让它自然回绕；我们只取差值 */

/* 方向符号：若实测"车往前走、计数往负走"，把对应一侧改成 -1。
 * ⚠ 侧框架翻转后（编码器到车头）左右相对关系没变，所以默认两个都是 +1。 */
#define ENC_L_SIGN  (+1)
#define ENC_R_SIGN  (+1)

static int16_t s_delta[ENC_COUNT];
static int32_t s_total[ENC_COUNT];
static uint16_t s_last[ENC_COUNT];

/* 取 16 位计数器的差值，自动处理回绕（正转/反转都对）*/
static int16_t cnt_delta(uint16_t now, uint16_t last)
{
    return (int16_t)((uint16_t)(now - last));
}

/* 配一个定时器为正交编码器模式：TI1/TI2 都计数（4 倍频）*/
static void tim_encoder_setup(TIM_TypeDef *tim)
{
    tim->PSC  = 0u;
    tim->ARR  = ENC_ARR;
    tim->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;   /* CH1/CH2 都映射到 TI1/TI2 */
    tim->CCER  = 0u;                                     /* 不检测极性（上升沿计数）*/
    tim->SMCR  = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;        /* SMS=011 编码器模式 3 */
    tim->CNT   = 0u;
    tim->CR1  |= TIM_CR1_CEN;
}

uint8_t encoder_init(void)
{
    /* 1) 时钟：GPIOA/GPIOB（APB2）、TIM2/TIM3（APB1）*/
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
    (void)RCC->APB2ENR;
    (void)RCC->APB1ENR;

    /* 2) 四个脚：浮空输入（编码器输出电平跟随它自己的 VCC=3.3V，直进 MCU 即可）
     *    ⚠ 这里**不能**配成上拉 —— 编码器是推挽输出，上拉没意义还多一路漏电。 */
    bsp_gpio_set_input_floating(GPIOA, 15u);   /* L_A */
    bsp_gpio_set_input_floating(GPIOB, 3u);    /* L_B */
    bsp_gpio_set_input_floating(GPIOB, 4u);    /* R_A */
    bsp_gpio_set_input_floating(GPIOB, 5u);    /* R_B */

    /* 3) ⚠ 部分重映射：这两句不写，脚就还连着 JTAG/别的功能，计数永远是 0。
     *    TIM2 部分重映射 01 → CH1/CH2 挪到 PA15/PB3
     *    TIM3 部分重映射 01 → CH1/CH2 挪到 PB4/PB5
     *    同时 JTAG 必须已关闭（bsp_afio_init 里做的）。
     *    ★2026-10-15 修正：原来写的是 TIM2_REMAP_1（位[9:8]=10 = **全重映射** PA0/PA1），
     *    左编码器根本不动；部分重映射是 01，应置 TIM2_REMAP_0（位 8）。
     *    实测证据：修之前 AFIO->MAPR = 0x02000604（TIM2=10），修完应为 0x02000504。 */
    AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_0;
    AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_0;

    tim_encoder_setup(TIM2);
    tim_encoder_setup(TIM3);

    s_last[ENC_L] = (uint16_t)TIM2->CNT;
    s_last[ENC_R] = (uint16_t)TIM3->CNT;
    s_delta[ENC_L] = s_delta[ENC_R] = 0;
    s_total[ENC_L] = s_total[ENC_R] = 0;
    return 1u;
}

void encoder_poll(void)
{
    uint16_t now;

    now = (uint16_t)TIM2->CNT;
    s_delta[ENC_L] = cnt_delta(now, s_last[ENC_L]);
    s_last[ENC_L] = now;
    s_total[ENC_L] += (int32_t)s_delta[ENC_L] * ENC_L_SIGN;

    now = (uint16_t)TIM3->CNT;
    s_delta[ENC_R] = cnt_delta(now, s_last[ENC_R]);
    s_last[ENC_R] = now;
    s_total[ENC_R] += (int32_t)s_delta[ENC_R] * ENC_R_SIGN;
}

int16_t encoder_delta(enc_id_t id)
{
    if ((uint8_t)id >= (uint8_t)ENC_COUNT) {
        return 0;
    }
    return (int16_t)(s_delta[id] * ((id == ENC_L) ? ENC_L_SIGN : ENC_R_SIGN));
}

int32_t encoder_total(enc_id_t id)
{
    if ((uint8_t)id >= (uint8_t)ENC_COUNT) {
        return 0;
    }
    return s_total[id];
}

void encoder_reset(void)
{
    s_total[ENC_L] = s_total[ENC_R] = 0;
}
