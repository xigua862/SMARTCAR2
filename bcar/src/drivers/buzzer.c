/**
  ******************************************************************************
  * @file    buzzer.c
  * @brief   蜂鸣器驱动实现（PB10，寄存器级）
  ******************************************************************************
  */
#include "buzzer.h"
#include "bcar_config.h"
#include "bsp_systick.h"
#include "stm32f1xx.h"

static uint8_t  s_on        = 0u;
static uint8_t  s_auto_beep = 0u;   /* 是否处于"限时鸣叫"中 */
static uint32_t s_stop_at   = 0u;   /* 到点自动停 */

static void buzzer_write(uint8_t on)
{
#if (BCAR_BUZZER_ON_LEVEL == 0)
    on = (uint8_t)(!on);
#endif
    /* BSRR：低 16 位写 1 = 置高，高 16 位写 1 = 拉低（原子，不用读改写）*/
    GPIOB->BSRR = on ? GPIO_PIN_10 : ((uint32_t)GPIO_PIN_10 << 16);
    s_on = on;
}

void buzzer_init(void)
{
    uint32_t shift;

    /* 时钟：GPIOB 在 APB2。GPIOB 其它脚（PB6/PB7 串口、PB10 蜂鸣器）已分别在
     * 各自驱动里开过时钟，这里再开一次是幂等的。 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    (void)RCC->APB2ENR;

    /* PB10 → 通用推挽输出 2MHz：CNF=00、MODE=10（对应 CRH 里的 0x2）*/
    shift = ((uint32_t)10u - 8u) * 4u;
    GPIOB->CRH = (GPIOB->CRH & ~(0xFu << shift)) | (0x2u << shift);

    buzzer_write(0u);
}

void buzzer_on(void)
{
    s_auto_beep = 0u;
    buzzer_write(1u);
}

void buzzer_off(void)
{
    s_auto_beep = 0u;
    buzzer_write(0u);
}

void buzzer_beep(uint32_t ms)
{
    buzzer_write(1u);
    s_stop_at   = bsp_millis() + ms;
    s_auto_beep = 1u;
}

void buzzer_poll(void)
{
    if (s_auto_beep && (int32_t)(bsp_millis() - s_stop_at) >= 0) {
        buzzer_write(0u);
        s_auto_beep = 0u;
    }
}

uint8_t buzzer_is_on(void)
{
    return s_on;
}
