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

/* 鸣叫序列：把"响多久/停多久/还剩几声"记下来，由 buzzer_poll() 一拍一拍推进。
 * 单声（buzzer_beep）= 序列里只有一声、间隔为 0 —— 两种用法共用一套状态。 */
static uint8_t  s_on         = 0u;
static uint8_t  s_seq_left   = 0u;   /* 还剩几声；0 = 没有序列在跑 */
static uint8_t  s_seq_on_ph  = 0u;   /* 1 = 现在在"响"段，0 = 在"间隔"段 */
static uint16_t s_seq_on_ms  = 0u;
static uint16_t s_seq_off_ms = 0u;
static uint32_t s_deadline   = 0u;   /* 下一动作的时刻 */

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
    s_seq_left = 0u;      /* 手动的开/关要能把序列掐掉 */
    buzzer_write(1u);
}

void buzzer_off(void)
{
    s_seq_left = 0u;
    buzzer_write(0u);
}

void buzzer_beeps(uint8_t n, uint16_t on_ms, uint16_t off_ms)
{
    if (n == 0u) {
        buzzer_off();
        return;
    }
    s_seq_left   = n;
    s_seq_on_ms  = on_ms;
    s_seq_off_ms = off_ms;
    s_seq_on_ph  = 1u;
    buzzer_write(1u);
    s_deadline = bsp_millis() + (uint32_t)on_ms;
}

void buzzer_beep(uint32_t ms)
{
    buzzer_beeps(1u, (uint16_t)ms, 0u);
}

void buzzer_poll(void)
{
    if (s_seq_left == 0u) {
        return;
    }
    if ((int32_t)(bsp_millis() - s_deadline) < 0) {
        return;
    }

    if (s_seq_on_ph) {
        buzzer_write(0u);          /* 这一声完了 */
        s_seq_left--;
        if (s_seq_left == 0u) {
            return;                /* 全放完：保持静音 */
        }
        s_seq_on_ph = 0u;          /* 进间隔段 */
        s_deadline  = bsp_millis() + (uint32_t)s_seq_off_ms;
    } else {
        s_seq_on_ph = 1u;          /* 间隔完了，响下一声 */
        buzzer_write(1u);
        s_deadline  = bsp_millis() + (uint32_t)s_seq_on_ms;
    }
}

uint8_t buzzer_busy(void)
{
    return (s_seq_left > 0u) ? 1u : 0u;
}

uint8_t buzzer_is_on(void)
{
    return s_on;
}
