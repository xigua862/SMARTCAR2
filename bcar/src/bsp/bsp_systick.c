/**
  ******************************************************************************
  * @file    bsp_systick.c
  * @brief   时基实现（5ms 节拍 + DWT 微秒）
  ******************************************************************************
  */
#include "bsp_systick.h"
#include "bcar_config.h"
#include "stm32f1xx.h"

static volatile uint32_t s_ms      = 0u;   /* 上电至今毫秒 */
static volatile uint8_t  s_tick5   = 0u;   /* 5ms 标志 */
static volatile uint8_t  s_div     = 0u;   /* 1ms → 5ms 的分频计数 */

void bsp_systick_init(void)
{
    /* DWT 周期计数器：CM3 要先开 TRCENA，再使能 CYCCNT。
     * 它数的是 CPU 时钟周期（本板 72MHz）→ 1µs = 72 个计数。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void bsp_tick_inc_ms(void)
{
    s_ms++;
    s_div++;
    if (s_div >= BCAR_TICK_MS) {
        s_div   = 0u;
        s_tick5 = 1u;
    }
}

uint8_t bsp_tick5ms_take(void)
{
    if (s_tick5) {
        s_tick5 = 0u;
        return 1u;
    }
    return 0u;
}

uint32_t bsp_millis(void)
{
    return s_ms;
}

uint32_t bsp_micros(void)
{
    /* SystemCoreClock = 72 000 000 → 每微秒 72 个周期。
     * 用除法而不是乘法，避免 32 位溢出（CYCCNT 本身就会回绕）。 */
    return DWT->CYCCNT / (SystemCoreClock / 1000000u);
}

uint8_t bsp_timer_expired(uint32_t *last_ms, uint32_t period_ms)
{
    uint32_t now = s_ms;
    if ((uint32_t)(now - *last_ms) >= period_ms) {
        *last_ms = now;
        return 1u;
    }
    return 0u;
}
