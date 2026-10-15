/**
  ******************************************************************************
  * @file    fan.c
  * @brief   风机（负压）驱动实现（PB1/PB2，寄存器级）
  ******************************************************************************
  */
#include "fan.h"
#include "bcar_config.h"
#include "stm32f1xx.h"

/* 软件 PWM：周期 FAN_PWM_MS 毫秒，占空比 BCAR_FAN_DUTY/1000。
 * ★必须每周期留一段低电平：AT8236 的高边驱动要"开关续命"，恒定高电平输出会垮。 */
#define FAN_PWM_MS   5u                            /* 200Hz */
static uint8_t s_level = 0u;                       /* 当前档位 0/1/2 */
static uint8_t s_dir   = (uint8_t)BCAR_FAN_DIR;    /* 发车时用哪个方向 */
static uint8_t s_phase = 0u;                       /* 0..FAN_PWM_MS-1 */
static uint8_t s_on_ms = (uint8_t)((BCAR_FAN_DUTY * FAN_PWM_MS) / 1000u);

/* 两脚电平：1 = 拉高、0 = 拉低。用 BSRR 原子写，不用读改写。 */
static void fan_write(uint8_t in1_high, uint8_t in2_high)
{
    uint32_t set = 0u;
    uint32_t clr = 0u;

    if (in1_high) { set |= GPIO_PIN_1; } else { clr |= GPIO_PIN_1; }
    if (in2_high) { set |= GPIO_PIN_2; } else { clr |= GPIO_PIN_2; }
    GPIOB->BSRR = set | ((uint32_t)clr << 16);
}

void fan_init(void)
{
    /* 时钟：GPIOB 在 APB2（别的驱动已经开过，幂等）*/
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    (void)RCC->APB2ENR;

    /* PB1 → CRL 位[7:4]、PB2 → CRL 位[11:8]，都配成"通用推挽输出 2MHz"（0b0010）*/
    GPIOB->CRL = (GPIOB->CRL & ~((0xFu << 4) | (0xFu << 8)))
               | (0x2u << 4) | (0x2u << 8);

    fan_write(0u, 0u);      /* 上电先关 */
    s_level = 0u;
}

void fan_set(uint8_t level)
{
    /* 电平由 fan_tick_isr() 按 PWM 打；这里只记档位和方向 */
    if (level > 2u) {
        level = 0u;
    }
    s_level = level;
    s_phase = FAN_PWM_MS;
    if (level == 0u) {
        fan_write(0u, 0u);      /* 关：两脚都低 ⇒ 输出高阻（滑行）*/
    } else {
        fan_write(1u, 0u);      /* 先给个"开"的电平，下一拍 PWM 接管 */
    }
}

void fan_tick_isr(void)
{
    uint8_t on;

    if (s_level == 0u) {
        return;                                     /* 关着就别折腾（ISR 要短）*/
    }
    if (s_phase == 0u) {
        s_phase = FAN_PWM_MS;
    }
    s_phase--;

    /* 这一 ms 该不该给高：占空比 = s_on_ms / FAN_PWM_MS。
     * ★s_on_ms 上限是 FAN_PWM_MS-1（见 fan.h 的说明），保证每周期都有一段低电平。 */
    on = (s_phase < s_on_ms) ? 1u : 0u;
    if (s_level == 1u) {
        fan_write(on, 0u);                          /* IN1 打 PWM、IN2 恒低 */
    } else {
        fan_write(0u, on);                          /* 方向反过来 */
    }
}

void fan_launch_on(void)
{
    fan_set(s_dir);
}

void fan_off(void)
{
    fan_set(0u);
}

void fan_set_dir(uint8_t level)
{
    s_dir = (level <= 2u) ? level : 1u;
    fan_set(s_dir);
}

uint8_t fan_level(void)
{
    return s_level;
}
