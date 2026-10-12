/**
  ******************************************************************************
  * @file    bsp_gpio.c
  * @brief   GPIO 底层配置（寄存器级实现）
  ******************************************************************************
  */
#include "bsp_gpio.h"

/* 一个脚的 4 位配置在 CRL/CRH 里的位置：脚 0~7 在 CRL、8~15 在 CRH，
 * 每个脚占 4 位 = [CNF1 CNF0 MODE1 MODE0]。 */
static void gpio_write_cfg(GPIO_TypeDef *port, uint8_t pin, uint8_t cfg4)
{
    volatile uint32_t *cr;
    uint32_t shift;

    if (pin < 8u) {
        cr    = &port->CRL;
        shift = (uint32_t)pin * 4u;
    } else {
        cr    = &port->CRH;
        shift = ((uint32_t)pin - 8u) * 4u;
    }
    *cr = (*cr & ~(0xFu << shift)) | ((uint32_t)cfg4 << shift);
}

void bsp_gpio_set_af_pp(GPIO_TypeDef *port, uint8_t pin, bsp_gpio_speed_t speed)
{
    /* CNF = 10（复用推挽），MODE = speed(2/10/50MHz，0 是输入所以不会是 0) */
    uint8_t cfg = (uint8_t)((0x2u << 2) | ((uint8_t)speed & 0x3u));
    gpio_write_cfg(port, pin, cfg);
}

void bsp_gpio_set_input_floating(GPIO_TypeDef *port, uint8_t pin)
{
    /* CNF = 01（浮空输入），MODE = 00 */
    gpio_write_cfg(port, pin, 0x4u);
}
