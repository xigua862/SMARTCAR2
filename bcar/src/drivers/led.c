/**
  ******************************************************************************
  * @file    led.c
  * @brief   状态灯驱动实现（寄存器级）
  ******************************************************************************
  */
#include "led.h"
#include "bcar_config.h"
#include "stm32f1xx.h"

/* 脚位对照（与 文档\引脚图-B车C8T6-20261002.html 一致）：
 *   LED1 = PC14、LED2 = PC15、LED3 = PA12
 * 注意 PC14/PC15 是弱驱动脚（输出速度上限 2MHz），点 LED 够用。 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} led_pin_t;

static const led_pin_t s_leds[LED_COUNT] = {
    { GPIOC, GPIO_PIN_14 },
    { GPIOC, GPIO_PIN_15 },
    { GPIOA, GPIO_PIN_12 }
};

static void led_write(led_id_t id, uint8_t on)
{
    uint16_t pin;
    if ((uint8_t)id >= (uint8_t)LED_COUNT) {
        return;
    }
#if (BCAR_LED_ACTIVE_HIGH == 0)
    on = (uint8_t)(!on);   /* 实测极性相反时用 bcar_config.h 里的开关翻转 */
#endif
    pin = s_leds[id].pin;
    /* BSRR 的写法：低 16 位写 1 = 置高，高 16 位写 1 = 拉低。原子操作，不用读改写。 */
    s_leds[id].port->BSRR = on ? pin : (uint32_t)pin << 16;
}

void led_init(void)
{
    uint8_t i;
    /* 时钟：GPIOA / GPIOC 在 APB2 上 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;
    (void)RCC->APB2ENR;

    for (i = 0u; i < (uint8_t)LED_COUNT; i++) {
        GPIO_TypeDef *port = s_leds[i].port;
        uint16_t pin = s_leds[i].pin;
        uint32_t shift;
        volatile uint32_t *cr;

        if (pin < 8u) { cr = &port->CRL; shift = (uint32_t)pin * 4u; }
        else          { cr = &port->CRH; shift = ((uint32_t)pin - 8u) * 4u; }

        /* 通用推挽输出：CNF=00、MODE=10(2MHz)。清 4 位后写入 0x2。 */
        *cr = (*cr & ~(0xFu << shift)) | (0x2u << shift);
    }
    led_all_off();
}

void led_on(led_id_t id)  { led_write(id, 1u); }
void led_off(led_id_t id) { led_write(id, 0u); }

void led_toggle(led_id_t id)
{
    uint16_t pin;
    if ((uint8_t)id >= (uint8_t)LED_COUNT) {
        return;
    }
    pin = s_leds[id].pin;
    /* ODR 异或：读改写。单主循环调用，不会和中断抢。 */
    s_leds[id].port->ODR ^= pin;
}

void led_all_off(void)
{
    uint8_t i;
    for (i = 0u; i < (uint8_t)LED_COUNT; i++) {
        led_write((led_id_t)i, 0u);
    }
}
