/**
  ******************************************************************************
  * @file    key.c
  * @brief   按键驱动实现（消抖 + 事件）
  ******************************************************************************
  */
#include "key.h"
#include "bcar_config.h"
#include "bsp_systick.h"
#include "stm32f1xx.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       stable;      /* 消抖后的稳定电平：1 = 按住 */
    uint8_t       last_raw;    /* 上一次采样的原始电平 */
    uint16_t      cnt;         /* 连续相同的采样次数 */
    uint8_t       ev_press;
    uint8_t       ev_release;
} key_state_t;

/* 按键接 GND + 内部上拉 ⇒ 按下读到 0 */
#define KEY_DOWN_WHEN_LOW  1
/* 主循环 5ms 一拍，连续 N 拍稳定才算数 */
#define KEY_STABLE_TICKS   (BCAR_KEY_DEBOUNCE_MS / BCAR_TICK_MS)

static key_state_t s_keys[KEY_COUNT];

static uint8_t key_raw(GPIO_TypeDef *port, uint16_t pin)
{
    uint8_t level = (port->IDR & pin) ? 1u : 0u;
#if (KEY_DOWN_WHEN_LOW == 1)
    return (uint8_t)(!level);
#else
    return level;
#endif
}

void key_init(void)
{
    uint8_t i;
    uint32_t shift;

    /* PC13 在 GPIOC、PB0 在 GPIOB；两个口都要时钟（幂等）*/
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
    (void)RCC->APB2ENR;

    /* PC13：浮空输入 + **内部上拉**（CNF=10 = 输入带上下拉，MODE=00，
     * ODR 置 1 选择上拉）。⚠ PC13 灌电流只有 3mA，只做输入不做输出。 */
    GPIOC->CRH = (GPIOC->CRH & ~(0xFu << 20)) | (0x8u << 20);   /* pin13 → 位 20~23 */
    GPIOC->BSRR = GPIO_PIN_13;                                   /* ODR=1 → 上拉 */

    /* PB0：同样处理（CRL 里 pin0 → 位 0~3）*/
    GPIOB->CRL = (GPIOB->CRL & ~(0xFu << 0)) | (0x8u << 0);
    GPIOB->BSRR = GPIO_PIN_0;

    s_keys[KEY_START].port = GPIOC;
    s_keys[KEY_START].pin  = GPIO_PIN_13;
    s_keys[KEY_TRIG].port  = GPIOB;
    s_keys[KEY_TRIG].pin   = GPIO_PIN_0;

    for (i = 0u; i < (uint8_t)KEY_COUNT; i++) {
        uint8_t raw;
        shift = 0u;   /* 只为消除未用告警 */
        (void)shift;
        raw = key_raw(s_keys[i].port, s_keys[i].pin);
        s_keys[i].stable  = raw;
        s_keys[i].last_raw = raw;
        s_keys[i].cnt     = 0u;
        s_keys[i].ev_press = 0u;
        s_keys[i].ev_release = 0u;
        /* 上电时若已经按住，不要立刻报"按下"，等真的松手再算 */
        if (raw == 1u) {
            s_keys[i].stable = 1u;
        }
    }
}

void key_poll(void)
{
    uint8_t i;
    for (i = 0u; i < (uint8_t)KEY_COUNT; i++) {
        uint8_t raw = key_raw(s_keys[i].port, s_keys[i].pin);

        if (raw != s_keys[i].last_raw) {
            s_keys[i].last_raw = raw;
            s_keys[i].cnt = 1u;
            continue;
        }
        if (s_keys[i].cnt < 0xFFFFu) {
            s_keys[i].cnt++;
        }
        if (s_keys[i].cnt >= (uint16_t)KEY_STABLE_TICKS && raw != s_keys[i].stable) {
            s_keys[i].stable = raw;
            s_keys[i].cnt    = 0u;
            if (raw == 1u) {
                s_keys[i].ev_press = 1u;      /* 按下沿 */
            } else {
                s_keys[i].ev_release = 1u;    /* 松开沿 */
            }
        }
    }
}

uint8_t key_take_press(key_id_t id)
{
    uint8_t v;
    if ((uint8_t)id >= (uint8_t)KEY_COUNT) {
        return 0u;
    }
    v = s_keys[id].ev_press;
    s_keys[id].ev_press = 0u;
    return v;
}

uint8_t key_take_release(key_id_t id)
{
    uint8_t v;
    if ((uint8_t)id >= (uint8_t)KEY_COUNT) {
        return 0u;
    }
    v = s_keys[id].ev_release;
    s_keys[id].ev_release = 0u;
    return v;
}

uint8_t key_is_down(key_id_t id)
{
    if ((uint8_t)id >= (uint8_t)KEY_COUNT) {
        return 0u;
    }
    return s_keys[id].stable;
}
