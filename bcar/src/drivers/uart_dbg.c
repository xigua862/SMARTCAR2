/**
  ******************************************************************************
  * @file    uart_dbg.c
  * @brief   调试串口实现（寄存器级 USART1 + 环形缓冲 + 简易 printf）
  ******************************************************************************
  */
#include "uart_dbg.h"
#include "bcar_config.h"
#include "bsp_afio.h"
#include "bsp_gpio.h"
#include "bsp_systick.h"
#include "stm32f1xx_hal.h"
#include <stdarg.h>

/* ---- 接收环形缓冲：ISR 写、主循环读。长度必须是 2 的幂，才能用位与代替取模。 ---- */
static volatile uint8_t  s_rx_buf[BCAR_UART_RX_BUF];
static volatile uint16_t s_rx_head = 0u;   /* 写入位置（ISR）*/
static volatile uint16_t s_rx_tail = 0u;   /* 读取位置（主循环）*/
static volatile uint32_t s_rx_drop = 0u;   /* 溢出丢弃计数（诊断用）*/

#define UART_RX_MASK  (BCAR_UART_RX_BUF - 1u)

void uart_dbg_init(void)
{
    uint32_t pclk2;
    uint32_t brr;

    /* 1) 开时钟：USART1 在 APB2 */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPBEN;
    (void)RCC->APB2ENR;

    /* 2) 脚的模式：TX = 复用推挽 50MHz；RX = 浮空输入
     *    （AFIO 的重映射由 bsp_afio_init() 负责，这里只管电气特性） */
    bsp_gpio_set_af_pp(GPIOB, 6u, BSP_GPIO_SPEED_50MHZ);
    bsp_gpio_set_input_floating(GPIOB, 7u);

    /* 3) 波特率：BRR = fPCLK2 / 波特率（72MHz / 115200 = 625，正好整数，无误差）
     *    USARTDIV 的整数部分放 [15:4]、小数部分放 [3:0]。 */
    pclk2 = HAL_RCC_GetPCLK2Freq();
    brr   = (pclk2 * 25u) / (4u * BCAR_UART_BAUD);   /* 等价于 pclk2/baud 的整数写法 */
    USART1->BRR = brr;

    /* 4) 8N1 + 收发使能 + 接收中断（RXNE） */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART1->CR2 = 0u;      /* 1 个停止位 */
    USART1->CR3 = 0u;      /* 无流控 */
    USART1->CR1 |= USART_CR1_UE;

    /* 5) 中断优先级：调试串口不参与实时闭环，给最低档（15）即可 */
    NVIC_SetPriority(USART1_IRQn, 15u);
    NVIC_EnableIRQ(USART1_IRQn);
}

void uart_dbg_isr_rx(void)
{
    uint32_t sr = USART1->SR;

    if (sr & USART_SR_RXNE) {
        uint8_t  b  = (uint8_t)(USART1->DR & 0xFFu);   /* 读 DR 顺带清 RXNE */
        uint16_t nh = (uint16_t)((s_rx_head + 1u) & UART_RX_MASK);
        if (nh == s_rx_tail) {
            s_rx_drop++;      /* 缓冲满了：丢新数据，别覆盖旧的 */
        } else {
            s_rx_buf[s_rx_head] = b;
            s_rx_head = nh;
        }
    } else {
        /* 不是 RXNE（多半是溢出 ORE 或是别的标志）：读一次 DR 清掉，避免死循环 */
        (void)USART1->DR;
    }
}

void uart_dbg_write(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    if (data == 0) {
        return;
    }
    for (i = 0u; i < len; i++) {
        while ((USART1->SR & USART_SR_TXE) == 0u) {
            /* 等发送寄存器空 —— 阻塞点：控制环里别发长串，
             * 遥测长串留给后面的 TX DMA 版本（阶段 6）。 */
        }
        USART1->DR = data[i];
    }
}

void uart_dbg_puts(const char *s)
{
    const char *p = s;
    while (p != 0 && *p != '\0') {
        while ((USART1->SR & USART_SR_TXE) == 0u) { }
        USART1->DR = (uint8_t)(*p);
        p++;
    }
}

uint16_t uart_dbg_available(void)
{
    return (uint16_t)((s_rx_head - s_rx_tail) & UART_RX_MASK);
}

uint16_t uart_dbg_read(uint8_t *dst, uint16_t max_len)
{
    uint16_t n = 0u;
    if (dst == 0) {
        return 0u;
    }
    while (n < max_len && s_rx_tail != s_rx_head) {
        dst[n] = s_rx_buf[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1u) & UART_RX_MASK);
        n++;
    }
    return n;
}

/* ---------------- 简易 printf ---------------- */

/* 把一个无符号数按 base 输出，返回写入的字符数 */
static uint16_t put_uint(uint32_t v, uint32_t base, uint8_t upper)
{
    char     tmp[11];
    uint16_t n = 0u;
    const char *digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";

    if (v == 0u) {
        while ((USART1->SR & USART_SR_TXE) == 0u) { }
        USART1->DR = '0';
        return 1u;
    }
    while (v > 0u && n < (uint16_t)sizeof(tmp)) {
        tmp[n] = digits[v % base];
        v /= base;
        n++;
    }
    {
        uint16_t i = n;
        while (i > 0u) {
            i--;
            while ((USART1->SR & USART_SR_TXE) == 0u) { }
            USART1->DR = (uint8_t)tmp[i];
        }
    }
    return n;
}

static void put_str(const char *s)
{
    uart_dbg_puts(s);
}

void uart_dbg_printf(const char *fmt, ...)
{
    va_list ap;
    const char *p = fmt;

    va_start(ap, fmt);
    while (p != 0 && *p != '\0') {
        if (*p != '%') {
            while ((USART1->SR & USART_SR_TXE) == 0u) { }
            USART1->DR = (uint8_t)(*p);
            p++;
            continue;
        }
        p++;
        switch (*p) {
        case 'd':
        case 'i': {
            int32_t v = va_arg(ap, int32_t);
            if (v < 0) {
                put_str("-");
                put_uint((uint32_t)(-v), 10u, 0u);
            } else {
                put_uint((uint32_t)v, 10u, 0u);
            }
            break;
        }
        case 'u':  put_uint(va_arg(ap, uint32_t), 10u, 0u); break;
        case 'x':  put_uint(va_arg(ap, uint32_t), 16u, 0u); break;
        case 'X':  put_uint(va_arg(ap, uint32_t), 16u, 1u); break;
        case 'c': {
            char c = (char)va_arg(ap, int32_t);
            while ((USART1->SR & USART_SR_TXE) == 0u) { }
            USART1->DR = (uint8_t)c;
            break;
        }
        case 's':  put_str(va_arg(ap, const char *)); break;
        case '%': {
            while ((USART1->SR & USART_SR_TXE) == 0u) { }
            USART1->DR = '%';
            break;
        }
        case 'l':
            /* %ld / %lu：本平台 int 和 long 都是 32 位，按同样处理 */
            p++;
            if (*p == 'd') {
                int32_t v = va_arg(ap, int32_t);
                if (v < 0) { put_str("-"); put_uint((uint32_t)(-v), 10u, 0u); }
                else       { put_uint((uint32_t)v, 10u, 0u); }
            } else {
                put_uint(va_arg(ap, uint32_t), 10u, 0u);
            }
            break;
        default:
            /* 不认识的格式：原样打出来，方便发现写错了 */
            if (*p != '\0') {
                while ((USART1->SR & USART_SR_TXE) == 0u) { }
                USART1->DR = (uint8_t)(*p);
            }
            break;
        }
        if (*p != '\0') {
            p++;
        }
    }
    va_end(ap);
}

uint8_t uart_dbg_loopback_test(uint32_t timeout_ms)
{
    uint8_t  b = 0u;
    uint32_t t0 = bsp_millis();

    /* 先把残留数据倒空，避免把上一轮的字节当成这次的回环 */
    while (uart_dbg_read(&b, 1u) > 0u) { }

    uart_dbg_write((const uint8_t *)"\xA5", 1u);

    while ((uint32_t)(bsp_millis() - t0) < timeout_ms) {
        if (uart_dbg_read(&b, 1u) > 0u) {
            return (b == 0xA5u) ? 1u : 0u;
        }
    }
    return 0u;
}
