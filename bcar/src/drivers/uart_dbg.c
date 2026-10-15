/**
  ******************************************************************************
  * @file    uart_dbg.c
  * @brief   调试串口实现（寄存器级 USART1 + 环形缓冲 + 简易 printf + TX DMA）
  *
  * 三种输出方式，按"会不会挡住控制环"选：
  *   uart_dbg_printf()      —— 阻塞式：上电横幅/错误信息（量小、不在控制路径）
  *   uart_dbg_snprintf()    —— 只格式化到内存缓冲，不输出
  *   uart_dbg_write_dma()   —— 缓冲整段交给 DMA 发，**不阻塞**（遥测 LAP 行用这个）
  *
  * 为什么非要 TX DMA：一条 LAP 行约 100 字节，115200 下阻塞发送要 ~9ms，
  *   等于丢掉两个 5ms 控制拍。发遥测把控制环卡住，是"车没事、数据把它弄坏"的典型。
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

/* ---- 发送：DMA 用的线性缓冲 + 忙标志 ---- */
static uint8_t           s_tx_buf[BCAR_UART_TX_BUF];
static volatile uint8_t  s_tx_busy = 0u;
static volatile uint32_t s_tx_drop = 0u;   /* 上一笔没发完又被塞新的 → 丢笔计数 */

#define UART_RX_MASK  (BCAR_UART_RX_BUF - 1u)

/* ---- 输出落点：0 = 直接写 DR（阻塞）；1 = 追加到内存缓冲（snprintf 用）---- */
static uint8_t  s_sink_mem = 0u;
static char    *s_sink_buf = 0;
static uint16_t s_sink_max = 0u;
static uint16_t s_sink_len = 0u;

/* ★两条发送路径必须互斥（2026-10-15 读码发现，待上板复测）：
 * TX DMA 正在发（s_tx_busy=1、CR3.DMAT=1）时，若 CPU 再直接写 DR，两边抢同一个发送寄存器，
 * 结果通常是 **DMA 那一笔丢掉一个字符、整行错位** —— TLM/LAP 行就这么悄悄坏掉，
 * 而且它不报错、只在解析端现形（调参器按行解析 LAP，最怕这个）。
 * 所以"阻塞式输出"动手前先等 DMA 收工；但**最多等 30ms**（发满一整个 TX 缓冲约 17ms），
 * 等不到就照旧发 —— 绝不许在这里死等，那会卡住控制环。 */
#define UART_TX_DMA_WAIT_MS  30u

static void tx_dma_idle_wait(void)
{
    uint32_t t0 = bsp_millis();
    while (s_tx_busy != 0u && (uint32_t)(bsp_millis() - t0) < UART_TX_DMA_WAIT_MS) {
        /* 等 DMA 把这一笔发完（RXNE 中断不受影响，收字节照收）*/
    }
}

static void putc_out(char c)
{
    if (s_sink_mem) {
        if (s_sink_buf != 0 && s_sink_len < (uint16_t)(s_sink_max - 1u)) {
            s_sink_buf[s_sink_len] = c;
            s_sink_len++;
            s_sink_buf[s_sink_len] = '\0';
        }
        return;      /* 缓冲写满就静默截断，绝不返回半截字符串 */
    }
    while ((USART1->SR & USART_SR_TXE) == 0u) {
        /* 等发送寄存器空 */
    }
    USART1->DR = (uint8_t)c;
}

void uart_dbg_init(void)
{
    uint32_t pclk2;
    uint32_t brr;

    /* 1) 开时钟：USART1 在 APB2、DMA1 在 AHB */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPBEN;
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
    (void)RCC->APB2ENR;
    (void)RCC->AHBENR;

    /* 2) 脚的模式：TX = 复用推挽 50MHz；RX = 浮空输入
     *    （AFIO 的重映射由 bsp_afio_init() 负责，这里只管电气特性） */
    bsp_gpio_set_af_pp(GPIOB, 6u, BSP_GPIO_SPEED_50MHZ);
    bsp_gpio_set_input_floating(GPIOB, 7u);

    /* 3) 波特率：OVER8=0 时 BRR = fPCLK2 / 波特率（低 4 位是小数部分，所以不能乘 16 之外的系数）。
     *    ★2026-10-15 修正：原式 (pclk2*25)/(4*baud) 会算出 3906（实际 18432 波特，串口全是乱码），
     *    正确值 72MHz/115200 = 625。四舍五入用 (pclk2 + baud/2)/baud。 */
    pclk2 = HAL_RCC_GetPCLK2Freq();
    brr   = (pclk2 + (BCAR_UART_BAUD / 2u)) / BCAR_UART_BAUD;
    USART1->BRR = brr;

    /* 4) 8N1 + 收发使能 + 接收中断（RXNE） */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART1->CR2 = 0u;
    USART1->CR3 = 0u;
    USART1->CR1 |= USART_CR1_UE;

    /* 5) 接收中断优先级：调试串口不参与实时闭环，给最低档（15）即可 */
    NVIC_SetPriority(USART1_IRQn, 15u);
    NVIC_EnableIRQ(USART1_IRQn);

    /* 6) TX DMA 通道先备好（USART1_TX 硬连线到 DMA1 通道4）*/
    DMA1_Channel4->CCR = 0u;
    NVIC_SetPriority(DMA1_Channel4_IRQn, 8u);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    s_tx_busy = 0u;
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
        (void)USART1->DR;     /* 多半是溢出 ORE：读一次清掉，避免中断里死循环 */
    }
}

/* ---------------- 阻塞式发送 ---------------- */

void uart_dbg_write(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    if (data == 0) {
        return;
    }
    tx_dma_idle_wait();      /* 别和 TX DMA 抢发送寄存器（见上面的说明）*/
    for (i = 0u; i < len; i++) {
        while ((USART1->SR & USART_SR_TXE) == 0u) { }
        USART1->DR = data[i];
    }
}

void uart_dbg_puts(const char *s)
{
    const char *p = s;
    if (p == 0) {
        return;
    }
    tx_dma_idle_wait();      /* 同上（回显/提示走这条路）*/
    while (*p != '\0') {
        putc_out(*p);
        p++;
    }
}

/* ---------------- DMA 发送（不阻塞） ---------------- */

uint8_t uart_dbg_write_dma(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == 0 || len == 0u || len > BCAR_UART_TX_BUF) {
        return 0u;
    }
    /* 上一笔还在发：不排队，直接丢这一笔并计数。
     * 遥测"丢一行"远好过"卡住控制环"；正常每圈一行，基本不会撞上。 */
    if (s_tx_busy) {
        s_tx_drop++;
        return 0u;
    }

    for (i = 0u; i < len; i++) {
        s_tx_buf[i] = data[i];
    }

    DMA1_Channel4->CCR   = 0u;
    DMA1_Channel4->CPAR  = (uint32_t)&USART1->DR;
    DMA1_Channel4->CMAR  = (uint32_t)s_tx_buf;
    DMA1_Channel4->CNDTR = len;
    DMA1_Channel4->CCR   = DMA_CCR_DIR            /* 内存 → 外设 */
                         | DMA_CCR_MINC           /* 内存地址自增 */
                         | DMA_CCR_TCIE           /* 发完进中断清忙标志 */
                         | DMA_CCR_EN;

    USART1->CR3 |= USART_CR3_DMAT;
    s_tx_busy = 1u;
    return 1u;
}

/** @brief DMA1_Channel4 中断入口调用：一笔发完，清忙标志 */
void uart_dbg_tx_dma_isr(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF4) {
        DMA1->IFCR = DMA_IFCR_CTCIF4;
        DMA1_Channel4->CCR = 0u;
        USART1->CR3 &= ~USART_CR3_DMAT;
        s_tx_busy = 0u;
    }
}

uint8_t uart_dbg_tx_busy(void)
{
    return s_tx_busy;
}

uint32_t uart_dbg_tx_drop_count(void)
{
    return s_tx_drop;
}

/* ---------------- 接收 ---------------- */

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

/* ---------------- 格式化：同一套代码，两种落点 ---------------- */

/* 按宽度输出无符号数：zero=1 用 '0' 左补，否则用空格。
 * ★必须支持宽度：本项目的横幅就用了 "%08X"，不支持会让 va_arg 错位（曾经因此上电就 HardFault）。 */
static void put_uint_w(uint32_t v, uint32_t base, uint8_t upper, int width, int zero)
{
    char     tmp[11];
    uint16_t n = 0u;
    const char *digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";

    if (v == 0u) {
        tmp[n++] = '0';
    } else {
        while (v > 0u && n < (uint16_t)sizeof(tmp)) {
            tmp[n] = digits[v % base];
            v /= base;
            n++;
        }
    }
    while ((int)n < width) {          /* 左补齐 */
        putc_out(zero ? '0' : ' ');
        width--;
    }
    while (n > 0u) {
        n--;
        putc_out(tmp[n]);
    }
}

/* 有符号：负号先打，数字本体按 (width-1) 补 */
static void put_signed_w(int32_t v, int width, int zero)
{
    if (v < 0) {
        putc_out('-');
        put_uint_w((uint32_t)0u - (uint32_t)v, 10u, 0u, (width > 0) ? width - 1 : 0, zero);
    } else {
        put_uint_w((uint32_t)v, 10u, 0u, width, zero);
    }
}

/* 解析 "%" 之后的 [0][宽度] 前缀，返回宽度（0 = 没写） */
static int fmt_parse_width(const char **pp, int *zero)
{
    const char *p = *pp;
    int w = 0;

    *zero = 0;
    if (*p == '0') {
        *zero = 1;
        p++;
    }
    while (*p >= '0' && *p <= '9') {
        w = w * 10 + (int)(*p - '0');
        p++;
    }
    *pp = p;
    return w;
}

static void fmt_core(const char *fmt, va_list ap)
{
    const char *p = fmt;

    while (p != 0 && *p != '\0') {
        if (*p != '%') {
            putc_out(*p);
            p++;
            continue;
        }
        p++;                                   /* 跳过 '%' */
        {
            int zero  = 0;
            int width = fmt_parse_width(&p, &zero);   /* ★解析 %[0][宽度] */

            switch (*p) {
            case 'd':
            case 'i':  put_signed_w(va_arg(ap, int32_t), width, zero);              break;
            case 'u':  put_uint_w(va_arg(ap, uint32_t), 10u, 0u, width, zero);      break;
            case 'x':  put_uint_w(va_arg(ap, uint32_t), 16u, 0u, width, zero);      break;
            case 'X':  put_uint_w(va_arg(ap, uint32_t), 16u, 1u, width, zero);      break;
            case 'c':  putc_out((char)va_arg(ap, int32_t));                         break;
            case 's': {
                const char *s = va_arg(ap, const char *);
                const char *q = s;
                int len = 0;
                while (q != 0 && *q != '\0') { len++; q++; }      /* 先数长度，好补空格 */
                while (len < width) { putc_out(' '); width--; }
                while (s != 0 && *s != '\0') { putc_out(*s); s++; }
                break;
            }
            case '%':  putc_out('%'); break;
            case 'l':
                /* %ld / %lu：本平台 int 和 long 都是 32 位，按同样处理 */
                p++;
                {
                    int z2 = 0;
                    int w2 = fmt_parse_width(&p, &z2);
                    if (*p == 'd' || *p == 'i') {
                        put_signed_w(va_arg(ap, int32_t), w2, z2);
                    } else if (*p == 'x') {
                        put_uint_w(va_arg(ap, uint32_t), 16u, 0u, w2, z2);
                    } else if (*p == 'X') {
                        put_uint_w(va_arg(ap, uint32_t), 16u, 1u, w2, z2);
                    } else {
                        put_uint_w(va_arg(ap, uint32_t), 10u, 0u, w2, z2);
                    }
                }
                break;
            default:
                /* 不认识的格式：原样打出来，方便发现写错了 */
                if (*p != '\0') {
                    putc_out(*p);
                }
                break;
            }
        }
        if (*p != '\0') {
            p++;
        }
    }
}

void uart_dbg_printf(const char *fmt, ...)
{
    va_list ap;
    s_sink_mem = 0u;
    tx_dma_idle_wait();      /* 同上（命令回复/横幅走这条路）*/
    va_start(ap, fmt);
    fmt_core(fmt, ap);
    va_end(ap);
}

uint16_t uart_dbg_snprintf(char *dst, uint16_t max, const char *fmt, ...)
{
    va_list ap;

    if (dst == 0 || max == 0u) {
        return 0u;
    }
    s_sink_mem = 1u;
    s_sink_buf = dst;
    s_sink_max = max;
    s_sink_len = 0u;
    dst[0] = '\0';

    va_start(ap, fmt);
    fmt_core(fmt, ap);
    va_end(ap);

    s_sink_mem = 0u;   /* 用完立刻切回"直连串口"，避免别的调用误写进缓冲 */
    return s_sink_len;
}

/* ---------------- 回环自检 ---------------- */

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
