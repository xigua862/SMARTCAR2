/**
  ******************************************************************************
  * @file    bsp_afio.c
  * @brief   AFIO 复用功能配置（寄存器级实现）
  ******************************************************************************
  */
#include "bsp_afio.h"
#include "stm32f1xx.h"

/* 把 SWJ_CFG 字段设成 010 = JTAG 关闭、SWD 保留。
 * 字段是"写保护"的：必须先清干净那 3 位再写新值，否则可能锁不进去。 */
#define SWJ_CFG_JTAG_OFF_SWD_ON   (0x2u << 24)

void bsp_afio_init(void)
{
    /* 1) AFIO 时钟：不改重映射/关闭 JTAG 都要它 */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    (void)RCC->APB2ENR;   /* 读一次，等时钟真正打开（写后读回，防乱序）*/

    /* 2) 关 JTAG、保 SWD（否则 PA15/PB3/PB4 不能当普通 IO，左编码器就没法接）*/
    AFIO->MAPR = (AFIO->MAPR & ~((uint32_t)0x7u << 24)) | SWJ_CFG_JTAG_OFF_SWD_ON;

    /* 3) USART1 重映射到 PB6/PB7（本板 PA9/PA10 给了电机 PWM，不重映射就没串口）*/
    AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
}

uint32_t bsp_afio_mapr(void)
{
    return AFIO->MAPR;
}

const char *bsp_afio_mapr_explain(uint32_t mapr)
{
    /* 只关心我们动过的两位，静态字符串避免在 ISR/早期日志里用 malloc */
    static char buf[48];
    uint32_t swj = (mapr >> 24) & 0x7u;
    uint8_t  usart1_remap = (mapr & AFIO_MAPR_USART1_REMAP) ? 1u : 0u;

    /* 手写格式化：上电早期还没别的库可用 */
    buf[0] = 'S'; buf[1] = 'W'; buf[2] = 'J'; buf[3] = '=';
    buf[4] = (char)('0' + (char)swj);
    buf[5] = ' ';
    buf[6] = 'U'; buf[7] = 'S'; buf[8] = '1'; buf[9] = 'R'; buf[10] = '=';
    buf[11] = usart1_remap ? '1' : '0';
    buf[12] = '\0';
    return buf;
}
