/**
  ******************************************************************************
  * @file    uart_dbg.h
  * @brief   调试串口（USART1 重映射到 PB6=TX / PB7=RX，115200 8N1）
  *
  * 为什么不用 CubeMX 生成的 USART 初始化：本板 USART1 必须走重映射
  *   （PA9/PA10 已给电机 PWM），而重映射是 bsp_afio.c 的活；这里自建一份
  *   寄存器级实现，意图全在明面上，也避免 CubeMX 重生成时把配置改回去。
  ******************************************************************************
  */
#ifndef __DRV_UART_DBG_H
#define __DRV_UART_DBG_H

#include <stdint.h>

/**
  * @brief  初始化 USART1（PB6/PB7 复用推挽 + RX 中断 + 环形缓冲）
  * @note   依赖 bsp_afio_init() 已经把 USART1 重映射到 PB6/PB7。
  *         顺序：先 bsp_afio_init()，再 uart_dbg_init()。
  */
void uart_dbg_init(void);

/** @brief 发送一段字节（阻塞式，逐字节等 TXE；控制环里别用长串）*/
void uart_dbg_write(const uint8_t *data, uint16_t len);

/**
  * @brief  用 DMA 发送一段字节（**不阻塞**）
  * @retval 1 = 已交给 DMA；0 = 上一笔还没发完（这笔被丢掉，进 tx_drop 计数）
  * @note   遥测行必须用这个：一条 LAP 行 100 字节阻塞发要 9ms = 两个控制拍。
  */
uint8_t uart_dbg_write_dma(const uint8_t *data, uint16_t len);

/** @brief DMA1_Channel4 中断入口调用 */
void uart_dbg_tx_dma_isr(void);

/** @brief 上一笔 DMA 是否还在发 */
uint8_t uart_dbg_tx_busy(void);

/** @brief 因上一笔没发完而丢弃的笔数（诊断：长期不为 0 说明发太密了）*/
uint32_t uart_dbg_tx_drop_count(void);

/** @brief 发送字符串（不含结尾 0）*/
void uart_dbg_puts(const char *s);

/**
  * @brief  简易 printf：支持 %d %u %x %c %s %ld %lu %%
  * @note   自己实现，不依赖 microlib 的 printf；且内部不等待整串发完之前
  *         不返回，所以逻辑上等价于"发完为止"。
  */
void uart_dbg_printf(const char *fmt, ...);

/**
  * @brief  格式化到内存缓冲（不输出）；返回写入长度（超长静默截断）
  * @note   遥测行先 snprintf 到缓冲、再 write_dma 发出去，就完全不挡控制环。
  */
uint16_t uart_dbg_snprintf(char *dst, uint16_t max, const char *fmt, ...);

/** @brief 从接收环形缓冲取数据，返回实际取出字节数 */
uint16_t uart_dbg_read(uint8_t *dst, uint16_t max_len);

/** @brief 当前缓冲里还有多少字节没取 */
uint16_t uart_dbg_available(void);

/** @brief 中断入口用：把收到的字节塞进环形缓冲（由 USART1_IRQHandler 调用）*/
void uart_dbg_isr_rx(void);

/**
  * @brief  回环自检：TX 发 0xA5 → 等 RX 收到 0xA5
  * @param  timeout_ms 等待上限
  * @retval 1 = 通了（说明"能发也能收"），0 = 超时（只发不收）
  * @note   需要把 USB-TTL 的 TX 和 RX 短接，或板上有回环跳线；测完要拆掉。
  */
uint8_t uart_dbg_loopback_test(uint32_t timeout_ms);

#endif /* __DRV_UART_DBG_H */
