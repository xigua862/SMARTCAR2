/**
 * @file isr.c
 * @brief PIT、摄像头、IMU 和串口中断服务。
 * @author 高冷马铃薯
 *
 * @par 所属项目
 * 2026 智能车区赛（摄像头）
 *
 * @par WegooAI 中转站
 * 本项目用户代码 99% 通过作者自建的 WegooAI 中转站，
 * 使用 Claude Code、Codex 等 AI 编程工具完成。
 * WegooAI 由作者高冷马铃薯搭建并维护，
 * 方便大家接入 AI 编程工具编写和修改代码，欢迎使用 :)
 * 官网：https://ai.wegoo.site/
 *
 * @par 联系作者
 * Bilibili：https://space.bilibili.com/513729639
 * QQ：1153626206
 * Email：moonserver@foxmail.com
 *
 * @note 中断内只保留有界工作，耗时处理放到后台任务。
 */

/*********************************************************************************************************************
 * 中断服务函数
 *********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"
#include "key.h"
#include "adc_key.h"
#include "task.h"
#include "control.h"
#include "code/prestart.h"
#include "gyroscope.h"
#include "globals.h"
#include "camera_pipeline.h"
#include "zf_device_mt9v03x.h"

/* CCU60_CH0：1ms 软件任务节拍。
 * 中断里只累加计数，实际任务在 CPU0 主循环里调度。
 */
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);
    task_loop_tick();
}

/* CCU60_CH1：5ms 发车状态机节拍。
 * 用于负压预热、校准前后延时和停车后负压延时关闭。
 */
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);
    prestart_tick_5ms();
}

/* CCU61_CH0：预留 PIT 中断，目前只清标志。 */
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH0);
}

/* CCU61_CH1：1ms 速度环。
 * 读取编码器、计算转向/速度控制并输出到 FOC。
 */
IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH1);
    pit_speed_pid();
}

/* ERU CH0/CH4：相机 VSYNC 与 IMU 数据就绪共用的外部中断。
 * 四元数已关闭，INT2 为陀螺数据就绪（480Hz）：IMU 在这里采样原始角速度、
 * 完成零飘采样、纯陀螺 yaw 积分，并推进发车校准状态。
 */
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);
    if (exti_flag_get(ERU_CH4_REQ8_P33_7))
    {
        uint32 capture_start_before = mt9v03x_capture_start_count;
        uint32 capture_ms = vision_clock_ms;
        float capture_pure_gyro_yaw_deg = gyro_get_pure_yaw_deg();

        exti_flag_clear(ERU_CH4_REQ8_P33_7);
        camera_vsync_handler();
        if (mt9v03x_capture_start_count != capture_start_before)
        {
            camera_pipeline_publish_capture_from_isr(mt9v03x_capture_start_count,
                                                     capture_ms,
                                                     capture_pure_gyro_yaw_deg);
        }
    }

    if (exti_flag_get(ERU_CH0_REQ0_P15_4))
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
        gyro_external_interrupt_update();
        prestart_gyro_update();
    }
}

/* ERU CH1/CH5：预留外部中断入口。
 * 当前只清标志，避免未处理标志反复进入中断。
 */
IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);

    if (exti_flag_get(ERU_CH1_REQ10_P14_3))
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);
    }

    if (exti_flag_get(ERU_CH5_REQ1_P15_8))
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);
    }
}

/* ERU CH2/CH6：预留外部中断入口。 */
IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
{
    interrupt_global_enable(0);
    if (exti_flag_get(ERU_CH2_REQ7_P00_4))
    {
        exti_flag_clear(ERU_CH2_REQ7_P00_4);
    }
    if (exti_flag_get(ERU_CH6_REQ9_P20_0))
    {
        exti_flag_clear(ERU_CH6_REQ9_P20_0);
    }
}

/* ERU CH3/CH7：预留外部中断入口。 */
IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);

    if (exti_flag_get(ERU_CH3_REQ3_P10_3))
    {
        exti_flag_clear(ERU_CH3_REQ3_P10_3);
    }

    if (exti_flag_get(ERU_CH7_REQ16_P15_1))
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
    }
}

/* DMA CH7：相机 DMA 完成中断。
 * 交给底层相机处理函数维护 finish_flag 和 DMA 状态。
 */
IFX_INTERRUPT(dma_ch7_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);
    camera_dma_handler();
}

/* UART0 发送完成中断，当前无额外处理。 */
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO) { interrupt_global_enable(0); }

/* UART0 接收中断。
 * DEBUG_UART_USE_INTERRUPT 打开时接收调试串口数据。
 */
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);

#if DEBUG_UART_USE_INTERRUPT
    debug_interrupr_handler();
#endif
}

/* UART1 发送完成中断，当前无额外处理。 */
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO) { interrupt_global_enable(0); }

/* UART1 接收中断，保留给摄像头串口回调。 */
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    camera_uart_handler();
}

/* UART2 发送完成中断，当前无额外处理。 */
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO) { interrupt_global_enable(0); }

/* UART2 接收中断，保留给无线模块。 */
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    wireless_module_uart_handler();
}

/* UART3 发送完成中断，当前无额外处理。 */
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO) { interrupt_global_enable(0); }

/* UART3 接收中断，保留给 GNSS 模块。 */
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    gnss_uart_callback();
}

/* UART0 错误中断。 */
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart0_handle);
}

/* UART1 错误中断。 */
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart1_handle);
}

/* UART2 错误中断。 */
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart2_handle);
}

/* UART3 错误中断。 */
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart3_handle);
}
