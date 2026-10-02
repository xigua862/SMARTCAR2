/**
 * @file isr.c
 * @brief 控制 PIT、发车节拍、IMU 和串口中断服务。
 * @author 高冷马铃薯
 *
 * @par 所属项目
 * 2026 智能车国赛（光电管）
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

// 中断服务函数 —— 全工程 ISR 集中在此。IMU ERU 服务CPU1，控制/PIT服务CPU0。
// 本工程两核BIV共用链接脚本装载的向量表0；服务核由SRC.TOS配置决定，不由向量表号决定。

#include "isr_config.h"                 // 中断优先级/服务核宏定义表
#include "isr.h"
#include "key.h"
#include "adc_key.h"
#include "task.h"
#include "control.h"
#include "code/prestart.h"
#include "gyroscope.h"
#include "globals.h"

// ---- PIT 定时器中断（CCU60 / CCU61）----

// CCU60 通道0：1ms 软件任务节拍。中断里只累加计数，实际任务在 CPU0 主循环调度。
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);         // 允许更高优先级中断嵌套
    pit_clear_flag(CCU60_CH0);          // 清标志，否则反复进中断
    task_loop_tick();
}

// CCU60 通道1：5ms 发车状态机节拍（负压预热、校准前后延时、停车后延时关负压）。
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH1);
    prestart_tick_5ms();
}

// CCU61 通道0：预留 PIT，只清标志。
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH0);
}

// CCU61 通道1：1ms 速度环（硬实时），读编码器、算转向/速度控制并输出到 FOC。
IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);
    pit_clear_flag(CCU61_CH1);
    pit_speed_pid();
}

// ---- 外部中断（ERU 引脚边沿触发）----

// ERU 通道0/4 共用（优先级最高 255）：当前只使用通道0的IMU数据就绪(INT2)。
// 四元数已关闭，INT2 为陀螺数据就绪(480Hz)：采样原始角速度、零飘采样、纯陀螺 yaw 积分并推进发车校准。
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);
    if (exti_flag_get(ERU_CH0_REQ0_P15_4))          // IMU 数据就绪
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);
        gyro_cpu1_external_interrupt_update();     // CPU1采样/VQF/姿态快照
    }
}

// ERU 通道1/5 共用（预留），只清标志。
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

// ERU 通道2/6 共用（预留），只清标志。
IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
{
    interrupt_global_enable(0);
    if (exti_flag_get(ERU_CH6_REQ9_P20_0))
    {
        exti_flag_clear(ERU_CH6_REQ9_P20_0);
    }
}

// ERU 通道3/7 共用（预留），只清标志。
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

// ---- 串口中断（UART0~UART3），每口有 TX/RX/ER 三类 ----

IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO) { interrupt_global_enable(0); }

// UART0 接收（调试串口）
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);

#if DEBUG_UART_USE_INTERRUPT
    debug_interrupr_handler();
#endif
}

IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO) { interrupt_global_enable(0); }

// UART1 当前未分配给应用设备
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);
}

IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO) { interrupt_global_enable(0); }

// UART2 接收：无线模块
IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    wireless_module_uart_handler();
}

IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO) { interrupt_global_enable(0); }

// UART3 接收：GNSS 模块
IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);
    gnss_uart_callback();
}

// 各串口错误中断（帧错误/溢出等）
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart0_handle);
}

IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart1_handle);
}

IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart2_handle);
}

IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart3_handle);
}
