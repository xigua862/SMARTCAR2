/**
 * @file adc_key.c
 * @brief ADC 按键采样、消抖和键值映射。
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
 */

// adc_key.c —— adc_key.h 的实现：init 配通道、scan 每 5ms 采样消抖、get 取走按键并清零。

#include "adc_key.h"


// 已确认、等待菜单层取走的按键结果。volatile：5ms 中断里写、主循环菜单里读。
static volatile adc_key_enum adc_key_result = ADC_KEY_NONE;

// 最近一次采到的原始 ADC 值，仅用于调试打印。
static volatile uint16 adc_key_last_raw = 0u;

/*
 * 电压区间法：先对 ADC 通道采 6 次取均值降噪得到 val，再用从小到大的阈值把 0~4095
 * 切成若干区间，val 落在哪个区间就是哪个方向。区间必须从小往大逐段判断。
 */
static adc_key_enum adc_key_read(void)
{
    uint16 val = adc_mean_filter_convert(ADC_KEY_CHANNEL, 6); // 采 6 次取均值降噪
    adc_key_last_raw = val;

    // 实测标定，无死区，VAREF=3.3V
    if (val < 100u)          // 中键下压
    {
        return ADC_KEY_PRESS;
    }
    else if (val < 402u)     // 向下
    {
        return ADC_KEY_DOWN;
    }
    else if (val < 821u)     // 向右
    {
        return ADC_KEY_RIGHT;
    }
    else if (val < 1223u)    // 向左
    {
        return ADC_KEY_LEFT;
    }

    else if (val < 1700u)    // 向上
    {
        return ADC_KEY_UP;
    }

    return ADC_KEY_NONE;     // 高于所有区间：摇杆回中
}

// 初始化 ADC 按键通道为 12 位并清空结果。开机调用一次。
void adc_key_init(void)
{
    adc_init(ADC_KEY_CHANNEL, ADC_12BIT);
    adc_key_result = ADC_KEY_NONE;
}

/*
 * ADC 按键消抖扫描，由 5ms 后台任务调用。连续稳定 ADC_KEY_STABLE_CNT 次才产生一次事件，
 * 且按住只触发一次。以下 static 变量在多次调用间记住进度：
 *   last_key     上次方向  stable_count 已连续稳定次数
 *   key_locked   已触发锁（按住不重复触发）  debug_tick 打印分频计数
 */
void adc_key_scan(void)
{
    static adc_key_enum last_key = ADC_KEY_NONE;
    static uint8 stable_count = 0u;
    static uint8 key_locked = 0u;
    static uint8 debug_tick = 0u;

    adc_key_enum current_key = adc_key_read();

    debug_tick++;                 // 每约 500ms 打印一次原始 ADC 值
    if (debug_tick >= 100u)
    {
        debug_tick = 0u;
        // printf("raw=%u\r\n", (unsigned int)adc_key_last_raw);
    }

    // 摇杆回中：消抖进度和锁全部清零，回到初始状态
    if (ADC_KEY_NONE == current_key)
    {
        key_locked = 0u;
        stable_count = 0u;
        last_key = ADC_KEY_NONE;
        return;
    }

    // 已触发，按住期间不重复触发，等松手解锁
    if (key_locked)
    {
        return;
    }

    if (current_key == last_key) // 方向和上次相同：可能是稳定按键
    {
        if (stable_count < ADC_KEY_STABLE_CNT)
        {
            stable_count++;
        }

        if (ADC_KEY_STABLE_CNT == stable_count) // 连续稳定够次数，确认一次按键
        {
            if (ADC_KEY_NONE == adc_key_result) // 上一次按键未被取走就不覆盖
            {
                adc_key_result = current_key;
            }
            key_locked = 1u;                    // 上锁
        }
    }
    else                          // 方向变了：还在抖或换了方向，重新计数
    {
        stable_count = 0u;
        last_key = current_key;
    }
}

// 读取并清除一次按键事件（一次性消费），菜单主循环轮询调用。无待处理时返回 ADC_KEY_NONE。
adc_key_enum adc_key_get(void)
{
    adc_key_enum key = adc_key_result;

    if (ADC_KEY_NONE != key)
    {
        // static const char *key_names[] = {"NONE", "PRESS", "UP", "DOWN", "LEFT", "RIGHT"};
        // printf("ADC_KEY raw=%u %s\r\n", (unsigned int)adc_key_last_raw, key_names[key]);
    }

    adc_key_result = ADC_KEY_NONE;     // 清零：本次按键已取走，不会再被读到
    return key;
}
