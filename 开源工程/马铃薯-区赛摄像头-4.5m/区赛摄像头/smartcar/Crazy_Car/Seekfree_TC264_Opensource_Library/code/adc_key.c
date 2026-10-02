/**
 * @file adc_key.c
 * @brief ADC 按键采样、消抖和键值映射。
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
 */

#include "adc_key.h"


static volatile adc_key_enum adc_key_result = ADC_KEY_NONE;
static volatile uint16 adc_key_last_raw = 0u;

/* 根据 ADC 值映射到键值（12 位 ADC，0~4095）。
 * 阈值基于 ADC 按键分压网络，并按实测值标定。
 */
static adc_key_enum adc_key_read(void)
{
    uint16 val = adc_mean_filter_convert(ADC_KEY_CHANNEL, 6);
    adc_key_last_raw = val;

    /* 实测标定，无死区，VAREF=3.3V */
    if (val < 100u)
    {
        return ADC_KEY_PRESS;
    }
    else if (val < 402u)
    {
        return ADC_KEY_DOWN;
    }
    else if (val < 821u)
    {
        return ADC_KEY_RIGHT;
    }
    else if (val < 1223u)
    {
        return ADC_KEY_LEFT;
    }

    else if (val < 1700u)
    {
        return ADC_KEY_UP;
    }

    return ADC_KEY_NONE;
}

/* 初始化 ADC 按键通道。
 * 上电后默认无按键事件，后续由 adc_key_scan() 周期扫描。
 */
void adc_key_init(void)
{
    adc_init(ADC_KEY_CHANNEL, ADC_12BIT);
    adc_key_result = ADC_KEY_NONE;
}

/* ADC 按键消抖扫描。
 * 由 5ms 后台任务调用，连续稳定 ADC_KEY_STABLE_CNT 次才产生一次按键事件。
 */
void adc_key_scan(void)
{
    static adc_key_enum last_key = ADC_KEY_NONE;
    static uint8 stable_count = 0u;
    static uint8 key_locked = 0u;
    static uint8 debug_tick = 0u;

    adc_key_enum current_key = adc_key_read();

    /* 由 5ms 周期调用时，每约 500ms 打印一次原始 ADC 值 */
    debug_tick++;
    if (debug_tick >= 100u)
    {
        debug_tick = 0u;
        // printf("raw=%u\r\n", (unsigned int)adc_key_last_raw);
    }

    /* 松手后才解锁，允许下一次触发 */
    if (ADC_KEY_NONE == current_key)
    {
        key_locked = 0u;
        stable_count = 0u;
        last_key = ADC_KEY_NONE;
        return;
    }

    /* 已触发过，按住不松手期间不再重复触发 */
    if (key_locked)
    {
        return;
    }

    if (current_key == last_key)
    {
        if (stable_count < ADC_KEY_STABLE_CNT)
        {
            stable_count++;
        }

        if (ADC_KEY_STABLE_CNT == stable_count)
        {
            if (ADC_KEY_NONE == adc_key_result)
            {
                adc_key_result = current_key;
            }
            key_locked = 1u;
        }
    }
    else
    {
        stable_count = 0u;
        last_key = current_key;
    }
}

/* 读取并清除一次 ADC 按键事件。
 * 菜单层调用后事件会被消费，避免同一次按键被重复处理。
 */
adc_key_enum adc_key_get(void)
{
    adc_key_enum key = adc_key_result;

    if (ADC_KEY_NONE != key)
    {
        // static const char *key_names[] = {"NONE", "PRESS", "UP", "DOWN", "LEFT", "RIGHT"};
        // printf("ADC_KEY raw=%u %s\r\n", (unsigned int)adc_key_last_raw, key_names[key]);
    }

    adc_key_result = ADC_KEY_NONE;
    return key;
}
