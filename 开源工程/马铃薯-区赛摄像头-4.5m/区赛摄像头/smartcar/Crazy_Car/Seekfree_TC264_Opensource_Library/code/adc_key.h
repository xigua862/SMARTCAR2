/**
 * @file adc_key.h
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

#ifndef CODE_ADC_KEY_H_
#define CODE_ADC_KEY_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ADC 按键键值枚举 */
typedef enum
{
    ADC_KEY_NONE = 0,
    ADC_KEY_PRESS,
    ADC_KEY_UP,
    ADC_KEY_DOWN,
    ADC_KEY_LEFT,
    ADC_KEY_RIGHT,
} adc_key_enum;
/* ADC 按键使用的通道 */

#define ADC_KEY_CHANNEL (ADC0_CH13_A13)

/* 消抖所需连续相同次数 */
#define ADC_KEY_STABLE_CNT (4u)

/* 初始化 ADC 按键（ADC0_CH7_A7，12 位） */
void adc_key_init(void);

/* 5ms 定时中断中调用，读取 ADC 并消抖 */
void adc_key_scan(void);

/* 获取当前消抖后的键值并清除（一次性消费） */
adc_key_enum adc_key_get(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_ADC_KEY_H_ */
