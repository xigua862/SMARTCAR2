/**
 * @file adc_key.h
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

// adc_key.h —— 五向摇杆按键（上/下/左/右/按下共用一个 ADC 引脚，用电压区间法区分方向）。

#ifndef CODE_ADC_KEY_H_
#define CODE_ADC_KEY_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ADC 按键键值，NONE 表示摇杆回中无按键。
typedef enum
{
    ADC_KEY_NONE = 0,   // 无按键 / 摇杆回中
    ADC_KEY_PRESS,      // 摇杆下压（中键）
    ADC_KEY_UP,
    ADC_KEY_DOWN,
    ADC_KEY_LEFT,
    ADC_KEY_RIGHT,
} adc_key_enum;

#define ADC_KEY_CHANNEL (ADC0_CH13_A13)

// 消抖：连续读到同一方向达此次数才确认（配合 5ms 扫描 ≈ 20ms）。
#define ADC_KEY_STABLE_CNT (4u)

void adc_key_init(void);            // 初始化 ADC 按键（12 位）
void adc_key_scan(void);            // 5ms 定时中断调用，读取并消抖
adc_key_enum adc_key_get(void);     // 取走当前消抖后的键值并清除（一次性消费）

#ifdef __cplusplus
}
#endif

#endif /* CODE_ADC_KEY_H_ */
