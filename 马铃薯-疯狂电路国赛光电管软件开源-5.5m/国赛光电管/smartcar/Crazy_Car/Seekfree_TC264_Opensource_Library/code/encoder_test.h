/**
 * @file encoder_test.h
 * @brief 编码器和地图控制链的台架诊断模式。
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

// encoder_test.h - 按地图运行的编码器/速度环诊断模式
//
// 开启后，KEY_1 短按会启动一段正常地图导航。左右轮目标速度仍由地图的
// 速度表和曲率控制生成，1 ms 速度 PI 正常输出 PWM；仅把航向估计替换为
// “左右编码器差速拟合航向”，用于验证高速运行时编码器是否丢脉冲。
// 车辆必须抬起驱动轮，严禁在地面或赛道上开启此模式。运行数据只进 EMEM，
// 不从调试串口打印；导出后分析 control.csv/navigation.csv 即可。

#ifndef ENCODER_TEST_H_
#define ENCODER_TEST_H_

#include "zf_common_headfile.h"
#include "competition_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 0=正常比赛固件；1=地图闭环编码器诊断模式。

// 菜单全局按键拦截：返回1表示本键已被测试模式消费。
uint8 encoder_test_handle_key(uint8 key);

// Duty_10ms() 调用一次；运行中采集编码器窗口、地图目标/实际速度和拟合航向。
void encoder_test_tick_10ms(void);

#ifdef __cplusplus
}
#endif

#endif
