/**
 * @file vqf_adapter.h
 * @brief 官方 VQF C++ 实现面向 C 固件的适配层。
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

// VQF C ABI adapter for the C-only application modules.

#ifndef CODE_VQF_ADAPTER_H_
#define CODE_VQF_ADAPTER_H_

#ifdef __cplusplus
extern "C" {
#endif

void vqf_adapter_init(void);
void vqf_adapter_update_gyro(const float gyro_radps[3], float sample_period_s);
void vqf_adapter_update_accel(const float accel_mps2[3]);
void vqf_adapter_get_quaternion(float quaternion_wxyz[4]);
float vqf_adapter_get_bias(float bias_radps[3]);
int vqf_adapter_rest_detected(void);
int vqf_adapter_rest_measurements_stable(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_VQF_ADAPTER_H_ */
