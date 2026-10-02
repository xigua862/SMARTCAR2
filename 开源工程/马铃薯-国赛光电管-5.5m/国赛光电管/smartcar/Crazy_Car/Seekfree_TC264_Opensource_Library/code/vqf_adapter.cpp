/**
 * @file vqf_adapter.cpp
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

// C ABI adapter around the official VQF C++ implementation.

#include "vqf_adapter.h"
#include "vqf.hpp"

#pragma section all "cpu1_dsram"

namespace
{
const vqf_real_t VQF_GYRO_SAMPLE_PERIOD_S = 1.0f / 480.0f;
const vqf_real_t VQF_ACCEL_SAMPLE_PERIOD_S = 1.0f / 120.0f;
VQF *vqf_filter = 0;
}

extern "C" void vqf_adapter_init(void)
{
    /* Function-local construction occurs on the CPU1 call path.  A global
     * C++ object would otherwise be constructed by the startup core before
     * core1_main(), violating CPU1's single-writer ownership. */
    static VQF filter_instance(VQF_GYRO_SAMPLE_PERIOD_S,
                               VQF_ACCEL_SAMPLE_PERIOD_S);
    vqf_filter = &filter_instance;
    vqf_filter->resetState();
}

extern "C" void vqf_adapter_update_gyro(const float gyro_radps[3],
                                        float sample_period_s)
{
    vqf_filter->updateGyr(gyro_radps, sample_period_s);
}

extern "C" void vqf_adapter_update_accel(const float accel_mps2[3])
{
    vqf_filter->updateAcc(accel_mps2);
}

extern "C" void vqf_adapter_get_quaternion(float quaternion_wxyz[4])
{
    vqf_filter->getQuat6D(quaternion_wxyz);
}

extern "C" float vqf_adapter_get_bias(float bias_radps[3])
{
    return vqf_filter->getBiasEstimate(bias_radps);
}

extern "C" int vqf_adapter_rest_detected(void)
{
    return vqf_filter->getRestDetected() ? 1 : 0;
}

extern "C" int vqf_adapter_rest_measurements_stable(void)
{
    vqf_real_t relative_deviation[2];

    if (0 == vqf_filter)
    {
        return 0;
    }
    vqf_filter->getRelativeRestDeviations(relative_deviation);
    /* NaN compares false, so an uninitialized filter cannot be reported as
     * stationary.  The absolute post-calibration rate is checked by the C
     * owner in addition to these VQF gyro/acceleration deviation gates. */
    return ((relative_deviation[0] < vqf_real_t(1.0)) &&
            (relative_deviation[1] < vqf_real_t(1.0)))
               ? 1
               : 0;
}

#pragma section all restore
