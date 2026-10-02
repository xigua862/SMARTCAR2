/**
 * @file telemetry_format.c
 * @brief EMEM 遥测记录的字节序、校验和格式编码。
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

#include "telemetry_format.h"

#include <math.h>

uint32_t telemetry_crc32_update(uint32_t state, const void *data, uint32_t size)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t index;

    if ((0u != size) && (0 == bytes))
    {
        return state;
    }

    for (index = 0u; index < size; index++)
    {
        uint8_t bit;

        state ^= bytes[index];
        for (bit = 0u; bit < 8u; bit++)
        {
            uint32_t mask = (uint32_t)(0u - (state & 1u));
            state = (state >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return state;
}

int16_t telemetry_float_to_i16(float value, float scale)
{
    float scaled;

    if ((value != value) || (scale <= 0.0f))
    {
        return 0;
    }
    scaled = value * scale;
    if (scaled >= 32767.0f)
    {
        return 32767;
    }
    if (scaled <= -32768.0f)
    {
        return -32768;
    }
    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    return (int16_t)scaled;
}

uint16_t telemetry_float_to_u16(float value, float scale)
{
    float scaled;

    if ((value != value) || (value <= 0.0f) || (scale <= 0.0f))
    {
        return 0u;
    }
    scaled = value * scale;
    if (scaled >= 65535.0f)
    {
        return 65535u;
    }
    return (uint16_t)(scaled + 0.5f);
}

int16_t telemetry_yaw_to_cdeg(float yaw_deg)
{
    float wrapped_yaw_deg = fmodf(yaw_deg, 360.0f);
    int16_t value;

    if (wrapped_yaw_deg != wrapped_yaw_deg)
    {
        return 0;
    }
    if (wrapped_yaw_deg > 180.0f)
    {
        wrapped_yaw_deg -= 360.0f;
    }
    else if (wrapped_yaw_deg <= -180.0f)
    {
        wrapped_yaw_deg += 360.0f;
    }
    value = telemetry_float_to_i16(wrapped_yaw_deg, 100.0f);

    return (value <= -18000) ? 18000 : value;
}
