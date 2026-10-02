/**
 * @file navigation_hash.h
 * @brief 路线和定位配置使用的稳定哈希计算。
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

#ifndef CODE_NAVIGATION_HASH_H_
#define CODE_NAVIGATION_HASH_H_

#include <stdint.h>

/* Scalar hashes are encoded little-endian so route/config IDs do not depend on host ABI. */
static inline uint32_t nav_hash_u8(uint32_t hash, uint8_t value)
{
    hash ^= value;
    return hash * 16777619u;
}

static inline uint32_t nav_hash_u16(uint32_t hash, uint16_t value)
{
    hash = nav_hash_u8(hash, (uint8_t)(value & 0xFFu));
    return nav_hash_u8(hash, (uint8_t)(value >> 8u));
}

static inline uint32_t nav_hash_u32(uint32_t hash, uint32_t value)
{
    hash = nav_hash_u8(hash, (uint8_t)(value & 0xFFu));
    hash = nav_hash_u8(hash, (uint8_t)((value >> 8u) & 0xFFu));
    hash = nav_hash_u8(hash, (uint8_t)((value >> 16u) & 0xFFu));
    return nav_hash_u8(hash, (uint8_t)(value >> 24u));
}

static inline uint32_t nav_hash_float(uint32_t hash, float value)
{
    union
    {
        float value;
        uint32_t word;
    } converted;

    converted.value = value;
    return nav_hash_u32(hash, converted.word);
}

#endif /* CODE_NAVIGATION_HASH_H_ */
