/**
 * @file flash.c
 * @brief 车辆运行参数的 Flash 读取、校验和保存。
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

/*********************************************************************************************************************
 * TC264 parameter flash helper migrated from the provincial-race project.
 *********************************************************************************************************************/

#include "flash.h"

#include <stddef.h>

/* 当前生效的参数副本，供控制和菜单逻辑直接使用。 */
CarParameters g_flash_parameters;
/* 最近一次 Flash 自检结果，供启动阶段打印。 */
volatile FlashSelfTestResult g_flash_self_test_result = FLASH_SELF_TEST_NOT_RUN;

typedef enum
{
    FLASH_PARAM_FLOAT = 0u,
    FLASH_PARAM_UINT8
} flash_parameter_type_enum;

typedef struct
{
    uint16 offset;
    flash_parameter_type_enum type;
} flash_parameter_descriptor;

#define FLASH_PARAM_DESCRIPTOR_FLOAT(field) {(uint16)offsetof(CarParameters, field), FLASH_PARAM_FLOAT}
#define FLASH_PARAM_DESCRIPTOR_UINT8(field) {(uint16)offsetof(CarParameters, field), FLASH_PARAM_UINT8}

/* 默认参数表，用于 Flash 读取失败或参数非法时回退。 */
static const CarParameters flash_default_parameters = {
    0.0f,   // turn_KP              / T_KP 完赛值 7.5 (2026-07-12,纯惯导+横向纠偏架构)
    0.0f,   // turn_KD              / T_KD
    0.0f,   // turn_KP2             / T_KP2
    0.0f,   // turn_GKD             / T_GKD 完赛值 11.8 (2026-07-12)
    800.0f, // turn_OutputLimit（旧版布局兼容）

    0.0f,   // angel_KP
    0.0f,   // angel_KD
    0.0f,   // angel_KP2
    0.0f,   // angel_GKD
    600.0f, // angel_OutputLimit

    15.0f,   // speed_KP             / S_KP 18.0
    1.8f,    // speed_KI             / S_KI 0.6
    9999.0f, // speed_IntegralLimit（旧版布局兼容）
    8000.0f, // speed_IncrementLimit / S_IncLim

    28u, // otsu_min
    30u, // otsu_max

    270.0f, // ring_speed（旧版布局兼容）
    280.0f, // ring_speed2（旧版布局兼容）
    270.0f, // min_speed_sudu_juece（旧版布局兼容）
    300.0f, // max_speed_sudu_juece（旧版布局兼容）

    0u,     // guandao_counts       / TrackCnt
    250.0f, // duanlu_speed         / BrokenSpd
};

/* Flash 持久化格式描述表。每个条目占用一个 32 位 Flash 槽位，顺序不能随意调整。 */
static const flash_parameter_descriptor flash_parameter_table[PARAM_FLASH_LENGTH] = {
    FLASH_PARAM_DESCRIPTOR_FLOAT(turn_KP),
    FLASH_PARAM_DESCRIPTOR_FLOAT(turn_KD),
    FLASH_PARAM_DESCRIPTOR_FLOAT(turn_KP2),
    FLASH_PARAM_DESCRIPTOR_FLOAT(turn_GKD),
    FLASH_PARAM_DESCRIPTOR_FLOAT(turn_OutputLimit),

    FLASH_PARAM_DESCRIPTOR_FLOAT(angel_KP),
    FLASH_PARAM_DESCRIPTOR_FLOAT(angel_KD),
    FLASH_PARAM_DESCRIPTOR_FLOAT(angel_KP2),
    FLASH_PARAM_DESCRIPTOR_FLOAT(angel_GKD),
    FLASH_PARAM_DESCRIPTOR_FLOAT(angel_OutputLimit),

    FLASH_PARAM_DESCRIPTOR_FLOAT(speed_KP),
    FLASH_PARAM_DESCRIPTOR_FLOAT(speed_KI),
    FLASH_PARAM_DESCRIPTOR_FLOAT(speed_IntegralLimit),
    FLASH_PARAM_DESCRIPTOR_FLOAT(speed_IncrementLimit),

    FLASH_PARAM_DESCRIPTOR_UINT8(otsu_min),
    FLASH_PARAM_DESCRIPTOR_UINT8(otsu_max),

    FLASH_PARAM_DESCRIPTOR_FLOAT(ring_speed),
    FLASH_PARAM_DESCRIPTOR_FLOAT(ring_speed2),
    FLASH_PARAM_DESCRIPTOR_FLOAT(min_speed_sudu_juece),
    FLASH_PARAM_DESCRIPTOR_FLOAT(max_speed_sudu_juece),

    FLASH_PARAM_DESCRIPTOR_UINT8(guandao_counts),
    FLASH_PARAM_DESCRIPTOR_FLOAT(duanlu_speed),
};

/* 检查浮点参数是否为有效业务范围值，过滤 NaN 和异常大值。 */
static uint8 flash_float_valid(float value)
{
    return (value == value) && (-1000000.0f < value) && (1000000.0f > value);
}

/* 校验缓冲区中的每个字是否都等于指定值，用于 Flash 擦写验证。 */
static uint8 flash_verify_same_word(const uint32 *buffer, uint16 length, uint32 expected_word)
{
    uint16 index = 0;

    zf_assert(NULL != buffer);

    for (index = 0; index < length; index++)
    {
        if (expected_word != buffer[index])
        {
            return 0;
        }
    }

    return 1;
}

/* 为 Flash 自检缓冲区生成测试写入模式，并避开擦除态字值。 */
static void flash_fill_self_test_pattern(uint32 *buffer, uint16 length, uint32 erased_word)
{
    uint16 index = 0;

    zf_assert(NULL != buffer);

    for (index = 0; index < length; index++)
    {
        uint32 value = (0x13579BDFu ^ ((uint32)(index + 1u) * 0x11111111u));

        if (erased_word == value)
        {
            value ^= 0xFFFFFFFFu;
        }

        buffer[index] = value;
    }
}

/* 根据描述表中的偏移量获取参数字段只读指针。
 * 这样打包函数不用为每个字段手写访问代码。
 */
static const void *flash_parameter_const_ptr(const CarParameters *params, uint16 offset)
{
    return (const void *)((const uint8 *)params + offset);
}

/* 根据描述表中的偏移量获取参数字段可写指针。
 * 解包 Flash 数据时通过它把不同类型字段写回结构体。
 */
static void *flash_parameter_ptr(CarParameters *params, uint16 offset)
{
    return (void *)((uint8 *)params + offset);
}

/* 按描述表把参数结构体打包到 Flash 写入缓冲区。 */
static void flash_pack_parameters(const CarParameters *params, flash_data_union *buffer)
{
    uint16 index = 0;

    zf_assert(NULL != params);
    zf_assert(NULL != buffer);

    for (index = 0; index < PARAM_FLASH_LENGTH; index++)
    {
        const flash_parameter_descriptor *descriptor = &flash_parameter_table[index];
        const void *field_ptr = flash_parameter_const_ptr(params, descriptor->offset);

        buffer[index].uint32_type = 0xFFFFFFFFu;
        if (FLASH_PARAM_FLOAT == descriptor->type)
        {
            buffer[index].float_type = *(const float *)field_ptr;
        }
        else
        {
            buffer[index].uint8_type = *(const uint8 *)field_ptr;
        }
    }
}

/* 按描述表从 Flash 数据缓冲区还原参数结构体。 */
static void flash_unpack_parameters(CarParameters *params, const flash_data_union *buffer)
{
    uint16 index = 0;

    zf_assert(NULL != params);
    zf_assert(NULL != buffer);

    for (index = 0; index < PARAM_FLASH_LENGTH; index++)
    {
        const flash_parameter_descriptor *descriptor = &flash_parameter_table[index];
        void *field_ptr = flash_parameter_ptr(params, descriptor->offset);

        if (FLASH_PARAM_FLOAT == descriptor->type)
        {
            *(float *)field_ptr = buffer[index].float_type;
        }
        else
        {
            *(uint8 *)field_ptr = buffer[index].uint8_type;
        }
    }
}

/* 校验参数结构体中的关键浮点值是否合法，防止异常数据进入控制环。 */
uint8 FlashParametersValid(const CarParameters *params)
{
    uint16 index = 0;

    zf_assert(NULL != params);

    for (index = 0; index < PARAM_FLASH_LENGTH; index++)
    {
        const flash_parameter_descriptor *descriptor = &flash_parameter_table[index];

        if ((FLASH_PARAM_FLOAT == descriptor->type) &&
            !flash_float_valid(*(const float *)flash_parameter_const_ptr(params, descriptor->offset)))
        {
            return 0;
        }
    }

    return 1;
}

/* 使用默认参数初始化传入的参数结构体。 */
void FlashInitDefaultParameters(CarParameters *params)
{
    zf_assert(NULL != params);
    *params = flash_default_parameters;
}

/* 将指定参数保存到 Flash 参数页，内容未变化时跳过擦写。 */
void FlashSaveParameters(const CarParameters *params)
{
    flash_data_union packed_data[PARAM_FLASH_LENGTH];
    uint32 copy_size = PARAM_FLASH_LENGTH * sizeof(flash_data_union);

    zf_assert(NULL != params);
    zf_assert(FlashParametersValid(params));

    memset(packed_data, 0xFF, sizeof(packed_data));
    flash_pack_parameters(params, packed_data);

    if (flash_check(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE))
    {
        flash_buffer_clear();
        flash_read_page(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, (uint32 *)flash_union_buffer, PARAM_FLASH_LENGTH);
        if (0 == memcmp(packed_data, flash_union_buffer, copy_size))
        {
            return;
        }
        flash_erase_page(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE);
    }

    flash_buffer_clear();
    memcpy(flash_union_buffer, packed_data, copy_size);
    flash_write_page(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, (const uint32 *)flash_union_buffer, PARAM_FLASH_LENGTH);
}

/* 从 Flash 参数页读取参数并校验合法性。 */
uint8 FlashLoadParameters(CarParameters *params)
{
    zf_assert(NULL != params);

    if (!flash_check(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE))
    {
        return 0;
    }

    flash_buffer_clear();
    flash_read_page(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, (uint32 *)flash_union_buffer, PARAM_FLASH_LENGTH);
    flash_unpack_parameters(params, flash_union_buffer);

    return FlashParametersValid(params);
}

/* 使用当前全局参数执行一次 Flash 保存。 */
void SaveParameters(void) { FlashSaveParameters(&g_flash_parameters); }

/* 从 Flash 加载参数并刷新全局副本。 */
uint8 LoadParameters(void)
{
    CarParameters loaded_parameters;

    if (!FlashLoadParameters(&loaded_parameters))
    {
        return 0;
    }

    g_flash_parameters = loaded_parameters;
    return 1;
}

/* 将全局参数恢复为默认值并同步保存到 Flash。 */
void InitDefaultParameters(void)
{
    FlashInitDefaultParameters(&g_flash_parameters);
    SaveParameters();
}

/* 启动时统一执行 Flash 自检、参数加载和默认值回退流程。 */
FlashParameterInitResult flash_parameter_init(void)
{
    FlashParameterInitResult result;

#if FLASH_SELF_TEST_ON_BOOT
    result.self_test_result = FlashSelfTest();
    result.self_test_ok = (FLASH_SELF_TEST_PASS == result.self_test_result) ? 1u : 0u;
#else
    g_flash_self_test_result = FLASH_SELF_TEST_NOT_RUN;
    result.self_test_result = FLASH_SELF_TEST_NOT_RUN;
    result.self_test_ok = 0u;
#endif
    result.load_ok = 0u;
    result.default_loaded = 0u;

    if (LoadParameters())
    {
        result.load_ok = 1u;
    }
    else
    {
        InitDefaultParameters();
        result.default_loaded = 1u;
    }

    return result;
}

/* 运行 Flash 自检，在独立测试页验证擦除、写入、读回和恢复。 */
FlashSelfTestResult FlashSelfTest(void)
{
    uint32 erased_buffer[FLASH_SELF_TEST_LENGTH];
    uint32 write_buffer[FLASH_SELF_TEST_LENGTH];
    uint32 readback_buffer[FLASH_SELF_TEST_LENGTH];
    uint32 erased_word = 0;

    if ((EEPROM_PAGE_NUM <= FLASH_SELF_TEST_PAGE) || (PARAM_FLASH_PAGE == FLASH_SELF_TEST_PAGE))
    {
        g_flash_self_test_result = FLASH_SELF_TEST_INVALID_PAGE;
        return g_flash_self_test_result;
    }

    flash_erase_page(PARAM_FLASH_SECTION, FLASH_SELF_TEST_PAGE);
    flash_read_page(PARAM_FLASH_SECTION, FLASH_SELF_TEST_PAGE, erased_buffer, FLASH_SELF_TEST_LENGTH);

    erased_word = erased_buffer[0];
    if (!flash_verify_same_word(erased_buffer, FLASH_SELF_TEST_LENGTH, erased_word))
    {
        g_flash_self_test_result = FLASH_SELF_TEST_ERASE_VERIFY_FAILED;
        return g_flash_self_test_result;
    }

    flash_fill_self_test_pattern(write_buffer, FLASH_SELF_TEST_LENGTH, erased_word);
    flash_write_page(PARAM_FLASH_SECTION, FLASH_SELF_TEST_PAGE, write_buffer, FLASH_SELF_TEST_LENGTH);
    flash_read_page(PARAM_FLASH_SECTION, FLASH_SELF_TEST_PAGE, readback_buffer, FLASH_SELF_TEST_LENGTH);

    if (0 != memcmp(write_buffer, readback_buffer, sizeof(write_buffer)))
    {
        g_flash_self_test_result = FLASH_SELF_TEST_WRITE_VERIFY_FAILED;
        return g_flash_self_test_result;
    }

    flash_erase_page(PARAM_FLASH_SECTION, FLASH_SELF_TEST_PAGE);
    flash_read_page(PARAM_FLASH_SECTION, FLASH_SELF_TEST_PAGE, readback_buffer, FLASH_SELF_TEST_LENGTH);

    if (!flash_verify_same_word(readback_buffer, FLASH_SELF_TEST_LENGTH, erased_word))
    {
        g_flash_self_test_result = FLASH_SELF_TEST_RESTORE_VERIFY_FAILED;
        return g_flash_self_test_result;
    }

    g_flash_self_test_result = FLASH_SELF_TEST_PASS;
    return g_flash_self_test_result;
}
