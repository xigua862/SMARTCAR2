/**
 * @file flash.c
 * @brief 速度 PI 等车辆参数的 Flash 读取、校验和保存。
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

// flash.c —— flash.h 的实现：参数打包/解包持久化，以及独立测试页上的 Flash 自检。

#include "flash.h"
#include "competition_config.h"

#include <stddef.h>   // offsetof：取字段在结构体内的字节偏移，是描述表的关键

CarParameters g_flash_parameters;   // 当前生效的参数副本，供控制和菜单直接使用
volatile FlashSelfTestResult g_flash_self_test_result = FLASH_SELF_TEST_NOT_RUN;

// 字段数据类型，打包/解包时决定按 float 处理，或保留为擦除态槽位。
typedef enum
{
    FLASH_PARAM_FLOAT = 0u,
    FLASH_PARAM_RESERVED
} flash_parameter_type_enum;

// 描述表的一行：字段在结构体内的字节偏移 + 类型。
typedef struct
{
    uint16 offset;                   // 由 offsetof 算出
    flash_parameter_type_enum type;
} flash_parameter_descriptor;

// 填一行描述表：传字段名自动算偏移并标类型，避免手写偏移出错。
#define FLASH_PARAM_DESCRIPTOR_FLOAT(field) {(uint16)offsetof(CarParameters, field), FLASH_PARAM_FLOAT}
#define FLASH_PARAM_DESCRIPTOR_RESERVED {(uint16)0u, FLASH_PARAM_RESERVED}

// 默认参数表，仅供用户主动执行“恢复默认参数”菜单操作。
static const CarParameters flash_default_parameters = {
    FLASH_DEFAULT_SPEED_KP,
    FLASH_DEFAULT_SPEED_KI,
    FLASH_DEFAULT_SPEED_INCREMENT_LIMIT,
};

// Flash 持久化格式描述表：每条目占一个 32 位槽位；只有槽位10、11、13
// 映射当前速度 PI 参数，槽位12及其它槽位保留为废弃槽。
// ★槽位顺序是持久化协议，不能把当前字段移动到旧槽位之前。★
static const flash_parameter_descriptor flash_parameter_table[PARAM_FLASH_LENGTH] = {
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,

    FLASH_PARAM_DESCRIPTOR_FLOAT(speed_KP),
    FLASH_PARAM_DESCRIPTOR_FLOAT(speed_KI),
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_FLOAT(speed_IncrementLimit),
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,
    FLASH_PARAM_DESCRIPTOR_RESERVED,
};

// 浮点参数合法性检查，过滤 NaN 和异常大值。(value == value) 为判 NaN 的经典写法（NaN 不等于自己）。
static uint8 flash_float_valid(float value)
{
    return (value == value) && (-1000000.0f < value) && (1000000.0f > value);
}

// 校验缓冲区每个字是否都等于 expected_word，用于 Flash 擦写验证。
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

// 为自检缓冲区生成每字各异的测试模式，并避开擦除态字值（否则写没写进去看不出来）。
static void flash_fill_self_test_pattern(uint32 *buffer, uint16 length, uint32 erased_word)
{
    uint16 index = 0;

    zf_assert(NULL != buffer);

    for (index = 0; index < length; index++)
    {
        uint32 value = (0x13579BDFu ^ ((uint32)(index + 1u) * 0x11111111u));

        if (erased_word == value)
        {
            value ^= 0xFFFFFFFFu;   // 撞上擦除态值就整字取反，确保和"没写"能区分开
        }

        buffer[index] = value;
    }
}

// 按描述表偏移取字段只读指针（结构体首地址当字节数组 + 偏移）。
static const void *flash_parameter_const_ptr(const CarParameters *params, uint16 offset)
{
    return (const void *)((const uint8 *)params + offset);
}

// 按描述表偏移取字段可写指针。
static void *flash_parameter_ptr(CarParameters *params, uint16 offset)
{
    return (void *)((uint8 *)params + offset);
}

// 按描述表把参数结构体打包到 Flash 写入缓冲区（有效字段写 float，废弃槽保持0xFF）。
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
        if (FLASH_PARAM_RESERVED == descriptor->type)
        {
            continue;
        }
        if (FLASH_PARAM_FLOAT == descriptor->type)
        {
            buffer[index].float_type = *(const float *)field_ptr;
        }
    }
}

// 按描述表从 Flash 缓冲区还原参数结构体。
static void flash_unpack_parameters(CarParameters *params, const flash_data_union *buffer)
{
    uint16 index = 0;

    zf_assert(NULL != params);
    zf_assert(NULL != buffer);

    for (index = 0; index < PARAM_FLASH_LENGTH; index++)
    {
        const flash_parameter_descriptor *descriptor = &flash_parameter_table[index];
        void *field_ptr = flash_parameter_ptr(params, descriptor->offset);

        if (FLASH_PARAM_RESERVED == descriptor->type)
        {
            continue;
        }
        if (FLASH_PARAM_FLOAT == descriptor->type)
        {
            *(float *)field_ptr = buffer[index].float_type;
        }
    }
}

// 校验当前有效浮点参数是否合法。任一非法返回 0，整体改用默认参数更安全。
uint8 flash_parameters_valid(const CarParameters *params)
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

    /* A numerically finite parameter is not necessarily a usable controller
     * parameter.  In particular, a blank/partially erased page decodes as
     * zero and would otherwise arm a speed loop with no feedback authority. */
    if ((params->speed_KP <= 0.0f) ||
        (params->speed_KI < 0.0f) ||
        (params->speed_IncrementLimit <= 0.0f))
    {
        return 0u;
    }

    return 1;
}

void flash_init_default_parameters(CarParameters *params)
{
    zf_assert(NULL != params);
    *params = flash_default_parameters;
}

// 将参数保存到 Flash 参数页。先读旧内容对比，完全相同则跳过擦写，省寿命。
void flash_save_parameters(const CarParameters *params)
{
    flash_data_union packed_data[PARAM_FLASH_LENGTH];
    uint32 copy_size = PARAM_FLASH_LENGTH * sizeof(flash_data_union);

    zf_assert(NULL != params);
    zf_assert(flash_parameters_valid(params));

    memset(packed_data, 0xFF, sizeof(packed_data));   // 缓冲区先铺擦除态，再打包填入有效字段
    flash_pack_parameters(params, packed_data);

    if (flash_check(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE))   // 页里已有数据：先比对，相同就跳过
    {
        flash_buffer_clear();
        flash_read_page(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, (uint32 *)flash_union_buffer, PARAM_FLASH_LENGTH);
        if (0 == memcmp(packed_data, flash_union_buffer, copy_size))
        {
            return;   // 内容没变，无需擦写
        }
        flash_erase_page(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE);   // 内容变了：先擦除旧页
    }

    flash_buffer_clear();
    memcpy(flash_union_buffer, packed_data, copy_size);
    flash_write_page(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, (const uint32 *)flash_union_buffer, PARAM_FLASH_LENGTH);
}

// 从 Flash 参数页读取并校验。页为空或校验失败返回 0。
uint8 flash_load_parameters(CarParameters *params)
{
    zf_assert(NULL != params);

    if (!flash_check(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE))   // 页里没有有效数据（比如第一次烧录）
    {
        return 0;
    }

    flash_buffer_clear();
    flash_read_page(PARAM_FLASH_SECTION, PARAM_FLASH_PAGE, (uint32 *)flash_union_buffer, PARAM_FLASH_LENGTH);
    flash_unpack_parameters(params, flash_union_buffer);

    return flash_parameters_valid(params);
}

void flash_params_save(void) { flash_save_parameters(&g_flash_parameters); }

// 从 Flash 加载并刷新全局副本。失败返回 0 且不改动全局副本。
uint8 flash_params_load(void)
{
    CarParameters loaded_parameters;

    if (!flash_load_parameters(&loaded_parameters))
    {
        return 0;
    }

    g_flash_parameters = loaded_parameters;
    return 1;
}

void flash_params_reset_default(void)
{
    flash_init_default_parameters(&g_flash_parameters);
    flash_params_save();
}

// cpu0 启动阶段调用一次：按需自检并加载参数；加载失败只报告失败，不自动替换参数。
FlashParameterInitResult flash_parameter_init(void)
{
    FlashParameterInitResult result;

#if FLASH_SELF_TEST_ON_BOOT
    result.self_test_result = flash_self_test();
    result.self_test_ok = (FLASH_SELF_TEST_PASS == result.self_test_result) ? 1u : 0u;
#else
    g_flash_self_test_result = FLASH_SELF_TEST_NOT_RUN;
    result.self_test_result = FLASH_SELF_TEST_NOT_RUN;
    result.self_test_ok = 0u;
#endif
    result.load_ok = 0u;
    result.default_loaded = 0u;

    if (flash_params_load())
    {
        result.load_ok = 1u;
    }

    return result;
}

/*
 * 在独立测试页运行 Flash 自检，会擦写 FLASH_SELF_TEST_PAGE（故该页必须避开参数页）。四步：
 *   1) 页号合法性（不能越界、不能撞上参数页）；
 *   2) 擦除后读回，确认全页都是同一擦除态字；
 *   3) 写入测试花样再读回，确认逐字一致；
 *   4) 再擦除并读回，确认恢复擦除态（不留残留）。任一步失败立即返回对应失败码。
 */
FlashSelfTestResult flash_self_test(void)
{
    uint32 erased_buffer[FLASH_SELF_TEST_LENGTH];
    uint32 write_buffer[FLASH_SELF_TEST_LENGTH];
    uint32 readback_buffer[FLASH_SELF_TEST_LENGTH];
    uint32 erased_word = 0;

    // 第 1 步：页号合法性——越界或撞上参数页都会毁掉真正的参数
    if ((EEPROM_PAGE_NUM <= FLASH_SELF_TEST_PAGE) || (PARAM_FLASH_PAGE == FLASH_SELF_TEST_PAGE))
    {
        g_flash_self_test_result = FLASH_SELF_TEST_INVALID_PAGE;
        return g_flash_self_test_result;
    }

    // 第 2 步：擦除后读回，验证全页都是同一擦除态字值
    flash_erase_page(PARAM_FLASH_SECTION, FLASH_SELF_TEST_PAGE);
    flash_read_page(PARAM_FLASH_SECTION, FLASH_SELF_TEST_PAGE, erased_buffer, FLASH_SELF_TEST_LENGTH);

    erased_word = erased_buffer[0];   // 以第 0 个字为擦除态基准值
    if (!flash_verify_same_word(erased_buffer, FLASH_SELF_TEST_LENGTH, erased_word))
    {
        g_flash_self_test_result = FLASH_SELF_TEST_ERASE_VERIFY_FAILED;
        return g_flash_self_test_result;
    }

    // 第 3 步：写入测试花样，读回逐字比对
    flash_fill_self_test_pattern(write_buffer, FLASH_SELF_TEST_LENGTH, erased_word);
    flash_write_page(PARAM_FLASH_SECTION, FLASH_SELF_TEST_PAGE, write_buffer, FLASH_SELF_TEST_LENGTH);
    flash_read_page(PARAM_FLASH_SECTION, FLASH_SELF_TEST_PAGE, readback_buffer, FLASH_SELF_TEST_LENGTH);

    if (0 != memcmp(write_buffer, readback_buffer, sizeof(write_buffer)))
    {
        g_flash_self_test_result = FLASH_SELF_TEST_WRITE_VERIFY_FAILED;
        return g_flash_self_test_result;
    }

    // 第 4 步：再擦除并读回，确认恢复擦除态（不给参数区留测试残留）
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
