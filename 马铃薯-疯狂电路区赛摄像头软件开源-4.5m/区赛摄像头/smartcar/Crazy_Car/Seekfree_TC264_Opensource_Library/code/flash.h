/**
 * @file flash.h
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

#ifndef CODE_FLASH_H_
#define CODE_FLASH_H_

#include "zf_common_headfile.h"

/* 参数页所在的 Flash 分区号。 */
#define PARAM_FLASH_SECTION (0u)
/* 参数长期保存使用的 Flash 页号。 */
#define PARAM_FLASH_PAGE (0u)
/* 参数数据占用的 32 位字数量。 */
#define PARAM_FLASH_LENGTH (22u)
/* Flash 自检专用页号，必须避开参数页。 */
#define FLASH_SELF_TEST_PAGE (1u)
/* Flash 自检使用的测试字数量。 */
#define FLASH_SELF_TEST_LENGTH (16u)
/* 是否在启动加载参数前执行 Flash 自检。自检会擦写 FLASH_SELF_TEST_PAGE，默认关闭。 */
#ifndef FLASH_SELF_TEST_ON_BOOT
#define FLASH_SELF_TEST_ON_BOOT (0u)
#endif

/*
 * Flash 自检结果码。
 * 记录自检未执行、通过、页号非法，以及擦除、写入、恢复校验失败等状态。
 */
typedef enum
{
    FLASH_SELF_TEST_NOT_RUN = 0u,
    FLASH_SELF_TEST_PASS,
    FLASH_SELF_TEST_INVALID_PAGE,
    FLASH_SELF_TEST_ERASE_VERIFY_FAILED,
    FLASH_SELF_TEST_WRITE_VERIFY_FAILED,
    FLASH_SELF_TEST_RESTORE_VERIFY_FAILED
} FlashSelfTestResult;

/*
 * Flash 参数初始化结果。
 * self_test_ok 表示自检是否通过，load_ok 表示是否从 Flash 成功加载，
 * default_loaded 表示是否因加载失败而写入默认参数。
 */
typedef struct
{
    uint8 self_test_ok;
    uint8 load_ok;
    uint8 default_loaded;
    FlashSelfTestResult self_test_result;
} FlashParameterInitResult;

/*
 * 小车运行参数表，也是持久化到 Flash 的参数数据格式。
 * 持久化顺序由 flash.c 内的参数描述表维护。
 */
typedef struct
{
    float turn_KP;
    float turn_KD;
    float turn_KP2;
    float turn_GKD;
    float turn_OutputLimit; /* 旧版布局兼容，当前控制不读取。 */

    /* 以下 angel_* 字段只为兼容既有 Flash 排列。 */
    float angel_KP;
    float angel_KD;
    float angel_KP2;
    float angel_GKD;
    float angel_OutputLimit;

    float speed_KP;
    float speed_KI;
    float speed_IntegralLimit; /* 旧版布局兼容，当前增量 PI 不读取。 */
    float speed_IncrementLimit;

    /* 以下字段只为兼容既有 Flash 排列。 */
    uint8 otsu_min;
    uint8 otsu_max;

    float ring_speed;
    float ring_speed2;
    float min_speed_sudu_juece;
    float max_speed_sudu_juece;

    uint8 guandao_counts;
    float duanlu_speed;
} CarParameters;

/* 当前生效的参数副本。 */
extern CarParameters g_flash_parameters;
/* 最近一次 Flash 自检结果。 */
extern volatile FlashSelfTestResult g_flash_self_test_result;

/* 保存当前全局参数到 Flash。 */
void SaveParameters(void);
/* 从 Flash 读取参数到全局副本。 */
uint8 LoadParameters(void);
/* 将默认参数写入全局副本并保存到 Flash。 */
void InitDefaultParameters(void);
/* 启动时执行 Flash 自检并加载参数，加载失败时写入默认参数。 */
FlashParameterInitResult flash_parameter_init(void);
/* 将指定参数打包后写入 Flash 参数页。 */
void FlashSaveParameters(const CarParameters *params);
/* 从 Flash 读取指定参数并执行合法性校验。 */
uint8 FlashLoadParameters(CarParameters *params);
/* 使用默认值初始化一份参数。 */
void FlashInitDefaultParameters(CarParameters *params);
/* 判断参数是否在允许范围内。 */
uint8 FlashParametersValid(const CarParameters *params);
/* 运行 Flash 自检，在自检页执行擦除、写入、读回和恢复校验。 */
FlashSelfTestResult FlashSelfTest(void);

#endif /* CODE_FLASH_H_ */
