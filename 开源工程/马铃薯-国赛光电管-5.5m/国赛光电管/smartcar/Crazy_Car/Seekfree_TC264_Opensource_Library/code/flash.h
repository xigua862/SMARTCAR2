/**
 * @file flash.h
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

// flash.h —— 小车运行参数（速度 PI）的 Flash 持久化：保存与断电重载。

#ifndef CODE_FLASH_H_
#define CODE_FLASH_H_

#include "zf_common_headfile.h"

// Flash 存储位置/容量：20 个物理槽位保持不变。槽位 0..9 和 14..19
// 作为废弃保留槽写入擦除态，速度 PI 继续固定在槽位 10、11、13，避免已有车端
// Flash 数据错位覆盖当前有效参数。
#define PARAM_FLASH_SECTION (0u)
#define PARAM_FLASH_PAGE (0u)
#define PARAM_FLASH_LENGTH (20u)      // 物理槽位数，一槽占一个 32 位字
#define FLASH_SELF_TEST_PAGE (1u)     // 自检专用页，必须避开参数页
#define FLASH_SELF_TEST_LENGTH (16u)
// 是否在加载参数前跑 Flash 自检（会擦写自检页），默认关闭。
#ifndef FLASH_SELF_TEST_ON_BOOT
#define FLASH_SELF_TEST_ON_BOOT (0u)
#endif

// Flash 自检结果码。成员的值和顺序不能改：其它地方靠数值判断自检结果。
typedef enum
{
    FLASH_SELF_TEST_NOT_RUN = 0u,             // 尚未执行（默认）
    FLASH_SELF_TEST_PASS,                     // 全部通过
    FLASH_SELF_TEST_INVALID_PAGE,             // 页号非法（越界或撞上参数页）
    FLASH_SELF_TEST_ERASE_VERIFY_FAILED,      // 擦除后校验失败
    FLASH_SELF_TEST_WRITE_VERIFY_FAILED,      // 写入后读回对不上
    FLASH_SELF_TEST_RESTORE_VERIFY_FAILED     // 收尾恢复擦除后校验失败
} FlashSelfTestResult;

// Flash 参数初始化结果。
typedef struct
{
    uint8 self_test_ok;                    // 自检是否通过
    uint8 load_ok;                         // 是否成功从 Flash 加载
    uint8 default_loaded;                  // 启动加载不自动写入默认值，保持为0
    FlashSelfTestResult self_test_result;  // 自检详细结果码
} FlashParameterInitResult;

/*
 * 小车当前运行参数表。Flash 仍保留 20 个物理槽位，但字段与槽位的对应关系由 flash.c 描述表维护；
 * 当前只保留速度 PI 三个参数。其它旧视觉/环道/断路参数已经删除，不再从 Flash 读取或写回。
 */
typedef struct
{
    // 速度 PI（增量式，喂给电机）
    float speed_KP;
    float speed_KI;
    float speed_IncrementLimit;

} CarParameters;

extern CarParameters g_flash_parameters;                     // 当前生效的参数副本
extern volatile FlashSelfTestResult g_flash_self_test_result; // 最近一次自检结果

// 对外接口：上半为操作全局副本的便捷函数，下半为面向结构体的底层函数（便捷函数多是其封装）。

void flash_params_save(void);            // 保存当前全局参数到 Flash
uint8 flash_params_load(void);           // 从 Flash 读取参数到全局副本
void flash_params_reset_default(void);   // 写入默认参数到全局副本并保存
FlashParameterInitResult flash_parameter_init(void); // 启动时自检+加载，失败只报告

void flash_save_parameters(const CarParameters *params);      // 打包写入 Flash 参数页
uint8 flash_load_parameters(CarParameters *params);           // 读取并合法性校验
void flash_init_default_parameters(CarParameters *params);
uint8 flash_parameters_valid(const CarParameters *params);
FlashSelfTestResult flash_self_test(void);   // 在自检页做擦除/写入/读回/恢复校验

#endif /* CODE_FLASH_H_ */
