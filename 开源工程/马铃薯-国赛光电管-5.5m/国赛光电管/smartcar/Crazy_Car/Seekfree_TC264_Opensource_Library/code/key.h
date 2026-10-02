/**
 * @file key.h
 * @brief 菜单、光电调试页、发车停车和日志导出交互。
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

#ifndef CODE_KEY_H_
#define CODE_KEY_H_

// key.h —— TFT 菜单 + 物理按键 + 参数注册 模块对外接口（CPU0 控制核）

#include "zf_common_headfile.h"
#include "flash.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 菜单按键抽象码：物理按键统一映射成这几个动作码，换硬件只改 key_get() 映射
#define KEY_UP ('U')           // 上移 / 编辑时 +1 步
#define KEY_DOWN ('D')         // 下移 / 编辑时 -1 步
#define KEY_LEFT ('L')         // 上一页 / 编辑时 -10 步
#define KEY_RIGHT ('R')        // 下一页 / 编辑时 +10 步
#define KEY_CONFIRM ('C')      // 确认（KEY1 短按）：进页面 / 进编辑 / 发车
#define KEY_RETURN ('B')       // 返回（ADC 摇杆按下）
#define KEY_CONFIRM_LONG ('X') // 长按确认（KEY1 长按）：急停 / 保存参数
#define KEY_NONE ('0')         // 无按键

// 参数注册：把一个 float 变量挂到调参菜单，注册顺序即菜单行顺序。name 截断到 19 字符，
// 调节自动夹在 [min,max]。返回成功 1u；表满或指针为空 0u。
uint8 add_parameter(const char *name, float *value_ptr, float min_value, float max_value, uint8 int_digits,
                    uint8 decimal_places);

void key_get(void);                      // 读物理按键生成菜单按键码（按键扫描节拍周期调用）
void menu_platform_init(void);           // 初始化按键、定时器和屏幕（开机一次）
void menu_init(void);                    // 初始化菜单状态并注册参数表
void menu_task(void);                    // 菜单周期任务（5ms 后台任务调用）
// 实体键是否已触发发车；可能还在预热，真正在跑要问 prestart_is_running()
extern volatile uint8 is_car_running;

#ifdef __cplusplus
}
#endif

#endif // CODE_KEY_H_
