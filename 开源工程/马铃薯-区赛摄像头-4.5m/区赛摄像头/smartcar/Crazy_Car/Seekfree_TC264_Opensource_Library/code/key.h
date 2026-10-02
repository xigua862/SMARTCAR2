/**
 * @file key.h
 * @brief 菜单、按键、发车停车和现场参数交互。
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

#ifndef CODE_KEY_H_
#define CODE_KEY_H_

#include "zf_common_headfile.h"
#include "flash.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* 菜单输入抽象层，将物理按键统一映射成菜单动作。 */
#define KEY_UP ('U')
#define KEY_DOWN ('D')
#define KEY_LEFT ('L')
#define KEY_RIGHT ('R')
#define KEY_CONFIRM ('C')
#define KEY_RETURN ('B')
#define KEY_CONFIRM_LONG ('X')
#define KEY_NONE ('0')

/* 注册一个浮点参数到菜单。 */
uint8 add_parameter(const char *name, float *value_ptr, float min_value, float max_value, uint8 int_digits,
                    uint8 decimal_places);
/* 读取物理按键状态并生成菜单按键码。 */
void getkey(void);
/* 初始化菜单所需的按键、定时器和屏幕。 */
void menu_platform_init(void);
/* 初始化菜单运行状态和默认参数。 */
void menu_init(void);
/* 菜单周期任务，处理输入并刷新界面。 */
void menu_task(void);
/* 通知菜单有新的图像调试帧可显示。 */
void menu_notify_frame_ready(void);
/* 更新菜单中的摄像头就绪状态。 */
void menu_set_camera_ready(uint8 camera_ready);
/* 写入图像调试页面使用的双缓冲帧数据。 */
void menu_update_image_debug_frame(const uint8 *raw_image, const uint8 *binary_image);
/* 车辆流程是否已经被用户启动；是否真正运行需再看 prestart_is_running()。 */
extern volatile uint8 is_car_running;
#ifdef __cplusplus
}
#endif

#endif /* CODE_KEY_H_ */
