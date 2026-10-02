/**
 * @file key.c
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

// key.c —— TFT 菜单 + 物理按键 + 参数注册 模块实现（CPU0 控制核）。对外接口见 key.h

#include "key.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "adc_key.h"
#include "phototube_array.h"
#include "control.h"
#include "motor.h"
#include "globals.h"
#include "gyroscope.h"
#include "navigation.h"
#include "phototube_scan.h"
#include "prestart.h"
#include "telemetry.h"
#include "encoder_test.h"

// current_menu 存的就是下面某个值，切页=改这个值
typedef enum
{
    MENU_MAIN = 0,          // 主菜单
    MENU_PARAM_ADJUST,      // 参数调节页：分页显示 parameters[] 并可编辑
    MENU_RUNTIME_INFO,      // 运行信息页：只读
    MENU_IMAGE_DEBUG,       // 模拟光电调试页，发车功能已并入本页
    MENU_LINE_SENSOR_INFO   // 16路线路传感器详情页
} menu_state_enum;

// 当前参数全部是 float；地图速度和旧整型参数已移出菜单协议。
typedef enum
{
    PARAM_TYPE_FLOAT = 0
} parameter_type_enum;

// 参数注册表的一行。字段顺序不要动——菜单显示/调节依赖这个数据布局
typedef struct
{
    char name[20]; // 显示名（最长 19 字符 + '\0'）
    union
    {
        float *float_ptr;
    } value_ptr;          // 指向被调变量本体，菜单直接读写它
    uint8 type;           // parameter_type_enum
    float min_value;      // 调节下限
    float max_value;      // 调节上限
    uint8 int_digits;     // 显示整数位数
    uint8 decimal_places; // 显示小数位数（整型恒为 0）
} parameter_struct;

#define SCREEN_WIDTH (240u)                    // 横屏画布宽
#define SCREEN_HEIGHT (135u)                   // 横屏画布高
#define CHAR_HEIGHT (16u)                      // 8x16 字体行高
#define STATUS_Y (SCREEN_HEIGHT - CHAR_HEIGHT) // 底部状态栏 y
#define MAIN_MENU_VISIBLE_LINES (7u)           // 主菜单一屏最多几行（超出滚动）
#define PARAMS_PER_PAGE (6u)                   // 参数页每页几行
#define MAX_PARAMETERS (32u)                   // 参数表最大容量
#define MENU_VALUE_X (152u)                    // 参数页数值列 x 起点
#define MAIN_MENU_START_Y (12u)                // 主菜单第一行 y 起点
#define MAIN_MENU_LINE_HEIGHT (14u)            // 主菜单行距
// 光电调试页单独用竖屏 135x240，其余菜单横屏 240x135。逐飞方向枚举命名与实际相反：
// IPS114_PORTAIT=240x135(横)，CROSSWISE/CROSSWISE_180=135x240(竖)。本车排线下竖屏正立
// 必须用 CROSSWISE_180（CROSSWISE 会上下颠倒）。故 DBG_PORTRAIT_W/H 与横屏常量长宽互换。
#define DBG_PORTRAIT_W (135u) // 竖屏画布宽
#define DBG_PORTRAIT_H (240u) // 竖屏画布高

#define DEBUG_IMG_W (112u)                                // 光电柱状图宽
#define DEBUG_IMG_H (72u)                                 // 光电柱状图高
#define DEBUG_BIN_X ((DBG_PORTRAIT_W - DEBUG_IMG_W) / 2u)
#define DEBUG_INFO_Y (DEBUG_IMG_H + 8u)                   // 信息区紧贴光电图下方
#define DEBUG_INFO_STEP (14u)                             // 信息区行距
#define DEBUG_INFO_LINES (11u)                            // 信息区行数，用于越界检查
#define FULL_WIDTH_SPACES "                              " // 整行空格，擦除旧文字用

// 编译期自检：坐标算错把图像/文字画出竖屏画布就直接报错，免得上车才发现越界
#if ((DEBUG_BIN_X + DEBUG_IMG_W) > DBG_PORTRAIT_W)
#error "debug sensor graph exceeds portrait width"
#endif

#if ((DEBUG_INFO_Y + DEBUG_INFO_LINES * DEBUG_INFO_STEP) > DBG_PORTRAIT_H)
#error "debug info text exceeds portrait height"
#endif

// 跨节拍共享、可能被按键扫描侧改写的量用 volatile 防读缓存
static volatile uint8 g_menu_key_result = KEY_NONE;      // 单槽按键事件缓存
static volatile uint8 g_menu_frame_ready = 0u;           // 有新光电扫描帧待刷新
static volatile uint8 g_phototube_ready = 0u;
static volatile uint8 g_image_debug_has_frame = 0u;      // 光电调试页是否已收到过帧
static uint32 g_menu_frame_seen_seq = 0u;
// 光电调试页只绘制16路ADC强度柱状图。
static uint16 g_image_debug_overlay_buffer[DEBUG_IMG_H][DEBUG_IMG_W];

// 当前所在页，即菜单状态机状态。切页：menu_task() 取按键交给当前页 handle_xxx_input()，
// 该函数改 current_menu 并置刷新标志，再由 menu_task() 按新页重画
static volatile menu_state_enum current_menu = MENU_MAIN;
static uint8 menu_initialized = 0u;
static uint8 menu_needs_refresh = 1u;                    // 置 1 表示这帧需整页重画
static uint8 main_menu_selection = 0u;
static uint8 param_menu_selection = 0u;
static uint8 param_menu_edit_mode = 0u;                  // 参数页是否处于编辑数值状态
static uint8 key3_long_press_count = 0u;                 // KEY1 长按去重计数
volatile uint8 is_car_running = 0u;                      // 见 key.h 说明
static uint8 parameter_count = 0u;                       // 参数表已注册行数
static uint32 last_dynamic_refresh_ms = 0u;
static uint32 status_expire_ms = 0u;                     // 底部状态消息过期时刻(ms)
static float param_menu_edit_original_value = 0.0f;      // 进编辑前原值，用于撤销
static char status_message[31] = {0};
static parameter_struct parameters[MAX_PARAMETERS];      // 参数注册表本体

// 主菜单选项文字表；第0项是光电调试/发车页，也是上电默认页和唯一发车入口。
static const char *main_menu_items[] = {
    "Phototube / Start",
    "16ch Sensor Info",
    "Parameter Adjust",
    "Runtime Info",
    "Save Parameters",
    "Reload Parameters",
    "Reset Defaults"};

static const uint8 main_menu_item_count =
    (uint8)(sizeof(main_menu_items) / sizeof(main_menu_items[0]));

static ips114_font_size_enum menu_display_font = IPS114_8X16_FONT;

// 设置菜单字体并同步到底层驱动。字体一律走这里切，menu_font_width()/height() 据此算裁剪边界
static void menu_set_font(ips114_font_size_enum font)
{
    menu_display_font = font;
    ips114_set_font(font);
}

static uint8 menu_font_width(void)
{
    return (IPS114_6X8_FONT == menu_display_font) ? 6u : 8u;
}

static uint8 menu_font_height(void)
{
    return (IPS114_6X8_FONT == menu_display_font) ? 8u : 16u;
}

// 安全显示字符串：对 x/y 和长度裁剪，避免写出有效区。text 为 NULL 忽略
static void menu_show_string(uint16 x, uint16 y, const char *text)
{
    char clipped[41];
    uint8 max_chars;
    uint8 i = 0u;

    // 用驱动随方向更新的实际宽高裁剪，不用写死横屏常量：旋转后长宽互换
    // (240x135 <-> 135x240)，写死会导致竖屏下 y>127 的行被误判越界整行不画
    if ((NULL == text) || (x >= ips114_width_max) || (y > (ips114_height_max - menu_font_height())))
    {
        return;
    }

    max_chars = (uint8)((ips114_width_max - x) / menu_font_width());
    if (max_chars >= sizeof(clipped))
    {
        max_chars = (uint8)(sizeof(clipped) - 1u);
    }

    while ((i < max_chars) && ('\0' != text[i]))
    {
        clipped[i] = text[i];
        i++;
    }
    clipped[i] = '\0';

    ips114_show_string(x, y, clipped);
}

// 往下所有 ips114_set_font()/ips114_show_string() 都被重定向到安全封装：set_font 被毒化成
// 不该直接用的名字（强制走 menu_set_font），show_string 走 menu_show_string（自带越界裁剪）
#define ips114_set_font(font) ips114_set_font_should_not_be_used_directly(font)
#define ips114_show_string(x, y, text) menu_show_string((x), (y), (text))

// 设置底部临时状态消息，默认显示 1.5s，超时由 menu_task() 清除。message==NULL 立即清空
static void set_status_message(const char *message)
{
    if (NULL == message)
    {
        status_message[0] = '\0';
        status_expire_ms = 0u;
        return;
    }

    strncpy(status_message, message, sizeof(status_message) - 1u);
    status_message[sizeof(status_message) - 1u] = '\0';
    status_expire_ms = system_getval_ms() + 1500u; // 1.5s 后过期
}

static void clear_status_message(void)
{
    status_message[0] = '\0';
    status_expire_ms = 0u;
}

// 取出并清空一次菜单按键事件。单槽缓存，消费后才允许下次事件进入（天然防连击）
static uint8 consume_menu_key(void)
{
    uint8 key = g_menu_key_result;
    g_menu_key_result = KEY_NONE;
    return key;
}

// 参数菜单总页数。空表也算 1 页，避免翻页出现 0 页
static uint8 get_parameter_page_count(void)
{
    if (0u == parameter_count)
    {
        return 1u;
    }
    return (uint8)(((uint16)parameter_count + PARAMS_PER_PAGE - 1u) / PARAMS_PER_PAGE); // 向上取整
}

// 单步调节量：浮点按小数位取 0.1/0.01/0.001，位数异常兜底 0.1
static float get_parameter_step(const parameter_struct *param)
{
    switch (param->decimal_places)
    {
    case 0u:
        return 1.0f;
    case 1u:
        return 0.1f;
    case 2u:
        return 0.01f;
    case 3u:
        return 0.001f;
    default:
        return 0.1f;
    }
}

// 以 float 形式读取当前参数。
static float get_parameter_value(const parameter_struct *param)
{
    return *param->value_ptr.float_ptr;
}

// 把 Flash 参数同步到运行时速度环变量，参数菜单一改就立即生效。
// 两组速度环 PI 共用同一套 Flash 增益。
static void apply_runtime_parameters(void)
{
    speed_PID1.KP = g_flash_parameters.speed_KP;
    speed_PID1.KI = g_flash_parameters.speed_KI;
    speed_PID1.IncrementLimit = g_flash_parameters.speed_IncrementLimit;

    speed_PID2.KP = g_flash_parameters.speed_KP;
    speed_PID2.KI = g_flash_parameters.speed_KI;
    speed_PID2.IncrementLimit = g_flash_parameters.speed_IncrementLimit;
}

// 写入参数：先把新值夹在 [min,max]，按类型写回，再同步到运行时控制结构
static void set_parameter_value(parameter_struct *param, float new_value)
{
    if (new_value < param->min_value)
    {
        new_value = param->min_value;
    }
    if (new_value > param->max_value)
    {
        new_value = param->max_value;
    }

    *param->value_ptr.float_ptr = new_value;

    apply_runtime_parameters();
}

// 注册当前仍参与控制的调参项：决定菜单顺序、显示名、范围、小数位。
// 地图速度由 nav_path.h 提供，旧 Speed/BaseSpd/环道/断路参数不再显示。
static void register_default_parameters(void)
{
    parameter_count = 0u; // 清空表，从头登记

    add_parameter("S_KP", &g_flash_parameters.speed_KP, 0.0f, 999.0f, 3u, 2u);
    add_parameter("S_KI", &g_flash_parameters.speed_KI, 0.0f, 999.0f, 3u, 2u);
    add_parameter("S_IncLim", &g_flash_parameters.speed_IncrementLimit, 0.0f, 9999.0f, 4u, 1u);
    add_parameter("MaxDuty", &max_duty_limit, 0.0f, 10000.0f, 5u, 1u);

    // 重登记后若选中项超出新表长度，拉回第一项避免越界
    if (param_menu_selection >= parameter_count)
    {
        param_menu_selection = 0u;
    }
}

// 绘制底部状态栏：未过期时优先显示临时状态消息，否则显示 footer 提示。先整行擦除再画
static void draw_status_or_footer(const char *footer_text)
{
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_show_string(0u, STATUS_Y, FULL_WIDTH_SPACES);

    if (('\0' != status_message[0]) && (system_getval_ms() < status_expire_ms))
    {
        ips114_set_color(RGB565_GREEN, RGB565_BLACK);
        ips114_show_string(0u, STATUS_Y, status_message);
    }
    else if (NULL != footer_text)
    {
        ips114_set_color(RGB565_WHITE, RGB565_BLACK);
        ips114_show_string(0u, STATUS_Y, footer_text);
    }
}

// 绘制主菜单：选项超过一屏(7行)时按选中项滚动使其始终可见，选中项黄底高亮
static void draw_main_menu(void)
{
    uint8 i;
    uint8 first_item = 0u;

    // 选中项落到第 8 项及以后时窗口下滚，让它停在可见区最后一行
    if (main_menu_selection >= MAIN_MENU_VISIBLE_LINES)
    {
        first_item = (uint8)(main_menu_selection - MAIN_MENU_VISIBLE_LINES + 1u);
    }

    menu_set_font(IPS114_6X8_FONT);
    ips114_clear();
    ips114_set_color(RGB565_CYAN, RGB565_BLACK);
    ips114_show_string(0u, 0u, "Key Menu");

    for (i = 0u;
         (i < MAIN_MENU_VISIBLE_LINES) &&
         ((uint8)(first_item + i) < main_menu_item_count);
         i++)
    {
        uint8 item_index = (uint8)(first_item + i);

        if (item_index == main_menu_selection)
        {
            ips114_set_color(RGB565_BLACK, RGB565_YELLOW); // 选中：黑字黄底
        }
        else
        {
            ips114_set_color(RGB565_WHITE, RGB565_BLACK); // 普通：白字黑底
        }
        ips114_show_string(4u,
                          (uint16)(MAIN_MENU_START_Y + i * MAIN_MENU_LINE_HEIGHT),
                          main_menu_items[item_index]);
    }

    draw_status_or_footer("U/D Move C Enter");
}

// 绘制参数调节菜单：选中项黄色高亮、编辑态绿色高亮，只画当前页 PARAMS_PER_PAGE 行，右上角显示页码
static void draw_param_adjust_menu(void)
{
    uint8 page;
    uint8 page_count;
    uint8 start;
    uint8 end;
    uint8 i;
    uint16 y;
    char page_text[20];
    parameter_struct *param;

    menu_set_font(IPS114_8X16_FONT);
    page = (uint8)(param_menu_selection / PARAMS_PER_PAGE); // 选中项落在第几页
    page_count = get_parameter_page_count();
    start = (uint8)(page * PARAMS_PER_PAGE); // 本页第一项下标
    end = (uint8)(start + PARAMS_PER_PAGE);  // 本页末项之后（可能越过表尾，下面夹）
    if (end > parameter_count)
    {
        end = parameter_count;
    }

    ips114_clear();
    ips114_set_color(RGB565_CYAN, RGB565_BLACK);
    ips114_show_string(0u, 0u, "Parameter Adjust");
    sprintf(page_text, "Page %u/%u", (unsigned int)(page + 1u), (unsigned int)page_count);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_show_string(144u, 0u, page_text);

    for (i = start; i < end; i++)
    {
        param = &parameters[i];
        y = (uint16)(16u + (i - start) * CHAR_HEIGHT); // 本页内第 (i-start) 行的 y

        if (i == param_menu_selection)
        {
            // 选中项：编辑中绿底、未编辑黄底
            ips114_set_color(RGB565_BLACK, param_menu_edit_mode ? RGB565_GREEN : RGB565_YELLOW);
        }
        else
        {
            ips114_set_color(RGB565_WHITE, RGB565_BLACK);
        }

        ips114_show_string(4u, y, param->name);

        ips114_show_float(MENU_VALUE_X, y, *param->value_ptr.float_ptr,
                          param->int_digits, param->decimal_places);
    }

    // 底部提示随编辑态切换
    draw_status_or_footer(param_menu_edit_mode ? "U/D +/- L/R fast C OK R Undo" : "U/D Sel L/R Pg C Edit");
}

// 绘制运行信息页：16路线路阵列、yaw、距离、处理耗时和采样帧率。
static void draw_runtime_info_menu(void)
{
    PhototubeArraySnapshot sensor;
    char line[40];
    uint8 have_sensor = phototube_array_get_snapshot(&sensor);

    menu_set_font(IPS114_8X16_FONT);
    ips114_clear();
    ips114_set_color(RGB565_CYAN, RGB565_BLACK);
    ips114_show_string(0u, 0u, "Runtime Info");
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);

    sprintf(line, "Line:%+6.1f mm", have_sensor ? sensor.front_offset_mm : 0.0f);
    ips114_show_string(0u, 16u, line);

    sprintf(line, "Valid:%u Edge:%u N:%02u",
            (unsigned int)(have_sensor && sensor.valid),
            (unsigned int)(have_sensor && sensor.edge_detected),
            (unsigned int)(have_sensor ? sensor.active_count : 0u));
    ips114_show_string(0u, 32u, line);

    sprintf(line, "Signal:%4.1f Contrast:%3u",
            have_sensor ? sensor.signal_sum : 0.0f,
            (unsigned int)(have_sensor ? sensor.contrast_gray : 0.0f));
    ips114_show_string(0u, 48u, line);

    sprintf(line, "Yaw:%7.1f", gyro_get_heading_deg());
    ips114_show_string(0u, 64u, line);

    sprintf(line, "Dist:%1.3f", total_dist);
    ips114_show_string(0u, 80u, line);

    sprintf(line, "Mask:%04X Proc:%3luus",
            (unsigned int)(have_sensor ? sensor.active_mask : 0u),
            (unsigned long)time_cost);
    ips114_show_string(0u, 96u, line);

    sprintf(line, "ADC:%u FPS:%3u", (unsigned int)g_phototube_ready,
            (unsigned int)PHOTOTUBE_ARRAY_SCAN_HZ);
    ips114_show_string(0u, 112u, line);

    draw_status_or_footer("R Back");
}

// 16路传感器详情页：显示12位ADC原值和固定阈值分割结果。
static void draw_line_sensor_info_menu(void)
{
    PhototubeArraySnapshot sensor;
    char line[41];
    uint8 have_sensor = phototube_array_get_snapshot(&sensor);
    uint8 group;

    if (!have_sensor)
    {
        memset(&sensor, 0, sizeof(sensor));
    }
    menu_set_font(IPS114_6X8_FONT);
    ips114_clear();
    ips114_set_color(RGB565_CYAN, RGB565_BLACK);
    ips114_show_string(0u, 0u, "16ch Analog Phototube");
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);

    sprintf(line, "A:007 S:006/005/004/003 sp:%umm f:%umm",
            (unsigned int)PHOTOTUBE_ARRAY_SPACING_MM,
            (unsigned int)PHOTOTUBE_ARRAY_FORWARD_MM);
    ips114_show_string(0u, 8u, line);
    sprintf(line, "frame:%lu fps:%lu",
            (unsigned long)sensor.frame_seq,
            (unsigned long)PHOTOTUBE_ARRAY_SCAN_HZ);
    ips114_show_string(0u, 16u, line);
    sprintf(line, "pos:%+5.1f V:%u E:%u N:%u",
            sensor.front_offset_mm,
            (unsigned int)sensor.valid,
            (unsigned int)sensor.edge_detected,
            (unsigned int)sensor.active_count);
    ips114_show_string(0u, 24u, line);
    sprintf(line, "sig:%4.1f con:%3u A:%02u T:%04u",
            sensor.signal_sum,
            (unsigned int)sensor.contrast_gray,
            (unsigned int)sensor.white_percent,
            (unsigned int)PHOTOTUBE_ARRAY_LINE_THRESHOLD_ADC);
    ips114_show_string(0u, 32u, line);
    sprintf(line, "mask:%04X P:%u F:%u",
            (unsigned int)sensor.active_mask,
            (unsigned int)sensor.peak_count,
            (unsigned int)sensor.full_coverage);
    ips114_show_string(0u, 40u, line);

    for (group = 0u; group < 4u; group++)
    {
        uint8 base = (uint8)(group * 4u);
        sprintf(line, "ADC%02u:%4u %4u %4u %4u",
                (unsigned int)base,
                (unsigned int)sensor.raw_adc[base],
                (unsigned int)sensor.raw_adc[base + 1u],
                (unsigned int)sensor.raw_adc[base + 2u],
                (unsigned int)sensor.raw_adc[base + 3u]);
        ips114_show_string(0u, (uint16)(48u + group * 8u), line);
    }
    draw_status_or_footer("R Back");
}

// 组装模拟光电调试图：每个柱直接表示一路12位原始ADC，黄色横线是固定判定阈值。
static void build_debug_sensor_image(const PhototubeArraySnapshot *sensor)
{
    uint16 row;
    uint16 col;
    uint8 index;
    uint16 threshold_y;

    for (row = 0u; row < DEBUG_IMG_H; row++)
    {
        for (col = 0u; col < DEBUG_IMG_W; col++)
        {
            g_image_debug_overlay_buffer[row][col] = RGB565_BLACK;
        }
    }

    if (NULL == sensor)
    {
        return;
    }

    for (index = 0u; index < PHOTOTUBE_ARRAY_COUNT; index++)
    {
        uint16 x_start = (uint16)index * (DEBUG_IMG_W / PHOTOTUBE_ARRAY_COUNT) + 1u;
        uint16 x_end = (uint16)(index + 1u) * (DEBUG_IMG_W / PHOTOTUBE_ARRAY_COUNT) - 2u;
        uint16 raw_adc = sensor->raw_adc[index];
        uint16 bar_height;
        uint16 y_start;
        uint16 color = RGB565_CYAN;
        uint16 x;
        uint16 y;

        if (raw_adc > PHOTOTUBE_ARRAY_ADC_FULL_SCALE)
        {
            raw_adc = PHOTOTUBE_ARRAY_ADC_FULL_SCALE;
        }
        bar_height = (uint16)((uint32)raw_adc * (DEBUG_IMG_H - 10u) /
                              PHOTOTUBE_ARRAY_ADC_FULL_SCALE);
        y_start = DEBUG_IMG_H - 1u - bar_height;

        if (0u != (sensor->active_mask & (uint16)(1u << index)))
        {
            color = RGB565_GREEN;
        }
        if (bar_height > 0u)
        {
            for (y = y_start; y < DEBUG_IMG_H; y++)
            {
                for (x = x_start; x <= x_end; x++)
                {
                    g_image_debug_overlay_buffer[y][x] = color;
                }
            }

        }

        // 即使ADC为0，也在零刻度处保留一条采样标记，避免命中线路后整柱消失。
        for (x = x_start; x <= x_end; x++)
        {
            g_image_debug_overlay_buffer[y_start][x] = color;
        }
    }

    threshold_y = DEBUG_IMG_H - 1u -
        (uint16)((uint32)PHOTOTUBE_ARRAY_LINE_THRESHOLD_ADC *
                 (DEBUG_IMG_H - 10u) / PHOTOTUBE_ARRAY_ADC_FULL_SCALE);
    for (col = 0u; col < DEBUG_IMG_W; col++)
    {
        g_image_debug_overlay_buffer[threshold_y][col] = RGB565_YELLOW;
    }
}

// 组装位姿补偿后的扫图：横轴为扫图坐标 x，纵轴为横向 y；绿点是实际命中点，
// 黄色线是 y=kx+b 拟合结果。坐标范围按这一段扫图自动缩放，停车后可直接看直线度。
static void build_debug_scan_image(const PhototubeScanSnapshot *scan)
{
    uint16 row;
    uint16 col;
    uint16 index;
    float x_span;
    float y_min;
    float y_max;
    float y_span;
    float line_y;

    for (row = 0u; row < DEBUG_IMG_H; row++)
    {
        for (col = 0u; col < DEBUG_IMG_W; col++)
        {
            g_image_debug_overlay_buffer[row][col] = RGB565_BLACK;
        }
    }

    if ((NULL == scan) || (0u == scan->point_count))
    {
        return;
    }

    x_span = scan->x_max_mm - scan->x_min_mm;
    if (x_span < 1.0f)
    {
        x_span = 1.0f;
    }

    y_min = 0.0f;
    y_max = 0.0f;
    for (index = 0u; index < scan->point_count; index++)
    {
        PhototubeScanPoint point;
        if (!phototube_scan_get_point(index, &point))
        {
            continue;
        }
        if ((0u == index) || (point.y_mm < y_min)) y_min = point.y_mm;
        if ((0u == index) || (point.y_mm > y_max)) y_max = point.y_mm;
    }
    if (scan->valid)
    {
        line_y = scan->line_slope * scan->x_min_mm + scan->line_intercept;
        if (line_y < y_min) y_min = line_y;
        line_y = scan->line_slope * scan->x_max_mm + scan->line_intercept;
        if (line_y > y_max) y_max = line_y;
    }
    y_span = y_max - y_min;
    if (y_span < 40.0f)
    {
        float center = 0.5f * (y_min + y_max);
        y_min = center - 20.0f;
        y_max = center + 20.0f;
        y_span = 40.0f;
    }
    y_min -= 0.10f * y_span;
    y_max += 0.10f * y_span;
    y_span = y_max - y_min;

    // 中心线 y=0；显示与地图编辑器一致：车辆右侧 y+ 朝屏幕下方。
    if ((0.0f >= y_min) && (0.0f <= y_max))
    {
        uint16 axis_y = (uint16)(((0.0f - y_min) / y_span) *
                                 (float)(DEBUG_IMG_H - 1u));
        if (axis_y < DEBUG_IMG_H)
        {
            for (col = 0u; col < DEBUG_IMG_W; col++)
            {
                g_image_debug_overlay_buffer[axis_y][col] = RGB565_BLUE;
            }
        }
    }

    if (scan->valid)
    {
        for (col = 0u; col < DEBUG_IMG_W; col++)
        {
            float x = scan->x_min_mm +
                      x_span * (float)col / (float)(DEBUG_IMG_W - 1u);
            float fitted_y = scan->line_slope * x + scan->line_intercept;
            int16 pixel_y = (int16)((fitted_y - y_min) / y_span *
                                    (float)(DEBUG_IMG_H - 1u) + 0.5f);
            if ((pixel_y >= 0) && (pixel_y < (int16)DEBUG_IMG_H))
            {
                g_image_debug_overlay_buffer[(uint16)pixel_y][col] = RGB565_YELLOW;
            }
        }
    }

    for (index = 0u; index < scan->point_count; index++)
    {
        PhototubeScanPoint point;
        int16 pixel_x;
        int16 pixel_y;
        if (!phototube_scan_get_point(index, &point))
        {
            continue;
        }
        pixel_x = (int16)((point.x_mm - scan->x_min_mm) / x_span *
                          (float)(DEBUG_IMG_W - 1u) + 0.5f);
        pixel_y = (int16)((point.y_mm - y_min) / y_span *
                          (float)(DEBUG_IMG_H - 1u) + 0.5f);
        if ((pixel_x >= 0) && (pixel_x < (int16)DEBUG_IMG_W) &&
            (pixel_y >= 0) && (pixel_y < (int16)DEBUG_IMG_H))
        {
            g_image_debug_overlay_buffer[(uint16)pixel_y][(uint16)pixel_x] =
                RGB565_GREEN;
            if (pixel_x + 1 < (int16)DEBUG_IMG_W)
            {
                g_image_debug_overlay_buffer[(uint16)pixel_y][(uint16)(pixel_x + 1)] =
                    RGB565_GREEN;
            }
        }
    }
}

static void draw_launch_status_line(uint16 y)
{
    char line[24];

    if (0u == is_car_running)
    {
        if (!gyro_cpu1_service_ready())
        {
            sprintf(line, "%-15s", "IMU NG");
        }
        else if (!gyro_zero_calibration_done())
        {
            sprintf(line, "%-15s", "IMU UNCAL");
        }
        else
        {
            sprintf(line, "%-15s", "IMU OK");
        }
    }
    else if (prestart_is_warming_up())
    {
        sprintf(line, "%-15s", "WARMUP wait..");
    }
    else
    {
#if ENCODER_TEST_MODE_ENABLE
        sprintf(line, "%-15s", "TEST K3:Stop");
#else
        sprintf(line, "%-15s", "RUN   K3:Stop");
#endif
    }
    ips114_show_string(2u, y, line);
}

// 模拟光电发车/调试页：显示16路强度柱和点云诊断。
static void draw_image_debug_menu(uint8 full_refresh)
{
    PhototubeArraySnapshot sensor;
    PhototubeScanSnapshot scan;
    char line[40];
    uint8 have_sensor;
    float map_tx_mm;
    float map_ty_mm;
    float map_yaw_deg;

    if (full_refresh)
    {
        ips114_clear();
    }

    if (!g_phototube_ready)
    {
        menu_set_font(IPS114_8X16_FONT);
        ips114_set_color(RGB565_RED, RGB565_BLACK);
        ips114_show_string(0u, 0u, "Phototube Source Off");
        menu_set_font(IPS114_6X8_FONT);
        ips114_set_color(RGB565_WHITE, RGB565_BLACK);
        draw_launch_status_line(24u);
        return;
    }

    if (!g_image_debug_has_frame)
    {
        menu_set_font(IPS114_8X16_FONT);
        ips114_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips114_show_string(0u, 0u, "Waiting Frame");
        menu_set_font(IPS114_6X8_FONT);
        ips114_set_color(RGB565_WHITE, RGB565_BLACK);
        draw_launch_status_line(24u);
        return;
    }

    have_sensor = phototube_array_get_snapshot(&sensor);
    memset(&scan, 0, sizeof(scan));
    phototube_scan_get_snapshot(&scan);
    if (!have_sensor)
    {
        memset(&sensor, 0, sizeof(sensor));
    }

    // 运行中始终显示16路实时光电，避免扫图旁路把传感器诊断页替换掉；
    // 停车后才显示本次运行累积的位姿补偿扫图结果。
    if ((0u == scan.active) && (scan.point_count > 0u))
    {
        build_debug_scan_image(&scan);
    }
    else
    {
        build_debug_sensor_image(have_sensor ? &sensor : NULL);
    }
    ips114_show_rgb565_image(DEBUG_BIN_X, 0u, g_image_debug_overlay_buffer[0], DEBUG_IMG_W, DEBUG_IMG_H, DEBUG_IMG_W,
                             DEBUG_IMG_H, 0u);

    // 竖屏下信息改为图像下方单列排布；字段全部定宽，配合分帧局部刷新原位覆盖旧值
    menu_set_font(IPS114_6X8_FONT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);

    sprintf(line, "Pos:%+5.1f V:%u E:%u",
            sensor.front_offset_mm,
            (unsigned int)sensor.valid,
            (unsigned int)sensor.edge_detected);
    ips114_show_string(2u, DEBUG_INFO_Y + 0u * DEBUG_INFO_STEP, line);

    sprintf(line, "Sig:%4.1f Con:%3u N:%02u",
            sensor.signal_sum,
            (unsigned int)sensor.contrast_gray,
            (unsigned int)sensor.active_count);
    ips114_show_string(2u, DEBUG_INFO_Y + 1u * DEBUG_INFO_STEP, line);

    sprintf(line, "M:%04X P:%u F:%u",
            (unsigned int)sensor.active_mask,
            (unsigned int)sensor.peak_count,
            (unsigned int)sensor.full_coverage);
    ips114_show_string(2u, DEBUG_INFO_Y + 2u * DEBUG_INFO_STEP, line);

    sprintf(line, "R0:%02X%02X%02X%02X%02X%02X%02X%02X",
            (unsigned int)(sensor.raw_adc[0] >> 4u),
            (unsigned int)(sensor.raw_adc[1] >> 4u),
            (unsigned int)(sensor.raw_adc[2] >> 4u),
            (unsigned int)(sensor.raw_adc[3] >> 4u),
            (unsigned int)(sensor.raw_adc[4] >> 4u),
            (unsigned int)(sensor.raw_adc[5] >> 4u),
            (unsigned int)(sensor.raw_adc[6] >> 4u),
            (unsigned int)(sensor.raw_adc[7] >> 4u));
    ips114_show_string(2u, DEBUG_INFO_Y + 3u * DEBUG_INFO_STEP, line);

    sprintf(line, "R8:%02X%02X%02X%02X%02X%02X%02X%02X",
            (unsigned int)(sensor.raw_adc[8] >> 4u),
            (unsigned int)(sensor.raw_adc[9] >> 4u),
            (unsigned int)(sensor.raw_adc[10] >> 4u),
            (unsigned int)(sensor.raw_adc[11] >> 4u),
            (unsigned int)(sensor.raw_adc[12] >> 4u),
            (unsigned int)(sensor.raw_adc[13] >> 4u),
            (unsigned int)(sensor.raw_adc[14] >> 4u),
            (unsigned int)(sensor.raw_adc[15] >> 4u));
    ips114_show_string(2u, DEBUG_INFO_Y + 4u * DEBUG_INFO_STEP, line);

    sprintf(line, "FPS:%3lu Proc:%3luus",
            (unsigned long)PHOTOTUBE_ARRAY_SCAN_HZ,
            (unsigned long)time_cost);
    ips114_show_string(2u, DEBUG_INFO_Y + 5u * DEBUG_INFO_STEP, line);

    sprintf(line, "Pts:%04u Fit:%u",
            (unsigned int)scan.point_count,
            (unsigned int)scan.valid);
    ips114_show_string(2u, DEBUG_INFO_Y + 6u * DEBUG_INFO_STEP, line);

    sprintf(line, "Stop:%u Loc:%03u",
            (unsigned int)g_stop_reason,
            (unsigned int)(nav_get_localization_update_sequence() % 1000u));
    ips114_show_string(2u, DEBUG_INFO_Y + 7u * DEBUG_INFO_STEP, line);

    nav_get_map_from_odom(&map_tx_mm, &map_ty_mm, &map_yaw_deg);
    sprintf(line, "Y%+4.0f M%+4.1f T%4.0f",
            gyro_get_heading_deg(), map_yaw_deg,
            sqrtf(map_tx_mm * map_tx_mm + map_ty_mm * map_ty_mm));
    ips114_show_string(2u, DEBUG_INFO_Y + 8u * DEBUG_INFO_STEP, line);

    // 第 9 行：校准状态/发车按键提示。停车/预热/待确认/运行四态各不同。
    draw_launch_status_line(DEBUG_INFO_Y + 9u * DEBUG_INFO_STEP);

    // 第 10 行：正常跑图旁路扫图点数/最近窗口拟合误差。RMS 只描述局部窗口，拐弯不会被误判为全图失败。
    sprintf(line, "Scan:%u P:%03u R:%4.1f",
            (unsigned int)scan.valid,
            (unsigned int)scan.point_count,
            scan.rms_error_mm);
    ips114_show_string(2u, DEBUG_INFO_Y + 10u * DEBUG_INFO_STEP, line);
}

// 处理主菜单按键：上/下循环移动；确认进对应页面或执行保存/加载/恢复默认这类一次性动作。
// 跑车时(is_car_running)禁止保存/加载/改默认，避免写 Flash 打断控制
static void handle_main_menu_input(uint8 key)
{
    if (KEY_UP == key)
    {
        // 上移，第 0 项再上绕到末项
        main_menu_selection = (0u == main_menu_selection)
                                  ? (uint8)(main_menu_item_count - 1u)
                                  : (uint8)(main_menu_selection - 1u);
        menu_needs_refresh = 1u;
    }
    else if (KEY_DOWN == key)
    {
        // 下移，越过末项绕回第 0 项
        main_menu_selection++;
        if (main_menu_selection >= main_menu_item_count)
        {
            main_menu_selection = 0u;
        }
        menu_needs_refresh = 1u;
    }
    else if (KEY_CONFIRM == key)
    {
        switch (main_menu_selection)
        {
        case 0u:
            // 虚拟光电页=发车页（唯一入口）：进入前先停车，再以当前摆位重开惯导调试会话。
            if (!prestart_is_idle())
            {
                prestart_stop();
            }
            control_disable();
            current_menu = MENU_IMAGE_DEBUG;
            // 开机已在 cpu0_main.c 开过一次推车会话(本页是开机默认页,从不经过这条按键分支);
            // 这里覆盖"退出到主菜单再按确认重进"场景——同样以当前摆位/朝向为原点重开,想重零就退出再进
            nav_start();
            break;
        case 1u:
            current_menu = MENU_LINE_SENSOR_INFO;
            break;
        case 2u:
            current_menu = MENU_PARAM_ADJUST;
            break;
        case 3u:
            current_menu = MENU_RUNTIME_INFO;
            break;
        case 4u:
            // 保存参数到 Flash（仅停车时允许）
            if (!is_car_running)
            {
                flash_params_save();
                set_status_message("Parameters saved");
            }
            else
            {
                set_status_message("Stop car before save");
            }
            break;
        case 5u:
            // 从 Flash 重新载入参数并同步到运行时（仅停车时允许）
            if (is_car_running)
            {
                set_status_message("Stop car before reload");
            }
            else if (flash_params_load())
            {
                apply_runtime_parameters();
                set_status_message("Parameters reloaded");
            }
            else
            {
                set_status_message("Reload failed");
            }
            break;
        case 6u:
            // 恢复出厂默认参数并同步（仅停车时允许，不自动写 Flash）
            if (!is_car_running)
            {
                flash_params_reset_default();
                apply_runtime_parameters();
                set_status_message("Defaults loaded");
            }
            else
            {
                set_status_message("Stop car before default");
            }
            break;
        default:
            break;
        }
        menu_needs_refresh = 1u;
    }
}

// 处理参数调节页按键：非编辑模式管选项和翻页，编辑模式管改值/保存/撤销。空表时只响应返回键
static void handle_param_adjust_input(uint8 key)
{
    parameter_struct *param;
    uint8 current_page;
    uint8 page_count;
    float step;
    float value;

    if (0u == parameter_count)
    {
        if (KEY_RETURN == key)
        {
            current_menu = MENU_MAIN;
            menu_needs_refresh = 1u;
        }
        return;
    }

    param = &parameters[param_menu_selection];
    current_page = (uint8)(param_menu_selection / PARAMS_PER_PAGE);
    page_count = get_parameter_page_count();

    if (!param_menu_edit_mode)
    {
        // 非编辑模式：选择 + 翻页
        if (KEY_UP == key)
        {
            // 上移一项，循环
            param_menu_selection = (0u == param_menu_selection) ? (parameter_count - 1u) : (param_menu_selection - 1u);
            menu_needs_refresh = 1u;
        }
        else if (KEY_DOWN == key)
        {
            // 下移一项，循环
            param_menu_selection++;
            if (param_menu_selection >= parameter_count)
            {
                param_menu_selection = 0u;
            }
            menu_needs_refresh = 1u;
        }
        else if (KEY_LEFT == key)
        {
            // 上一页首项，第一页则绕到最后一页首项，再夹到表尾内
            param_menu_selection = (0u == current_page) ? (uint8)((page_count - 1u) * PARAMS_PER_PAGE)
                                                        : (uint8)((current_page - 1u) * PARAMS_PER_PAGE);
            if (param_menu_selection >= parameter_count)
            {
                param_menu_selection = parameter_count - 1u;
            }
            menu_needs_refresh = 1u;
        }
        else if (KEY_RIGHT == key)
        {
            // 下一页首项，最后一页则绕回第 0 项，再夹到表尾内
            param_menu_selection =
                ((current_page + 1u) >= page_count) ? 0u : (uint8)((current_page + 1u) * PARAMS_PER_PAGE);
            if (param_menu_selection >= parameter_count)
            {
                param_menu_selection = parameter_count - 1u;
            }
            menu_needs_refresh = 1u;
        }
        else if (KEY_CONFIRM == key)
        {
            // 进编辑模式：先记下原值方便撤销
            param_menu_edit_original_value = get_parameter_value(param);
            param_menu_edit_mode = 1u;
            menu_needs_refresh = 1u;
        }
        else if (KEY_RETURN == key)
        {
            current_menu = MENU_MAIN;
            menu_needs_refresh = 1u;
        }
        return;
    }

    // 编辑模式：改数值 / 确认 / 撤销
    step = get_parameter_step(param);
    value = get_parameter_value(param);

    if (KEY_UP == key)
    {
        set_parameter_value(param, value + step); // +1 步
        menu_needs_refresh = 1u;
    }
    else if (KEY_DOWN == key)
    {
        set_parameter_value(param, value - step); // -1 步
        menu_needs_refresh = 1u;
    }
    else if (KEY_RIGHT == key)
    {
        set_parameter_value(param, value + step * 10.0f); // +10 步（快调）
        menu_needs_refresh = 1u;
    }
    else if (KEY_LEFT == key)
    {
        set_parameter_value(param, value - step * 10.0f); // -10 步（快调）
        menu_needs_refresh = 1u;
    }
    else if (KEY_CONFIRM == key)
    {
        // 确认：退出编辑，停车时顺手存盘，跑车时只运行期生效不写 Flash
        param_menu_edit_mode = 0u;
        if (!is_car_running)
        {
            flash_params_save();
            set_status_message("Saved");
        }
        else
        {
            set_status_message("Runtime only");
        }
        menu_needs_refresh = 1u;
    }
    else if (KEY_RETURN == key)
    {
        // 撤销：还原成进编辑前的原值，退出编辑
        set_parameter_value(param, param_menu_edit_original_value);
        param_menu_edit_mode = 0u;
        set_status_message("Edit cancelled");
        menu_needs_refresh = 1u;
    }
}

// 运行信息页只响应返回键（只读页）
static void handle_runtime_info_input(uint8 key)
{
    if (KEY_RETURN == key)
    {
        current_menu = MENU_MAIN;
        menu_needs_refresh = 1u;
    }
}

static uint8 begin_prestart_transaction(void)
{
    if (!prestart_begin())
    {
        return 0u;
    }
    is_car_running = 1u;
    return 1u;
}

// 光电调试页按键（发车页已并入本页）：KEY_1短按依次开始、停止/冻结、导出。
// ADC扫描帧通过 globals.c 中的 CPU1→CPU0 信号驱动本页刷新。
static void handle_image_debug_input(uint8 key)
{
    if (KEY_RETURN == key)
    {
        // 返回主菜单前先确保停车，避免带着运行状态离开本页
        if (!prestart_is_idle())
        {
            prestart_stop();
        }
        control_disable();
        current_menu = MENU_MAIN;
        menu_needs_refresh = 1u;
    }
    else if (KEY_CONFIRM == key)
    {
        // 短按确认在默认流程中执行：手动起步→预热/发车→运行中停车→冻结后导出
        if (0u == is_car_running)
        {
            /* A stop request keeps the state machine in STOPPING for 500 ms;
             * do not let the next one-key press reopen it while EMEM queues
             * are still being drained. */
            if (TELEMETRY_STATE_FROZEN == telemetry_get_state())
            {
                /* One-key sequence: start -> stop/freeze -> export. */
                if (telemetry_request_export())
                {
                    set_status_message("Export requested");
                }
                else
                {
                    set_status_message("Log busy");
                }
            }
            else if (!prestart_is_idle())
            {
                set_status_message("Saving");
            }
            else if (TELEMETRY_STATE_EXPORTING == telemetry_get_state())
            {
                set_status_message("Saving");
            }
            else if (TELEMETRY_STATE_DISABLED == telemetry_get_state())
            {
                /* Do not enter a no-op bench session when EMEM was rejected
                 * by the hardware gate or its self-test.  That state used to
                 * look like a frozen run with no serial export possible. */
                set_status_message("EMEM disabled");
            }
            else
            {
                // 第一次按：实车模式开始预热；台架模式直接进入静止 EMEM 测试流程
                if (!begin_prestart_transaction())
                {
                    if (!gyro_cpu1_service_ready())
                    {
                        set_status_message("IMU not ready");
                    }
                    else if (!gyro_zero_calibration_done())
                    {
                        set_status_message("IMU not calibrated");
                    }
                    else
                    {
                        set_status_message("IMU not ready");
                    }
                    return;
                }
#if PRESTART_BENCH_TEST_MODE
                set_status_message("Bench test");
#else
                set_status_message("Warmup");
#endif
            }
        }
        else if (prestart_is_warming_up())
        {
            set_status_message("Wait warmup");
        }
        else
        {
            // 已在运行：本次短按当作手动停车
            g_stop_reason = STOP_REASON_MANUAL;
            prestart_stop();
            set_status_message("Car stopped");
        }
        menu_needs_refresh = 1u;
    }
    else if (KEY_CONFIRM_LONG == key)
    {
        // 长按确认：无条件急停（记手动停车原因）
        g_stop_reason = STOP_REASON_MANUAL;
        prestart_stop();
        set_status_message("E-STOP");
        menu_needs_refresh = 1u;
    }
}

// 线路传感器详情页只读，返回主菜单。
static void handle_line_sensor_info_input(uint8 key)
{
    if (KEY_RETURN == key)
    {
        current_menu = MENU_MAIN;
        menu_needs_refresh = 1u;
    }
}

// 当前菜单只登记仍参与控制的 float 参数。

uint8 add_parameter(const char *name, float *value_ptr, float min_value, float max_value, uint8 int_digits,
                    uint8 decimal_places)
{
    if ((NULL == name) || (NULL == value_ptr) || (parameter_count >= MAX_PARAMETERS))
    {
        return 0u;
    }

    strncpy(parameters[parameter_count].name, name, sizeof(parameters[parameter_count].name) - 1u);
    parameters[parameter_count].name[sizeof(parameters[parameter_count].name) - 1u] = '\0';
    parameters[parameter_count].value_ptr.float_ptr = value_ptr;
    parameters[parameter_count].type = PARAM_TYPE_FLOAT;
    parameters[parameter_count].min_value = min_value;
    parameters[parameter_count].max_value = max_value;
    parameters[parameter_count].int_digits = int_digits;
    parameters[parameter_count].decimal_places = decimal_places;
    parameter_count++;
    return 1u;
}

// 汇总实体按键和 ADC 按键事件：KEY1 短按=确认、长按=急停；ADC 五向键=方向/返回。
// 结果写进单槽 g_menu_key_result（仅槽空时写，先到先得防覆盖）
void key_get(void)
{
    adc_key_enum adc_key;
    key_state_enum key3_state = key_get_state(KEY_1);

    if (KEY_SHORT_PRESS == key3_state)
    {
        // KEY1 短按 → 确认。写完清状态并复位长按计数
        if (KEY_NONE == g_menu_key_result)
        {
            g_menu_key_result = KEY_CONFIRM;
        }
        key_clear_state(KEY_1);
        key3_long_press_count = 0u;
    }
    else if (KEY_LONG_PRESS == key3_state)
    {
        // KEY1 长按 → 急停，一次长按只触发一次（key3_long_press_count 去重）
        if (0u == key3_long_press_count)
        {
            if (KEY_NONE == g_menu_key_result)
            {
                g_menu_key_result = KEY_CONFIRM_LONG;
            }
            key3_long_press_count = 1u;
        }
    }
    else if (0u != key3_long_press_count)
    {
        // 长按松手后：复位去重计数并清状态，为下次长按做准备
        key3_long_press_count = 0u;
        key_clear_state(KEY_1);
    }

    // ADC 五向摇杆 → 上/下/左/右/返回。同样只在槽空时写
    adc_key = adc_key_get();
    if (ADC_KEY_NONE != adc_key)
    {
        if (KEY_NONE == g_menu_key_result)
        {
            switch (adc_key)
            {
            case ADC_KEY_UP:
                g_menu_key_result = KEY_UP;
                break;
            case ADC_KEY_DOWN:
                g_menu_key_result = KEY_DOWN;
                break;
            case ADC_KEY_LEFT:
                g_menu_key_result = KEY_LEFT;
                break;
            case ADC_KEY_RIGHT:
                g_menu_key_result = KEY_RIGHT;
                break;
            case ADC_KEY_PRESS:
                g_menu_key_result = KEY_RETURN;
                break;
            default:
                break;
            }
        }
    }
}

// 初始化菜单平台资源：ADC 按键、KEY1、发车状态机 5ms PIT、屏幕方向和菜单状态
void menu_platform_init(void)
{
    // 屏幕未连接背光控制脚。P15_4在SPI初始化时仅作伪MISO，随后由CPU1配置为IMU INT2输入。
    // 本函数必须先于gyro_cpu1_init()，上电后不得再次调用ips114_init()抢走ERU配置。
    // 上电后任何地方都不得再调 ips114_init()，运行期切方向一律用 ips114_change_dir()
    ips114_set_dir(IPS114_PORTAIT);
    ips114_init();
    menu_init();
}

// 初始化菜单内部状态和参数表，默认进入光电调试/发车页。
void menu_init(void)
{
    apply_runtime_parameters();
    register_default_parameters();
    g_menu_key_result = KEY_NONE;
    g_menu_frame_ready = 0u;
    g_image_debug_has_frame = 0u;
    g_menu_frame_seen_seq = phototube_menu_frame_seq;
    current_menu = MENU_IMAGE_DEBUG;
    main_menu_selection = 0u; // 光电调试/发车页是主菜单第一项
    param_menu_selection = 0u;
    param_menu_edit_mode = 0u;
    param_menu_edit_original_value = 0.0f;
    menu_needs_refresh = 1u;
    last_dynamic_refresh_ms = 0u;
    clear_status_message();
    menu_set_font(IPS114_8X16_FONT);
    menu_initialized = 1u;
}

// 当前物理显示方向，初值与 menu_platform_init() 首次 ips114_init() 一致（横屏）
static ips114_dir_enum g_menu_display_dir = IPS114_PORTAIT;

// 按当前页切换屏幕方向：光电调试页竖屏，其余页面横屏。
// 必须用ips114_change_dir()（只发MADCTL+清屏），严禁重复调用ips114_init()：init会把P15_4
// 重配为SPI伪MISO，而该脚运行期是IMU660RC INT2的ERU输入。
// 清屏走 SPI 仍有几十 ms，保持只在停车时切换
static void menu_reconcile_orientation(void)
{
    // 只有光电调试页使用竖屏(135x240)，其余页面使用横屏(240x135)。
    // CROSSWISE 才是竖屏，本车排线下竖屏正立必须用 CROSSWISE_180(CROSSWISE 会上下颠倒)
    ips114_dir_enum want = (MENU_IMAGE_DEBUG == current_menu) ? IPS114_CROSSWISE_180 : IPS114_PORTAIT;

    if (want == g_menu_display_dir)
    {
        return; // 已是目标方向
    }
    if (is_car_running)
    {
        return; // 跑车中不切方向（清屏耗时几十 ms，会打断显示）
    }

    ips114_change_dir(want);
    g_menu_display_dir = want;
    menu_needs_refresh = 1u;
}

// 菜单主任务（5ms 后台任务调用）：状态消息过期清理 → 取按键 → 特殊长按处理 → 分发给当前页 handle →
// 同步屏幕方向 → 需要则整页重画，否则按页面做低频动态刷新
void menu_task(void)
{
    uint8 key;
    uint32 now;

    if (!menu_initialized)
    {
        return; // menu_init() 还没跑过就什么都不做
    }

    if (g_phototube_ready != phototube_menu_ready_signal)
    {
        g_phototube_ready = phototube_menu_ready_signal;
        menu_needs_refresh = 1u;
    }
    if (g_menu_frame_seen_seq != phototube_menu_frame_seq)
    {
        g_menu_frame_seen_seq = phototube_menu_frame_seq;
        g_image_debug_has_frame = 1u;
        g_menu_frame_ready = 1u;
    }

    apply_runtime_parameters();
    now = system_getval_ms();
    // 底部状态消息到点就清掉；停车时顺便标记需重画（跑车时不打断显示）
    if ((status_expire_ms > 0u) && (now >= status_expire_ms))
    {
        clear_status_message();
        if (!is_car_running)
        {
            menu_needs_refresh = 1u;
        }
    }

    key = consume_menu_key();

#if ENCODER_TEST_MODE_ENABLE
    /* Map encoder test owns KEY_1 on every page so the normal menu handler
     * cannot start the ordinary suction/launch workflow a second time. */
    if ((KEY_NONE != key) && encoder_test_handle_key(key))
    {
        menu_needs_refresh = 1u;
        key = KEY_NONE;
    }
#endif

#if PRESTART_BENCH_TEST_MODE
    /* The third short press is an export command regardless of which menu
     * page is visible.  This keeps the complete bench workflow on KEY_1 and
     * prevents the same press from also entering a menu item. */
    if ((KEY_CONFIRM == key) &&
        (0u == is_car_running) &&
        (TELEMETRY_STATE_FROZEN == telemetry_get_state()))
    {
        if (telemetry_request_export())
        {
            set_status_message("Export requested");
        }
        else
        {
            set_status_message("Log busy");
        }
        menu_needs_refresh = 1u;
        key = KEY_NONE;
    }
#endif

    // 长按语义：发车页急停，其余普通页面保存参数。
    if ((KEY_CONFIRM_LONG == key) &&
        (MENU_IMAGE_DEBUG != current_menu) &&
        (MENU_LINE_SENSOR_INFO != current_menu) &&
        !is_car_running)
    {
        flash_params_save();
        set_status_message("Parameters saved");
        menu_needs_refresh = 1u;
        key = KEY_NONE; // 已在此消费掉，别再传给页面 handle 函数
    }

    if (KEY_NONE != key)
    {
        // 把按键交给当前所在页的处理函数——切页就是在这些函数里改 current_menu
        switch (current_menu)
        {
        case MENU_MAIN:
            handle_main_menu_input(key);
            break;
        case MENU_PARAM_ADJUST:
            handle_param_adjust_input(key);
            break;
        case MENU_RUNTIME_INFO:
            handle_runtime_info_input(key);
            break;
        case MENU_IMAGE_DEBUG:
            handle_image_debug_input(key);
            break;
        case MENU_LINE_SENSOR_INFO:
            handle_line_sensor_info_input(key);
            break;
        default:
            break;
        }
    }

    // 按键可能已切换页面，据新页面同步屏幕方向
    menu_reconcile_orientation();

    if (menu_needs_refresh)
    {
        // 整页重画：按当前页调对应 draw 函数，画完清标志并记刷新时间后返回
        switch (current_menu)
        {
        case MENU_MAIN:
            draw_main_menu();
            break;
        case MENU_PARAM_ADJUST:
            draw_param_adjust_menu();
            break;
        case MENU_RUNTIME_INFO:
            draw_runtime_info_menu();
            break;
        case MENU_IMAGE_DEBUG:
            draw_image_debug_menu(1u);
            break;
        case MENU_LINE_SENSOR_INFO:
            draw_line_sensor_info_menu();
            break;
        default:
            break;
        }
        menu_needs_refresh = 0u;
        g_menu_frame_ready = 0u;
        last_dynamic_refresh_ms = now;
        return;
    }

    // 无需整页重画时，各页面按需做低频局部刷新（不清屏，只覆盖动态字段）
    if (!is_car_running && (MENU_RUNTIME_INFO == current_menu))
    {
        // 运行信息页：每 100ms 刷一次
        if ((now - last_dynamic_refresh_ms) >= 100u)
        {
            draw_runtime_info_menu();
            last_dynamic_refresh_ms = now;
        }
    }
    else if (is_car_running && !prestart_is_running() && (MENU_IMAGE_DEBUG == current_menu))
    {
        // 预热/待确认阶段（车未动、速度环未开）：低频刷新发车状态行，WARMUP→READY 不按键也能看到；
        // 真正跑起来后不再刷屏省时间
        if ((now - last_dynamic_refresh_ms) >= 100u)
        {
            draw_image_debug_menu(0u);
            last_dynamic_refresh_ms = now;
        }
    }
    else if (!is_car_running && (MENU_IMAGE_DEBUG == current_menu) && g_menu_frame_ready)
    {
        // 停车看图：有新帧就局部刷新一次
        draw_image_debug_menu(0u);
        g_menu_frame_ready = 0u;
    }
    else if (!is_car_running && (MENU_LINE_SENSOR_INFO == current_menu) &&
              ((now - last_dynamic_refresh_ms) >= 100u))
    {
        // 传感器详情页：每100ms刷新一次16路模拟量。
        draw_line_sensor_info_menu();
        last_dynamic_refresh_ms = now;
    }
}
