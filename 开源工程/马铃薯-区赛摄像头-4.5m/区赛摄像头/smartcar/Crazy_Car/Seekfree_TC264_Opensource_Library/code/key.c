/**
 * @file key.c
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

#include "key.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "adc_key.h"
#include "camera_pipeline.h"
#include "control.h"
#include "motor.h"
#include "globals.h"
#include "image.h"
#include "navigation.h"
#include "prestart.h"
#include "calibration.h"
#include "wifi_assistant.h"

typedef enum
{
    MENU_MAIN = 0,
    MENU_PARAM_ADJUST,
    MENU_RUNTIME_INFO,
    MENU_IMAGE_TRANSFER,
    MENU_IMAGE_DEBUG, /* 图像调试页，发车功能已并入本页 */
    MENU_CAMERA_CALIBRATION
} menu_state_enum;

typedef struct
{
    char name[20];
    float *value_ptr;
    float min_value;
    float max_value;
    uint8 int_digits;
    uint8 decimal_places;
} parameter_struct;

#define SCREEN_WIDTH (240u)
#define SCREEN_HEIGHT (135u)
#define CHAR_HEIGHT (16u)
#define STATUS_Y (SCREEN_HEIGHT - CHAR_HEIGHT)
#define MAIN_MENU_VISIBLE_LINES (7u)
#define PARAMS_PER_PAGE (6u)
#define MAX_PARAMETERS (32u)
#define MENU_VALUE_X (152u)
#define MAIN_MENU_START_Y (12u)
#define MAIN_MENU_LINE_HEIGHT (14u)
/* 图像调试页单独使用竖屏画布 135x240（其余菜单仍是横屏 240x135）。
 * 逐飞驱动方向枚举命名与实际相反：IPS114_PORTAIT=240x135(横)，CROSSWISE/CROSSWISE_180=135x240(竖)。
 * 本车排线方向下，竖屏正立需用 IPS114_CROSSWISE_180（CROSSWISE 会上下颠倒）。 */
#define DBG_PORTRAIT_W (135u)
#define DBG_PORTRAIT_H (240u)

#define DEBUG_IMG_W (112u)
#define DEBUG_IMG_H (72u)
#define DEBUG_BIN_X ((DBG_PORTRAIT_W - DEBUG_IMG_W) / 2u) /* 叠加图水平居中 */
#define DEBUG_INFO_Y (DEBUG_IMG_H + 8u)                   /* 信息区紧贴图像下方，留 8px 间隔 */
#define DEBUG_INFO_STEP (14u)                             /* 6x8 字体单行行距，竖屏拉开更易读 */
#define DEBUG_INFO_LINES (10u)                            /* 信息区行数，用于越界检查 */
#define FULL_WIDTH_SPACES "                              "

#if ((DEBUG_BIN_X + DEBUG_IMG_W) > DBG_PORTRAIT_W)
#error "debug overlay image exceeds portrait width"
#endif

#if ((DEBUG_INFO_Y + DEBUG_INFO_LINES * DEBUG_INFO_STEP) > DBG_PORTRAIT_H)
#error "debug info text exceeds portrait height"
#endif

static volatile uint8 g_menu_key_result = KEY_NONE;
static volatile uint8 g_menu_frame_ready = 0u;
static volatile uint8 g_camera_ready = 0u;
static volatile uint8 g_image_debug_has_frame = 0u;
static volatile uint8 g_image_debug_display_index = 0u;

static uint8 g_image_debug_binary_buffers[2][DEBUG_IMG_H][DEBUG_IMG_W];
static uint16 g_image_debug_overlay_buffer[DEBUG_IMG_H][DEBUG_IMG_W];
static volatile uint8 g_image_debug_fit_valid[2];
static volatile float g_image_debug_fit_position_px[2];
static volatile float g_image_debug_fit_slope_px_per_row[2];

static volatile menu_state_enum current_menu = MENU_MAIN;
static uint8 menu_initialized = 0u;
static uint8 menu_needs_refresh = 1u;
static uint8 main_menu_selection = 0u;
static uint8 param_menu_selection = 0u;
static uint8 param_menu_edit_mode = 0u;
static uint8 camera_calibration_show_model_details = 0u;
static uint8 key3_long_press_count = 0u;
volatile uint8 is_car_running = 0u;
static uint8 parameter_count = 0u;
static uint32 last_dynamic_refresh_ms = 0u;
static uint32 status_expire_ms = 0u;
static float param_menu_edit_original_value = 0.0f;
static char status_message[31] = {0};
static parameter_struct parameters[MAX_PARAMETERS];

static const char *main_menu_items[] = {
    "Image Debug / Start",
    "Camera Heading Cal",
    "Parameter Adjust",
    "Runtime Info",
    "Image Transfer",
    "Save Parameters",
    "Reload Parameters",
    "Reset Defaults"};

static const uint8 main_menu_item_count =
    (uint8)(sizeof(main_menu_items) / sizeof(main_menu_items[0]));

static ips114_font_size_enum menu_display_font = IPS114_8X16_FONT;

/* 设置菜单当前字体，并同步到底层 IPS114 驱动。 */
static void menu_set_font(ips114_font_size_enum font)
{
    menu_display_font = font;
    ips114_set_font(font);
}

/* 获取当前字体宽度，用于字符串裁剪和坐标边界判断。 */
static uint8 menu_font_width(void)
{
    return (IPS114_6X8_FONT == menu_display_font) ? 6u : 8u;
}

/* 获取当前字体高度，用于防止文字写出屏幕底部。 */
static uint8 menu_font_height(void)
{
    return (IPS114_6X8_FONT == menu_display_font) ? 8u : 16u;
}

/* 安全显示字符串。
 * 对 x/y 和字符串长度做裁剪，避免 IPS114 显示函数写出有效区域。
 */
static void menu_show_string(uint16 x, uint16 y, const char *text)
{
    char clipped[41];
    uint8 max_chars;
    uint8 i = 0u;

    /* 用驱动随方向更新的实际宽高裁剪，而不是写死的横屏常量：
     * 旋转后长宽会互换（横屏 240x135 <-> 竖屏 135x240），
     * 写死会导致竖屏下 y>127 的行被误判越界而整行不画。 */
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

#define ips114_set_font(font) ips114_set_font_should_not_be_used_directly(font)
#define ips114_show_string(x, y, text) menu_show_string((x), (y), (text))

/* 设置底部临时状态消息。
 * 消息默认显示 1.5 秒，超时后由 menu_task() 清除。
 */
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
    status_expire_ms = system_getval_ms() + 1500u;
}

/* 清除底部临时状态消息。 */
static void clear_status_message(void)
{
    status_message[0] = '\0';
    status_expire_ms = 0u;
}

/* 取出并清空一次菜单按键事件。
 * 菜单事件是单槽缓存，消费后才允许下一次事件进入。
 */
static uint8 consume_menu_key(void)
{
    uint8 key = g_menu_key_result;
    g_menu_key_result = KEY_NONE;
    return key;
}

/* 计算参数菜单总页数。 */
static uint8 get_parameter_page_count(void)
{
    if (0u == parameter_count)
    {
        return 1u;
    }
    return (uint8)(((uint16)parameter_count + PARAMS_PER_PAGE - 1u) / PARAMS_PER_PAGE);
}

/* 根据小数位数决定单步调节量。 */
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

/* 读取当前参数值。 */
static float get_parameter_value(const parameter_struct *param)
{
    return *param->value_ptr;
}

/* 将 Flash 参数同步到运行时 PID/控制变量。
 * 参数菜单修改后会立即调用，让调参不必重启。
 */
static void apply_runtime_parameters(void)
{
    turn_PID.KP = g_flash_parameters.turn_KP;
    turn_PID.KD = g_flash_parameters.turn_KD;
    turn_PID.KP2 = g_flash_parameters.turn_KP2;
    turn_PID.GKD = g_flash_parameters.turn_GKD;

    speed_PID1.KP = g_flash_parameters.speed_KP;
    speed_PID1.KI = g_flash_parameters.speed_KI;
    speed_PID1.IncrementLimit = g_flash_parameters.speed_IncrementLimit;

    speed_PID2.KP = g_flash_parameters.speed_KP;
    speed_PID2.KI = g_flash_parameters.speed_KI;
    speed_PID2.IncrementLimit = g_flash_parameters.speed_IncrementLimit;
}

/* 写入参数值并做范围限制。
 * 写入后立即同步到运行时控制结构。
 */
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

    *param->value_ptr = new_value;

    apply_runtime_parameters();
}

/* 注册参数菜单中的所有可调项。
 * 这里决定菜单顺序、显示名称、范围和小数位数。
 */
static void register_default_parameters(void)
{
    parameter_count = 0u;

    add_parameter("T_KP", &g_flash_parameters.turn_KP, 0.0f, 999.0f, 3u, 2u);
    add_parameter("T_KD", &g_flash_parameters.turn_KD, 0.0f, 999.0f, 3u, 1u);
    add_parameter("T_KP2", &g_flash_parameters.turn_KP2, 0.0f, 999.0f, 3u, 3u);
    add_parameter("T_GKD", &g_flash_parameters.turn_GKD, 0.0f, 999.0f, 3u, 1u);
    add_parameter("S_KP", &g_flash_parameters.speed_KP, 0.0f, 999.0f, 3u, 1u);
    add_parameter("S_KI", &g_flash_parameters.speed_KI, 0.0f, 999.0f, 3u, 1u);
    add_parameter("S_IncLim", &g_flash_parameters.speed_IncrementLimit, 0.0f, 9999.0f, 4u, 1u);
    add_parameter("MaxDuty", &max_duty_limit, 0.0f, 10000.0f, 5u, 1u);

    if (param_menu_selection >= parameter_count)
    {
        param_menu_selection = 0u;
    }
}

/* 绘制底部状态栏。
 * 优先显示临时状态消息，没有消息时显示传入的页脚提示。
 */
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

/* 绘制主菜单。 */
static void draw_main_menu(void)
{
    uint8 i;
    uint8 first_item = 0u;

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
            ips114_set_color(RGB565_BLACK, RGB565_YELLOW);
        }
        else
        {
            ips114_set_color(RGB565_WHITE, RGB565_BLACK);
        }
        ips114_show_string(4u,
                          (uint16)(MAIN_MENU_START_Y + i * MAIN_MENU_LINE_HEIGHT),
                          main_menu_items[item_index]);
    }

    draw_status_or_footer("U/D Move C Enter");
}

/* 绘制参数调节菜单。
 * 选中项用黄色高亮，编辑模式用绿色高亮，数值显示按参数类型选择格式。
 */
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
    page = (uint8)(param_menu_selection / PARAMS_PER_PAGE);
    page_count = get_parameter_page_count();
    start = (uint8)(page * PARAMS_PER_PAGE);
    end = (uint8)(start + PARAMS_PER_PAGE);
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
        y = (uint16)(16u + (i - start) * CHAR_HEIGHT);

        if (i == param_menu_selection)
        {
            ips114_set_color(RGB565_BLACK, param_menu_edit_mode ? RGB565_GREEN : RGB565_YELLOW);
        }
        else
        {
            ips114_set_color(RGB565_WHITE, RGB565_BLACK);
        }

        ips114_show_string(4u, y, param->name);

        ips114_show_float(MENU_VALUE_X, y, *param->value_ptr, param->int_digits, param->decimal_places);
    }

    draw_status_or_footer(param_menu_edit_mode ? "U/D +/- L/R fast C OK R Undo" : "U/D Sel L/R Pg C Edit");
}

/* 绘制运行信息页。
 * 用于快速查看视觉误差、路口状态、yaw、距离、图像耗时和相机帧率。
 */
static void draw_runtime_info_menu(void)
{
    char line[40];

    menu_set_font(IPS114_8X16_FONT);
    ips114_clear();
    ips114_set_color(RGB565_CYAN, RGB565_BLACK);
    ips114_show_string(0u, 0u, "Runtime Info");
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);

    sprintf(line, "Line:%6.1f", image_lat_offset_px);
    ips114_show_string(0u, 16u, line);

    sprintf(line, "Nav:%u P:%u/%u", (unsigned int)nav_get_state(),
            (unsigned int)nav_get_progress(), (unsigned int)nav_get_total_points());
    ips114_show_string(0u, 32u, line);

    sprintf(line, "Corr:%6.1f Lat:%+d", nav_get_correction(),
            (int)nav_get_lat_corr_mm());
    ips114_show_string(0u, 48u, line);

    sprintf(line, "Yaw:%7.1f", yaw_total);
    ips114_show_string(0u, 64u, line);

    sprintf(line, "Dist:%1.3f", total_dist);
    ips114_show_string(0u, 80u, line);

    sprintf(line, "Proc:%3lu", (unsigned long)time_cost);
    ips114_show_string(0u, 96u, line);

    sprintf(line, "Cam:%u FPS:%3lu", (unsigned int)g_camera_ready, (unsigned long)camera_pipeline_get_fps());
    ips114_show_string(0u, 112u, line);

    draw_status_or_footer("R Back");
}

/* 绘制图传信息页。
 * 当前只显示工程配置和相机状态，不在这里启动或停止图传。
 */
static void draw_image_transfer_menu(void)
{
    char line[40];

    menu_set_font(IPS114_8X16_FONT);
    ips114_clear();
    ips114_set_color(RGB565_CYAN, RGB565_BLACK);
    ips114_show_string(0u, 0u, "Image Transfer");
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);

    ips114_show_string(0u, 16u, "Stream: CPU0 managed");
    ips114_show_string(0u, 32u, "Type: Seekfree Assistant");
    sprintf(line, "Cam: %ux%u", (unsigned int)COL, (unsigned int)ROW);
    ips114_show_string(0u, 48u, line);
    sprintf(line, "FPS: %3lu", (unsigned long)camera_pipeline_get_fps());
    ips114_show_string(0u, 64u, line);
    ips114_show_string(0u, 80u, "Wifi SPI by current proj");
    ips114_show_string(0u, 96u, wifi_spi_ip_addr_port[0] ? wifi_spi_ip_addr_port : "--");

    draw_status_or_footer("Read Only R Back");
}

static void draw_camera_calibration_menu(void)
{
    CameraCalibrationStatus status;
    CameraCalibrationModelDetails details;
    char line[41];

    camera_calibration_get_status(&status);
    camera_calibration_get_model_details(&details);
    menu_set_font(IPS114_6X8_FONT);
    ips114_clear();
    ips114_set_color(RGB565_CYAN, RGB565_BLACK);
    ips114_show_string(0u, 0u,
                       camera_calibration_show_model_details
                           ? "Camera Heading Model Details"
                           : "Camera Heading Calibration");
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);

    if (camera_calibration_show_model_details)
    {
        sprintf(line, "Source: %s",
                details.saved ? "SAVED FLASH" :
                details.available ? "PENDING / NOT SAVED" : "NO MODEL");
        ips114_show_string(0u, 8u, line);
        if (details.available)
        {
            sprintf(line, "P0 position: %+.4f px",
                    details.reference_position_px);
            ips114_show_string(0u, 16u, line);
            sprintf(line, "S0 slope: %+.6f px/row",
                    details.reference_slope_px_per_row);
            ips114_show_string(0u, 24u, line);
            sprintf(line, "B0 intercept: %+.6e", details.intercept);
            ips114_show_string(0u, 32u, line);
            sprintf(line, "BP position gain: %+.6e", details.position_gain);
            ips114_show_string(0u, 40u, line);
            sprintf(line, "BS slope gain: %+.6e", details.slope_gain);
            ips114_show_string(0u, 48u, line);
            sprintf(line, "Bias:%+.2f  RMSE:%.2f deg",
                    details.bias_deg, details.rmse_deg);
            ips114_show_string(0u, 56u, line);
            sprintf(line, "P95:%.2f deg  Cond:%.3f",
                    details.p95_deg, details.feature_independence);
            ips114_show_string(0u, 64u, line);
            sprintf(line, "Valid frame rate: %.1f%%",
                    details.valid_rate_percent);
            ips114_show_string(0u, 72u, line);
            sprintf(line, "Samples train/valid: %lu / %lu",
                    (unsigned long)details.training_samples,
                    (unsigned long)details.validation_samples);
            ips114_show_string(0u, 80u, line);
        }
        else
        {
            ips114_show_string(0u, 24u, "No fitted or saved camera model");
        }
        sprintf(line, "Model:%s  Fusion:%s",
                status.model_valid ? "VALID" : "NONE",
                status.fusion_enabled ? "ON" : "SHADOW");
        ips114_show_string(0u, 88u, line);
        sprintf(line, "Step: %s", camera_calibration_phase_text(status.phase));
        ips114_show_string(0u, 96u, line);
        sprintf(line, "WiFi: %s",
                wifi_assistant_state_text(wifi_assistant_get_state()));
        ips114_show_string(0u, 104u, line);
        sprintf(line, "Result: %s",
                camera_calibration_error_text(status.error, status.fit_status));
        ips114_show_string(0u, 112u, line);
        draw_status_or_footer("Down Live  Right Fusion  ADC Back");
        return;
    }

    sprintf(line, "Step: %s", camera_calibration_phase_text(status.phase));
    ips114_show_string(0u, 8u, line);
    sprintf(line, "Valid samples: %u / %u",
            (unsigned int)status.valid_samples,
            (unsigned int)status.minimum_samples);
    ips114_show_string(0u, 16u, line);
    sprintf(line, "Observed frames: %u",
            (unsigned int)status.observed_frames);
    ips114_show_string(0u, 24u, line);
    sprintf(line, "Valid frame rate: %5.1f%%", status.valid_rate_percent);
    ips114_show_string(0u, 32u, line);
    sprintf(line, "Gyro range: %+.1f to %+.1f deg",
            status.minimum_gyro_deg, status.maximum_gyro_deg);
    ips114_show_string(0u, 40u, line);
    sprintf(line, "Line pos:%+.2f Fit:%u/%u", status.live_position_px,
            (unsigned int)status.live_valid_rows,
            (unsigned int)status.live_candidate_rows);
    ips114_show_string(0u, 48u, line);
    sprintf(line, "Line slope:%+.4f MSE:%.1f",
            status.live_slope_px_per_row, status.live_fit_mse_px2);
    ips114_show_string(0u, 56u, line);
    sprintf(line, "Gyro / camera: %+.1f / %+.1f",
            status.live_gyro_deg, status.predicted_heading_deg);
    ips114_show_string(0u, 64u, line);
    sprintf(line, "Validation bias: %+.2f deg", status.bias_deg);
    ips114_show_string(0u, 72u, line);
    sprintf(line, "Validation RMSE: %.2f deg", status.rmse_deg);
    ips114_show_string(0u, 80u, line);
    sprintf(line, "Validation P95: %.2f deg", status.p95_deg);
    ips114_show_string(0u, 88u, line);
    sprintf(line, "Model:%s Cond:%.3f Fusion:%s",
            status.model_valid ? "VALID" : "NONE",
            status.feature_independence,
            status.fusion_enabled ? "ON" : "SHADOW");
    ips114_show_string(0u, 96u, line);

    sprintf(line, "WiFi: %s  Up only in WAIT",
            wifi_assistant_state_text(wifi_assistant_get_state()));
    ips114_show_string(0u, 104u, line);
    sprintf(line, "Result: %s",
            camera_calibration_error_text(status.error, status.fit_status));
    ips114_show_string(0u, 112u, line);

    draw_status_or_footer("Down Model  K3 Next  Right Fusion");
}

static uint16 debug_image_local_x(uint8 col)
{
    uint16 x = (uint16)((uint32)col * DEBUG_IMG_W / COL);
    if (x >= DEBUG_IMG_W)
    {
        x = DEBUG_IMG_W - 1u;
    }
    return x;
}

static uint16 debug_image_local_y(uint8 row)
{
    uint16 y = (uint16)((uint32)row * DEBUG_IMG_H / ROW);
    if (y >= DEBUG_IMG_H)
    {
        y = DEBUG_IMG_H - 1u;
    }
    return y;
}

static void debug_overlay_set_point(uint16 x, uint16 y, uint16 color)
{
    if ((x < DEBUG_IMG_W) && (y < DEBUG_IMG_H))
    {
        g_image_debug_overlay_buffer[y][x] = color;
    }
}

static void debug_overlay_set_point_i(int x, int y, uint16 color)
{
    if ((x >= 0) && (x < (int)DEBUG_IMG_W) && (y >= 0) && (y < (int)DEBUG_IMG_H))
    {
        g_image_debug_overlay_buffer[y][x] = color;
    }
}

static void debug_overlay_draw_cross(uint8 col, uint8 row, uint16 color)
{
    int x;
    int y;
    int i;

    if ((col >= COL) || (row >= ROW))
    {
        return;
    }

    x = (int)debug_image_local_x(col);
    y = (int)debug_image_local_y(row);
    /* 臂长 ±3 像素：缩放图上原来的 ±1 十字几乎不可见。 */
    for (i = -3; i <= 3; i++)
    {
        debug_overlay_set_point_i(x + i, y, color);
        debug_overlay_set_point_i(x, y + i, color);
    }
}

static void debug_overlay_draw_hline(uint16 x1, uint16 x2, uint16 y, uint16 color)
{
    uint16 x;
    uint16 left = x1;
    uint16 right = x2;

    if (left > right)
    {
        left = x2;
        right = x1;
    }
    if (y >= DEBUG_IMG_H)
    {
        return;
    }
    if (right >= DEBUG_IMG_W)
    {
        right = DEBUG_IMG_W - 1u;
    }

    for (x = left; x <= right; x++)
    {
        debug_overlay_set_point(x, y, color);
        if (x == 65535u)
        {
            break;
        }
    }
}

static void debug_overlay_draw_vline(uint16 x, uint16 y1, uint16 y2, uint16 color)
{
    uint16 y;
    uint16 top = y1;
    uint16 bottom = y2;

    if (top > bottom)
    {
        top = y2;
        bottom = y1;
    }
    if (x >= DEBUG_IMG_W)
    {
        return;
    }
    if (bottom >= DEBUG_IMG_H)
    {
        bottom = DEBUG_IMG_H - 1u;
    }

    for (y = top; y <= bottom; y++)
    {
        debug_overlay_set_point(x, y, color);
        if (y == 65535u)
        {
            break;
        }
    }
}

static void debug_overlay_draw_planning_rect(void)
{
    uint16 left = debug_image_local_x(RECT_LEFT);
    uint16 right = debug_image_local_x(RECT_RIGHT);
    uint16 top = debug_image_local_y(RECT_TOP);
    uint16 bottom = debug_image_local_y(RECT_BOTTOM);
    uint16 top2 = (top < (DEBUG_IMG_H - 1u)) ? (uint16)(top + 1u) : top;
    uint16 bottom2 = (bottom > 0u) ? (uint16)(bottom - 1u) : bottom;
    uint16 left2 = (left < (DEBUG_IMG_W - 1u)) ? (uint16)(left + 1u) : left;
    uint16 right2 = (right > 0u) ? (uint16)(right - 1u) : right;
    uint16 top_color = path_info.top_exist ? RGB565_GREEN : RGB565_RED;
    uint16 bottom_color = path_info.bottom_exist ? RGB565_GREEN : RGB565_RED;
    uint16 left_color = path_info.left_exist ? RGB565_GREEN : RGB565_RED;
    uint16 right_color = path_info.right_exist ? RGB565_GREEN : RGB565_RED;

    debug_overlay_draw_hline(left, right, top, top_color);
    debug_overlay_draw_hline(left, right, top2, top_color);
    debug_overlay_draw_hline(left, right, bottom, bottom_color);
    debug_overlay_draw_hline(left, right, bottom2, bottom_color);
    debug_overlay_draw_vline(left, top, bottom, left_color);
    debug_overlay_draw_vline(left2, top, bottom, left_color);
    debug_overlay_draw_vline(right, top, bottom, right_color);
    debug_overlay_draw_vline(right2, top, bottom, right_color);
}
static void debug_overlay_draw_fitted_center_line(uint8 valid,
                                                  float position_px,
                                                  float slope_px_per_row)
{
    uint16 y;

    if (!valid)
    {
        return;
    }
    for (y = 0u; y < DEBUG_IMG_H; y++)
    {
        uint16 source_row = (uint16)((uint32)y * ROW / DEBUG_IMG_H);
        float source_col = (float)(COL / 2u) + position_px +
                           slope_px_per_row *
                               ((float)source_row - (float)(ROW / 3u));

        if ((source_col >= 0.0f) && (source_col <= (float)(COL - 1u)))
        {
            int x = (int)debug_image_local_x((uint8)(source_col + 0.5f));

            debug_overlay_set_point_i(x, (int)y, RGB565_RED);
            debug_overlay_set_point_i(x + 1, (int)y, RGB565_RED);
        }
    }
}

static void build_debug_overlay_image(const uint8 *binary_image,
                                      uint8 fit_valid,
                                      float fit_position_px,
                                      float fit_slope_px_per_row)
{
    uint16 row;
    uint16 col;

    if (NULL == binary_image)
    {
        return;
    }

    for (row = 0u; row < DEBUG_IMG_H; row++)
    {
        for (col = 0u; col < DEBUG_IMG_W; col++)
        {
            g_image_debug_overlay_buffer[row][col] =
                binary_image[row * DEBUG_IMG_W + col] >= 127u ? RGB565_WHITE : RGB565_BLACK;
        }
    }

    debug_overlay_draw_planning_rect();
    /* 拐点搜索窗上下界（黄色横线）：只有夹在两线之间的边界突变才会成为拐点，
     * 弯太近掉到下线以下时拐点消失属于正常现象。 */
    debug_overlay_draw_hline(0u, DEBUG_IMG_W - 1u, debug_image_local_y(GUAIDIAN_SEARCH_START), RGB565_YELLOW);
    debug_overlay_draw_hline(0u, DEBUG_IMG_W - 1u, debug_image_local_y(GUAIDIAN_SEARCH_END), RGB565_YELLOW);

    if (l_guaidian)
    {
        debug_overlay_draw_cross(l_border[l_guaidian], l_guaidian, RGB565_PURPLE);
    }
    if (r_guaidian)
    {
        debug_overlay_draw_cross(r_border[r_guaidian], r_guaidian, RGB565_CYAN);
    }

    /* 最后画红线，确保本帧有效拟合不会被其他调试标记盖住。 */
    debug_overlay_draw_fitted_center_line(fit_valid,
                                          fit_position_px,
                                          fit_slope_px_per_row);
}

/* 绘制图像调试页。停车时显示居中的二值图；有效拟合中线用红色叠加。 */
static void draw_image_debug_menu(uint8 full_refresh)
{
    char line[40];
    uint8 display_index;
    uint8 otsu_far;
    uint8 otsu_mid;
    uint8 otsu_near;
    uint8 fit_valid;
    float fit_position_px;
    float fit_slope_px_per_row;

    if (full_refresh)
    {
        ips114_clear();
    }

    if (!g_camera_ready)
    {
        menu_set_font(IPS114_8X16_FONT);
        ips114_set_color(RGB565_RED, RGB565_BLACK);
        ips114_show_string(0u, 0u, "Camera Off");
        return;
    }

    if (!g_image_debug_has_frame)
    {
        menu_set_font(IPS114_8X16_FONT);
        ips114_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips114_show_string(0u, 0u, "Waiting Frame");
        return;
    }

    /* 双缓冲索引只在 menu_update_image_debug_frame() 写完后切换。 */
    display_index = g_image_debug_display_index;
    __dsync();
    fit_valid = g_image_debug_fit_valid[display_index];
    fit_position_px = g_image_debug_fit_position_px[display_index];
    fit_slope_px_per_row =
        g_image_debug_fit_slope_px_per_row[display_index];
    image_get_otsu_thresholds(&otsu_far, &otsu_mid, &otsu_near);

    build_debug_overlay_image(g_image_debug_binary_buffers[display_index][0],
                              fit_valid,
                              fit_position_px,
                              fit_slope_px_per_row);
    ips114_show_rgb565_image(DEBUG_BIN_X, 0u, g_image_debug_overlay_buffer[0], DEBUG_IMG_W, DEBUG_IMG_H, DEBUG_IMG_W,
                             DEBUG_IMG_H, 0u);

    /* 竖屏下信息改为图像下方单列排布；字段全部定宽，配合分帧局部刷新原位覆盖旧值。 */
    menu_set_font(IPS114_6X8_FONT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);

    sprintf(line, "Line:%6.1f", image_lat_offset_px);
    ips114_show_string(2u, DEBUG_INFO_Y + 0u * DEBUG_INFO_STEP, line);

    sprintf(line, "Otsu F%3u M%3u N%3u", (unsigned int)otsu_far, (unsigned int)otsu_mid, (unsigned int)otsu_near);
    ips114_show_string(2u, DEBUG_INFO_Y + 1u * DEBUG_INFO_STEP, line);

    sprintf(line, "FPS:%3lu Proc:%2lu White:%2u",
            (unsigned long)camera_pipeline_get_fps(),
            (unsigned long)time_cost,
            (unsigned int)image_get_frame_white_percent());
    ips114_show_string(2u, DEBUG_INFO_Y + 2u * DEBUG_INFO_STEP, line);

    sprintf(line, "Fit rows:%02u mse:%3.1f",
            (unsigned int)nav_get_vision_valid_rows(),
            nav_get_vision_fit_mse_px2());
    ips114_show_string(2u, DEBUG_INFO_Y + 3u * DEBUG_INFO_STEP, line);

    {
        sprintf(line, "Accepted:%03lu Drop:%03lu",
                (unsigned long)(nav_get_vision_accept_count() % 1000u),
                (unsigned long)(nav_get_vision_drop_count() % 1000u));
    }
    ips114_show_string(2u, DEBUG_INFO_Y + 4u * DEBUG_INFO_STEP, line);

    /* Stop 为停车原因码，Reject 为视觉帧拒绝原因，Age 为帧龄。 */
    {
        uint32 age_ms = nav_get_vision_age_ms();
        if (age_ms > 99u)
        {
            age_ms = 99u;
        }
        sprintf(line, "Stop:%u Reject:%u Age:%02lu",
                (unsigned int)g_stop_reason,
                (unsigned int)nav_get_vision_last_drop_reason(),
                (unsigned long)age_ms);
    }
    ips114_show_string(2u, DEBUG_INFO_Y + 5u * DEBUG_INFO_STEP, line);

    sprintf(line, "Yaw:%+5.0f HeadBias:%+3.1f", yaw_total,
            nav_get_heading_bias_deg());
    ips114_show_string(2u, DEBUG_INFO_Y + 6u * DEBUG_INFO_STEP, line);

    /* P=回放进度 X=当前帧是否看到直行路口(1=看到) N=本圈路口检测累计次数。
     * 推车跑一圈:N 应约等于路线实际经过的直行路口数(远多=误检,远少=漏检);
     * X 慢推时应只在真路口处闪 1、直道/弯道保持 0,据此判有无误检位置。 */
    sprintf(line, "St%+4.1f P%04u X%u N%lu", nav_get_correction(),
            (unsigned int)nav_get_progress(),
            (unsigned int)((nav_get_vision_flags() & VISION_OBS_FLAG_CROSSING) ? 1u : 0u),
            (unsigned long)nav_get_crossing_count());
    ips114_show_string(2u, DEBUG_INFO_Y + 7u * DEBUG_INFO_STEP, line);

    {
        sprintf(line, "Line:%+3dpx LatFix:%+3dmm", (int)image_lat_offset_px,
                (int)nav_get_lat_corr_mm());
        ips114_show_string(2u, DEBUG_INFO_Y + 8u * DEBUG_INFO_STEP, line);
    }

    /* 第 9 行：发车状态 + 按键提示（发车页已并入本页）。 */
    if (0u == is_car_running)
    {
        sprintf(line, "%-15s", "STOP  K3:Start");
    }
    else if (prestart_is_warming_up())
    {
        sprintf(line, "%-15s", "WARMUP wait..");
    }
    else if (prestart_is_waiting_confirm())
    {
        sprintf(line, "%-15s", "READY K3:Go!");
    }
    else
    {
        sprintf(line, "%-15s", "RUN   K3:Stop");
    }
    ips114_show_string(2u, DEBUG_INFO_Y + 9u * DEBUG_INFO_STEP, line);
}

/* 处理主菜单按键。
 * 确认键进入页面或执行保存/加载/恢复默认参数等一次性动作。
 */
static void handle_main_menu_input(uint8 key)
{
    if (KEY_UP == key)
    {
        main_menu_selection = (0u == main_menu_selection)
                                  ? (uint8)(main_menu_item_count - 1u)
                                  : (uint8)(main_menu_selection - 1u);
        menu_needs_refresh = 1u;
    }
    else if (KEY_DOWN == key)
    {
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
            /* 图像调试页 = 发车页（唯一入口）：进入前先停车，并重置
             * 视觉测量、路口几何诊断和导航会话。想重零时退出再进。 */
            if (!prestart_is_idle())
            {
                prestart_stop();
            }
            control_disable();
            current_menu = MENU_IMAGE_DEBUG;
            image_request_vision_state_reset();
            /* 开机已在 cpu0_main.c 里开过一次推车会话(本页是开机默认页,
             * 从不经过这条按键分支);这里覆盖"退出到主菜单再按确认重进"的
             * 场景——同样以当前摆位/朝向为原点重开,老规矩:想重零就退出再进。 */
            nav_start();
            break;
        case 1u:
            camera_calibration_show_model_details = 0u;
            camera_calibration_enter();
            wifi_assistant_camera_page_enter();
            current_menu = MENU_CAMERA_CALIBRATION;
            break;
        case 2u:
            current_menu = MENU_PARAM_ADJUST;
            break;
        case 3u:
            current_menu = MENU_RUNTIME_INFO;
            break;
        case 4u:
            current_menu = MENU_IMAGE_TRANSFER;
            break;
        case 5u:
            if (!is_car_running)
            {
                SaveParameters();
                set_status_message("Parameters saved");
            }
            else
            {
                set_status_message("Stop car before save");
            }
            break;
        case 6u:
            if (is_car_running)
            {
                set_status_message("Stop car before reload");
            }
            else if (LoadParameters())
            {
                apply_runtime_parameters();
                set_status_message("Parameters reloaded");
            }
            else
            {
                set_status_message("Reload failed");
            }
            break;
        case 7u:
            if (!is_car_running)
            {
                InitDefaultParameters();
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

/* 处理参数调节页按键。
 * 非编辑模式负责选项和翻页；编辑模式负责修改数值、保存或撤销。
 */
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
        if (KEY_UP == key)
        {
            param_menu_selection = (0u == param_menu_selection) ? (parameter_count - 1u) : (param_menu_selection - 1u);
            menu_needs_refresh = 1u;
        }
        else if (KEY_DOWN == key)
        {
            param_menu_selection++;
            if (param_menu_selection >= parameter_count)
            {
                param_menu_selection = 0u;
            }
            menu_needs_refresh = 1u;
        }
        else if (KEY_LEFT == key)
        {
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

    step = get_parameter_step(param);
    value = get_parameter_value(param);

    if (KEY_UP == key)
    {
        set_parameter_value(param, value + step);
        menu_needs_refresh = 1u;
    }
    else if (KEY_DOWN == key)
    {
        set_parameter_value(param, value - step);
        menu_needs_refresh = 1u;
    }
    else if (KEY_RIGHT == key)
    {
        set_parameter_value(param, value + step * 10.0f);
        menu_needs_refresh = 1u;
    }
    else if (KEY_LEFT == key)
    {
        set_parameter_value(param, value - step * 10.0f);
        menu_needs_refresh = 1u;
    }
    else if (KEY_CONFIRM == key)
    {
        param_menu_edit_mode = 0u;
        if (!is_car_running)
        {
            SaveParameters();
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
        set_parameter_value(param, param_menu_edit_original_value);
        param_menu_edit_mode = 0u;
        set_status_message("Edit cancelled");
        menu_needs_refresh = 1u;
    }
}

/* 运行信息页只响应返回键。 */
static void handle_runtime_info_input(uint8 key)
{
    if (KEY_RETURN == key)
    {
        current_menu = MENU_MAIN;
        menu_needs_refresh = 1u;
    }
}

/* 图传信息页只响应返回键。 */
static void handle_image_transfer_input(uint8 key)
{
    if (KEY_RETURN == key)
    {
        current_menu = MENU_MAIN;
        menu_needs_refresh = 1u;
    }
}

/* 图像调试页按键（发车页已并入本页）。
 * 短按确认走 预热→二次确认→停车 流程，长按确认直接急停，返回键回主菜单。
 * 图像帧刷新由 camera_pipeline_task() 通过 menu_update_image_debug_frame() 驱动。
 */
static void handle_image_debug_input(uint8 key)
{
    if (KEY_RETURN == key)
    {
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
        if (0u == is_car_running)
        {
            prestart_begin();
            is_car_running = 1u;
            set_status_message("Warmup");
        }
        else if (prestart_is_warming_up())
        {
            set_status_message("Wait warmup");
        }
        else if (prestart_is_waiting_confirm())
        {
            if (prestart_confirm_launch())
            {
                set_status_message("Suction settle");
            }
            else
            {
                set_status_message("Gyro fail");
            }
        }
        else
        {
            g_stop_reason = STOP_REASON_MANUAL;
            prestart_stop();
            set_status_message("Car stopped");
        }
        menu_needs_refresh = 1u;
    }
    else if (KEY_CONFIRM_LONG == key)
    {
        g_stop_reason = STOP_REASON_MANUAL;
        prestart_stop();
        set_status_message("E-STOP");
        menu_needs_refresh = 1u;
    }
}

static void handle_camera_calibration_input(uint8 key)
{
    if (KEY_RETURN == key)
    {
        camera_calibration_abort();
        camera_calibration_leave();
        wifi_assistant_camera_page_leave();
        current_menu = MENU_MAIN;
        menu_needs_refresh = 1u;
    }
    else if (KEY_CONFIRM == key)
    {
        CameraCalibrationStatus status;

        camera_calibration_confirm();
        camera_calibration_get_status(&status);
        camera_calibration_show_model_details =
            (uint8)((CAMERA_CAL_PASS_NOT_SAVED == status.phase) ||
                    (CAMERA_CAL_SAVED == status.phase));
        menu_needs_refresh = 1u;
    }
    else if (KEY_DOWN == key)
    {
        camera_calibration_show_model_details =
            (uint8)!camera_calibration_show_model_details;
        menu_needs_refresh = 1u;
    }
    else if (KEY_RIGHT == key)
    {
        if (camera_heading_set_fusion_enabled(
                (uint8)!camera_heading_fusion_enabled()))
        {
            set_status_message(camera_heading_fusion_enabled()
                                   ? "Camera fusion ON"
                                   : "Camera fusion SHADOW");
        }
        else
        {
            set_status_message("Need saved camera model");
        }
        menu_needs_refresh = 1u;
    }
    else if (KEY_UP == key)
    {
        CameraCalibrationStatus status;

        camera_calibration_get_status(&status);
        if ((CAMERA_CAL_WAIT_TRAIN_ZERO != status.phase) &&
            (CAMERA_CAL_WAIT_VALIDATION_ZERO != status.phase))
        {
            set_status_message("WiFi only in WAIT step");
        }
        else if (wifi_assistant_connect())
        {
            set_status_message("WiFi connected");
        }
        else
        {
            set_status_message("WiFi not connected");
        }
        menu_needs_refresh = 1u;
    }
    else if (KEY_CONFIRM_LONG == key)
    {
        camera_calibration_abort();
        set_status_message("Camera cal aborted");
        menu_needs_refresh = 1u;
    }
}

/* 注册 float 类型参数到参数菜单。 */
uint8 add_parameter(const char *name, float *value_ptr, float min_value, float max_value, uint8 int_digits,
                    uint8 decimal_places)
{
    if ((NULL == name) || (NULL == value_ptr) || (parameter_count >= MAX_PARAMETERS))
    {
        return 0u;
    }

    strncpy(parameters[parameter_count].name, name, sizeof(parameters[parameter_count].name) - 1u);
    parameters[parameter_count].name[sizeof(parameters[parameter_count].name) - 1u] = '\0';
    parameters[parameter_count].value_ptr = value_ptr;
    parameters[parameter_count].min_value = min_value;
    parameters[parameter_count].max_value = max_value;
    parameters[parameter_count].int_digits = int_digits;
    parameters[parameter_count].decimal_places = decimal_places;
    parameter_count++;
    return 1u;
}

/* 汇总实体按键和 ADC 按键事件。
 * KEY1 短按映射确认，长按映射急停；ADC 五向键映射菜单方向/返回。
 */
void getkey(void)
{
    adc_key_enum adc_key;
    key_state_enum key3_state = key_get_state(KEY_1);

    if (KEY_SHORT_PRESS == key3_state)
    {
        if (KEY_NONE == g_menu_key_result)
        {
            g_menu_key_result = KEY_CONFIRM;
        }
        key_clear_state(KEY_1);
        key3_long_press_count = 0u;
    }
    else if (KEY_LONG_PRESS == key3_state)
    {
        if (0u == key3_long_press_count)
        {
            if (KEY_NONE == g_menu_key_result)
            {
                g_menu_key_result = KEY_CONFIRM_LONG;
            }
            key3_long_press_count = 1u;
        }
    }
    else if ((KEY_LONG_PRESS != key3_state) && (0u != key3_long_press_count))
    {
        key3_long_press_count = 0u;
        key_clear_state(KEY_1);
    }

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

/* 初始化菜单平台资源。
 * 包含 ADC 按键、KEY1、发车状态机 5ms PIT、屏幕方向和菜单状态。
 */
void menu_platform_init(void)
{
    adc_key_init();
    key_init(5);
    pit_ms_init(CCU60_CH1, 5);
    pit_disable(CCU60_CH1);
    /* ips114_init() 会把 P15_4 配成背光输出，而该脚同时是 IMU660RC INT2 中断输入。
     * 依赖 cpu0_main 的顺序：本函数必须先于 gyro_init() 执行，让 IMU 的 exti_init
     * 最后接管 P15_4。上电后任何地方都不得再调 ips114_init()，
     * 运行期切方向一律用 ips114_change_dir()。 */
    ips114_set_dir(IPS114_PORTAIT);
    ips114_init();
    menu_init();
}

/* 初始化菜单内部状态和参数列表。 */
void menu_init(void)
{
    apply_runtime_parameters();
    register_default_parameters();
    g_menu_key_result = KEY_NONE;
    g_menu_frame_ready = 0u;
    g_image_debug_has_frame = 0u;
    g_image_debug_display_index = 0u;
    g_image_debug_fit_valid[0] = 0u;
    g_image_debug_fit_valid[1] = 0u;
    g_image_debug_fit_position_px[0] = 0.0f;
    g_image_debug_fit_position_px[1] = 0.0f;
    g_image_debug_fit_slope_px_per_row[0] = 0.0f;
    g_image_debug_fit_slope_px_per_row[1] = 0.0f;
    current_menu = MENU_IMAGE_DEBUG;
    main_menu_selection = 0u; /* Image Debug（发车页）现在是主菜单第一项。 */
    param_menu_selection = 0u;
    param_menu_edit_mode = 0u;
    param_menu_edit_original_value = 0.0f;
    menu_needs_refresh = 1u;
    last_dynamic_refresh_ms = 0u;
    clear_status_message();
    menu_set_font(IPS114_8X16_FONT);
    menu_initialized = 1u;
}

/* 通知菜单有新图像帧可刷新。
 * 相机任务处理完一帧后调用。
 */
void menu_notify_frame_ready(void) { g_menu_frame_ready = 1u; }

/* 更新相机就绪状态。
 * 状态变化时强制刷新菜单，避免页面显示旧状态。
 */
void menu_set_camera_ready(uint8 camera_ready)
{
    if (g_camera_ready != camera_ready)
    {
        g_camera_ready = camera_ready;
        menu_needs_refresh = 1u;
    }
}

/* 更新图像调试页的双缓冲图像。
 * 只有停车且停留在图像调试页时才复制，避免跑车时额外耗时。
 */
void menu_update_image_debug_frame(const uint8 *raw_image, const uint8 *binary_image)
{
    uint8 write_index;
    uint8 fit_valid;
    uint16 y;
    uint16 x;
    float fit_position_px = 0.0f;
    float fit_slope_px_per_row = 0.0f;

    (void)raw_image;

    if (NULL == binary_image)
    {
        return;
    }
    if (is_car_running || (MENU_IMAGE_DEBUG != current_menu))
    {
        return;
    }

    write_index = (uint8)(g_image_debug_display_index ^ 1u);
    fit_valid = image_get_fitted_center_line(&fit_position_px,
                                             &fit_slope_px_per_row);

    for (y = 0u; y < DEBUG_IMG_H; y++)
    {
        uint16 src_y = (uint16)(y * ROW / DEBUG_IMG_H);
        for (x = 0u; x < DEBUG_IMG_W; x++)
        {
            /* 将摄像头原尺寸缩放到调试窗口尺寸。 */
            uint16 src_x = (uint16)(x * COL / DEBUG_IMG_W);
            g_image_debug_binary_buffers[write_index][y][x] = binary_image[src_y * COL + src_x];
        }
    }
    g_image_debug_fit_position_px[write_index] = fit_position_px;
    g_image_debug_fit_slope_px_per_row[write_index] =
        fit_slope_px_per_row;
    g_image_debug_fit_valid[write_index] = fit_valid;
    __dsync();
    g_image_debug_display_index = write_index;
    __dsync();
    g_image_debug_has_frame = 1u;
    g_menu_frame_ready = 1u;
}

/* 当前物理显示方向，初值与 menu_platform_init() 里首次 ips114_init() 保持一致（横屏）。 */
static ips114_dir_enum g_menu_display_dir = IPS114_PORTAIT;

/* 按当前页面把屏幕方向调到目标状态：图像调试页竖屏，其余页面横屏。
 * 集中在这里做，进入/退出该页的所有跳转点都无需各自处理方向。
 * 必须用 ips114_change_dir()（只发 MADCTL 指令+清屏），严禁用 ips114_init()：
 * init 会重配 P15_4（屏幕 BLK/伪 MISO），而该脚同时是 IMU660RC INT2 的
 * ERU 中断输入，被抢走后四元数读取失去数据就绪同步，撕裂出 NaN。
 * 清屏走 SPI 仍有几十 ms，保持只在停车时切换。
 */
static void menu_reconcile_orientation(void)
{
    ips114_dir_enum want = (MENU_IMAGE_DEBUG == current_menu) ? IPS114_CROSSWISE_180 : IPS114_PORTAIT;

    if (want == g_menu_display_dir)
    {
        return;
    }
    if (is_car_running)
    {
        return;
    }

    ips114_change_dir(want);
    g_menu_display_dir = want;
    menu_needs_refresh = 1u;
}

/* 菜单主任务。
 * 由 5ms 后台任务调用，负责状态消息过期、按键分发和页面动态刷新。
 */
void menu_task(void)
{
    uint8 key;
    uint32 now;

    if (!menu_initialized)
    {
        return;
    }

    apply_runtime_parameters();
    now = system_getval_ms();
    if ((status_expire_ms > 0u) && (now >= status_expire_ms))
    {
        clear_status_message();
        if (!is_car_running)
        {
            menu_needs_refresh = 1u;
        }
    }

    key = consume_menu_key();

    /* 长按语义：发车页急停、标定页立即中止，其余普通页面保存参数。 */
    if ((KEY_CONFIRM_LONG == key) &&
        (MENU_IMAGE_DEBUG != current_menu) &&
        (MENU_CAMERA_CALIBRATION != current_menu) &&
        !is_car_running)
    {
        SaveParameters();
        set_status_message("Parameters saved");
        menu_needs_refresh = 1u;
        key = KEY_NONE;
    }

    if (KEY_NONE != key)
    {
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
        case MENU_IMAGE_TRANSFER:
            handle_image_transfer_input(key);
            break;
        case MENU_IMAGE_DEBUG:
            handle_image_debug_input(key);
            break;
        case MENU_CAMERA_CALIBRATION:
            handle_camera_calibration_input(key);
            break;
        default:
            break;
        }
    }

    /* 按键可能已切换页面，这里据新页面同步屏幕方向（并在需要时置刷新标志）。 */
    menu_reconcile_orientation();

    if (menu_needs_refresh)
    {
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
        case MENU_IMAGE_TRANSFER:
            draw_image_transfer_menu();
            break;
        case MENU_IMAGE_DEBUG:
            draw_image_debug_menu(1u);
            break;
        case MENU_CAMERA_CALIBRATION:
            draw_camera_calibration_menu();
            break;
        default:
            break;
        }
        menu_needs_refresh = 0u;
        g_menu_frame_ready = 0u;
        last_dynamic_refresh_ms = now;
        return;
    }

    if (!is_car_running && (MENU_RUNTIME_INFO == current_menu))
    {
        if ((now - last_dynamic_refresh_ms) >= 100u)
        {
            draw_runtime_info_menu();
            last_dynamic_refresh_ms = now;
        }
    }
    else if (is_car_running && !prestart_is_running() && (MENU_IMAGE_DEBUG == current_menu))
    {
        /* 预热/待确认阶段（车未动、速度环未开）：低频刷新发车状态行，
         * WARMUP→READY 的切换不按键也能看到；真正跑起来后不再刷屏省时间。 */
        if ((now - last_dynamic_refresh_ms) >= 100u)
        {
            draw_image_debug_menu(0u);
            last_dynamic_refresh_ms = now;
        }
    }
    else if (!is_car_running && (MENU_IMAGE_DEBUG == current_menu) && g_menu_frame_ready)
    {
        draw_image_debug_menu(0u);
        g_menu_frame_ready = 0u;
    }
    else if (!is_car_running && (MENU_CAMERA_CALIBRATION == current_menu) &&
             ((now - last_dynamic_refresh_ms) >= 100u))
    {
        draw_camera_calibration_menu();
        last_dynamic_refresh_ms = now;
    }
}
