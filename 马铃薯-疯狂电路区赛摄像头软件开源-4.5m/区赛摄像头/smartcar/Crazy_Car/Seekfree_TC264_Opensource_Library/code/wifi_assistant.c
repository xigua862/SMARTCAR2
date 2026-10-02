/**
 * @file wifi_assistant.c
 * @brief 无线调参与运行数据发送接口。
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

#include "wifi_assistant.h"

#include <string.h>

#include "calibration.h"
#include "control.h"
#include "globals.h"
#include "image.h"
#include "prestart.h"
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "zf_device_wifi_spi.h"

/* 配置留在一个地方。空字符串表示尚未填写，不会初始化 WiFi 模块。 */
typedef struct
{
    char ssid[32];
    char password[64];
    char target_ip[16];
    char target_port[8];
    char local_port[8];
} WifiAssistantConfig;

static WifiAssistantConfig wifi_config = {
    "", "", "", "8080", "6666"};

enum
{
    WIFI_IMAGE_SLOT_COUNT = 2,
    WIFI_SCOPE_CHANNEL_COUNT = 8,
    WIFI_IMAGE_PERIOD_MS = 200,
    WIFI_SCOPE_PERIOD_MS = 50
};

#pragma section all "cpu0_dsram"

static IFX_ALIGN(4) uint8 wifi_image_slots[WIFI_IMAGE_SLOT_COUNT][ROW * COL];
static volatile uint32 wifi_image_slot_seq[WIFI_IMAGE_SLOT_COUNT];
static volatile uint32 wifi_image_slot_version[WIFI_IMAGE_SLOT_COUNT];
static volatile uint32 wifi_image_publish_seq = 0u;
static volatile uint8 wifi_image_slot_in_use = 0xFFu;
static volatile uint8 wifi_image_publish_enabled = 0u;

static volatile WifiAssistantState wifi_state = WIFI_ASSISTANT_OFF;
static uint8 wifi_camera_page_active = 0u;
static uint8 wifi_socket_connected = 0u;
static uint32 wifi_last_image_seq = 0u;
static volatile uint32 wifi_last_image_capture_ms = 0u;
static uint32 wifi_last_scope_ms = 0u;
static seekfree_assistant_camera_struct wifi_camera;
static seekfree_assistant_oscilloscope_struct wifi_scope;
static float wifi_scope_data[WIFI_SCOPE_CHANNEL_COUNT];

extern seekfree_assistant_transfer_callback_function
    seekfree_assistant_transfer_callback;

#pragma section all restore

static uint8 wifi_config_ready(void)
{
    return (uint8)((wifi_config.ssid[0] != '\0') &&
                   (wifi_config.target_ip[0] != '\0') &&
                   (wifi_config.target_port[0] != '\0') &&
                   (wifi_config.local_port[0] != '\0'));
}

static void wifi_assistant_restore_uart(void)
{
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
}

static uint8 wifi_connection_phase_allowed(void)
{
    CameraCalibrationStatus status;

    camera_calibration_get_status(&status);
    return (uint8)((CAMERA_CAL_WAIT_TRAIN_ZERO == status.phase) ||
                   (CAMERA_CAL_WAIT_VALIDATION_ZERO == status.phase));
}

static uint32 wifi_assistant_transfer(const uint8 *buffer, uint32 length)
{
    uint32 remaining;

    if (WIFI_ASSISTANT_CONNECTED != wifi_state)
    {
        return length;
    }
    remaining = wifi_spi_send_buffer(buffer, length);
    if (0u != remaining)
    {
        wifi_image_publish_enabled = 0u;
        wifi_state = WIFI_ASSISTANT_ERROR;
    }
    return remaining;
}

void wifi_assistant_camera_page_enter(void)
{
    wifi_camera_page_active = 1u;
    wifi_image_publish_enabled = 0u;
    wifi_last_image_seq = wifi_image_publish_seq;
    wifi_last_image_capture_ms = 0u;
    wifi_last_scope_ms = 0u;
    if (!wifi_config_ready())
    {
        wifi_state = WIFI_ASSISTANT_CONFIG_MISSING;
    }
    else
    {
        wifi_state = WIFI_ASSISTANT_DISCONNECTED;
    }
}

void wifi_assistant_camera_page_leave(void)
{
    wifi_image_publish_enabled = 0u;
    wifi_camera_page_active = 0u;
    if (wifi_socket_connected)
    {
        (void)wifi_spi_socket_disconnect();
        wifi_socket_connected = 0u;
    }
    wifi_assistant_restore_uart();
    wifi_state = WIFI_ASSISTANT_OFF;
    wifi_image_slot_in_use = 0xFFu;
}

uint8 wifi_assistant_connect(void)
{
    uint8 result;

    if (!wifi_camera_page_active || !prestart_is_idle() ||
        !wifi_connection_phase_allowed())
    {
        wifi_state = WIFI_ASSISTANT_ERROR;
        return 0u;
    }
    if (!wifi_config_ready())
    {
        wifi_state = WIFI_ASSISTANT_CONFIG_MISSING;
        return 0u;
    }
    if (WIFI_ASSISTANT_CONNECTED == wifi_state)
    {
        return 1u;
    }
    if (wifi_socket_connected)
    {
        (void)wifi_spi_socket_disconnect();
        wifi_socket_connected = 0u;
    }

    control_disable();
    wifi_state = WIFI_ASSISTANT_CONNECTING;
    result = wifi_spi_init(wifi_config.ssid,
                           wifi_config.password[0] ? wifi_config.password : NULL);
    if (0u == result)
    {
        result = wifi_spi_socket_connect("TCP",
                                         wifi_config.target_ip,
                                         wifi_config.target_port,
                                         wifi_config.local_port);
        if (0u == result)
        {
            wifi_socket_connected = 1u;
        }
    }
    if (0u != result)
    {
        wifi_state = WIFI_ASSISTANT_ERROR;
        wifi_assistant_restore_uart();
        return 0u;
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    seekfree_assistant_transfer_callback = wifi_assistant_transfer;
    seekfree_assistant_camera_config(&wifi_camera,
                                     SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
                                     COL,
                                     ROW,
                                     wifi_image_slots[0]);
    seekfree_assistant_oscilloscope_config(&wifi_scope,
                                           WIFI_SCOPE_CHANNEL_COUNT,
                                           wifi_scope_data);
    wifi_image_publish_enabled = 1u;
    wifi_state = WIFI_ASSISTANT_CONNECTED;
    return 1u;
}

WifiAssistantState wifi_assistant_get_state(void)
{
    return wifi_state;
}

uint8 wifi_assistant_is_active(void)
{
    return (uint8)(wifi_camera_page_active &&
                   (WIFI_ASSISTANT_CONNECTED == wifi_state));
}

const char *wifi_assistant_state_text(WifiAssistantState state)
{
    switch (state)
    {
    case WIFI_ASSISTANT_CONFIG_MISSING:
        return "WIFI CFG";
    case WIFI_ASSISTANT_DISCONNECTED:
        return "WIFI OFF";
    case WIFI_ASSISTANT_CONNECTING:
        return "WIFI BUSY";
    case WIFI_ASSISTANT_CONNECTED:
        return "WIFI ON";
    case WIFI_ASSISTANT_ERROR:
        return "WIFI ERR";
    default:
        return "WIFI --";
    }
}

void wifi_assistant_publish_image_cpu1(const uint8 *gray_image,
                                       uint32 frame_seq,
                                       uint32 capture_ms)
{
    uint32 next_sequence;
    uint32 write_version;
    uint8 slot;

    if (!wifi_image_publish_enabled || (NULL == gray_image))
    {
        return;
    }
    if ((0u != wifi_last_image_capture_ms) &&
        ((capture_ms - wifi_last_image_capture_ms) < WIFI_IMAGE_PERIOD_MS))
    {
        return;
    }

    next_sequence = wifi_image_publish_seq + 1u;
    if (0u == next_sequence)
    {
        next_sequence = 1u;
    }
    slot = (uint8)(next_sequence & 1u);
    if (slot == wifi_image_slot_in_use)
    {
        next_sequence++;
        if (0u == next_sequence)
        {
            next_sequence = 1u;
        }
        slot ^= 1u;
    }

    write_version = wifi_image_slot_version[slot] + 1u;
    if (0u == (write_version & 1u))
    {
        write_version++;
    }
    wifi_image_slot_version[slot] = write_version;
    __dsync();
    if (!wifi_image_publish_enabled || (slot == wifi_image_slot_in_use))
    {
        wifi_image_slot_version[slot] = write_version + 1u;
        __dsync();
        return;
    }
    memcpy(wifi_image_slots[slot], gray_image, ROW * COL);
    __dsync();
    wifi_image_slot_seq[slot] = next_sequence;
    __dsync();
    wifi_image_slot_version[slot] = write_version + 1u;
    __dsync();
    wifi_image_publish_seq = next_sequence;
    wifi_last_image_capture_ms = capture_ms;
    (void)frame_seq;
}

static uint8 wifi_acquire_latest_image(uint8 *slot_out, uint32 *sequence_out)
{
    uint32 sequence;
    uint8 slot;
    uint32 sequence_after;
    uint32 version_before;
    uint32 version_after;

    if ((NULL == slot_out) || (NULL == sequence_out))
    {
        return 0u;
    }
    sequence = wifi_image_publish_seq;
    if ((0u == sequence) || (sequence == wifi_last_image_seq))
    {
        return 0u;
    }
    slot = (uint8)(sequence & 1u);
    version_before = wifi_image_slot_version[slot];
    if (0u != (version_before & 1u))
    {
        return 0u;
    }
    wifi_image_slot_in_use = slot;
    __dsync();
    sequence_after = wifi_image_publish_seq;
    version_after = wifi_image_slot_version[slot];
    if ((sequence != sequence_after) ||
        (wifi_image_slot_seq[slot] != sequence) ||
        (version_before != version_after) ||
        (0u != (version_after & 1u)))
    {
        wifi_image_slot_in_use = 0xFFu;
        __dsync();
        return 0u;
    }
    *slot_out = slot;
    *sequence_out = sequence;
    return 1u;
}

void wifi_assistant_task_20ms(const WifiAssistantTelemetry *telemetry)
{
    uint8 image_slot;
    uint32 image_sequence;
    uint32 now_ms;

    if (!wifi_assistant_is_active() || !prestart_is_idle())
    {
        wifi_image_publish_enabled = 0u;
        return;
    }

    now_ms = vision_clock_ms;
    if ((NULL != telemetry) &&
        ((0u == wifi_last_scope_ms) ||
         ((now_ms - wifi_last_scope_ms) >= WIFI_SCOPE_PERIOD_MS)))
    {
        wifi_scope_data[0] = telemetry->pure_gyro_deg;
        wifi_scope_data[1] = telemetry->line_position_px;
        wifi_scope_data[2] = telemetry->line_slope_px_per_row;
        wifi_scope_data[3] = telemetry->camera_heading_deg;
        wifi_scope_data[4] = (float)telemetry->valid_rows;
        wifi_scope_data[5] = (float)telemetry->fit_mse_x100 * 0.01f;
        wifi_scope_data[6] = (float)telemetry->frame_valid;
        wifi_scope_data[7] = (float)telemetry->sample_count;
        seekfree_assistant_oscilloscope_send(&wifi_scope);
        if (WIFI_ASSISTANT_CONNECTED != wifi_state)
        {
            wifi_assistant_restore_uart();
            return;
        }
        wifi_last_scope_ms = now_ms;
    }

    if (wifi_acquire_latest_image(&image_slot, &image_sequence))
    {
        wifi_camera.data_buffer = wifi_image_slots[image_slot];
        wifi_camera.buffer_byte_size = ROW * COL;
        seekfree_assistant_camera_send(&wifi_camera);
        if (WIFI_ASSISTANT_CONNECTED == wifi_state)
        {
            wifi_last_image_seq = image_sequence;
        }
        wifi_image_slot_in_use = 0xFFu;
        __dsync();
        if (WIFI_ASSISTANT_CONNECTED != wifi_state)
        {
            wifi_assistant_restore_uart();
        }
    }
}
