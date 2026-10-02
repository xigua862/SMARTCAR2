/**
 * @file wifi_assistant.h
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

#ifndef CODE_WIFI_ASSISTANT_H_
#define CODE_WIFI_ASSISTANT_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    WIFI_ASSISTANT_OFF = 0,
    WIFI_ASSISTANT_CONFIG_MISSING,
    WIFI_ASSISTANT_DISCONNECTED,
    WIFI_ASSISTANT_CONNECTING,
    WIFI_ASSISTANT_CONNECTED,
    WIFI_ASSISTANT_ERROR
} WifiAssistantState;

typedef struct
{
    float pure_gyro_deg;
    float line_position_px;
    float line_slope_px_per_row;
    float camera_heading_deg;
    uint16 valid_rows;
    uint16 fit_mse_x100;
    uint8 frame_valid;
    uint16 sample_count;
} WifiAssistantTelemetry;

/* 只有相机标定页调用这两个页面生命周期接口。 */
void wifi_assistant_camera_page_enter(void);
void wifi_assistant_camera_page_leave(void);
uint8 wifi_assistant_connect(void);
WifiAssistantState wifi_assistant_get_state(void);
const char *wifi_assistant_state_text(WifiAssistantState state);
uint8 wifi_assistant_is_active(void);

/* CPU0 每 20ms 调用；这里才允许访问 WiFi-SPI/逐飞助手。 */
void wifi_assistant_task_20ms(const WifiAssistantTelemetry *telemetry);

/* CPU1 每处理完一帧调用；只复制到共享双缓冲，不调用任何通信驱动。 */
void wifi_assistant_publish_image_cpu1(const uint8 *gray_image,
                                       uint32 frame_seq,
                                       uint32 capture_ms);

#ifdef __cplusplus
}
#endif

#endif /* CODE_WIFI_ASSISTANT_H_ */
