/**
 * @file task.c
 * @brief CPU0 软件任务调度、诊断和运行保护。
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

#include "task.h"
#include "globals.h"
#include <string.h>
#include "control.h"
#include "key.h"
#include "adc_key.h"
#include "prestart.h"
#include "gyroscope.h"
#include "navigation.h"
#include "calibration.h"
#include "wifi_assistant.h"

#pragma section all "cpu0_dsram"

loop_t loop;

#define ASSISTANT_SCOPE_CHANNEL_MAX (2u)
#define ASSISTANT_PARAM_SPEED_KP_INDEX (1u)
#define ASSISTANT_PARAM_SPEED_KI_INDEX (2u)
#define ASSISTANT_PARAM_ESTOP_INDEX (3u)
#define ASSISTANT_SCOPE_SEND_ENABLE (0u)
#define ASSISTANT_ESTOP_TRIGGER_VALUE (0.5f)
#define ATTITUDE_STOP_LIMIT_DEG (50.0f)
/* 姿态(横滚/俯仰)超限停车，R=5/R=6。
 * 姿态源 = 加速度计重低通倾角（四元数已关闭），取相对发车零点的差值。 */
#define ATTITUDE_STOP_ENABLE (1u)
/* 图像帧看门狗超时：正常 150fps 左右一帧约 6~7ms，
 * 超过该时间没有新帧说明相机链路异常（排线松动/DMA 停止）。 */
#define IMAGE_FRAME_TIMEOUT_MS (100u)
#define IMU_DEBUG_PRINT_ENABLE (0u)

static seekfree_assistant_debug_param_struct assistant_params;
static seekfree_assistant_oscilloscope_struct assistant_scope;
static float assistant_scope_data[ASSISTANT_SCOPE_CHANNEL_MAX];

/* 判断横滚角是否触发姿态保护。
 * 只有陀螺仪零飘完成后才启用，避免校准前原始安装角度误停车。
 */
static uint8 attitude_roll_need_stop(void)
{
#if ATTITUDE_STOP_ENABLE
    float roll = gyro_get_roll_delta();

    if (!gyro_zero_calibration_done())
    {
        return 0u;
    }

    return (uint8)((roll > ATTITUDE_STOP_LIMIT_DEG) || (roll < -ATTITUDE_STOP_LIMIT_DEG));
#else
    return 0u; /* 姿态横滚保护已临时关闭（ATTITUDE_STOP_ENABLE=0）。 */
#endif
}

/* 判断俯仰角是否触发姿态保护。
 * 与横滚保护分开写，方便后续单独调试是哪一个姿态轴触发停车。
 */
static uint8 attitude_pitch_need_stop(void)
{
#if ATTITUDE_STOP_ENABLE
    float pitch = gyro_get_pitch_delta();

    if (!gyro_zero_calibration_done())
    {
        return 0u;
    }

    return (uint8)((pitch > ATTITUDE_STOP_LIMIT_DEG) || (pitch < -ATTITUDE_STOP_LIMIT_DEG));
#else
    return 0u; /* 姿态俯仰保护已临时关闭（ATTITUDE_STOP_ENABLE=0）。 */
#endif
}

#if IMU_DEBUG_PRINT_ENABLE
/* 临时 IMU 调试打印。
 * 默认关闭，避免运行时串口打印占用时间；需要看姿态数据时再打开宏。
 */
static void imu_debug_print(void)
{
    int roll_x10 = (int)(IMU_Data.roll * 10.0f);
    int pitch_x10 = (int)(IMU_Data.pitch * 10.0f);
    int yaw_x10 = (int)(IMU_Data.yaw * 10.0f);
    int droll_x10 = (int)(gyro_get_roll_delta() * 10.0f);
    int dpitch_x10 = (int)(gyro_get_pitch_delta() * 10.0f);

    printf("\r\nIMU run=%u cal=%u/%u roll10=%d pitch10=%d yaw10=%d droll10=%d dpitch10=%d\r\n",
           (unsigned int)is_car_running,
           (unsigned int)gyro_zero_calibration_done(),
           (unsigned int)gyro_zero_calibration_running(),
           roll_x10,
           pitch_x10,
           yaw_x10,
           droll_x10,
           dpitch_x10);
}
#endif

/* 初始化逐飞助手调参结构。
 * 当前示波器发送默认关闭，但参数解析仍保留用于上位机急停。
 */
static void assistant_task_init(void)
{
    memset(&assistant_params, 0, sizeof(assistant_params));
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    seekfree_assistant_oscilloscope_config(&assistant_scope, ASSISTANT_SCOPE_CHANNEL_MAX, assistant_scope_data);
}

/* 解析逐飞助手下发参数。
 * 运行时仍解析急停通道，保证上位机可以强制停车。
 */
static void assistant_apply_debug_params(void)
{
    seekfree_assistant_debug_param_analysis(&assistant_params);

    if (assistant_params.update_flag[ASSISTANT_PARAM_SPEED_KP_INDEX])
    {
        assistant_params.update_flag[ASSISTANT_PARAM_SPEED_KP_INDEX] = 0u;
        speed_PID1.KP = assistant_params.data[ASSISTANT_PARAM_SPEED_KP_INDEX] * 0.1f;
        speed_PID2.KP = speed_PID1.KP;
    }

    if (assistant_params.update_flag[ASSISTANT_PARAM_SPEED_KI_INDEX])
    {
        assistant_params.update_flag[ASSISTANT_PARAM_SPEED_KI_INDEX] = 0u;
        speed_PID1.KI = assistant_params.data[ASSISTANT_PARAM_SPEED_KI_INDEX] * 0.1f;
        speed_PID2.KI = speed_PID1.KI;
    }

    if (assistant_params.update_flag[ASSISTANT_PARAM_ESTOP_INDEX])
    {
        assistant_params.update_flag[ASSISTANT_PARAM_ESTOP_INDEX] = 0u;
        if (assistant_params.data[ASSISTANT_PARAM_ESTOP_INDEX] > ASSISTANT_ESTOP_TRIGGER_VALUE)
        {
            /* 上位机急停通道：保留运行中解析，收到触发值后立即关闭控制输出。 */
            g_stop_reason = STOP_REASON_ASSISTANT;
            prestart_stop();
        }
    }
}

#if ASSISTANT_SCOPE_SEND_ENABLE
/* 发送逐飞助手示波器数据。
 * 默认关闭，避免跑车时无线/串口发送拖慢控制和图像处理。
 */
static void assistant_send_scope(void)
{
    assistant_scope_data[0] = actual_l_debug;
    assistant_scope_data[1] = actual_r_debug;

    seekfree_assistant_oscilloscope_send(&assistant_scope);
}
#endif

/* 初始化任务调度器。
 * CCU60_CH0 每 1ms 进一次中断，只累加软件计数，具体任务在主循环执行。
 */
void task_init(void)
{
    pit_ms_init(CCU60_CH0, 1);
    assistant_task_init();
}

/* 1ms 系统节拍中断回调。
 * 只做计数累加和超时检测，不在中断里执行耗时任务。
 */
void task_loop_tick(void)
{
    vision_clock_ms++;
    loop.cnt_1ms++;
    loop.cnt_2ms++;
    loop.cnt_4ms++;
    loop.cnt_5ms++;
    loop.cnt_10ms++;
    loop.cnt_20ms++;
    loop.cnt_50ms++;
    loop.cnt_100ms++;
    loop.cnt_200ms++;
    loop.cnt_500ms++;
    loop.cnt_1000ms++;

    if (loop.check_flag >= 1)
    {
        loop.err_flag++;
    }
    else
    {
        loop.check_flag = 1;
    }
}

/* 主循环任务调度器。
 * 根据各周期计数触发 Duty_xms，执行完后清 check_flag 表示本轮调度完成。
 */
void task_main_loop(void)
{
    if (loop.cnt_1ms >= 1)
    {
        loop.cnt_1ms = 0;
        Duty_1ms();
    }
    if (loop.cnt_2ms >= 2)
    {
        loop.cnt_2ms = 0;
        Duty_2ms();
    }
    if (loop.cnt_4ms >= 4)
    {
        loop.cnt_4ms = 0;
        Duty_4ms();
    }
    if (loop.cnt_5ms >= 5)
    {
        loop.cnt_5ms = 0;
        Duty_5ms();
    }
    if (loop.cnt_10ms >= 10)
    {
        loop.cnt_10ms = 0;
        Duty_10ms();
    }
    if (loop.cnt_20ms >= 20)
    {
        loop.cnt_20ms = 0;
        Duty_20ms();
    }
    if (loop.cnt_50ms >= 50)
    {
        loop.cnt_50ms = 0;
        Duty_50ms();
    }
    if (loop.cnt_100ms >= 100)
    {
        loop.cnt_100ms = 0;
        Duty_100ms();
    }
    if (loop.cnt_200ms >= 200)
    {
        loop.cnt_200ms = 0;
        Duty_200ms();
    }
    if (loop.cnt_500ms >= 500)
    {
        loop.cnt_500ms = 0;
        Duty_500ms();
    }
    if (loop.cnt_1000ms >= 1000)
    {
        loop.cnt_1000ms = 0;
        Duty_1000ms();
    }

    loop.check_flag = 0;
}

/* 1ms 后台任务入口。
 * 当前空置，1ms 实时控制在 CCU61_CH1 的 pit_speed_pid() 中完成。
 */
void Duty_1ms(void) {}

/* 2ms 后台任务入口，当前预留。 */
void Duty_2ms(void) {}

/* 4ms 后台任务入口，当前预留。 */
void Duty_4ms(void) {}

/* 5ms 后台任务。
 * 菜单刷新、按键扫描和 ADC 按键消抖都放这里，避免放进硬实时速度环。
 * IMU 采样/校准/yaw 积分在数据就绪外部中断完成，不走软任务。
 */
void Duty_5ms(void)
{
    menu_task();
    key_scanner();
    adc_key_scan();
    getkey();
    calibration_task_5ms();
}

/* 10ms 后台任务。
 * 解析上位机参数、处理图像/姿态停车请求，并按需发送助手示波器。
 */
void Duty_10ms(void)
{
    static uint16 image_frame_age_ms = 0u;
    static uint32 image_processed_seq_last = 0u;
    uint32 image_processed_seq = vision_processed_seq;

    if (!wifi_assistant_is_active())
    {
        assistant_apply_debug_params();
    }

    /* 停车时代读编码器累计里程，速度环运行时该函数内部直接返回。 */
    control_odometer_idle_update();

    /* 图像帧看门狗：CPU1 每处理完一帧发布单调序号，这里比较并累计帧龄。
     * 跑车中帧龄超时说明相机不再出帧，必须立即停车。
     * 序号不会像单比特事件那样在跨核置位/清零竞争中丢帧。 */
    if (image_processed_seq != image_processed_seq_last)
    {
        image_processed_seq_last = image_processed_seq;
        image_frame_age_ms = 0u;
    }
    else if (image_frame_age_ms < IMAGE_FRAME_TIMEOUT_MS)
    {
        image_frame_age_ms += 10u;
    }

    if (is_car_running && prestart_is_running() && (image_frame_age_ms >= IMAGE_FRAME_TIMEOUT_MS))
    {
        g_stop_reason = STOP_REASON_FRAME_WATCHDOG;
        prestart_stop();
    }
    else if (is_car_running && prestart_is_running() && motor_fault_request)
    {
        /* 电机/编码器失控（编码器线松/堵转导致占空比满幅无反馈），立即停车。 */
        motor_fault_request = 0u;
        g_stop_reason = STOP_REASON_MOTOR_FAULT;
        prestart_stop();
    }
    else if (is_car_running && prestart_is_running() && nav_is_complete())
    {
        /* 惯导路径播完，正常终点停车。nav_stop() 在 prestart_stop() 内统一执行。 */
        g_stop_reason = STOP_REASON_PATH_DONE;
        prestart_stop();
    }
    else if (is_car_running && prestart_is_running() && image_stop_request)
    {
        /* 原因码已在 image.c 的极值帧保护处写好，此处不覆盖。 */
        image_stop_request = 0u;
        prestart_stop();
    }
    else if (is_car_running && prestart_is_running() && attitude_roll_need_stop())
    {
        g_stop_reason = STOP_REASON_ROLL;
        prestart_stop();
    }
    else if (is_car_running && prestart_is_running() && attitude_pitch_need_stop())
    {
        g_stop_reason = STOP_REASON_PITCH;
        prestart_stop();
    }
    else if (image_stop_request)
    {
        image_stop_request = 0u;
    }
    else if (motor_fault_request)
    {
        /* 非运行状态下清掉残留请求，避免下次发车瞬间被旧标志停车。 */
        motor_fault_request = 0u;
    }
#if ASSISTANT_SCOPE_SEND_ENABLE
    assistant_send_scope();
#endif
}

/* 20ms 后台任务：相机标定页可选地发送逐飞助手示波器数据。 */
void Duty_20ms(void)
{
    CameraCalibrationStatus camera_status;
    WifiAssistantTelemetry telemetry;
    float fit_mse_x100;

    if (!wifi_assistant_is_active())
    {
        return;
    }
    camera_calibration_get_status(&camera_status);
    telemetry.pure_gyro_deg = camera_status.live_gyro_deg;
    telemetry.line_position_px = camera_status.live_position_px;
    telemetry.line_slope_px_per_row = camera_status.live_slope_px_per_row;
    telemetry.camera_heading_deg = camera_status.predicted_heading_deg;
    telemetry.valid_rows = camera_status.live_valid_rows;
    fit_mse_x100 = camera_status.live_fit_mse_px2 * 100.0f;
    if ((fit_mse_x100 != fit_mse_x100) || (fit_mse_x100 <= 0.0f))
    {
        telemetry.fit_mse_x100 = 0u;
    }
    else if (fit_mse_x100 >= 65535.0f)
    {
        telemetry.fit_mse_x100 = 0xFFFFu;
    }
    else
    {
        telemetry.fit_mse_x100 = (uint16)fit_mse_x100;
    }
    telemetry.frame_valid = camera_status.live_frame_valid;
    telemetry.sample_count = camera_status.valid_samples;
    wifi_assistant_task_20ms(&telemetry);
}

/* 50ms 后台任务入口，当前预留。 */
void Duty_50ms(void) {}

/* 100ms 后台任务。
 * 仅在打开 IMU_DEBUG_PRINT_ENABLE 时输出姿态调试信息。
 */
void Duty_100ms(void)
{
#if IMU_DEBUG_PRINT_ENABLE
    imu_debug_print();
#endif
}

/* 200ms 后台任务入口，当前预留。 */
void Duty_200ms(void) {}

/* 500ms 后台任务入口，当前预留。 */
void Duty_500ms(void) {}

/* 1000ms 后台任务入口，当前预留。 */
void Duty_1000ms(void) {}

#pragma section all restore
