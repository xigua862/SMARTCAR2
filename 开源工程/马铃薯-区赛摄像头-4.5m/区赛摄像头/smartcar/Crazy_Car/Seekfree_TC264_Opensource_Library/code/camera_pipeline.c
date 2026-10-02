/**
 * @file camera_pipeline.c
 * @brief 摄像头采集元数据、双核帧交接和图像处理调度。
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

#include "camera_pipeline.h"

#include "globals.h"
#include "image.h"
#include "key.h"
#include "wifi_assistant.h"

/* 图像耗时串口打印，默认关闭：printf 走阻塞串口，每秒一次约 1.5~5ms，
 * 足够卡掉一帧图像处理，只在台架分析耗时时临时打开。 */
#define CAMERA_PIPELINE_PROFILE_UART_ENABLE (0u)

/* 共享存储定义在 globals.c 的 CPU0 DSPR，两个核心只通过下方 API 访问。 */
extern volatile CameraCaptureMetadata vision_capture_metadata_slots[2];
extern volatile uint32 vision_capture_metadata_versions[2];

static uint8 camera_pipeline_ready = 0u;
static uint32 camera_pipeline_frame_count = 0u;
static volatile uint32 camera_pipeline_fps = 0u;
static uint32 camera_pipeline_last_fps_ms = 0u;
#if CAMERA_PIPELINE_PROFILE_UART_ENABLE
static uint32 camera_pipeline_debug_last_ms = 0u;
static uint32 camera_pipeline_debug_frame_count = 0u;
static uint32 camera_pipeline_debug_total_ms = 0u;
#endif

/* 清空相机帧率统计。
 * 相机初始化前调用一次，重新开始按 1 秒窗口统计实际处理帧率。
 */
static void camera_pipeline_reset_fps(void)
{
    camera_pipeline_frame_count = 0u;
    camera_pipeline_fps = 0u;
    camera_pipeline_last_fps_ms = system_getval_ms();
#if CAMERA_PIPELINE_PROFILE_UART_ENABLE
    camera_pipeline_debug_last_ms = camera_pipeline_last_fps_ms;
    camera_pipeline_debug_frame_count = 0u;
    camera_pipeline_debug_total_ms = 0u;
#endif
}

/* 更新相机处理帧率。
 * 每处理完一帧调用一次，elapsed_ms 满 1000ms 后刷新 camera_pipeline_fps。
 */
static void camera_pipeline_update_fps(uint32 now_ms)
{
    uint32 elapsed_ms;

    camera_pipeline_frame_count++;
    elapsed_ms = now_ms - camera_pipeline_last_fps_ms;
    if (1000u <= elapsed_ms)
    {
        camera_pipeline_fps = (camera_pipeline_frame_count * 1000u) / elapsed_ms;
        camera_pipeline_frame_count = 0u;
        camera_pipeline_last_fps_ms = now_ms;
    }
}

/* 初始化 MT9V03X 相机。
 * 按配置重试多次，成功后 camera_pipeline_task() 才会处理图像帧。
 */
void camera_pipeline_init(void)
{
    uint8 camera_init_retry = 0u;

    camera_pipeline_ready = 0u;
    camera_pipeline_reset_fps();
    for (camera_init_retry = 0u; camera_init_retry < CAMERA_PIPELINE_INIT_RETRY_MAX; camera_init_retry++)
    {
        if (0 == mt9v03x_init())
        {
            camera_pipeline_ready = 1u;
            break;
        }
        system_delay_ms(CAMERA_PIPELINE_INIT_RETRY_DELAY_MS);
    }
}

/* 查询相机是否初始化成功。 */
uint8 camera_pipeline_is_ready(void) { return camera_pipeline_ready; }

/* 获取最近 1 秒窗口统计出的实际处理帧率。 */
uint32 camera_pipeline_get_fps(void) { return camera_pipeline_fps; }

/* 发布一次真实 DMA 启动对应的采集快照。
 * 槽版本号先置奇数，载荷写完并执行屏障后置偶数，最后才更新全局场序号。
 * 即使读端恰好跨过一次写入，也只会重试，不会接受撕裂的 float/时间戳。 */
void camera_pipeline_publish_capture_from_isr(uint32 frame_seq,
                                              uint32 capture_ms,
                                              float pure_gyro_yaw_deg)
{
    uint8 slot = (uint8)(frame_seq & 1u);
    uint32 write_version = vision_capture_metadata_versions[slot] + 1u;

    if (0u == (write_version & 1u))
    {
        write_version++;
    }
    vision_capture_metadata_versions[slot] = write_version;
    __dsync();

    vision_capture_metadata_slots[slot].frame_seq = frame_seq;
    vision_capture_metadata_slots[slot].capture_ms = capture_ms;
    vision_capture_metadata_slots[slot].pure_gyro_yaw_deg = pure_gyro_yaw_deg;
    __dsync();

    vision_capture_metadata_versions[slot] = write_version + 1u;
    __dsync();
    vision_capture_seq = frame_seq;
}

/* 读取指定场的完整采集快照。双槽只保证最近两场，因此更早场直接失效。 */
uint8 camera_pipeline_read_capture_metadata(uint32 frame_seq,
                                            CameraCaptureMetadata *metadata)
{
    static const uint8 read_retry_count = 3u;
    uint8 retry;

    if ((0u == frame_seq) || (0 == metadata))
    {
        return 0u;
    }

    for (retry = 0u; retry < read_retry_count; retry++)
    {
        uint32 latest_seq = vision_capture_seq;
        uint32 frame_age = latest_seq - frame_seq;
        uint8 slot = (uint8)(frame_seq & 1u);
        uint32 version_before;
        uint32 version_after;

        if (frame_age > 1u)
        {
            return 0u;
        }

        /* 发布帧号是生产者的最后一次写；先完成该读取，再碰槽内数据。 */
        __dsync();
        version_before = vision_capture_metadata_versions[slot];
        if (0u != (version_before & 1u))
        {
            continue;
        }
        __dsync();
        metadata->frame_seq = vision_capture_metadata_slots[slot].frame_seq;
        metadata->capture_ms = vision_capture_metadata_slots[slot].capture_ms;
        metadata->pure_gyro_yaw_deg = vision_capture_metadata_slots[slot].pure_gyro_yaw_deg;
        __dsync();
        version_after = vision_capture_metadata_versions[slot];

        if ((version_before == version_after) &&
            (0u == (version_after & 1u)) &&
            (metadata->frame_seq == frame_seq))
        {
            return 1u;
        }
    }
    return 0u;
}

#if CAMERA_PIPELINE_PROFILE_UART_ENABLE
/* 打印图像处理耗时统计。
 * 默认关闭，避免跑车时串口输出拖慢图像处理。
 */
static void camera_pipeline_debug_print(uint32 process_ms, uint32 now_ms)
{
    uint32 elapsed_ms;

    camera_pipeline_debug_frame_count++;
    camera_pipeline_debug_total_ms += process_ms;
    elapsed_ms = now_ms - camera_pipeline_debug_last_ms;
    if (1000u <= elapsed_ms)
    {
        uint32 avg_x10 = 0u;
        uint32 max_fps = 9999u;

        if (0u < camera_pipeline_debug_total_ms)
        {
            avg_x10 = (camera_pipeline_debug_total_ms * 10u) / camera_pipeline_debug_frame_count;
            max_fps = (camera_pipeline_debug_frame_count * 1000u) / camera_pipeline_debug_total_ms;
        }

        printf("\r\nIMG frame=%lu avg=%lu.%lu ms max=%lu fps actual=%lu\r\n",
               (unsigned long)camera_pipeline_debug_frame_count,
               (unsigned long)(avg_x10 / 10u),
               (unsigned long)(avg_x10 % 10u),
               (unsigned long)max_fps,
               (unsigned long)camera_pipeline_fps);

        camera_pipeline_debug_last_ms = now_ms;
        camera_pipeline_debug_frame_count = 0u;
        camera_pipeline_debug_total_ms = 0u;
    }
}
#endif

/* 从底层相机就绪缓冲复制一帧到用户图像缓冲。
 * 驱动是乒乓双缓冲：finish_flag=1 期间就绪缓冲不会被 DMA 触碰（DMA 只写
 * 另一块），复制完成并屏障后清标志，驱动才能发布下一帧到本缓冲。 */
static void camera_pipeline_copy_frame(void)
{
    memcpy(orig_image[0], mt9v03x_ready_image(), MT9V03X_IMAGE_SIZE);
    __dsync();
    mt9v03x_finish_flag = 0u;
}

/* 相机处理任务。
 * 主循环调用；检测到新帧后复制图像、执行视觉处理、刷新菜单调试帧。
 */
void camera_pipeline_task(void)
{
    if (!camera_pipeline_ready)
    {
        system_delay_ms(CAMERA_PIPELINE_IDLE_DELAY_MS);
        return;
    }

    if (mt9v03x_finish_flag)
    {
        uint32 time_start = system_getval_ms();
        uint32 frame_seq = mt9v03x_ready_capture_count;
        CameraCaptureMetadata capture;

        /* 乒乓采集不等消费，最新采集场号可能已领先就绪帧一场，双槽元数据
         * 仍能读到本帧快照。读取失败说明处理落后两场以上、槽已被覆盖：
         * 放弃本帧并释放就绪缓冲去追新帧，否则该帧永远读不出会卡死流水线。 */
        if (!camera_pipeline_read_capture_metadata(frame_seq, &capture))
        {
            mt9v03x_finish_flag = 0u;
            return;
        }

        camera_pipeline_copy_frame();
        /* WiFi 页开启时才复制一份 150x80 原图到 CPU0 双缓冲；运行期不产生额外开销。 */
        wifi_assistant_publish_image_cpu1(orig_image[0],
                                          capture.frame_seq,
                                          capture.capture_ms);
        image_proc_copied(capture.frame_seq, capture.capture_ms);

        time_cost = system_getval_ms() - time_start;
        camera_pipeline_update_fps(system_getval_ms());
#if CAMERA_PIPELINE_PROFILE_UART_ENABLE
        camera_pipeline_debug_print(time_cost, system_getval_ms());
#endif
        vision_processed_seq = frame_seq;
        __dsync();
        menu_update_image_debug_frame(orig_image[0], bin_image[0]);
        menu_notify_frame_ready();
    }
}
