/**
 * @file camera_pipeline.h
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

#ifndef CODE_CAMERA_PIPELINE_H_
#define CODE_CAMERA_PIPELINE_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    uint32 frame_seq;             /* 真正启动 DMA 的场序号。 */
    uint32 capture_ms;            /* VSYNC 到达时的共享毫秒时钟。 */
    float pure_gyro_yaw_deg;      /* 同一 VSYNC 时刻的纯陀螺连续航向。 */
} CameraCaptureMetadata;

/* 摄像头初始化失败时的最大重试次数。 */
#ifndef CAMERA_PIPELINE_INIT_RETRY_MAX
#define CAMERA_PIPELINE_INIT_RETRY_MAX (3u)
#endif

/* 每次重试之间的等待时间，避免连续初始化压死总线。 */
#ifndef CAMERA_PIPELINE_INIT_RETRY_DELAY_MS
#define CAMERA_PIPELINE_INIT_RETRY_DELAY_MS (200u)
#endif

/* 摄像头未就绪时，后台任务的空转延时。 */
#ifndef CAMERA_PIPELINE_IDLE_DELAY_MS
#define CAMERA_PIPELINE_IDLE_DELAY_MS (20u)
#endif

/* 初始化单摄采集链路，并记录最终是否成功。 */
void camera_pipeline_init(void);
/* 返回摄像头采集链路是否已经准备就绪。 */
uint8 camera_pipeline_is_ready(void);
/* 返回采集链路最近统计到的处理帧率。 */
uint32 camera_pipeline_get_fps(void);
/* VSYNC 中断确认 DMA 已启动后调用；载荷完成后才发布场序号。 */
void camera_pipeline_publish_capture_from_isr(uint32 frame_seq,
                                              uint32 capture_ms,
                                              float pure_gyro_yaw_deg);
/* 按场序号读取最近两场之一；槽正被改写或已过期时返回 0。 */
uint8 camera_pipeline_read_capture_metadata(uint32 frame_seq,
                                            CameraCaptureMetadata *metadata);
/* 单次执行采集、处理和菜单调试画面的更新。 */
void camera_pipeline_task(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_CAMERA_PIPELINE_H_ */
