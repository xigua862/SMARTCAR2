/**
 * @file phototube_array.h
 * @brief CPU1 端 16 路模拟光电扫描、候选提取和发布。
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
 *
 * @note CPU1 以 1 kHz 扫描，发布时必须保持跨核提交顺序。
 */

// phototube_array.h - 16-channel analog phototube array and MUX wiring.

#ifndef CODE_PHOTOTUBE_ARRAY_H_
#define CODE_PHOTOTUBE_ARRAY_H_

#include "zf_common_headfile.h"
#include "competition_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define PHOTOTUBE_ARRAY_COUNT (16u)
#define PHOTOTUBE_ARRAY_ALGORITHM_VERSION (13u)
#define PHOTOTUBE_ARRAY_SCAN_HZ (1000u)
#define PHOTOTUBE_ARRAY_SCAN_PERIOD_MS (1u)

#if (PHOTOTUBE_ARRAY_SCAN_HZ * PHOTOTUBE_ARRAY_SCAN_PERIOD_MS) != 1000u
#error "Phototube scan frequency and millisecond period are inconsistent"
#endif

/* S3 S2 S1 S0 = binary channel number, Y0..Y15. */
#define PHOTOTUBE_ARRAY_ADC_CHANNEL (ADC3_CH5_P00_7)
#define PHOTOTUBE_ARRAY_ADC_GROUP (3u)
#define PHOTOTUBE_ARRAY_ADC_GROUP_CHANNEL (5u)
#define PHOTOTUBE_ARRAY_S0_PIN (P00_6)
#define PHOTOTUBE_ARRAY_S1_PIN (P00_5)
#define PHOTOTUBE_ARRAY_S2_PIN (P00_4)
#define PHOTOTUBE_ARRAY_S3_PIN (P00_3)
#define PHOTOTUBE_ARRAY_MUX_SELECTOR_COUNT (4u)
#define PHOTOTUBE_ARRAY_MUX_SETTLE_US (3u)
#define PHOTOTUBE_ARRAY_ADC_SAMPLE_COUNT (2u)

/* Physical geometry confirmed for this installation. */
/* Installed orientation: Y0 is the physical left channel, so array index
 * grows toward vehicle right.  This is also the screen display order and the
 * raw-odom convention used by the point cloud (right lateral is positive). */
#define PHOTOTUBE_ARRAY_Y0_IS_LEFT (1u)
/* On-car measurement: the track line is near ADC=0 and the background is
 * near ADC=4095.  The midpoint is fixed; 16 samples are too few for a
 * meaningful per-frame histogram threshold. */
#define PHOTOTUBE_ARRAY_ADC_FULL_SCALE (4095u)
/* No-line protection thresholds are centralized in competition_config.h. */

#define PHOTOTUBE_ARRAY_SPAN_MM \
    ((PHOTOTUBE_ARRAY_COUNT - 1u) * PHOTOTUBE_ARRAY_SPACING_MM)
#define PHOTOTUBE_ARRAY_MAX_CANDIDATES ((PHOTOTUBE_ARRAY_COUNT + 1u) / 2u)
/* Boundary points remain in telemetry for width diagnostics, but the centre
 * keeps most of the frame confidence because it is the localization sample. */

#if PHOTOTUBE_ARRAY_LINE_THRESHOLD_ADC >= PHOTOTUBE_ARRAY_ADC_FULL_SCALE
#error "Phototube line threshold must be below ADC full scale"
#endif

typedef struct
{
    uint32 frame_seq;
    uint32 capture_ms;
    float front_offset_mm;
    float signal_sum;
    float contrast_gray;
    float variance_mm2;
    uint16 active_mask;
    uint16 raw_adc[PHOTOTUBE_ARRAY_COUNT];
    uint8 strength[PHOTOTUBE_ARRAY_COUNT];
    uint8 active_count;
    uint8 edge_detected;
    uint8 valid;
    uint8 white_percent;       /* Legacy field name: active low-line coverage, 0..100. */
    uint8 peak_count;
    uint8 full_coverage;
    uint8 threshold_gray;       /* Fixed ADC threshold quantized by /16. */
} PhototubeArraySnapshot;

void phototube_array_init(void);
void phototube_array_task(void);
uint8 phototube_array_get_snapshot(PhototubeArraySnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* CODE_PHOTOTUBE_ARRAY_H_ */
