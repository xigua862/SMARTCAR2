/**
 * @file image.c
 * @brief 图像二值化、边线提取、补线和中线观测生成。
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

/*
 * image.c
 *
 * Migrated from the 264 project and adapted to the current chassis project.
 */

#include "image.h"

#include <stdlib.h>
#include <string.h>

#include "globals.h"
#include "task.h"
#include "key.h"
#include "navigation.h"
#include "camera_pipeline.h"
#include "vision_fusion_math.h"

#define IMAGE_STOP_CONFIRM_FRAMES (3u)
#define IMAGE_STOP_EXTREME_RATIO_PERCENT (1u)
/* Otsu 阈值每 N 帧重算一次，其余帧复用；1 表示每帧都算。 */
#define OTSU_UPDATE_INTERVAL_FRAMES (1u)

/* 图像缓冲和状态放 CPU1 本地 DSRAM，避免视觉处理逐像素跨核访问。 */
#pragma section all "cpu1_dsram"

uint8 bin_image[ROW][COL];
uint8 orig_image[ROW][COL];

uint8 l_border[ROW];
uint8 r_border[ROW];
uint8 mid_line[ROW];

static uint8 thresh_far = 128;
static uint8 thresh_mid = 128;
static uint8 thresh_near = 128;
static uint8 image_extreme_frame = 0u;
static volatile uint8 frame_white_percent = 0u; /* 全图白占比（%），调试页显示用。 */

/* 极值帧的连续确认计数。放文件级而不是函数内静态：
 * 发车重置（image_reset_vision_state_local）必须能清掉它，
 * 否则预发车阶段攒下的计数会带进新一轮运行。 */
static uint8 image_stop_confirm_count = 0u;

/* 读取全图白占比，供菜单调试显示。 */
uint8 image_get_frame_white_percent(void)
{
    return frame_white_percent;
}

/* 读取当前三段 Otsu 阈值。
 * 菜单调试页面用它显示远/中/近区域的二值化阈值。
 */
void image_get_otsu_thresholds(uint8 *far, uint8 *mid, uint8 *near)
{
    if (far)
    {
        *far = thresh_far;
    }
    if (mid)
    {
        *mid = thresh_mid;
    }
    if (near)
    {
        *near = thresh_near;
    }
}

uint8 l_guaidian = 0;
uint8 r_guaidian = 0;

PathInfo path_info = {0};
static volatile uint8 image_turn_reset_request = 0u;

/* FKDL523 风格的全图 Otsu：先把极暗/极亮灰度钳进当前相机的有效曝光带，
 * 再完整搜索类间方差最大值。最终阈值仍钳在本车 THRESH_MIN..THRESH_MAX。 */
static uint8 global_otsu(uint8 (*src_image)[COL])
{
    uint32 histogram[256] = {0};
    uint16 row;
    uint16 col;
    uint32 pixel_count = (uint32)ROW * COL;
    uint32 pixel_sum = 0u;
    uint16 threshold;
    uint32 background_count = 0u;
    uint32 background_sum = 0u;
    float max_variance = 0.0f;
    uint8 best_threshold = THRESH_MIN;
    uint8 min_gray = OTSU_MAX_GRAYSCALE;
    uint8 max_gray = OTSU_MIN_GRAYSCALE;

    for (row = 0u; row < ROW; row++)
    {
        for (col = 0u; col < COL; col++)
        {
            uint8 gray = src_image[row][col];

            if (gray < OTSU_MIN_GRAYSCALE)
            {
                gray = OTSU_MIN_GRAYSCALE;
            }
            else if (gray > OTSU_MAX_GRAYSCALE)
            {
                gray = OTSU_MAX_GRAYSCALE;
            }
            histogram[gray]++;
            pixel_sum += gray;
            if (gray < min_gray)
            {
                min_gray = gray;
            }
            if (gray > max_gray)
            {
                max_gray = gray;
            }
        }
    }

    if (min_gray == max_gray)
    {
        if (min_gray < THRESH_MIN)
        {
            return THRESH_MIN;
        }
        if (min_gray > THRESH_MAX)
        {
            return THRESH_MAX;
        }
        return min_gray;
    }

    best_threshold = min_gray;
    for (threshold = min_gray; threshold < max_gray; threshold++)
    {
        uint32 foreground_count;
        uint32 foreground_sum;
        float background_mean;
        float foreground_mean;
        float variance;

        background_count += histogram[threshold];
        background_sum += histogram[threshold] * threshold;

        foreground_count = pixel_count - background_count;
        if (0u == foreground_count || 0u == background_count)
            continue;

        foreground_sum = pixel_sum - background_sum;
        background_mean = (float)background_sum / (float)background_count;
        foreground_mean = (float)foreground_sum / (float)foreground_count;
        variance = (float)background_count * (float)foreground_count * (background_mean - foreground_mean) *
                   (background_mean - foreground_mean);

        if (variance > max_variance)
        {
            max_variance = variance;
            best_threshold = (uint8)threshold;
        }
    }

    if (best_threshold < THRESH_MIN)
    {
        best_threshold = THRESH_MIN;
    }
    else if (best_threshold > THRESH_MAX)
    {
        best_threshold = THRESH_MAX;
    }
    return best_threshold;
}

/* ==================== 二值图去噪 ====================
 * 3x3 邻域白点投票,一遍扫过:
 *   孤立白噪点(白像素,8 邻域白点 <= DENOISE_WHITE_MAX)抹黑;
 *   线内黑孔  (黑像素,8 邻域白点 >= DENOISE_BLACK_MIN)补白。
 * 阈值故意保守:远场导线可能细到 1 像素宽(竖向线每点恰有 2 个白邻),
 * WHITE_MAX 取 1 只删真正的孤立点/斜对点,绝不吃线;
 * BLACK_MIN 取 7 只填几乎全包围的孔,不在路口缝隙处乱粘线。
 * 就地处理:上一行的修正会进入下一行的判断(等效轻微迭代),无碍。
 * 用每列竖向三连白计数滑窗实现,全图 80x150 约 0.4ms。 */
#define IMAGE_DENOISE_ENABLE (1u)
#define DENOISE_WHITE_MAX (1u)
#define DENOISE_BLACK_MIN (7u)

#if IMAGE_DENOISE_ENABLE
static void image_denoise(uint8 (*img)[COL])
{
    uint8 tri[COL]; /* 每列 (row-1,row,row+1) 三连白计数 0..3 */
    int16 row;
    int16 col;

    for (row = 1; row < (int16)(ROW - 1); row++)
    {
        for (col = 0; col < (int16)COL; col++)
        {
            tri[col] = (uint8)(((WHITE == img[row - 1][col]) ? 1u : 0u) +
                               ((WHITE == img[row][col]) ? 1u : 0u) +
                               ((WHITE == img[row + 1][col]) ? 1u : 0u));
        }
        for (col = 1; col < (int16)(COL - 1); col++)
        {
            /* 9 宫格白数减掉中心自己 = 8 邻域白数 */
            uint8 center_white = (uint8)((WHITE == img[row][col]) ? 1u : 0u);
            uint8 neighbors = (uint8)(tri[col - 1] + tri[col] + tri[col + 1] - center_white);

            if (center_white && (neighbors <= DENOISE_WHITE_MAX))
            {
                img[row][col] = BLACK;
            }
            else if ((0u == center_white) && (neighbors >= DENOISE_BLACK_MIN))
            {
                img[row][col] = WHITE;
            }
        }
    }
}
#endif

/* 将灰度图按远/中/近三段阈值二分到 bin_image。
 * 本函数只生成原始二值并统计白占比；随后仅做保守的孤立点去噪。
 * 注意：bin_image 与 orig_image 保持同一坐标系，
 * 否则后续边线搜索和视觉拟合的行列假设全部失效。
 */
void image_binarize(uint8 (*src_image)[COL])
{
    static uint8 thresh_update_cnt = OTSU_UPDATE_INTERVAL_FRAMES; /* 首帧强制算一次。 */
    int16 row;
    int16 col;
    uint32 white_count = 0u;
    uint32 total = (uint32)ROW * COL;

    /* Otsu 阈值按帧分频更新：赛道光照在相邻几帧内基本不变，
     * 每帧全图重算直方图是主要的固定耗时，分频后其余帧直接复用旧阈值。 */
    if (++thresh_update_cnt >= OTSU_UPDATE_INTERVAL_FRAMES)
    {
        int16 base = (int16)global_otsu(src_image);
        int16 threshold;

        thresh_update_cnt = 0u;

        threshold = base + OTSU_FAR_OFFSET;
        thresh_far = (uint8)((threshold < THRESH_MIN) ? THRESH_MIN :
                             ((threshold > THRESH_MAX) ? THRESH_MAX : threshold));
        threshold = base + OTSU_MID_OFFSET;
        thresh_mid = (uint8)((threshold < THRESH_MIN) ? THRESH_MIN :
                             ((threshold > THRESH_MAX) ? THRESH_MAX : threshold));
        threshold = base + OTSU_NEAR_OFFSET;
        thresh_near = (uint8)((threshold < THRESH_MIN) ? THRESH_MIN :
                              ((threshold > THRESH_MAX) ? THRESH_MAX : threshold));
    }

    /* 按当前行阈值直接二分；安全统计在形态学之前完成。 */
    for (row = 0; row < ROW; row++)
    {
        uint8 threshold = thresh_near;

        if (row < BLOCK_FAR_END)
        {
            threshold = thresh_far;
        }
        else if (row < BLOCK_MID_END)
        {
            threshold = thresh_mid;
        }

        for (col = 0; col < COL; col++)
        {
            if (src_image[row][col] > threshold)
            {
                bin_image[row][col] = WHITE;
                white_count++;
            }
            else
            {
                bin_image[row][col] = BLACK;
            }
        }
    }

    image_extreme_frame =
        (uint8)((white_count >= total * (100u - IMAGE_STOP_EXTREME_RATIO_PERCENT) / 100u) ||
                (white_count <= total * IMAGE_STOP_EXTREME_RATIO_PERCENT / 100u));

    /* 全图白占比，调试页显示用（复用已数好的 white_count，无额外遍历）。 */
    frame_white_percent = (uint8)(((uint32)white_count * 100u) / (uint32)total);
}

/* 逐行搜索左右边界并计算中线。
 * 从底部到顶部扫描黑白跳变，默认找不到边界时贴到图像左右边缘。
 */
void search_line(void)
{
    int16 row;

    memset(l_border, 0, sizeof(l_border));
    memset(r_border, COL - 1, sizeof(r_border));
    memset(mid_line, COL / 2, sizeof(mid_line));

    for (row = ROW - 1; row >= 0; row--)
    {
        uint8 l_find_flag = 0;
        uint8 r_find_flag = 0;
        int16 col;

        for (col = 0; col < (COL - 1); col++)
        {
            if ((BLACK == bin_image[row][col]) && (WHITE == bin_image[row][col + 1]))
            {
                l_find_flag = 1;
                l_border[row] = (uint8)col;
                break;
            }
        }

        if (!l_find_flag)
        {
            l_border[row] = 0;
        }

        for (col = COL - 1; col >= 2; col--)
        {
            if ((BLACK == bin_image[row][col]) && (WHITE == bin_image[row][col - 1]))
            {
                r_find_flag = 1;
                r_border[row] = (uint8)col;
                break;
            }
        }

        if (!r_find_flag)
        {
            r_border[row] = COL - 1;
        }

        mid_line[row] = (uint8)((l_border[row] + r_border[row]) / 2u);
    }
}

/* 根据左右边线突变寻找拐点。
 * start/end 允许传反，函数内部会修正，并限制最小行号避免
 * i - GUAIDIAN_JUMP_ROWS 下溢。
 */
void find_guaidian(uint8 start, uint8 end)
{
    int16 i;
    uint8 t;

    l_guaidian = 0;
    r_guaidian = 0;

    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }

    if (start >= (ROW - 1u - 5u))
        start = ROW - 1u - 5u;
    /* 下限保护跟随跳变基线：保证 i - GUAIDIAN_JUMP_ROWS 不越界到负行。 */
    if (end <= GUAIDIAN_JUMP_ROWS)
        end = GUAIDIAN_JUMP_ROWS;

    for (i = start; i >= end; i--)
    {
        if ((0u == l_guaidian) &&
            (abs((int)l_border[i] - (int)l_border[i - GUAIDIAN_JUMP_ROWS]) >= GUAIDIAN_JUMP_MIN_PX))
        {
            l_guaidian = (uint8)i;
        }

        if ((0u == r_guaidian) &&
            (abs((int)r_border[i] - (int)r_border[i - GUAIDIAN_JUMP_ROWS]) >= GUAIDIAN_JUMP_MIN_PX))
        {
            r_guaidian = (uint8)i;
        }

        if ((0u != l_guaidian) && (0u != r_guaidian))
        {
            break;
        }
    }
}

/* 统计矩形检测框四条边上的白色通道数量。
 * 结果写入 path_info，用于判断路口方向和当前可走路径数量。
 */
uint8 find_rect_path(uint8 (*bin_image_data)[COL])
{
    uint8 col;
    uint8 row;

    memset(&path_info, 0, sizeof(path_info));

    // ===== 上边 =====
    // 弯中顶边常同时有两个开口（原直行走廊 + 目标分支），扫全整条边：
    // top_col 记最左开口中心（左转锚点），top_col_r 记最右（右转锚点）。
    {
        uint8 found_enter = 0;
        uint8 enter_col = 0;
        uint8 first_done = 0;
        for (col = RECT_LEFT; col < (RECT_RIGHT - 1u); col++)
        {
            if (!found_enter)
            {
                if ((BLACK == bin_image_data[RECT_TOP][col]) && (WHITE == bin_image_data[RECT_TOP][col + 1u]))
                {
                    found_enter = 1;
                    enter_col = col + 1u;
                }
            }
            else
            {
                if ((WHITE == bin_image_data[RECT_TOP][col]) && (BLACK == bin_image_data[RECT_TOP][col + 1u]))
                {
                    uint8 run_center = (uint8)(((uint16)enter_col + col) / 2u);

                    if (!first_done)
                    {
                        path_info.top_exist = 1;
                        path_info.top_col = run_center;
                        path_info.count++;
                        first_done = 1;
                    }
                    path_info.top_col_r = run_center; /* 持续覆盖 = 最右一段 */
                    found_enter = 0;                  /* 继续找下一段开口 */
                }
            }
        }
    }

    // ===== 下边 =====
    {
        uint8 found_enter = 0;
        uint8 enter_col = 0;
        for (col = RECT_LEFT; col < (RECT_RIGHT - 1u); col++)
        {
            if (!found_enter)
            {
                if ((BLACK == bin_image_data[RECT_BOTTOM][col]) && (WHITE == bin_image_data[RECT_BOTTOM][col + 1u]))
                {
                    found_enter = 1;
                    enter_col = col + 1u;
                }
            }
            else
            {
                if ((WHITE == bin_image_data[RECT_BOTTOM][col]) && (BLACK == bin_image_data[RECT_BOTTOM][col + 1u]))
                {
                    path_info.bottom_exist = 1;
                    path_info.bottom_col = (uint8)(((uint16)enter_col + col) / 2u);
                    path_info.count++;
                    break;
                }
            }
        }
    }

    // ===== 左边 =====
    {
        uint8 found_enter = 0;
        uint8 enter_row = 0;
        for (row = RECT_TOP; row < (RECT_BOTTOM - 1u); row++)
        {
            if (!found_enter)
            {
                if ((BLACK == bin_image_data[row][RECT_LEFT]) && (WHITE == bin_image_data[row + 1u][RECT_LEFT]))
                {
                    found_enter = 1;
                    enter_row = row + 1u;
                }
            }
            else
            {
                if ((WHITE == bin_image_data[row][RECT_LEFT]) && (BLACK == bin_image_data[row + 1u][RECT_LEFT]))
                {
                    path_info.left_exist = 1;
                    path_info.left_row = (uint8)(((uint16)enter_row + row) / 2u);
                    path_info.count++;
                    break;
                }
            }
        }
    }

    // ===== 右边 =====
    {
        uint8 found_enter = 0;
        uint8 enter_row = 0;
        for (row = RECT_TOP; row < (RECT_BOTTOM - 1u); row++)
        {
            if (!found_enter)
            {
                if ((BLACK == bin_image_data[row][RECT_RIGHT]) && (WHITE == bin_image_data[row + 1u][RECT_RIGHT]))
                {
                    found_enter = 1;
                    enter_row = row + 1u;
                }
            }
            else
            {
                if ((WHITE == bin_image_data[row][RECT_RIGHT]) && (BLACK == bin_image_data[row + 1u][RECT_RIGHT]))
                {
                    path_info.right_exist = 1;
                    path_info.right_row = (uint8)(((uint16)enter_row + row) / 2u);
                    path_info.count++;
                    break;
                }
            }
        }
    }

    return path_info.count;
}

/* 横向纠偏测量帧处理（替代查表状态机）：惯导全程转向,摄像头唯一职责是
 * 每帧给出"车相对贴线的横向偏差"及其有效性,CPU0 在编辑器规划的摄像头区
 * 内消费(navigation.c)。路口对表/弯口识别均已撤销(2026-07-11,见 navigation.h)。
 */

/* 测量行：参考行在图像约 1/3 高度，中心位置和斜率由同一次拟合给出。 */
enum
{
    VISION_FIT_ROW_LO = 16,
    VISION_FIT_ROW_HI = 50,
    VISION_FIT_ROW_STEP = 2,
    VISION_FIT_MIN_ROWS = 8,
    VISION_FIT_MIN_SPAN = 12,
    VISION_FIT_FAR_MAX_ROW = 28,
    VISION_FIT_NEAR_MIN_ROW = 38,
    VISION_FIT_MIN_SEGMENT_ROWS = 3
};

/* 只拟合中线(用户定,2026-07-12):中线本就是左右边界的平均,
 * 左/右边线独立拟合+三线一致性门是冗余计算,已删。 */
typedef struct
{
    float center_residual_limit_px;
    uint8 minimum_inlier_percent;
} ImageVisionFitConfig;

static const ImageVisionFitConfig image_vision_fit_config = {
    3.0f, /* center_residual_limit_px */
    50u   /* minimum_inlier_percent */
};

static uint32 image_frame_seq_current = 0u;
static uint32 image_capture_ms_current = 0u;
static uint32 image_publish_seq_counter = 0u;
static float image_fitted_center_slope = 0.0f;

typedef struct
{
    VisionFitWorkspace fit_workspace;
    float rows[VISION_FIT_MAX_POINTS];
    float center_offsets[VISION_FIT_MAX_POINTS];
} ImageVisionFitWorkspace;

static ImageVisionFitWorkspace image_vision_workspace;

/* ==================== 直行三叉路口补线状态机 ====================
 * 直行道上前方出现拐点(三叉分支/箭头把边线打断)时按拐点行号分三段补线,
 * 让鲁棒拟合与横向纠偏在穿越三叉时不断流:
 *   SJ_FAR : 拐点仍在上1/3 → 下1/3实测中线向上按斜率外推,只盖远端乱区
 *   SJ_MID : 拐点进入中1/3 → 底端锚点直连顶边开口,整幅桥接
 *   SJ_NEAR: 拐点进入下1/3 → 上1/3实测中线向下按斜率外推,盖掉近端乱区
 * 拐点从下1/3消失(连续确认)即穿越完成退出。
 * 路线侧由 CPU0 的 nav_straight_ahead 把关:前方0.4m(视距35cm+余量)
 * 非直行立即放手,真弯道拐点不会被补线盖掉,补出的"合格"假观测也就
 * 不会在区尾污染纠偏。行号约定:0=图像顶(远端),ROW-1=底(近端)。 */
enum
{
    SJ_TOP_THIRD_ROW = (ROW / 3),        /* 26:行号<=此值属上1/3 */
    SJ_BOTTOM_THIRD_ROW = (ROW * 2 / 3), /* 53:行号>此值属下1/3 */
    SJ_ENTER_CONFIRM_FRAMES = 2,         /* 拐点连续帧确认后才进状态机 */
    SJ_EXIT_CONFIRM_FRAMES = 2,          /* 下1/3拐点连续消失确认后退出 */
    SJ_TIMEOUT_FRAMES = 600,             /* 兜底:拐点跟踪丢失时约3s强制退出 */
    SJ_FIT_MIN_ROWS = 6,                 /* 源段最少实测行数,不足不补 */
    SJ_PATCH_MARGIN_ROWS = 3,            /* 拐点行附近的过渡带一并盖掉 */
    SJ_DEFAULT_HALF_WIDTH_PX = 8         /* 无实测半宽时的兜底线半宽 */
};

typedef enum
{
    SJ_IDLE = 0,
    SJ_FAR,
    SJ_MID,
    SJ_NEAR
} SjPhase;

static SjPhase sj_phase = SJ_IDLE;
static uint8 sj_enter_confirm = 0u;
static uint8 sj_exit_confirm = 0u;
static uint16 sj_frame_count = 0u;
static uint8 sj_peak_row = 0u; /* 会话内拐点最大行号,单调只进不退 */

static void sj_reset(void)
{
    sj_phase = SJ_IDLE;
    sj_enter_confirm = 0u;
    sj_exit_confirm = 0u;
    sj_frame_count = 0u;
    sj_peak_row = 0u;
}

/* 对[row_lo,row_hi]内左右边界齐全的行做中线最小二乘,同时给出平均半宽。 */
static uint8 sj_fit_mid_segment(uint8 row_lo, uint8 row_hi,
                                float *mean_row, float *mean_col,
                                float *slope_col_per_row, float *half_width)
{
    uint8 row;
    uint8 n = 0u;
    float sum_r = 0.0f;
    float sum_c = 0.0f;
    float sum_rr = 0.0f;
    float sum_rc = 0.0f;
    float sum_w = 0.0f;

    for (row = row_lo; row <= row_hi; row++)
    {
        if ((l_border[row] > 0u) && (r_border[row] < (COL - 1u)))
        {
            float r = (float)row;
            float c = 0.5f * ((float)l_border[row] + (float)r_border[row]);

            sum_r += r;
            sum_c += c;
            sum_rr += r * r;
            sum_rc += r * c;
            sum_w += 0.5f * ((float)r_border[row] - (float)l_border[row]);
            n++;
        }
    }
    if (n < SJ_FIT_MIN_ROWS)
    {
        return 0u;
    }
    {
        float fn = (float)n;
        float var = sum_rr - sum_r * sum_r / fn;

        if (var < 1.0f)
        {
            return 0u;
        }
        *mean_row = sum_r / fn;
        *mean_col = sum_c / fn;
        *slope_col_per_row = (sum_rc - sum_r * sum_c / fn) / var;
        *half_width = sum_w / fn;
    }
    return 1u;
}

/* 把一行改写成补出的中线±半宽(l/r_border 与 mid_line 一起改,
 * 拟合采样"两侧边界齐全"的条件才能继续成立)。 */
static void sj_write_row(uint8 row, float mid_col, float half_width)
{
    float l = mid_col - half_width;
    float r = mid_col + half_width;

    if (l < 1.0f)
    {
        l = 1.0f;
    }
    if (r > (float)(COL - 2))
    {
        r = (float)(COL - 2);
    }
    if (r < l)
    {
        r = l;
    }
    l_border[row] = (uint8)l;
    r_border[row] = (uint8)r;
    mid_line[row] = (uint8)(0.5f * (l + r));
}

/* 斜率式补线:参考点+斜率把[row_from,row_to]整段改写。 */
static void sj_patch_slope(uint8 row_from, uint8 row_to,
                           float mean_row, float mean_col,
                           float slope_col_per_row, float half_width)
{
    uint8 row;

    for (row = row_from; row <= row_to; row++)
    {
        sj_write_row(row,
                     mean_col + slope_col_per_row * ((float)row - mean_row),
                     half_width);
    }
}

/* 桥接补线:底端锚点(ROW-1行)直连顶边开口(RECT_TOP行),整幅改写,
 * RECT_TOP 以上按同斜率延伸。 */
static void sj_patch_bridge(float bottom_col, float top_col, float half_width)
{
    uint8 row;
    float k = (top_col - bottom_col) /
              ((float)RECT_TOP - (float)(ROW - 1));

    for (row = 0u; row < ROW; row++)
    {
        sj_write_row(row,
                     bottom_col + k * ((float)row - (float)(ROW - 1)),
                     half_width);
    }
}

/* 直行三叉补线在拐点/开口刷新后、拟合采样前调用。 */
static void sj_update(void)
{
    uint8 g = (l_guaidian > r_guaidian) ? l_guaidian : r_guaidian;
    float mean_row;
    float mean_col;
    float slope;
    float halfw;

    if (SJ_IDLE == sj_phase)
    {
        if ((0u == g) || (0u == nav_straight_ahead))
        {
            sj_enter_confirm = 0u;
            return;
        }
        if (++sj_enter_confirm < SJ_ENTER_CONFIRM_FRAMES)
        {
            return;
        }
        sj_enter_confirm = 0u;
        sj_exit_confirm = 0u;
        sj_frame_count = 0u;
        sj_phase = SJ_FAR;
        sj_peak_row = g;
    }
    else
    {
        /* 跟踪期换宽窗重找拐点:默认窗近端截在50行,进不了下1/3。
         * 近场假突变(元件符号)只影响会话内的相位推进,准入仍用严窗。 */
        find_guaidian((uint8)(ROW - 1u), GUAIDIAN_SEARCH_END);
        g = (l_guaidian > r_guaidian) ? l_guaidian : r_guaidian;
        if (g > sj_peak_row)
        {
            sj_peak_row = g;
        }
        if (0u == nav_straight_ahead)
        {
            sj_reset();
            return;
        }
        if (++sj_frame_count > SJ_TIMEOUT_FRAMES)
        {
            sj_reset();
            return;
        }
    }

    /* 相位随拐点最大行号单调推进(拐点只会越来越近,不回退)。 */
    if ((SJ_FAR == sj_phase) && (sj_peak_row > SJ_TOP_THIRD_ROW))
    {
        sj_phase = SJ_MID;
    }
    if ((SJ_MID == sj_phase) && (sj_peak_row > SJ_BOTTOM_THIRD_ROW))
    {
        sj_phase = SJ_NEAR;
    }

    /* 退出:拐点从下1/3消失,连续确认防单帧闪断。 */
    if (SJ_NEAR == sj_phase)
    {
        if ((0u == g) || (g <= SJ_BOTTOM_THIRD_ROW))
        {
            if (++sj_exit_confirm >= SJ_EXIT_CONFIRM_FRAMES)
            {
                sj_reset();
                return;
            }
        }
        else
        {
            sj_exit_confirm = 0u;
        }
    }

    switch (sj_phase)
    {
    case SJ_FAR:
        /* 下1/3实测中线向上外推,只盖拐点行以上的乱区,中段实测保留。 */
        if (sj_fit_mid_segment((uint8)(SJ_BOTTOM_THIRD_ROW + 1), (uint8)(ROW - 1u),
                               &mean_row, &mean_col, &slope, &halfw))
        {
            uint8 patch_top = (uint8)(sj_peak_row + SJ_PATCH_MARGIN_ROWS);

            if (patch_top > SJ_BOTTOM_THIRD_ROW)
            {
                patch_top = SJ_BOTTOM_THIRD_ROW;
            }
            sj_patch_slope(0u, patch_top, mean_row, mean_col, slope, halfw);
        }
        break;

    case SJ_MID:
    {
        float bottom_col;
        uint8 have_bottom = sj_fit_mid_segment((uint8)(SJ_BOTTOM_THIRD_ROW + 1),
                                               (uint8)(ROW - 1u),
                                               &mean_row, &mean_col, &slope, &halfw);

        if (have_bottom)
        {
            bottom_col = mean_col + slope * ((float)(ROW - 1) - mean_row);
        }
        else if (path_info.bottom_exist)
        {
            bottom_col = (float)path_info.bottom_col;
            halfw = (float)SJ_DEFAULT_HALF_WIDTH_PX;
        }
        else
        {
            break; /* 底端没有可信锚点,本帧不补 */
        }

        if (path_info.top_exist)
        {
            /* 顶边可能同时有直行开口和分支开口,取与底端锚点更对齐的那个。 */
            float cand_l = (float)path_info.top_col;
            float cand_r = (float)path_info.top_col_r;
            float top_col = (fabsf(cand_l - bottom_col) <= fabsf(cand_r - bottom_col))
                                ? cand_l
                                : cand_r;

            sj_patch_bridge(bottom_col, top_col, halfw);
        }
        else if (have_bottom)
        {
            /* 顶边开口尚未露出:先按底段斜率外推顶住,露出后自动转桥接。 */
            sj_patch_slope(0u, (uint8)SJ_BOTTOM_THIRD_ROW,
                           mean_row, mean_col, slope, halfw);
        }
        break;
    }

    case SJ_NEAR:
        /* 上1/3实测中线向下外推,近端乱区整段盖掉。 */
        if (sj_fit_mid_segment(0u, (uint8)SJ_TOP_THIRD_ROW,
                               &mean_row, &mean_col, &slope, &halfw))
        {
            sj_patch_slope((uint8)(SJ_TOP_THIRD_ROW + 1), (uint8)(ROW - 1u),
                           mean_row, mean_col, slope, halfw);
        }
        break;

    default:
        break;
    }
}

static void image_publish_pose_observation(uint16 flags,
                                           float line_position_px,
                                           float line_slope_px_per_row,
                                           float fit_mse_px2,
                                           uint16 valid_rows,
                                           uint16 candidate_rows,
                                           uint16 row_span)
{
    uint32 publish_seq = image_publish_seq_counter + 1u;
    uint8 slot;
    CameraCaptureMetadata capture;
    volatile VisionPoseObservation *dst;

    if (!camera_pipeline_read_capture_metadata(image_frame_seq_current, &capture))
    {
        /* 没有同一场的 IMU 快照时仍发布无效帧，不能沿用旧槽中的航向。 */
        flags = 0u;
        capture.pure_gyro_yaw_deg = 0.0f;
    }

    if (0u == publish_seq)
    {
        /* UINT32 wrap follows an odd token, so resume on an even token. */
        publish_seq = 2u;
    }
    image_publish_seq_counter = publish_seq;
    slot = (uint8)(publish_seq & 1u);
    dst = &vision_pose_slots[slot];

    dst->publish_seq = publish_seq;
    dst->frame_seq = image_frame_seq_current;
    dst->capture_ms = image_capture_ms_current;
    dst->capture_gyro_yaw_deg = capture.pure_gyro_yaw_deg;
    dst->line_position_px = line_position_px;
    dst->line_slope_px_per_row = line_slope_px_per_row;
    dst->fit_mse_px2 = fit_mse_px2;
    dst->valid_rows = valid_rows;
    dst->candidate_rows = candidate_rows;
    dst->row_span = row_span;
    dst->flags = flags;
    __dsync();
    vision_pose_publish_seq = publish_seq;
}

static void image_update_vision_observation(uint8 (*bin_image_data)[COL])
{
    uint8 row;
    uint8 fit_count = 0u;
    uint8 far_rows = 0u;
    uint8 near_rows = 0u;
    uint8 side_open;
    VisionLineFit center_fit;
    uint16 flags = 0u;

    /* 拐点/矩形开口每帧照常刷新：横向测量的视野门要用,调试页也看它们。 */
    find_guaidian(GUAIDIAN_SEARCH_START, GUAIDIAN_SEARCH_END);
    find_rect_path(bin_image_data);

    /* 侧开口只作为诊断标记。导航只在规划直行区内消费观测，因此三岔直行
     * 仍应继续拟合前向白线；分支造成的异常行交给鲁棒拟合质量门排除。 */
    side_open = (uint8)(path_info.left_exist || path_info.right_exist);
    if (side_open)
    {
        flags |= VISION_OBS_FLAG_SIDE_OPEN;
    }

    /* 直行路口检测(四边开口数≥3 且顶边有开口)：地图门控弧长重置的触发
     * 信号。判据沿用实车验证过的直行路口图案。
     * 无论下面的横向拟合是否有效,本标志都随观测发布,穿越路口时拟合常
     * 失效,信号不能依赖它。 */
    if ((path_info.count >= 3u) && path_info.top_exist)
    {
        flags |= VISION_OBS_FLAG_CROSSING;
    }

    /* 直行三叉补线:需要时改写 l/r_border 与 mid_line,后面的拟合采样
     * 直接使用补好的线。 */
    sj_update();

    image_lat_valid = 0u;

    for (row = VISION_FIT_ROW_LO;
         (row <= VISION_FIT_ROW_HI) && (fit_count < VISION_FIT_MAX_POINTS);
         row = (uint8)(row + VISION_FIT_ROW_STEP))
    {
        if ((l_border[row] > 0u) && (r_border[row] < (COL - 1u)))
        {
            image_vision_workspace.rows[fit_count] = (float)row;
            image_vision_workspace.center_offsets[fit_count] =
                0.5f * ((float)l_border[row] + (float)r_border[row]) -
                (float)(COL / 2u);
            if (row <= VISION_FIT_FAR_MAX_ROW)
            {
                far_rows++;
            }
            if (row >= VISION_FIT_NEAR_MIN_ROW)
            {
                near_rows++;
            }
            fit_count++;
        }
    }

    if ((fit_count >= VISION_FIT_MIN_ROWS) &&
        (far_rows >= VISION_FIT_MIN_SEGMENT_ROWS) &&
        (near_rows >= VISION_FIT_MIN_SEGMENT_ROWS) &&
        vision_fit_line_reject(image_vision_workspace.rows,
                               image_vision_workspace.center_offsets,
                               fit_count, (float)(ROW / 3u),
                               image_vision_fit_config.center_residual_limit_px,
                               image_vision_fit_config.minimum_inlier_percent,
                               &image_vision_workspace.fit_workspace, &center_fit) &&
        (center_fit.used_count >= VISION_FIT_MIN_ROWS) &&
        ((uint32)center_fit.used_count * 100u >=
         (uint32)fit_count *
             (uint32)image_vision_fit_config.minimum_inlier_percent) &&
        (center_fit.row_span >= VISION_FIT_MIN_SPAN) &&
        (center_fit.mse <= (image_vision_fit_config.center_residual_limit_px *
                            image_vision_fit_config.center_residual_limit_px)))
    {
        /* P 与 S 必须来自同一次鲁棒拟合，不能再混用五行简单平均。 */
        image_lat_offset_px = center_fit.offset_at_reference;
        image_fitted_center_slope = center_fit.slope;
        image_lat_valid = 1u;
        flags |= VISION_OBS_FLAG_LATERAL_VALID |
                 VISION_OBS_FLAG_FIT_VALID |
                 VISION_OBS_FLAG_COVERAGE |
                 VISION_OBS_FLAG_EDGE_VALID;
        image_publish_pose_observation(flags,
                                       center_fit.offset_at_reference,
                                       center_fit.slope,
                                       center_fit.mse,
                                       center_fit.used_count,
                                       fit_count,
                                       center_fit.row_span);
    }
    else
    {
        image_lat_valid = 0u;
        image_publish_pose_observation(flags,
                                       0.0f,
                                       0.0f, 0.0f, 0u, fit_count, 0u);
    }
}
/* 横向纠偏跨核量清零:发车/全量重置共用。 */
static void image_hybrid_reset(void)
{
    image_lat_offset_px = 0.0f;
    image_fitted_center_slope = 0.0f;
    image_lat_valid = 0u;
    sj_reset(); /* 直行三叉补线状态机同步归零 */
    __dsync();
    vision_pose_publish_seq = 0u;
}

/* 重置摄像头测量和路口几何诊断状态。 */
static void image_reset_vision_state_local(void)
{
    l_guaidian = 0;
    r_guaidian = 0;

    memset(&path_info, 0, sizeof(path_info));
    image_stop_confirm_count = 0u;

    image_hybrid_reset();
}

/* 请求在下一帧图像处理前重置转弯状态。
 * 用标志位跨模块通知，避免控制中断直接改图像状态机内部变量。
 */
void image_request_vision_state_reset(void)
{
    image_turn_reset_request = 1u;
}

/* 查询当前帧是否属于全白/全黑异常帧。
 * image_proc_copied() 会连续确认多帧后才真正请求停车。
 */
uint8 image_need_stop(void)
{
    return image_extreme_frame;
}

uint8 image_get_fitted_center_line(float *position_px,
                                   float *slope_px_per_row)
{
    if (!image_lat_valid || (NULL == position_px) ||
        (NULL == slope_px_per_row))
    {
        return 0u;
    }
    *position_px = image_lat_offset_px;
    *slope_px_per_row = image_fitted_center_slope;
    return 1u;
}

/* 处理已经复制到 orig_image 的一帧图像。
 * 主流程：二值化、异常帧确认、边线搜索、路口状态机、误差计算。
 */
void image_proc_copied(uint32 frame_seq, uint32 capture_ms)
{
    image_frame_seq_current = frame_seq;
    image_capture_ms_current = capture_ms;

    if (image_turn_reset_request)
    {
        image_turn_reset_request = 0u;
        image_reset_vision_state_local();
    }

    image_binarize(orig_image);
#if IMAGE_DENOISE_ENABLE
    image_denoise(bin_image);
#endif

    if (image_need_stop())
    {
        if (image_stop_confirm_count < IMAGE_STOP_CONFIRM_FRAMES)
        {
            image_stop_confirm_count++;
        }
        /* 停车仍需连续确认，但任何极值候选帧都不得进入位姿融合。 */
        image_lat_valid = 0u;
        image_publish_pose_observation(0u, 0.0f, 0.0f, 0.0f,
                                       0u, 0u, 0u);
        if (image_stop_confirm_count >= IMAGE_STOP_CONFIRM_FRAMES)
        {
            g_stop_reason = STOP_REASON_EXTREME_FRAME;
            image_stop_request = 1u;
            return;
        }
        return;
    }
    else
    {
        image_stop_confirm_count = 0u;
    }

    search_line();
    image_update_vision_observation(bin_image);
}

#pragma section all restore
