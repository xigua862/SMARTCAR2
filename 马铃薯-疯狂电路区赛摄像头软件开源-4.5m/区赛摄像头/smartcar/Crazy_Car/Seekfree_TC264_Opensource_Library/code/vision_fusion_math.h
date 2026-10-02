/**
 * @file vision_fusion_math.h
 * @brief 视觉定位融合使用的坐标变换和鲁棒计算。
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

#ifndef CODE_VISION_FUSION_MATH_H_
#define CODE_VISION_FUSION_MATH_H_

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

enum
{
    VISION_FIT_MAX_POINTS = 20,
    VISION_FIT_MAX_PAIRS =
        (VISION_FIT_MAX_POINTS * (VISION_FIT_MAX_POINTS - 1)) / 2
};

typedef struct
{
    float offset_at_reference;
    float slope;
    float mse;
    uint16_t used_count;
    uint16_t row_span;
} VisionLineFit;

typedef struct
{
    float pair_slopes[VISION_FIT_MAX_PAIRS];
    float values[VISION_FIT_MAX_POINTS];
    uint8_t inlier[VISION_FIT_MAX_POINTS];
} VisionFitWorkspace;

typedef struct
{
    float slope;
    float intercept;
    float squared_error;
    unsigned int inlier_count;
    unsigned int row_span;
} VisionLineCandidate;

static inline float vision_median(float *values, unsigned int count)
{
    unsigned int i;

    for (i = 1u; i < count; i++)
    {
        float value = values[i];
        unsigned int position = i;

        while ((position > 0u) && (values[position - 1u] > value))
        {
            values[position] = values[position - 1u];
            position--;
        }
        values[position] = value;
    }
    if (0u == (count & 1u))
    {
        return 0.5f * (values[count / 2u - 1u] + values[count / 2u]);
    }
    return values[count / 2u];
}

static inline void vision_score_line(const float *rows,
                                     const float *offsets,
                                     unsigned int count,
                                     float slope,
                                     float intercept,
                                     float residual_limit,
                                     VisionLineCandidate *candidate)
{
    unsigned int i;
    float min_row = 0.0f;
    float max_row = 0.0f;

    candidate->slope = slope;
    candidate->intercept = intercept;
    candidate->squared_error = 0.0f;
    candidate->inlier_count = 0u;
    candidate->row_span = 0u;
    for (i = 0u; i < count; i++)
    {
        float residual = offsets[i] - (intercept + slope * rows[i]);

        if (fabsf(residual) <= residual_limit)
        {
            candidate->squared_error += residual * residual;
            if (0u == candidate->inlier_count)
            {
                min_row = rows[i];
                max_row = rows[i];
            }
            else
            {
                if (rows[i] < min_row)
                {
                    min_row = rows[i];
                }
                if (rows[i] > max_row)
                {
                    max_row = rows[i];
                }
            }
            candidate->inlier_count++;
        }
    }
    if (candidate->inlier_count > 0u)
    {
        candidate->row_span =
            (unsigned int)(max_row - min_row + 0.5f);
    }
}

static inline int vision_candidate_better(const VisionLineCandidate *candidate,
                                          const VisionLineCandidate *best)
{
    return (candidate->inlier_count > best->inlier_count) ||
           ((candidate->inlier_count == best->inlier_count) &&
            (candidate->row_span > best->row_span)) ||
           ((candidate->inlier_count == best->inlier_count) &&
            (candidate->row_span == best->row_span) &&
            (candidate->squared_error < best->squared_error));
}

static inline void vision_search_pair_lines(const float *rows,
                                            const float *offsets,
                                            unsigned int count,
                                            float residual_limit,
                                            const uint8_t *allowed_points,
                                            VisionLineCandidate *best)
{
    unsigned int i;
    unsigned int j;

    for (i = 0u; i < count; i++)
    {
        for (j = i + 1u; j < count; j++)
        {
            float row_delta;
            float slope;
            float intercept;
            VisionLineCandidate candidate;

            if ((NULL != allowed_points) &&
                (!allowed_points[i] || !allowed_points[j]))
            {
                continue;
            }
            row_delta = rows[j] - rows[i];
            if (fabsf(row_delta) <= 1.0e-6f)
            {
                continue;
            }
            slope = (offsets[j] - offsets[i]) / row_delta;
            intercept = offsets[i] - slope * rows[i];
            vision_score_line(rows, offsets, count, slope, intercept,
                              residual_limit, &candidate);
            if (vision_candidate_better(&candidate, best))
            {
                *best = candidate;
            }
        }
    }
}

static inline int vision_refit_selected_line(const float *rows,
                                             const float *offsets,
                                             unsigned int count,
                                             float reference_row,
                                             const uint8_t *selected,
                                             VisionLineFit *result)
{
    unsigned int selected_count = 0u;
    unsigned int i;
    float sum_row = 0.0f;
    float sum_offset = 0.0f;
    float mean_row;
    float mean_offset;
    float covariance = 0.0f;
    float variance = 0.0f;
    float min_row;
    float max_row;
    unsigned int range_initialized = 0u;

    for (i = 0u; i < count; i++)
    {
        if (selected[i])
        {
            sum_row += rows[i];
            sum_offset += offsets[i];
            selected_count++;
        }
    }
    if (selected_count < 3u)
    {
        return 0;
    }
    mean_row = sum_row / (float)selected_count;
    mean_offset = sum_offset / (float)selected_count;
    min_row = 0.0f;
    max_row = 0.0f;
    for (i = 0u; i < count; i++)
    {
        if (selected[i])
        {
            float row_delta = rows[i] - mean_row;
            covariance += row_delta * (offsets[i] - mean_offset);
            variance += row_delta * row_delta;
            if (!range_initialized)
            {
                min_row = rows[i];
                max_row = rows[i];
                range_initialized = 1u;
            }
            else
            {
                if (rows[i] < min_row)
                {
                    min_row = rows[i];
                }
                if (rows[i] > max_row)
                {
                    max_row = rows[i];
                }
            }
        }
    }
    if (variance <= 1.0e-6f)
    {
        return 0;
    }

    result->slope = covariance / variance;
    result->offset_at_reference =
        mean_offset + result->slope * (reference_row - mean_row);
    result->mse = 0.0f;
    for (i = 0u; i < count; i++)
    {
        if (selected[i])
        {
            float fitted = result->offset_at_reference +
                           result->slope * (rows[i] - reference_row);
            float residual = offsets[i] - fitted;
            result->mse += residual * residual;
        }
    }
    result->mse /= (float)selected_count;
    result->used_count = (uint16_t)selected_count;
    result->row_span = (uint16_t)(max_row - min_row + 0.5f);
    return 1;
}

/* 先用两两斜率和截距中位数得到粗线，再只从粗线附近点生成候选。
 * 若附近候选达不到最低一致率，才回退到全部点对，避免粗线偶尔偏离真线。 */
static inline int vision_fit_line_reject(const float *rows,
                                         const float *offsets,
                                         unsigned int count,
                                         float reference_row,
                                         float residual_limit,
                                         unsigned int minimum_inlier_percent,
                                         VisionFitWorkspace *workspace,
                                         VisionLineFit *result)
{
    unsigned int pair_count = 0u;
    unsigned int nearby_count = 0u;
    unsigned int required_count;
    unsigned int i;
    unsigned int j;
    float rough_slope;
    float rough_intercept;
    float rough_mad;
    float nearby_limit;
    float adaptive_limit;
    VisionLineCandidate best;
    VisionLineFit refined;

    if ((NULL == rows) || (NULL == offsets) || (NULL == workspace) ||
        (NULL == result) || (count < 3u) ||
        (count > VISION_FIT_MAX_POINTS) || (residual_limit <= 0.0f) ||
        (minimum_inlier_percent < 1u) || (minimum_inlier_percent > 100u))
    {
        return 0;
    }

    for (i = 0u; i < count; i++)
    {
        for (j = i + 1u; j < count; j++)
        {
            float row_delta = rows[j] - rows[i];

            if (fabsf(row_delta) > 1.0e-6f)
            {
                workspace->pair_slopes[pair_count++] =
                    (offsets[j] - offsets[i]) / row_delta;
            }
        }
    }
    if (0u == pair_count)
    {
        return 0;
    }
    rough_slope = vision_median(workspace->pair_slopes, pair_count);

    for (i = 0u; i < count; i++)
    {
        workspace->values[i] = offsets[i] - rough_slope * rows[i];
    }
    rough_intercept = vision_median(workspace->values, count);
    for (i = 0u; i < count; i++)
    {
        workspace->values[i] =
            fabsf(offsets[i] - (rough_intercept + rough_slope * rows[i]));
    }
    rough_mad = vision_median(workspace->values, count);

    nearby_limit = 2.0f * residual_limit;
    adaptive_limit = 4.4478f * rough_mad;
    if (adaptive_limit > nearby_limit)
    {
        nearby_limit = adaptive_limit;
    }
    if (nearby_limit > 4.0f * residual_limit)
    {
        nearby_limit = 4.0f * residual_limit;
    }

    for (i = 0u; i < count; i++)
    {
        float residual = offsets[i] -
                         (rough_intercept + rough_slope * rows[i]);

        workspace->inlier[i] =
            (uint8_t)(fabsf(residual) <= nearby_limit);
        if (workspace->inlier[i])
        {
            nearby_count++;
        }
    }

    vision_score_line(rows, offsets, count, rough_slope, rough_intercept,
                      residual_limit, &best);
    if (nearby_count >= 2u)
    {
        vision_search_pair_lines(rows, offsets, count, residual_limit,
                                 workspace->inlier, &best);
    }

    required_count =
        (count * minimum_inlier_percent + 99u) / 100u;
    if (best.inlier_count < required_count)
    {
        vision_search_pair_lines(rows, offsets, count, residual_limit,
                                 NULL, &best);
    }
    if (best.inlier_count < 3u)
    {
        return 0;
    }

    for (i = 0u; i < count; i++)
    {
        float residual = offsets[i] -
                         (best.intercept + best.slope * rows[i]);
        workspace->inlier[i] =
            (uint8_t)(fabsf(residual) <= residual_limit);
    }
    if (!vision_refit_selected_line(rows, offsets, count, reference_row,
                                    workspace->inlier, result))
    {
        return 0;
    }

    for (i = 0u; i < count; i++)
    {
        float fitted = result->offset_at_reference +
                       result->slope * (rows[i] - reference_row);
        workspace->inlier[i] =
            (uint8_t)(fabsf(offsets[i] - fitted) <= residual_limit);
    }
    if (vision_refit_selected_line(rows, offsets, count, reference_row,
                                   workspace->inlier, &refined))
    {
        *result = refined;
    }
    return 1;
}

static inline uint32_t vision_age_ms(uint32_t now_ms, uint32_t capture_ms)
{
    return (uint32_t)(now_ms - capture_ms);
}

static inline float vision_rate_limited_step(float innovation,
                                             float gain,
                                             float rate_mm_per_s,
                                             uint32_t frame_dt_ms)
{
    float max_step = rate_mm_per_s * (float)frame_dt_ms * 0.001f;
    float step = innovation * gain;

    if (step > max_step)
    {
        step = max_step;
    }
    else if (step < -max_step)
    {
        step = -max_step;
    }
    return step;
}

#endif /* CODE_VISION_FUSION_MATH_H_ */
