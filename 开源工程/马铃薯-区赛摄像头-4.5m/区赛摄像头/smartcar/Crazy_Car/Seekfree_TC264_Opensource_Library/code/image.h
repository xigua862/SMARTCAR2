/**
 * @file image.h
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
 * image.h
 *
 * Migrated from the 264 project and adapted to the current chassis project.
 */

#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"

#define COL 150
#define ROW 80

#define BLACK 0
#define WHITE 255

/* 拐点搜索窗（行号，近端→远端）：窗外的边界突变不参与拐点判定。
 * 近场（大于 START 的行）被排除——元件符号的边在近场被透视放大成假突变。
 * 调试页会把这两条界线用黄色横线画在二值图上。 */
#define GUAIDIAN_SEARCH_START (50u)
#define GUAIDIAN_SEARCH_END (20u)

/* 拐点判定：相隔 GUAIDIAN_JUMP_ROWS 行的边界横向跳变达到
 * GUAIDIAN_JUMP_MIN_PX 像素即记为拐点行。
 * 放宽方式：基线 7→14 行、门限 42 不动（用户定）——三叉/箭头口的边界
 * 斜向岔开，拉长基线让斜边累积出足够横移，陡峭度要求折半，
 * 比直接降门限更不容易被单行毛刺误触。误检另有 nav_straight_ahead
 * 准入门+连续帧确认兜底；实车嫌敏感/迟钝直接调这两个数。
 * find_guaidian 的行号下限保护已跟随本宏，改这里不用再动别处。 */
#define GUAIDIAN_JUMP_ROWS (14)
#define GUAIDIAN_JUMP_MIN_PX (42)

#define THRESH_MIN 35
#define THRESH_MAX 75

/* 全局 Otsu 曝光搜索钳位：把直方图搜索范围锁进合理灰度带，
 * 眩光/深阴影的极端灰度不参与阈值计算，抑制强光干扰。 */
#define OTSU_MIN_GRAYSCALE 0
#define OTSU_MAX_GRAYSCALE 100

/* 单全局 Otsu 基准阈值的三段偏移量（草莽 close/mid/far_Threshold）。
 * 三段统一：阈值 = 基准 + OFFSET，偏移量带符号——
 *   正值 → 阈值更高、白区更小（更严）；负值 → 阈值更低、白区更大（更敏感）。
 * 远处想看更远给 FAR 负值，近处过曝想更稳给 NEAR 正值，中段一般 0。
 * 默认全 0 即三段共用同一全局阈值。负值请写成 (-10) 形式，避免宏展开歧义。 */
#define OTSU_NEAR_OFFSET (5)
#define OTSU_MID_OFFSET 0
#define OTSU_FAR_OFFSET 0

/* 远/中/近三段的分界行（半开区间）：远 [0,27) 中 [27,54) 近 [54,80)。 */
#define BLOCK_FAR_END 27
#define BLOCK_MID_END 54

#define RECT_LEFT 10
#define RECT_RIGHT 140
#define RECT_TOP 5
#define RECT_BOTTOM 75

/* 矩形检测框四边开口信息。 */
typedef struct
{
    uint8 count;
    uint8 top_exist;
    uint8 bottom_exist;
    uint8 left_exist;
    uint8 right_exist;
    uint8 top_col;    /* 顶边最左开口中心列：左转弯中锚点（弯中顶边常有两个开口） */
    uint8 top_col_r;  /* 顶边最右开口中心列：右转弯中锚点；只有一个开口时等于 top_col */
    uint8 bottom_col; /* 底边开口中心列 */
    uint8 left_row;   /* 左边开口中心行：左转拉线的首选锚点（分支线真实位置） */
    uint8 right_row;  /* 右边开口中心行：右转拉线的首选锚点 */
} PathInfo;

extern PathInfo path_info;

extern uint8 bin_image[ROW][COL];
extern uint8 orig_image[ROW][COL];

extern uint8 l_border[ROW];
extern uint8 r_border[ROW];
extern uint8 mid_line[ROW];
extern uint8 l_guaidian;
extern uint8 r_guaidian;
/* 读取远/中/近三段二值化阈值。 */
void image_get_otsu_thresholds(uint8 *far, uint8 *mid, uint8 *near);
/* 灰度图二值化，并统计全白/全黑异常帧。 */
void image_binarize(uint8 (*src_image)[COL]);
/* 搜索左右边界和中线。 */
void search_line(void);
/* 查找左右拐点。 */
void find_guaidian(uint8 start, uint8 end);
/* 统计矩形框四边开口数量。 */
uint8 find_rect_path(uint8 (*bin_image_data)[COL]);
/* 请求下一帧前重置视觉测量状态。 */
void image_request_vision_state_reset(void);
/* 查询当前帧是否为异常帧。 */
uint8 image_need_stop(void);
/* 读取本帧通过50%一致率门的拟合中线；无有效拟合时返回0。 */
uint8 image_get_fitted_center_line(float *position_px,
                                   float *slope_px_per_row);
/* 处理已复制的相机帧。 */
void image_proc_copied(uint32 frame_seq, uint32 capture_ms);
/* 读取全图白占比（%），供菜单调试显示。 */
uint8 image_get_frame_white_percent(void);

#endif /* CODE_IMAGE_H_ */
