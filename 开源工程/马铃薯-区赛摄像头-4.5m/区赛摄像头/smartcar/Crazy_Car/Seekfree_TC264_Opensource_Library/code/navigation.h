/**
 * @file navigation.h
 * @brief 地图投影、视觉定位修正和 Pure Pursuit 导航。
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
 * navigation.h
 *
 * 惯性导航模块:纯追踪(pure pursuit)。
 * 地图编辑器离线规划路线并导出 nav_path.h(航向表+坐标表);
 * 车上按编码器+yaw_total 航位推算 (x,y),追前方 NAV_PP_LOOKAHEAD_MM
 * 处的路径点,追踪角作为转向修正输出。横向偏差(碰撞/打滑侧移/
 * 发车摆位偏)会被闭环拉回路径。
 *
 * 数据约定(与 map_editor.html 导出一致):
 *   采样间隔 NAV_PATH_SAMPLE_MM(默认 5mm);航向 0.01 度存 int16,
 *   相对发车朝向,右转为正 —— 与 gyroscope.c 的 yaw_total 同号;
 *   坐标 nav_path_x/y 单位 mm,发车坐标系:原点=路线起点,
 *   x=发车朝向,y=其右侧。
 *
 * 接入方式:
 *   nav_init()   上电初始化(cpu0_main)
 *   nav_start()  发车进入 RUNNING 时调用(prestart.c)
 *   nav_update() 1ms 速度环内调用,传入本周期编码器脉冲增量(control.c)
 *   nav_get_correction() 角度修正输出,乘 NAV_CORRECTION_TO_DIFF 作为转向输入
 *   nav_is_complete()    路径跑完,Duty_10ms 消费后按 STOP_REASON_PATH_DONE 停车
 */

#ifndef CODE_NAVIGATION_H_
#define CODE_NAVIGATION_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ==================== 摄像头区横向纠偏 ====================
 * 转向全程由惯导导航输出；摄像头在编辑器规划的摄像头区内
 * (nav_zone_*,掐头去尾留余量)连续测中线偏差，按限速步长把惯导
 * 横向坐标拉向真值。没有“识别→事件”环节。
 * 需要编辑器导出的 nav_path.h(含 NAV_HYBRID_TABLES 区表)。
 * ⚠ 历史(同日,勿重试):①"直道摄像头巡线/弯道惯导"分区握盘+弯口拐点识别
 * 交接,实车三连败撤销(70cm 提前交盘出界/280mm 前瞻紧弯跟不住出界/弯口
 * 识别只在 25~30cm 甜点可见);②直行路口对表(三通道识别→一次性校准),
 * 因路口误识别撤销;③从线扫描取"纵向"信息是历史五连败毒根,本机制只取
 * 横向,区门保证视野是直道,限步保证单帧垃圾拉不动坐标。 */
/* 推车调试是运行态行为,无需开关:进图像调试页(发车页)即以当前摆位重开
 * 惯导会话,未发车时 10ms 步距推进(control_odometer_idle_update),手推车
 * 沿线走实时看 P(测量px)/V(有效)/A(累计纠偏);发车时 nav_start 重开正式
 * 会话,自动退出推车态。只往前推别倒车;想重零退出再进本页。 */

/* 摄像头融合总开关。具体门限、像素尺度和相机模型由 navigation.c 的
 * 配置结构管理；模型无效或处于 SHADOW 时，航向融合自动保持关闭。 */
#define NAV_VISION_FUSION_ENABLE (1u)

typedef enum
{
    NAV_VISION_DROP_NONE = 0u,
    NAV_VISION_DROP_PRESESSION,
    NAV_VISION_DROP_STALE,
    NAV_VISION_DROP_QUALITY,
    NAV_VISION_DROP_HISTORY,
    NAV_VISION_DROP_ZONE,
    NAV_VISION_DROP_YAW,
    NAV_VISION_DROP_INNOVATION,
    /* 追加在末尾:菜单 Reject 显示原始数字,老代码含义(ZONE=5/YAW=6/INNO=7)不变 */
    NAV_VISION_DROP_NOCAL /* 禁校区(地图 nocal 节点入前一段):元件污染视野,只信赖惯导 */
} VisionRejectReason;

/* 惯导角度修正(度)换算到转向环输入的比例,上车标定。
 * turn_PID 原本吃图像中线偏差(像素量纲),惯导给的是航向角误差,
 * 先低速跑,按转向响应强弱调这个系数。
 * 1.0 是完赛验证配置(配 150mm 前瞻);混动调试期曾临时改 2.0,已随
 * "回到原来那一套"一并复原——嫌转向软再往上加,一次只动一个数。 */
#define NAV_CORRECTION_TO_DIFF (1.0f)

/* 纯追踪前瞻距离(mm):追前方这么远的路径点。
 * 太短抖、太长切内弯,经验:取值 ≈ 编辑器规划的转弯半径。
 * 150=完赛验证值,4.5m/s 提速也先保持 150(2026-07-12 用户定,曾短暂
 * 改 190 又撤回);若 4.5 直道蛇形,再考虑 150→170→190 逐级试,
 * 190(≈规划弯半径194)是上限量级。
 * ⚠ 教训(2026-07-11 深夜):曾为治"转弯太晚"单独放大弯道前瞻到 280,
 * 实车 194mm 紧弯直接跟不住、出界全黑——前瞻超出弯半径太多则纯追踪
 * 轨迹外扩,紧弯必炸,勿再往 280 摸。 */
#define NAV_PP_LOOKAHEAD_MM (150.0f)

/* 纯追踪最近点搜索窗口(mm):每周期从上次进度向前搜这么远找最近路径点。
 * 只需覆盖一个控制周期的最大位移再留余量,窗口小才不会在自交路线上跳点。 */
#define NAV_PP_SEARCH_MM (200.0f)

typedef enum
{
    NAV_STATE_IDLE = 0,   /* 空闲 */
    NAV_STATE_NAVIGATING, /* 回放导航中 */
    NAV_STATE_FINISHED,   /* 路径播完(等待停车消费) */
    NAV_STATE_ERROR       /* 错误:无有效路径 */
} NavSystemState;

/* 上电初始化,默认选路线 0。 */
void nav_init(void);
/* 选择路线 0..NAV_PATH_COUNT-1,导航中不允许切换;返回 1 成功。 */
uint8 nav_select_path(uint8 path_id);
/* 开始回放。应在 control_enable() 之后立即调用:
 * 彼时 yaw_total 归零请求已发出,发车朝向即 0 度基准。 */
void nav_start(void);
/* 停止回放,回到空闲。停车路径统一调用,可重入。 */
void nav_stop(void);
/* 推进回放。1ms 速度环内调用,传入本周期左右编码器脉冲增量(前进为正)。
 * 非导航态内部直接返回。 */
void nav_update(int16 pulses_l, int16 pulses_r);
/* 当前状态。 */
NavSystemState nav_get_state(void);
/* 当前角度修正输出(度):目标相对航向 - 当前相对航向,右转为正。 */
float nav_get_correction(void);
/* 路径是否播完。 */
uint8 nav_is_complete(void);

/* ==================== 横向纠偏 ==================== */
/* 本圈累计横向纠偏量(mm,带符号,发车系右手方向为正),调试显示。 */
float nav_get_lat_corr_mm(void);
float nav_get_heading_bias_deg(void);
/* 本圈摄像头检测到直行路口的次数(上升沿)。验证路口检测是否可靠:
 * 推车跑一圈,该值应约等于路线实际经过的直行路口数;数值远多=误检、
 * 远少=漏检。 */
uint32 nav_get_crossing_count(void);
uint32 nav_get_vision_age_ms(void);
uint32 nav_get_vision_accept_count(void);
uint32 nav_get_vision_drop_count(void);
VisionRejectReason nav_get_vision_last_drop_reason(void);
uint16 nav_get_vision_valid_rows(void);
uint16 nav_get_vision_flags(void);
float nav_get_vision_fit_mse_px2(void);

/* ==================== 调试/菜单显示 ==================== */
/* 已回放的采样点序号。 */
uint16 nav_get_progress(void);
/* 当前路线总点数。 */
uint16 nav_get_total_points(void);
/* 本圈 |修正| 峰值(度)，nav_start 时清零；停车后读它评估全程跟踪质量。 */
float nav_get_max_abs_correction(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_NAVIGATION_H_ */
