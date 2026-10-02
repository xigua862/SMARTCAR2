/**
 * @file navigation.h
 * @brief 点云地图定位、路径投影和 v17 横向控制。
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
 * @note 点云匹配在后台计算，定位修正在 1 ms 导航中断原子提交。
 */

/*
 * navigation.h —— 惯性导航、点云定位、连续路径投影与地图循迹接口。
 * v17 由离线曲率生成目标偏航率，并用 Frenet 误差回正和陀螺角速度闭环
 * 形成唯一转向输出；16路线路观测只生成定位点云、遥测和全黑保护。
 */

#ifndef CODE_NAVIGATION_H_
#define CODE_NAVIGATION_H_

#include "zf_common_headfile.h"
#include "competition_config.h"
#include "drive_odometry.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 惯导地图回放是唯一控制形态；单排线路阵列只用于点云定位和保护。

/* 推车调试是运行态行为,无需开关:进光电调试页(发车页)即以当前摆位重开
 * 惯导会话,未发车时 10ms 步距推进(control_odometer_idle_update),手推车
 * 沿线走实时看 P(测量px)/V(有效)/A(累计纠偏);发车时 nav_start 重开正式
 * 会话,自动退出推车态。只往前推别倒车;想重零退出再进本页。 */

// 当前模拟光电点云定位配置的只读快照，供 EMEM 固件指纹和日志头使用。
typedef struct
{
    uint32 algorithm_version;
    uint8 sensor_count;
    uint8 adc_group;
    uint8 adc_channel;
    uint8 mux_selector_count;
    uint32 scan_hz;
    float sensor_spacing_mm;
    float sensor_forward_mm;
    float scan_min_step_mm;
    float match_travel_window_mm;
    float feature_radius_mm;
    float max_feature_point_residual_mm;
    float max_rms_mm;
    float max_correction_mm;
    float max_correction_yaw_deg;
    float black_stop_distance_mm;
} NavLocalizationConfigSnapshot;

/* ============ 离线曲率目标偏航率 + 陀螺闭环 ============
 * 地图生成器预先给出 kappa(s)。控制器按车速和已辨识的执行链延迟，在
 * s + v*tau 读取曲率并生成目标偏航率；当前 Frenet 横向/航向误差只作为
 * 回正项。差速运动学前馈和陀螺角速度误差反馈仍形成唯一 turn：
 *
 *   omega_ff  = v_actual*kappa(s + v_preview*tau)
 *   omega_cmd = omega_ff - K_psi*epsi
 *                         - K_y*ey/max(v_actual, v_min)
 *   turn_ff   = -(b/2)*omega_cmd
 *   turn_fb   = K_omega*(omega_measured-omega_cmd)
 *   turn      = turn_ff+turn_fb
 *
 * b、v 使用一致长度单位；turn 为 cm/s 半轮差。回正增益是现有固定参数，
 * 不随速度再次放大；达到目标角速度时反馈严格归零。地图定位只改变 map pose
 * 和下一拍曲率取样进度。
 */
/* Tracking gains and preview geometry are centralized in competition_config.h. */
/* The phototube array is 98.5 mm ahead of the axle.  At the 5 m/s
 * operating speed the measured speed-loop/actuator delay is about 35 ms, so
 * the curvature feedforward must read roughly 175 mm ahead.  Keep this as a time
 * delay (not a speed-scaled gain): distance = v * tau grows automatically
 * with speed, while the electronics and motor delay do not. */
/* Curvature preview timing is centralized in competition_config.h. */
/* Straight speed-test heading hold; equal wheel references alone are
 * open-loop when the two motors do not produce identical speed. */
/* epsi=车体航向-路径切线、ey=车体位于路径右侧为正；二者均应产生反向回正偏航率。 */

/* 以下常量只保留给路线哈希和旧日志兼容重算；v17 不读取它们控制车辆。 */
#define NAV_TRACK_KD_CMPS_PER_DEGPS_PER_SPEED_SCALE (0.012422f)
#define NAV_TRACK_GKD_CMPS_PER_RADPS (1.5f)

/* 下列前瞻常量保留给路径投影、地图接管区协议与遥测兼容；v17 的目标偏航率
 * 不再通过 Pure Pursuit 前视点生成。 */
/* Keep the map-takeover lead at 150mm.  Curves use a separate 200mm floor so
 * closely spaced alternating turns do not look through the following bend. */
/* v17 uses delayed offline curvature feedforward plus Frenet error recovery. */
#define NAV_TRACKING_REFERENCE_ALGORITHM_VERSION (17u)

// 连续进度预测后的唯一投影窗；投影失败进入 NAV_STATE_ERROR，不切换搜索策略。

/* ================= 地图速度上限 + 车端在线速度剖面 =================
 * 地图生成器按全局速度、逐点曲率和横向加速度生成每个位置的速度上限；
 * 车端再按前方低速段的可达性和最大
 * 加/减速度逐周期跟踪，保留安全余量，避免速度表段切换在控制ISR中形成
 * 瞬时车速阶跃。NAV_SPEED_TABLES 仍是唯一地图速度来源，不回退到 Flash 旧表。
 * 段值 <50cm/s 视为导出异常并进入导航错误，不替换成默认速度继续运行。
 * ⚠ 需要含 NAV_SPEED_TABLES 的 nav_path.h；地图协议不提供运行时旧表回退。
 * (2026-07-19 用户定案删除开关宏,不留回头路;回退只剩 git 层。) */


// 导航状态机的四个状态。
typedef enum
{
    NAV_STATE_IDLE = 0,   // 空闲
    NAV_STATE_NAVIGATING, // 回放导航中
    NAV_STATE_FINISHED,   // 路径播完(等待停车消费)
    NAV_STATE_ERROR       // 错误:无有效路径
} NavSystemState;

// 生命周期与主循环接口
void nav_init(void);                              // 上电初始化并装载 nav_path.h 中的唯一导出路线
uint8 nav_ready_for_start(void);                  // 发车前只读检查，不修改里程/导航状态
// nav_start 应在 control_enable() 之后立即调用:彼时 VQF yaw 发车零点已锁存。
void nav_start(void);
void nav_stop(void);                              // 停止回放回空闲,可重入
// 推进回放,1ms 速度环内调用,传入本周期左右编码器脉冲增量(前进为正);非导航态直接返回。
void nav_update(int16 pulses_l, int16 pulses_r, float sample_period_s);
// 直线速度测试在不启动地图导航时，仅推进同一份差速里程计；正常扫图不走此接口。
void nav_update_odometry_only(int16 pulses_l, int16 pulses_r,
                              float sample_period_s);
NavSystemState nav_get_state(void);
// 读取未经过地图匹配修正的 CPU0 原始航位推算，供扫描时间补偿使用。
uint8 nav_get_raw_odometry_pose(DriveOdometryPose *pose);
/* CPU0 background diagnostic update.  The line observation is not a control
 * input and therefore must not run from the 1 ms control ISR. */
void nav_update_line_diagnostics(void);
/* A single missed seqlock read is tolerated.  This flag becomes sticky for
 * the run only when no fresh CPU1 IMU publication has arrived for 10 ms. */
uint8 nav_imu_stale_faulted(void);
uint8 nav_is_complete(void);
// 速度决策:当前回放点所在速度段的目标速度(cm/s);非导航态/无表/参数未配置返回 -1。
// NAVIGATING 与 FINISHED 都有效(FINISHED 保持末段值,避免停车消费前的几毫秒速度回跳)。
float nav_get_target_speed_cmps(void);
// 生成器写入的离线速度轨迹物理约束，供遥测快照和配置核对。
float nav_get_speed_profile_max_accel_cmps2(void);
float nav_get_speed_profile_max_decel_cmps2(void);
float nav_get_speed_profile_safety_margin_mm(void);
// 目标偏航率由延迟补偿的离线曲率前馈和Frenet误差回正生成；
// 差速运动学前馈和陀螺目标偏航率误差反馈形成唯一横向输出(cm/s半轮差)。
float nav_compute_tracking_turn_cmps(float plan_v_cmps,
                                     float actual_v_cmps);
/* Heading/gyro hold used only by the straight speed-test mode. */
float nav_compute_straight_hold_turn_cmps(void);
// 当前角速度闭环的实际误差：目标偏航率减同一控制拍VQF偏航率(deg/s)。
float nav_get_tracking_yaw_rate_error_degps(void);
// 当前控制拍复用的同一份VQF偏航率，供CPU0做编码器-陀螺一致性诊断。
float nav_get_control_yaw_rate_degps(void);
uint8 nav_control_imu_is_valid(void);
// CPU0 10ms任务调用：将最近一段光电扫图与当前路线局部折线匹配。任何可信匹配
// 都完整更新T/map pose并从新地图位姿重投影进度；直角/路口还可校正完整SE(2)。
void nav_scan_map_match_service(void);
// 停车冻结日志前收尾定位事务；已提交结果写账本，未进入控制拍的候选明确记为 stopped。
void nav_scan_finalize_pending_transaction(void);
// 仅在最近有可信点云匹配且当前位置处于地图接管区时返回1，用于避免单排阵列
// 在已知物理特征内短暂全黑就误触发跑线停车。
uint8 nav_scan_map_match_allows_blackout(void);
/* 固定连续全黑停车距离；地图接管区的可信特征豁免由上面的接口独立处理。 */
float nav_get_line_loss_stop_distance_mm(void);
void nav_fail_invalid_reference(void);

// 点云定位只读诊断。地图位姿 = T_map_from_odom * 原始里程计位姿。
void nav_get_map_from_odom(float *tx_mm, float *ty_mm, float *yaw_deg);
uint16 nav_get_localization_update_sequence(void);
void nav_get_localization_config_snapshot(
    NavLocalizationConfigSnapshot *snapshot);
uint32 nav_get_localization_config_hash(void);
uint16 nav_get_scan_feature_count(void);
uint8 nav_get_scan_feature_info(uint16 occurrence,
                                uint8 *feature_type,
                                uint16 *feature_progress_points);

// 调试/菜单显示
uint16 nav_get_total_points(void);
uint8 nav_get_source_route_id(void);              // 地图编辑器中导出来源路线号，仅供日志标识
uint32 nav_get_route_hash(void);                  // 当前 yaw/x/y/速度表的 FNV-1a 标识
float nav_get_path_sample_mm(void);               // 路径表采样间隔(mm)

#ifdef __cplusplus
}
#endif

#endif /* CODE_NAVIGATION_H_ */
