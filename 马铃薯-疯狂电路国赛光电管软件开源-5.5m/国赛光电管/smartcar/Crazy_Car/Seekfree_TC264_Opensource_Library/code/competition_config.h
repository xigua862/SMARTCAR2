/**
 * @file competition_config.h
 * @brief 国赛现场可调的控制、光电、定位和保护参数。
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
 */

/*
 * competition_config.h —— 赛场运行参数唯一入口。
 *
 * 这里集中放置需要在赛场快速切换的固件参数。控制、导航、光电、发车
 * 和 IMU 模块的竞赛调参项只在这里定义，避免同一参数散落在多个模块中改漏。
 * 协议/算法版本、硬件引脚、队列容量和地图导出表仍保留在各自模块，避免
 * 把不可现场调节的兼容性常量误当成比赛参数。
 *
 * 速度上限、路线方向和路线几何不在这里覆盖：它们必须由地图生成器导出
 * 到 nav_path.h，否则会产生“车端偷偷改速/路线哈希不一致”的问题。
 *
 * 单位约定：MM=毫米，CMPS=厘米/秒，DEGPS=度/秒，RADPS=弧度/秒，
 * MS/US=毫秒/微秒，PERCENT=百分比。带“兼容字段”字样的宏只为旧日志
 * 和哈希协议保留，不应作为现场调参旋钮。
 */

#ifndef CODE_COMPETITION_CONFIG_H_
#define CODE_COMPETITION_CONFIG_H_

/* ================= 赛场曲率前馈 =================
 * 当前决赛基线：4.2 m/s 正向地图使用 35 ms 曲率提前量。
 * 固件控制和路线哈希只使用秒值；遥测毫秒字段由该秒值现场换算，
 * 避免改了控制参数却忘记同步显示值。 */
#define NAV_TRACK_CURVATURE_PREVIEW_TIME_S (0.035f) /* 曲率取样提前量；单位 s */
/*
 * 前瞻时间按“空间前视距离”换算：当前 4.2 m/s × 35 ms ≈ 147 mm。
 * 如果提高车速，建议先保持约 147 mm 的空间前视，避免车速越高越提前拐：
 *   4.5 m/s：0.032~0.033 s，建议初始值 0.032f；
 *   5.0 m/s：0.029~0.030 s，建议初始值 0.029f。
 * 若实车仍明显提前，可分别试 0.030f / 0.027f；每次只调整 2~3 ms。
 * 该参数参与控制配置哈希，修改后必须重新编译、刷写并同步遥测解析配置。
 */

/* ================= 全黑/丢线停车 =================
 * 完赛基线固定为连续全黑 200 mm，不按地图特征放宽。 */
#define PHOTOTUBE_ARRAY_BLACK_STOP_DISTANCE_MM (200.0f) /* 全黑停车距离；mm */

/* ================= 电机堵转保护 =================
 * 按目标/实际轮速比例判断，连续 60 ms。 */
#define CONTROL_STALL_DUTY_PERCENT (70u) /* 进入堵转候选的 PWM 百分比 */
#define CONTROL_STALL_TARGET_MIN_CMPS (120.0f) /* 低于此目标速度不判堵转；cm/s */
#define CONTROL_STALL_SPEED_RATIO_PERCENT (40u) /* 实际/目标低于此百分比时判异常 */
#define CONTROL_STALL_MIN_ACTUAL_CMPS (5.0f) /* 实际速度绝对下限；cm/s */
#define CONTROL_STALL_CONFIRM_MS (60u) /* 连续满足时间；ms，越小越敏感 */
#define CONTROL_STALL_START_GRACE_MS (150u) /* 发车后忽略启动瞬态；ms */
#define CONTROL_STALL_ENCODER_COUNT_MAX (1) /* 每拍允许的最大编码器计数 */

/* ================= 车辆几何/速度执行 =================
 * 这些是标定项或车端在线执行器约束，不会覆盖地图导出的空间速度表。
 * 地图侧的 NAV_SPEED_PROFILE_SAFETY_MARGIN_MM 等值仍由 nav_path.h 导出，
 * 这里不再保留同名的无效副本。 */
#define DRIVE_WHEEL_TRACK_MM (119.0f) /* 左右轮接地点中心距；mm */
#define WHEEL_DIAMETER_MM (23.5f) /* 里程换算使用的等效滚动直径；mm */
#define ENCODER_PPR (1024u) /* 编码器每轮端等效脉冲数 */
#define GEAR_RATIO (4.1f) /* 电机到轮端的减速比 */
#define ENCODER_YAW_DEG_PER_DIFF_PULSE (0.008285f) /* 左右差 1 脉冲对应航向角；deg */
#define MOTOR_REFERENCE_LEAD_TIME_MS (22u) /* 速度目标提前下发时间；ms */
#define SPEED_PROFILE_MAX_ACCEL_CMPS2 (3000.0f) /* 车端目标加速度上限；cm/s² */
#define SPEED_PROFILE_MAX_DECEL_CMPS2 (5000.0f) /* 车端目标减速度上限；cm/s² */
#define SPEED_PROFILE_RECONCILE_ERROR_CMPS (25.0f) /* 地图目标与实际速度误差门限；cm/s */

/* 高速滑移保护：只在编码器横摆率与陀螺横摆率明显分叉时增加阻尼。 */
#define SLIP_GUARD_MIN_SPEED_CMPS (300.0f) /* 低于此车速不启用滑移判定；cm/s */
#define SLIP_GUARD_MIN_YAW_RATE_DEGPS (600.0f) /* 低于此横摆率不启用；deg/s */
#define SLIP_GUARD_RATE_FILTER_TAU_S (0.006f) /* 横摆率低通时间常数；s */
#define SLIP_GUARD_MISMATCH_DEADBAND_DEGPS (250.0f) /* 编码器/陀螺差值死区；deg/s */
#define SLIP_GUARD_MISMATCH_RELEASE_DEGPS (180.0f) /* 解除保护的差值；deg/s */
#define SLIP_GUARD_GAIN_CMPS_PER_RADPS (4.0f) /* 滑移误差转阻尼量的增益 */
#define SLIP_GUARD_MAX_CORRECTION_CMPS (25.0f) /* 滑移保护最大附加半差；cm/s */

/* ================= 横向控制/投影 ================= */
#define NAV_TRACK_YAW_RATE_FB_CMPS_PER_RADPS (9.0f) /* 陀螺横摆率反馈增益 */
#define NAV_STRAIGHT_HOLD_HEADING_RADPS_PER_RAD (4.0f) /* 直线航向保持增益 */
#define NAV_TRACK_HEADING_CORRECTION_RADPS_PER_RAD (20.0f) /* 航向误差回正增益 */
#define NAV_TRACK_CROSS_TRACK_CORRECTION_MMPS_PER_MM (100.0f) /* 横向误差回正增益 */
#define NAV_TRACK_MIN_CORRECTION_SPEED_MMPS (500.0f) /* 低于此速度才限制回正量；mm/s */
#define NAV_PP_PREVIEW_TIME_S (0.065f) /* 旧投影诊断前视时间；s */
#define NAV_PP_MIN_LOOKAHEAD_MM_INT (150u) /* 地图接管区最小前视；mm */
#define NAV_PP_MIN_LOOKAHEAD_MM (150.0f) /* 动态前视下限；mm */
#define NAV_PP_MAX_LOOKAHEAD_MM (350.0f) /* 动态前视上限；mm */
#define NAV_PP_CURVE_MIN_LOOKAHEAD_MM (200.0f) /* 弯道前视下限；mm */
#define NAV_TRACK_TARGET_RATE_WINDOW_MS (5u) /* 目标横摆率计算窗口；ms */
#define NAV_TRACK_TARGET_RATE_WINDOW_SAMPLES (5u) /* 目标横摆率窗口样本数 */
#define NAV_PP_PREDICT_SEARCH_MM (80.0f) /* 连续投影预测搜索窗；mm */
#define NAV_TURN_LIMIT_RATIO (1.0f) /* 半差/基础速度上限；1.0=内侧轮最低为0 */

/* ================= 光电/扫图硬件与观测门限 =================
 * 阵列数量、MUX引脚和ADC时钟仍留在 phototube_array.h；这里放机械安装
 * 和可调观测门限。 */
#define PHOTOTUBE_ARRAY_SPACING_MM (8.0f) /* 相邻光电管中心距；mm */
#define PHOTOTUBE_ARRAY_FORWARD_MM (98.5f) /* 光电阵列相对车轴前置距离；mm */
#define PHOTOTUBE_ARRAY_LINE_THRESHOLD_ADC (2047u) /* 黑线判定 ADC 阈值 */
#define PHOTOTUBE_ARRAY_ACTIVE_RATIO (0.20f) /* 单管归一化有效比例阈值 */
#define PHOTOTUBE_ARRAY_FULL_COVERAGE_PERCENT (90u) /* 全黑/全覆盖判定比例；% */
#define PHOTOTUBE_ARRAY_RUN_GEOMETRY_MIN_SPAN_MM (30.0f) /* 有效线段最小横向跨度；mm */
#define PHOTOTUBE_ARRAY_BOUNDARY_QUALITY_RATIO (0.125f) /* 边界点相对中心点的权重 */
#define PHOTOTUBE_ARRAY_FULL_SCAN_HIT_COUNT (8u) /* 一帧至少命中数量才算完整覆盖 */
#define PHOTOTUBE_ARRAY_TRACK_HOLD_MS (60u) /* 观测短暂跳变时保持上一帧；ms */
#define PHOTOTUBE_ARRAY_TRACK_MAX_YAW_STEP_DEG (30.0f) /* 相邻帧允许的最大角跳；deg */
#define PHOTOTUBE_ARRAY_TRACK_MAX_CENTER_STEP_MM (3.0f * PHOTOTUBE_ARRAY_SPACING_MM) /* 中心最大跳变；mm */
#define PHOTOTUBE_ARRAY_TRACK_MAX_WIDTH_STEP_MM (2.0f * PHOTOTUBE_ARRAY_SPACING_MM) /* 宽度最大跳变；mm */
#define PHOTOTUBE_ARRAY_TRACK_UNIQUE_MARGIN_MM (1.0f * PHOTOTUBE_ARRAY_SPACING_MM) /* 候选唯一性余量；mm */
#define PHOTOTUBE_ARRAY_TRACK_WIDTH_SCORE_GAIN (0.5f) /* 宽度变化在候选评分中的权重 */
#define PHOTOTUBE_SCAN_MIN_STEP_MM (5.0f) /* 扫图点最小里程间隔；mm */
#define PHOTOTUBE_SCAN_MAX_POINTS (1280u) /* CPU0 在线扫图环形缓存容量 */
#define PHOTOTUBE_SCAN_POSE_MAX_AGE_MS (15u) /* 允许的位姿插值最大年龄；ms */
#define PHOTOTUBE_SCAN_FIT_WINDOW_POINTS (32u) /* 局部直线拟合最多使用点数 */
#define PHOTOTUBE_SCAN_MOUNTING_DEG (0.0f) /* 阵列相对车体横轴的安装角；deg */
#define PHOTOTUBE_SCAN_MIN_FIT_SPAN_MM (20.0f) /* 局部拟合的最小空间跨度；mm */
#define PHOTOTUBE_SCAN_AUTO_RUN (1u) /* 1=正常运行自动记录扫图；0=仅手动记录 */

/* ================= 点云定位门限 =================
 * 这些参数会进入定位配置哈希；比赛现场不建议随意改，改动后必须重新
 * 编译刷写并用同一份固件参考解码日志。算法/协议版本号仍保留在
 * navigation.c，不放在调参区。 */
#define NAV_SCAN_MATCH_TRAVEL_WINDOW_MM (1000.0f) /* 特征匹配总里程窗；mm */
#define NAV_SCAN_MATCH_STRAIGHT_TRAVEL_WINDOW_MM (120.0f) /* 直线拟合最近里程窗；mm */
/* 直线/转角候选的最小证据量：数量越大越稳，但定位响应越慢。 */
#define NAV_SCAN_MATCH_MIN_POINT_COUNT (6u) /* 单次匹配最少点数 */
#define NAV_SCAN_MATCH_MIN_CORNER_SIDE_COUNT (3u) /* 角点每条边最少有效点数 */
#define NAV_SCAN_MATCH_MIN_SIDE_FRAME_COUNT (3u) /* 每条边最少来源帧数 */
#define NAV_SCAN_MATCH_MIN_RUN_SIDE_FRAME_COUNT (2u) /* 连续直线边最少帧数 */
#define NAV_SCAN_MATCH_MIN_SHORT_SIDE_CLEAN_POINTS (2u) /* 短边清洗后的最少点数 */
#define NAV_SCAN_MATCH_RUN_FRAME_MAX_GAP (2u) /* 连续边允许的最大丢帧数 */
#define NAV_SCAN_MATCH_MIN_STRAIGHT_SPAN_MM (75.0f) /* 直线拟合最小跨度；mm */
#define NAV_SCAN_MATCH_MIN_STRAIGHT_FRAME_COUNT (12u) /* 直线确认最少帧数 */
/* 以下是直线观测的运动显著性、鲁棒拟合和异常点门限。 */
#define NAV_SCAN_MATCH_STRAIGHT_CONFIRM_TRAVEL_MM (0.0f) /* 兼容字段，当前不二次确认 */
#define NAV_SCAN_MATCH_STRAIGHT_CONFIRM_POSITION_MM (0.0f) /* 兼容字段 */
#define NAV_SCAN_MATCH_STRAIGHT_CONFIRM_YAW_DEG (0.0f) /* 兼容字段；deg */
#define NAV_SCAN_MATCH_STRAIGHT_MIN_TRANSLATION_MM (4.0f) /* 直线运动显著位移；mm */
#define NAV_SCAN_MATCH_STRAIGHT_MIN_YAW_DEG (0.5f) /* 直线运动显著转角；deg */
#define NAV_SCAN_MATCH_STRAIGHT_SIGNIFICANCE_SIGMA (3.0f) /* 显著性倍数 */
#define NAV_SCAN_MATCH_STRAIGHT_CORRECTION_BLEND (0.35f) /* 每次直线修正的融合比例 */
#define NAV_SCAN_MATCH_STRAIGHT_MAX_POSE_STEP_MM (8.0f) /* 单次最大平移修正；mm */
#define NAV_SCAN_MATCH_STRAIGHT_EDGE_WEIGHT_RATIO (0.35f) /* 边界点降权比例 */
#define NAV_SCAN_MATCH_MIN_FEATURE_SIDE_SPAN_MM (8.0f) /* 物理边最小跨度；mm */
#define NAV_SCAN_MATCH_MAX_RMS_MM (25.0f) /* 拟合最大 RMS 残差；mm */
#define NAV_SCAN_MATCH_MAX_STRAIGHT_POINT_RESIDUAL_MM (100.0f) /* 直线点最大残差；mm */
#define NAV_SCAN_MATCH_STRAIGHT_ROBUST_MIN_GATE_MM (18.0f) /* 鲁棒门限下限；mm */
#define NAV_SCAN_MATCH_STRAIGHT_ROBUST_MAX_GATE_MM (25.0f) /* 鲁棒门限上限；mm */
#define NAV_SCAN_MATCH_STRAIGHT_ROBUST_MAD_SCALE (3.0f) /* MAD 异常点倍数 */
#define NAV_SCAN_MATCH_STRAIGHT_MIN_INLIER_RATIO (0.45f) /* 直线内点最小比例 */
#define NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_MM (5.0f) /* 位移众数分箱宽度；mm */
#define NAV_SCAN_MATCH_STRAIGHT_MODE_BIN_COUNT (41u) /* 位移众数分箱数量 */
#define NAV_SCAN_MATCH_STRAIGHT_MODE_HALF_WINDOW_BINS (3u) /* 众数局部半窗 */
#define NAV_SCAN_MATCH_STRAIGHT_MODE_MIN_SEPARATION_BINS (7u) /* 两候选最小分离箱数 */
#define NAV_SCAN_MATCH_STRAIGHT_MODE_SEED_GATE_MM (25.0f) /* 众数种子门限；mm */
#define NAV_SCAN_MATCH_STRAIGHT_AMBIGUITY_WEIGHT_RATIO (0.75f) /* 次优候选权重比例 */
#define NAV_SCAN_MATCH_MAX_FEATURE_POINT_RESIDUAL_MM (150.0f) /* 特征点最大残差；mm */
/* 角点/路口只在物理特征附近搜索；搜索窗越大，容错越强但误匹配风险越高。 */
#define NAV_SCAN_MATCH_FEATURE_RADIUS_MM (225.0f) /* 特征候选发现半径；mm */
#define NAV_SCAN_MATCH_FEATURE_LOCAL_RADIUS_MM (150.0f) /* 角点局部求解半径；mm */
#define NAV_SCAN_MATCH_FEATURE_SEARCH_TRANSLATION_MM (150.0f) /* 粗搜索平移窗；mm */
#define NAV_SCAN_MATCH_FEATURE_SEARCH_YAW_DEG (15.0f) /* 粗搜索航向窗；deg */
#define NAV_SCAN_MATCH_FEATURE_SEARCH_COARSE_STEP_MM (75.0f) /* 粗搜索平移步长；mm */
#define NAV_SCAN_MATCH_FEATURE_SEARCH_COARSE_STEP_DEG (7.0f) /* 粗搜索航向步长；deg */
#define NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_TRANSLATION_MM (20.0f) /* 细搜索平移窗；mm */
#define NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_YAW_DEG (2.0f) /* 细搜索航向窗；deg */
#define NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_MM (10.0f) /* 细搜索平移步长；mm */
#define NAV_SCAN_MATCH_FEATURE_SEARCH_FINE_STEP_DEG (1.0f) /* 细搜索航向步长；deg */
#define NAV_SCAN_MATCH_FEATURE_SEARCH_POINT_LIMIT (32u) /* 单次搜索最多使用点数 */
#define NAV_SCAN_MATCH_FEATURE_SEARCH_IDENTITY_MAX_SCORE_MM2 (1600.0f) /* 允许直接局部细化的残差平方；mm² */
#define NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_POINT_COUNT (3u) /* 候选最少点数 */
#define NAV_SCAN_MATCH_FEATURE_CANDIDATE_MIN_FRAME_COUNT (2u) /* 候选最少来源帧数 */
#define NAV_SCAN_MATCH_FEATURE_PROGRESS_AMBIGUITY_MM (120.0f) /* 进度歧义门限；mm */
#define NAV_SCAN_MATCH_FEATURE_ASSOCIATION_MARGIN_MM (12.0f) /* 特征关联余量；mm */
#define NAV_SCAN_MATCH_FEATURE_AMBIGUOUS_WEIGHT_RATIO (0.35f) /* 歧义候选权重比例 */
#define NAV_SCAN_MATCH_FEATURE_MIN_INLIER_WEIGHT_RATIO (0.60f) /* 最小内点权重比例 */
#define NAV_SCAN_MATCH_FEATURE_BOUNDARY_FALLBACK_MM (225.0f) /* 边界点兜底半径；mm */
/* 进度、候选排序和位姿修正约束。正常比赛不建议调整。 */
#define NAV_SCAN_MATCH_FEATURE_MARGIN_POINTS (4.0f) /* 候选前后路径点余量 */
#define NAV_SCAN_MATCH_SEGMENT_MARGIN_MM (180.0f) /* 物理线段候选余量；mm */
#define NAV_SCAN_MATCH_MAX_CORRECTION_MM (400.0f) /* 单次最大平移修正；mm */
#define NAV_SCAN_MATCH_MAX_CORRECTION_YAW_DEG (30.0f) /* 单次最大航向修正；deg */
#define NAV_SCAN_MATCH_COMPLETE_FEATURE_MAX_YAW_DEG (8.0f) /* 完整特征允许的航向误差；deg */
#define NAV_SCAN_MATCH_MIN_EFFECTIVE_WEIGHT (3.0f) /* 总有效权重下限 */
#define NAV_SCAN_MATCH_MIN_SIDE_EFFECTIVE_WEIGHT (0.90f) /* 单边有效权重下限 */
#define NAV_SCAN_MATCH_JUNCTION_MIN_SIDE_EFFECTIVE_WEIGHT (0.60f) /* 路口单边权重下限 */
#define NAV_SCAN_MATCH_PROGRESS_BEHIND_POINTS (80u) /* 当前进度后方保留点数 */
#define NAV_SCAN_MATCH_PROGRESS_SEARCH_POINTS (200u) /* 当前进度前方搜索点数 */
#define NAV_SCAN_COMMIT_HINT_BEHIND_POINTS (8u) /* 快速提交提示后方点数 */
#define NAV_SCAN_COMMIT_HINT_SEARCH_POINTS (24u) /* 快速提交提示前方点数 */
#define NAV_SCAN_MATCH_BLACKOUT_HOLD_MS (400u) /* 定位黑名单保持时间；ms */
#define NAV_SCAN_MATCH_MIN_RMS_IMPROVEMENT_MM (0.5f) /* 新拟合至少改善的 RMS；mm */
#define NAV_SCAN_MATCH_FEATURE_CONFIRM_TRAVEL_MM (0.0f) /* 兼容字段，保持 0 */
#define NAV_SCAN_MATCH_FEATURE_CONFIRM_POSITION_MM (0.0f) /* 兼容字段，保持 0 */
#define NAV_SCAN_MATCH_FEATURE_CONFIRM_YAW_DEG (0.0f) /* 兼容字段，保持 0 */
#define NAV_SCAN_MATCH_CONFIRM_OVERLAP_MM (0.0f) /* 兼容字段，保持 0 */
#define NAV_SCAN_MATCH_MAX_PROGRESS_BACKTRACK_MM (75.0f) /* 最大允许进度回退；mm */
/* 下面几项是历史兼容字段；当前强确认逻辑已关闭，保持 0 不要改。 */
#define NAV_SCAN_MATCH_STRONG_CORRECTION_YAW_DEG (0.0f) /* 兼容字段，保持 0；deg */
#define NAV_SCAN_MATCH_STRONG_CORRECTION_POSE_JUMP_MM (0.0f) /* 兼容字段，保持 0；mm */
#define NAV_SCAN_MATCH_STRONG_CONFIRM_TRAVEL_MM (0.0f) /* 兼容字段，保持 0；mm */
#define NAV_SCAN_MATCH_STRONG_CONFIRM_MAX_RMS_MM (0.0f) /* 兼容字段，保持 0；mm */
#define NAV_SCAN_MATCH_STRONG_CONFIRM_MIN_INLIER_COUNT (0u) /* 兼容字段，保持 0 */
#define NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_COUNT (3u) /* 物理特征最少支撑边数 */
#define NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_EFFECTIVE_WEIGHT (0.90f) /* 支撑边有效权重比例 */
#define NAV_SCAN_MATCH_MIN_SUPPORT_SIDE_SPAN_MM (8.0f) /* 支撑边最小跨度；mm */
/* 物理特征修正后必须仍贴合平滑路线，防止定位跳变把车辆带离赛道。 */
#define NAV_SCAN_MATCH_PATH_CROSS_TRACK_TOLERANCE_MM (8.0f) /* 普通修正后的路径横向误差；mm */
#define NAV_SCAN_MATCH_COMPLETE_FEATURE_PATH_CROSS_TRACK_TOLERANCE_MM (45.0f) /* 完整特征允许误差；mm */
#define NAV_SCAN_MATCH_MAX_PATH_CROSS_TRACK_AFTER_MM (120.0f) /* 修正后绝对横向误差上限；mm */
#define NAV_SCAN_MATCH_STRAIGHT_DIAGNOSTIC_INTERVAL_MM (100.0f) /* 直线诊断记录间隔；mm */
#define NAV_IMU_MAX_EXTRAPOLATION_US (5000u) /* IMU 快照允许外推时间；us */
#define NAV_IMU_HARD_STALE_US (10000u) /* 超过此时间禁用陀螺反馈；us */

/* ================= 发车、测试和末端停车 ================= */
#define PRESTART_BENCH_TEST_MODE (0u) /* 1=仅采集静止 EMEM，不驱动车轮 */
#define PRESTART_STRAIGHT_SPEED_TEST_MODE (0u) /* 1=台架真实 PWM 直线速度测试 */
#define PRESTART_STRAIGHT_SPEED_TEST_TARGET_CMPS (600.0f) /* 直线测试目标速度；cm/s */
#define PRESTART_STRAIGHT_SPEED_TEST_PROFILE (3u) /* 0=阶跃，1=斜坡，2=工作点，3=在线辨识 */
#define PRESTART_STRAIGHT_SPEED_TEST_BASE_CMPS (300.0f) /* PROFILE 2 基准速度；cm/s */
#define PRESTART_STRAIGHT_SPEED_TEST_STEP_HIGH_CMPS (450.0f) /* PROFILE 2 阶跃高速；cm/s */
#define PRESTART_STRAIGHT_SPEED_TEST_RAMP_TO_BASE_MS (250u) /* 到基准速度的斜坡时间；ms */
#define PRESTART_STRAIGHT_SPEED_TEST_BASE_SETTLE_MS (350u) /* 基准速度稳定时间；ms */
#define PRESTART_STRAIGHT_SPEED_TEST_HIGH_HOLD_MS (450u) /* 高速段保持时间；ms */
#define PRESTART_STRAIGHT_SPEED_TEST_LOW_HOLD_MS (450u) /* 低速段保持时间；ms */
#define PRESTART_STRAIGHT_SPEED_TEST_ID_BASE_CMPS (400.0f) /* PROFILE 3 辨识基准；cm/s */
#define PRESTART_STRAIGHT_SPEED_TEST_ID_BASE_SETTLE_MS (200u) /* 辨识基准稳定时间；ms */
#define PRESTART_STRAIGHT_SPEED_TEST_ID_SEGMENT_MS (100u) /* 每个伪随机片段时长；ms */
#define PRESTART_STRAIGHT_SPEED_TEST_ID_SEGMENT_COUNT (8u) /* 伪随机片段数量 */
#define PRESTART_STRAIGHT_SPEED_TEST_ID_POSTSTEP_MS (500u) /* 辨识结束后等待；ms */
#define PRESTART_STRAIGHT_SPEED_TEST_PRESTEP_MS (1000u) /* 阶跃前静止基线时间；ms */
#define PRESTART_STRAIGHT_SPEED_TEST_HOLD_MS (700u) /* PROFILE 1 目标保持时间；ms */
#define PRESTART_STRAIGHT_SPEED_TEST_POSTSTEP_MS (500u) /* 目标归零后的等待；ms */
#define PRESTART_STRAIGHT_SPEED_TEST_RAMP_ACCEL_MS (175u) /* PROFILE 1 加速时间；ms */
#define PRESTART_STRAIGHT_SPEED_TEST_RAMP_DECEL_MS (175u) /* PROFILE 1 减速时间；ms */
#define PRESTART_STRAIGHT_SPEED_TEST_STOP_DISTANCE_M (6.0f) /* 异常兜底最大行驶距离；m */
#define PRESTART_PATH_BRAKE_SPEED_THRESHOLD_CMPS (5.0f) /* 末端主动制动停止速度；cm/s */
#define PRESTART_PATH_BRAKE_STABLE_MS (20u) /* 低于停止速度需持续时间；ms */
#define PRESTART_PATH_BRAKE_TIMEOUT_MS (300u) /* 主动制动最长等待；ms */
#define PRESTART_RUNNING_SUCTION_PERCENT (80u) /* 正式运行负压占空比；% */
#define PRESTART_WARMUP_SUCTION_PERCENT (10u) /* 发车预热负压占空比；% */
#define PRESTART_WARMUP_MS (2000u) /* 低档负压预热时间；ms */
#define PRESTART_SUCTION_SETTLING_MS (500u) /* 运行档负压稳定时间；ms */
#define PRESTART_STOP_SUCTION_HOLD_MS (500u) /* 停车后继续吸附时间；ms */

#if PRESTART_STRAIGHT_SPEED_TEST_PROFILE > 3u
#error "PRESTART_STRAIGHT_SPEED_TEST_PROFILE must be 0, 1, 2, or 3"
#endif

#define ENCODER_TEST_MODE_ENABLE (0u) /* 1=抬起驱动轮后按地图做编码器诊断 */
#define ATTITUDE_STOP_LIMIT_DEG (50.0f) /* 相对发车姿态超限角度；deg */
#define ATTITUDE_STOP_ENABLE (1u) /* 1=横滚/俯仰超限立即请求停车 */

/* ================= Flash 恢复默认值 ================= */
#define FLASH_DEFAULT_SPEED_KP (13.010018f) /* 恢复默认时写入的速度环 KP */
#define FLASH_DEFAULT_SPEED_KI (0.300000f) /* 恢复默认时写入的速度环 KI */
#define FLASH_DEFAULT_SPEED_INCREMENT_LIMIT (8000.0f) /* 速度环单拍增量限幅 */

/* ================= IMU 校准/安装标定 =================
 * 这些值与板上安装方向有关；改动后应重新做原地转圈和零偏实验。 */
#define GYRO_NOMINAL_RATE_HZ (480u) /* IMU 名义输出频率；Hz，仅用于计数/兜底 */
#define GYRO_ZERO_CALIBRATION_MS (1000u) /* 静止零偏有效采样窗口；ms */
#define GYRO_STARTUP_STATIONARY_RATE_RADPS (0.5f * PI / 180.0f) /* 静止角速度门限；rad/s */
#define GYRO_SERVICE_FAILURE_LIMIT (5u) /* 连续读 IMU 失败次数后报故障 */
#define GYRO_ISR_PERIOD_US (2083u) /* IMU 中断周期监测参考；us */
#define GYRO_SCALE (1.0f) /* 陀螺乘性标定；正常应接近 1.0 */
#define VQF_YAW_RIGHT_POSITIVE (0u) /* 右转时 VQF yaw 是否为正；安装方向开关 */
#define VQF_YAW_MAP_SIGN (-1.0f) /* VQF yaw 映射到地图坐标的符号 */
#define VQF_YAW_RATE_MAP_SIGN (1.0f) /* VQF 横摆率映射到地图坐标的符号 */

/* ================= 诊断/后台任务开关 =================
 * 比赛默认关闭调试发送；队列预算只影响后台吞吐，不改变路线目标。 */
#define ASSISTANT_SCOPE_SEND_ENABLE (0u) /* 1=向上位机发送示波器通道 */
#define ASSISTANT_ESTOP_TRIGGER_VALUE (0.5f) /* 上位机急停通道触发阈值 */
#define IMU_DEBUG_PRINT_ENABLE (0u) /* 1=调试串口输出 IMU 姿态，比赛保持 0 */
#define PHOTOTUBE_SCAN_STAGING_CAPACITY (64u) /* CPU0 扫图暂存环容量，必须为 2 的幂 */
#define PHOTOTUBE_SCAN_INGRESS_BUDGET (4u) /* 每次后台最多接收的扫图帧数 */
#define PHOTOTUBE_SCAN_PROCESS_BUDGET (4u) /* 每次后台最多处理的扫图帧数 */
#define PHOTOTUBE_SCAN_EPOCH_ACK_TIMEOUT_US (3000u) /* CPU1 扫图 epoch 应答超时；us */
#define NAV_SCAN_MATCH_SERVICE_BUDGET_US (5000u) /* 定位服务单次 CPU 时间预算；us */
#define DUTY_10MS_BUDGET_US (10000u) /* 10ms 后台任务时间预算；us */
#define PHOTOTUBE_SCAN_POSE_HISTORY_CAPACITY (64u) /* 扫图位姿历史缓存容量 */

#endif /* CODE_COMPETITION_CONFIG_H_ */
