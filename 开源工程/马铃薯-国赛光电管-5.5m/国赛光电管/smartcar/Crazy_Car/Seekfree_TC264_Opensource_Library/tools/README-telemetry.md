# TC26x 遥测导出

运行日志使用 `0x50000` 字节的逻辑 EMEM 镜像，保存速度环、导航、视觉、事件和 IMU 姿态数据；IMU/VQF仍以480 Hz运行，EMEM只按2分频保存240 Hz日志。正常跑图期间，扫图点阵和拟合结果会作为旁路数据写入 EMEM 保留区，不保存原始图像。车端只有在 `CHIPID.EEA=1`、白名单 `CHID=0x68` 且 EMEM 边界允许时才启用日志；普通 TC264D 没有 EMEM，日志会安全禁用，不应影响控制。

## 按键说明

- `KEY_1/S1/P20_6` 是统一流程键：正常模式短按一次后自动完成预热/发车，运行中短按停车/冻结，随后短按导出；台架模式短按依次执行开始、停止/冻结、导出。
- `KEY_2/S2/P20_7` 在当前台架流程中不参与遥测导出；它也不会触发发车。

当前工程 `code/prestart.h` 的 `PRESTART_BENCH_TEST_MODE` 为 `0`，正常发车会保持
地图导航运行，同时旁路记录光电扫图；如果要做静止 EMEM/IMU/导航测试，可临时改为
`1`。当前工程为速度环测试配置：`PRESTART_STRAIGHT_SPEED_TEST_MODE=1`，会绕过
地图转向并按测试波形自动停车。当前 `PROFILE=3` 先对 `400 cm/s` 做硬阶跃并保持
`400 cm/s`，稳定 `200 ms` 后执行 `440/400/480/400/460/400/480/400 cm/s`
的 `100 ms` 短阶跃，专门用于拟合左右轮静态增益、时间常数和执行延迟；完整测试
约 `2.75 s`，并保留 `6 m` 里程兜底。`PROFILE=2` 保留 `300↔450 cm/s`
工作点响应测试；`PROFILE=1` 保留 `0→600 cm/s` 斜坡，主要用于辨识 `kA`。

## 导出步骤

1. 根据 `prestart.h` 当前模式操作：正常模式短按一次 `KEY_1/P20_6` 开始预热，约 2.5 秒后自动进入
   正常跑图，光电扫图会同步记录；运行中再短按一次停车/冻结；`PRESTART_BENCH_TEST_MODE=1` 时进入静止台架会话；
   `PRESTART_STRAIGHT_SPEED_TEST_MODE=1` 时才运行独立速度测试并自动停车。日志冻结后
   可再次短按 `KEY_1/P20_6` 请求导出；程序会等待 500 ms 收尾和 CRC 提交完成后自动发送，
   不需要额外确认键。
2. 将 CPU0 的 UART0 接到上位机，串口参数为 `921600 8N1`。
3. 在仓库根目录执行自动识别命令（不需要填写 COM 号）：

   ```powershell
   python -B -X utf8 tools/telemetry_capture.py capture --out output
   ```

   程序会扫描串口并等待车端的 `TLD1` 识别信标。若要指定端口，仍可使用
   `capture --port COM6 --out output`。
4. 看到 `TC264 telemetry detected` 后，确认车辆处于 idle。若屏幕状态仍为
   `Saving`，先等回到 `STOP`/idle，再短按物理按键 `KEY_1/P20_6` 请求导出；
   不要在 `Bench test`/`RUN` 状态连续按键。
   程序先发送 `TLG1` 前缀，再用协议 v2 的零游程块连续发送；上位机重建并校验完整 `0x50000` 字节逻辑镜像。导出期间 CPU0 暂停菜单、屏幕和扫图后台服务，命令行会显示按重建字节数计算的进度条。未使用记录仍在 EMEM 中保持为零，但不再逐字节占用串口；最坏情况下仍接近 921600 8N1 发送完整镜像的约3.56秒，正常稀疏日志会明显更快。

串口中断或断流时，原始 EMEM 内容不会被清除；车辆重新回到 idle 后再次按键即可重试。

## 解码已有镜像

```powershell
python -B -X utf8 tools/telemetry_capture.py decode output\telemetry.bin --out decoded
```

输出包括：

- `metadata.json`：运行状态、路线标识、电机逆模型和速度 PI 参数快照、`TRK1`版本2轮距/兼容前瞻字段/角速度反馈增益/限幅配置、单位和丢包计数；算法v17把固定布局中的旧KD槽写0、旧GKD槽解释为`K_omega`，旧target-rate-window槽解释为20ms曲率前馈提前量；
- `control.csv`：控制 ISR 每 5 个周期保留 1 条，约 200 Hz；`vision`线路诊断也将1kHz光电采集按5分频保存为200Hz，原始扫描命中与点云入口仍为1kHz。车端22字节控制记录保存编码器增量、左右轮目标速度/诊断加速度、最终PWM、控制器实际使用的`target_yaw_rate-measured_yaw_rate`和唯一转向输出。`plan_v`与`final_turn`由左右轮目标均值/半差恢复，实际速度由编码器比例恢复；当前电机前馈只由header中的kV恢复，固定布局中的kS/kA字段为0，PI贡献由最终PWM减kV前馈恢复。算法v16/v17恢复`target/measured_yaw_rate`、`yaw_rate_feedforward_turn_cmps`和`yaw_rate_feedback_turn_cmps`；v17另从导航记录恢复离线曲率前馈偏航率，v15及以前才恢复旧`P/D/GKD`分量。按时间关联`phototube.csv`恢复重复的线路诊断。所有近邻关联均输出`*_alignment_age_ms`，只有`yaw_rate_error_degps`是车端控制拍直接记录值；
- `navigation.csv`：schema 21 的18字节车端记录保存四分之一路径进度、校正后的地图位姿 `actual_*`、不可被定位改写的 `raw_*` 和`localization_update_seq`。上位机先按日志 `route_hash` 校验单路线`nav_path.h`，再按最近控制拍实际速度和v17的20ms曲率提前量复算`curvature_preview_progress_points`、`curvature_feedforward_yaw_rate_dps`、速度/加速度段及 Frenet 路径误差；`lookahead_*`、`pure_pursuit_*`字段只为旧日志诊断保留。`lookahead_control_alignment_age_ms`明确记录关联时间差。同拍raw/map位姿继续恢复`map_from_odom_*`及定位事务前后状态。上位机不兼容旧schema；
- `phototube.csv`：schema 21 每帧10字节；线路位置、活动位图和有效哨兵保持原值，阈值量化为7位、峰数4位、活动占比量化为4位、full-coverage保留1位，其余当前观测flags由这些量恢复；16位帧序号使用session header中的首帧32位基准展开，跨`0xFFFF`不会断裂。上位机还从活动位图计算首末通道、通道跨度、活动质心、连续块数及左右半区数量，供双峰/十字/边缘诊断；
- `events.csv`：发车、停车、溢出和其他状态事件；
- `localization_transactions.csv`：schema 21定位事务账本；车端用512条44字节环形记录保存最新的接受/拒绝尝试，容量满后覆盖最旧事务并累计`localization_overwrite_count`。记录带全程事务序号、物理槽、状态/拒绝原因、提交时间、证据帧/里程、更新序号、feature occurrence、匹配类型、`T_before/delta_odom/raw pose`、候选投影progress、RMS和内点数。上位机先按最旧槽恢复时间顺序，再分别重建候选变换与真实`T_after`；接受时真实after等于候选，拒绝时真实after严格等于before。上位机还用同一份`nav_path`反算before/candidate/actual的横向误差、纵向误差、航向误差、路径距离及变化量；旧日志的Pure Pursuit P项仅作为兼容诊断，不代表v17控制量。这些字段不扩张EMEM记录。普通直线使用自身证据水位后的最新120mm窗口，至少需要75mm沿线跨度和12个独立帧，通过几何、显著性及RMS门后立即提交；定位提交允许75mm以内的进度回退以修正编码器过度前进，超过该范围仍记录`progress_regression`；`straight_not_significant`表示候选低于4mm物理分辨率与3sigma统计门，没有写入定位状态；
- `localization_feature_diagnostics.csv`：每个CORNER/JUNCTION固定一行，不受事务数量影响；记录是否经过/接受、尝试次数、最后及全部拒绝原因、最后及全部详细失败门限、三条候选物理边最后一次的点数/独立帧数和失败实际值。line0/line1分别为路线入边/出边，line2为地图导出的真实额外支路；算法版本69把完整SE(2)和单边法向`partial`校准分开记录，partial不会伪装成完整成功；唯一关联点不足、唯一关联权重不足和非共线边归属歧义分别记录为`total_points`、`total_weight`和`ambiguous`，不再统一伪装成歧义；`disabled_by_map`表示路线掩码明确禁止该物理特征，`passed_without_candidate`表示车辆进度已越过该点但从未进入校准候选门，`not_reached`表示本趟尚未跑到；
- `imu.csv`：日志240 Hz、每条20字节；传感器和VQF仍运行于480 Hz。车端每2个姿态样本保留1条，保存相邻已记录样本之间的4us时间增量、原始acc/gyro和smallest-three 14位四元数；上位机累加恢复相对发车微秒时间及完整四元数，再计算模长和Roll/Pitch/Yaw；
- `phototube_scan.csv`：正常地图运行期间不可变的 raw odom 扫图归档。版本14使用5120条16字节环形记录保存最新点，容量满后逐点覆盖最旧EMEM记录并累计`archive_overwrite_point_count`；原始点本身仍只保存相对首帧号、相对采集时间、`x/y`、capture raw yaw、横向命中、按模65536保存的16位累计里程、置信度和flags。算法版本69复用flags区分`centerline_hit`和`boundary_hit`：连续活动带端点、宽带、多峰和全覆盖只用于宽度/传感器诊断；窄单峰在阵列边缘截断时可作为普通直线定位的35%低权重保守证据，但不能参与角点/路口完整特征匹配；junction的MULTIPLE/FULL_COVERAGE支路有效权重门为0.60，同时仍要求3点、8mm跨度、2个独立帧和完整特征yaw不超过8度；两类匹配使用独立证据水位，普通直线接受不会裁掉完整特征仍需使用的入边证据，完整或partial特征接受以及驶过特征边界才同时推进两类水位；角点/路口按地图导出的有限物理线段固定关联，点云历史保留完整1m行程，不再用顶点150mm圆截断提前入边；每条特征ray在地图坐标中限制为顶点向外300mm，并以75mm端点容差吸收当前位姿误差；使用25mm Huber降权和最终60%内点权重门；完整SE(2)不足时，连续主导单边可作为零yaw的partial法向校准并单独记账，partial也不与直线修正在同一拍叠加。上位机按头部保存的精确最旧物理槽恢复时间顺序，并对相邻记录的16位帧号/采集时间/里程模差做展开，因此保留窗口内跨过65.535m不会饱和；如果最旧保留点本身已跨多个65536mm周期，仅靠点记录仍无法恢复绝对高位。覆盖数量从独立drop summary恢复；再由raw点、raw yaw、安装前距和横向命中反推采集车轴`raw_axle_x/y`。若定位账本也已覆盖，第一条保留事务的`T_before`作为保留时间窗的变换基准，随后按接受事务恢复采集时刻及最终地图投影。在线匹配仍使用独立1280点DSRAM环；TLS/PCA、范围及末帧由重排后的点重算。CPU1源帧先进入CPU0 staging ring，点云归档和定位账本均由`telemetry_service()`后台分批写EMEM；staging回压、位姿等待和后台归档队列丢失在56字节版本14扫图头中分别计数。每轮发车/停车使用独立run-open/run-close epoch隔离跨边界帧；非当前epoch的过滤帧数和关闭epoch确认超时数由终止runtime ledger的`type=10/code=9`记录，不占用扫图头字段。
- `planned_route.csv`：从当前工程 `code/nav_path.h` 的单一车载数组下标0读取完整离线参考；除逐点`progress/x/y`外，还展开距离、航向、曲率/曲率导数/半径、速度、加速度、速度/加速度段号、地图接管区及物理特征编号。日志中的 `route_index` 是地图编辑器来源路线号，不是生成数组下标；即使车辆没有真正移动，也能显示整张规划地图；
- `telemetry.bin`：原始 `0x50000` 字节逻辑镜像（不是对物理 EMEM 总容量的声明）。
- `codex_packet.json`：给 Codex 二级下钻的结构化分析包，包含参数快照、数据完整性、各域统计、诊断标记和稀疏的关键字段时序索引；它不复制完整 CSV，定位异常时间窗后应回到对应 CSV 查询；
- `codex_prompt.md`：可直接连同 `codex_packet.json` 发给 Codex 的分析任务模板。
- `telemetry_dashboard.html`：离线可视化页面。地图系主视图固定显示物理feature map和nav_path、在线校正轨迹、所选T重投影的raw点云、事务前后位置标记及delta/RMS/内点/progress账本；raw odom诊断视图保持raw轨迹/点云不动，仅对物理地图和nav_path应用`inverse(T)`。页面同时保留速度、执行器、IMU、误差、视觉和事件视图。
- `analysis_summary.json`：给 Codex 首轮读取的紧凑摘要，只含运行有效性、参数、关键指标、诊断标记和按问题域下钻的文件名，不包含大段时序。
- `telemetry_overview.svg`：固定一页的赛场调车概览图。顶部先判断本趟数据是否有效，主体按 `A 路径与转向 → B 动力执行 → C 感知与安全` 分区；应渲染查看，不能把 SVG/XML 源码读进上下文。
- `telemetry_overview.png`：与 SVG 内容一致的直接预览图，是 Codex 默认读取的第一份产物，不需要浏览器或额外渲染步骤。

## 交给 Codex 做参数分析

一次导出完成后，优先把同一目录中的 `codex_prompt.md`、`analysis_summary.json` 和 `telemetry_overview.png` 交给 Codex。CSV 和 `codex_packet.json` 保留在目录中，只有首轮概览无法解释具体异常时才下钻，不要默认全部上传或整文件读取。发送：

```text
请先查看 telemetry_overview.png，再读取 analysis_summary.json，并按 codex_prompt.md 分析。不要读取 SVG/HTML 源码或整份 CSV；需要下钻时只查询相关列和时间窗。只给出下一轮调参建议，不修改代码；每次最多改 1～2 个主参数，并写明当前值、建议值、证据、风险、回滚条件和下一轮验收指标。
```

建议保留每一轮完整目录，例如 `runs/run_001`、`runs/run_002`。下一轮测试时保持路线、起步方式和速度档位一致，再把新旧两个目录一起给 Codex，让它比较 `navigation.progress_index` 对齐后的指标，而不是只比较单次峰值。

## 调参顺序

1. 先看 `telemetry_overview.png` 和 `analysis_summary.json` 的数据有效性、丢记录、保护事件和控制限幅事件；数据不完整时不要依据它改参数。
2. 速度问题先只调 `speed_KP`：看实际平均速度与 `plan_v_cmps` 的误差、PWM 和限幅事件。响应慢再小步增加 `speed_KP`；出现明显振荡或 PWM 高频变化则回退。
3. 稳态仍有同方向误差，再小步增加 `speed_KI`；出现累积冲过目标、松油后恢复慢或保护/限幅增加，先降低 `speed_KI`。
4. 如果主要问题是输出突变或电机响应过猛，优先检查 `speed_IncrementLimit`，不要用增大积分去掩盖限幅问题。
5. 路径问题先用 `localization_feature_diagnostics.csv` 确认每个路口是否完整校准及失败门限，再看 `cross_track_mm`、`heading_error_deg`、`raw_*` 与 `actual_*` 的差异、`map_from_odom_*`、`localization_update_seq`、`localization_transactions.csv`、`final_turn_cmps` 和控制限幅事件；一次只动一个路径相关参数，并让 Codex 结合源码确认该参数确实在当前固件生效。光电不生成控制目标，始终只有离线曲率目标偏航率、Frenet误差回正、差速前馈、陀螺角速度误差反馈和一个最终turn限幅；地图曲率不能在最终turn上再次叠加。
6. 当前 Flash 调参项只有 `speed_KP`、`speed_KI`、`speed_IncrementLimit`；地图速度来自 `nav_path.h`，不再从 `Speed`、`BaseSpd`、
   `RingSpd1/2` 或 `BrokenSpd` 读取。参数快照中的 `reserved_*` 只是为保持旧 Flash
   物理槽位而保留，不是调参项。

每次只改变一组小步长参数，记录“旧值 → 新值”，跑同一条路线并重新导出。验收时优先比较：横向误差 `p95_abs`、速度误差 `rms`、视觉 `accepted_ratio`、控制限幅事件数和保护事件数。

`events.csv` 的 `type` 定义为：`1` 发车、`2` 停车、`3` 发车状态、`4` 导航状态、`5` 视觉状态、`6` 缓冲区丢弃、`7` 位置饱和、`8` 控制限幅、`9` 保护触发、`10` 终止runtime ledger。`source` 中 `1/2/3/4/5/6` 分别表示 prestart/logger/navigation/control/vision/safety。点云定位不再编码进通用事件；接受/拒绝修正读取 `localization_transactions.csv`，逐特征聚合失败原因读取 `localization_feature_diagnostics.csv`。保护事件的 `code` 是 `STOP_REASON_*`。控制限幅事件的 `code` 和 `value1` 是当前位图：bit0=PID权限帽、bit1=横向加速度/转向变化率总限幅、bit2=左PWM软限幅、bit3=右PWM软限幅，`value0` 是前一个位图。runtime ledger的`code=1..6`依次记录控制执行、控制周期、光电扫描、10ms任务、地图匹配和EMEM服务的最大耗时/超限次数；`code=7`记录IMU一致快照读取失败，`code=8`记录IMU快照最大/末次年龄，`code=9`的`value0/value1`分别是run epoch过滤帧数和关闭确认超时数。

控制拍只在最近一致IMU快照年龄不超过5ms时使用其偏航率并最多外推航向5ms；连续10ms没有新的可用VQF发布会置位粘性`STOP_REASON_IMU_STALE=13`并停车。10ms后台任务先完成全黑、IMU、导航、电机、终点和姿态等保护裁决，再调用最多1280点的地图匹配器，因此最坏匹配耗时不会把本轮保护判断排到后面。

停车原因 `2`（`line_lost_after_200mm_black_scan`）表示从最后一次任意光电通道检测到低电平线路后，车辆又连续前进至少200mm且16路始终没有线路；它不再表示连续图像极值帧，16路全在线上也不会触发该原因。可用停车前的 `vision.active_mask_hex`、`white_percent` 和 `threshold_gray` 核对是否为真实出界或固定阈值造成的漏检。

事件区为发车/停车、停车状态、保护原因和最终丢包汇总预留空间。若视觉原因或控制限幅在短时间内反复切换，普通转换事件可能先被舍弃，但 `metadata.json` 的 `event_drop_count` 和 `events.csv` 的 buffer-drop 汇总会标明数量，终止与保护事件仍优先保留。

schema 21 的session header固定为256字节；header region在`0x100`保存64字节`LCF1`，从`0x140`保存`LFD1`头和116个32字节固定槽，并在紧随其后的`0xFD0`保存48字节`TRK1`版本2跟踪配置，正好在`0x1000`结束。其末4字节以两个`uint16`分别保存`path_sample_mm*100`和固定的算法附加槽；v17把后者解释为20ms曲率前馈提前量。主区容量不变：3296条22字节控制、1648条18字节导航、3296条10字节光电、256条16字节事件和3960条20字节IMU；IMU日志240 Hz可覆盖16.5秒。尾区保存最新512条44字节定位事务和最新5120条16字节点云；容量耗尽时覆盖最旧记录。解码器严格校验唯一布局、`TRK1`哈希和路线哈希，不提供旧schema兼容。

`firmware_config_fingerprint` 是 firmware/config fingerprint，不是编译日期或时间戳。为保证按键发车、屏幕刷新和串口服务不被阻塞，当前启动路径不再扫描整片 PFlash；镜像哈希分量固定为 `0`，指纹仍由遥测布局、常量快照、参数快照、路线 hash 和点云定位配置 hash 组合。旧视觉横向融合配置和兼容字段已经删除。普通无 EMEM 芯片在门控阶段直接退出，不会访问 `0xBF000000`。

控制/导航/光电/事件时间字段统一使用本轮发车起点的 `run-relative` 毫秒时钟；导航采样发生在控制 ISR 内且早于同拍控制记录，因此相同 `tick_ms`/`timestamp_ms` 的控制行给出了该导航姿态实际使用的规划速度。记录队列丢样只会让对应时间点缺行，不会改变其时间基准。`metadata.json` 中的 `start_uptime_ms` 和 `end_uptime_ms` 是系统上电后的时钟值，`duration_ms` 是本轮时长。

`navigation.actual_yaw_deg` 和 `raw_yaw_deg` 都是 `(-180, 180]` 范围内的包络航向，不是连续累计转角；`map_from_odom_yaw_unwrapped_deg`是上位机按相邻样本展开的诊断量。`heading_error_deg`表示实际车身yaw减路径切线yaw，`pure_pursuit_error_deg`才是当前唯一横向控制器使用的前视目标yaw减实际yaw，两列不能混用。`vision.reject_reason_name` 由当前候选选择状态导出：`accepted`、`selection_ambiguous`、`full_coverage` 或 `no_candidate`；应优先读取同一行的原始活动位图、观测标志和峰数。

IMU区与200Hz控制/视觉主区都按约16.48秒配置；超过容量后`metadata.json`的对应`drop_count`和诊断标记会明确提示，不能把截断后的波形当成完整运行。

对比不同运行时按 `navigation.csv` 的 `progress_index` 对齐路径位置；姿态与保护事件则按 `imu.csv` 的相对微秒时间对齐，不要按墙钟时间直接对齐。
