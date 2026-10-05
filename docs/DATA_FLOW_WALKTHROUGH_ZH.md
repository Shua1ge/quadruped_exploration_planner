# SCAN-Planner 数据流全程讲解:从传感器到 Go2 行为

> 基线:分支 `codex/observation-task-local-planner-20261003`(HEAD `8895627`),行号以该工作树为准。
> 本文按数据流动顺序组织:仿真输入 → C++ 感知与局部规划 → 执行侧 → Python 探索侧,
> 最后给一条"从看到 frontier 到轮子转动"的端到端时间线,以及两个常见现象
> (滚动重规划何时触发/被抑制、窄通道走走停停)的代码级映射。
> 辅助节点只列清单,细节后续单独成文。

---

## 0. 全链路总览(频率图)

```
Gazebo (gpu_lidar 10Hz, IMU 200Hz, odom 100Hz)
   └─ ros_gz_bridge (bridge.yaml)
        ├─ /quad_0/cloud (10Hz PointCloud2) ──┬─> grid_map.cpp cloudCallback ─> 20Hz raycast 融合 ─> LocalMapPatch (0.5s 节流)
        │                                      └─> frontier_explorer.py cloudCallback (0.5s 节流) ─> explorer/map
        ├─ /quad_0/body_pose (100Hz) ──> FSM safetyOdometryCallback / Controller odomCallback / Explorer odomCallback
        └─ /go2/imu, /tf, /go2/*_foot_contact (foot contact 无消费者)
Explorer(frontier_explorer.py, 0.25s 决策拍) ── /initial_path 或 /observation_task ──> SCANReplanFSM
SCANReplanFSM (exec_timer 100Hz + safety_timer 20Hz) ── /planning/bspline ──> ClosedLoopController
ClosedLoopController (cmdCallback 100Hz) ── /planning/cmd_vel_raw ──> Go2LocomotionSupervisor
Go2LocomotionSupervisor (PD 200Hz / ONNX 50Hz) ── /quad_0/cmd_vel (kinematic) 或 /joint_group_effort_controller/commands (rl_effort)
```

---

## 1. 仿真输入侧

### 1.1 Gazebo / 传感器源
- **gpu_lidar**:`src/simulator/Utils/go2_description/xacro/gazebo.xacro:69-97`,挂 `lidar_link`,topic `/quad_0/lidar/points`,**10 Hz**,720×1 条射线,量程 0.20–7.50 m。gz-transport 会在配置 topic 后追加 `/points`,因此真实云在 `/quad_0/lidar/points/points`(bridge.yaml:47-51 注释)。
- **IMU**:gazebo.xacro:60-67,200 Hz,topic `/go2/imu`。
- **里程计**:`OdometryPublisher` 系统以 100 Hz 发布 gz `/model/go2/odometry`,3 维(gazebo.xacro:47-52)。
- **足端接触**:gazebo.xacro:3-22 + 99-104,四个 100 Hz contact sensor,topic `/go2/{FL,FR,RL,RR}_foot_contact`。**注意:全栈没有任何规划/控制节点订阅它们**——当前是"桥接但无消费者"。
- **执行器两种模式**(`locomotion_mode` 参数,run.launch.py:359):
  - `planning_kinematic`:gazebo.xacro:17-56,base 标记 kinematic+无重力,加载 `gz::sim::systems::VelocityControl`,直接订阅 gz `/model/go2/cmd_vel` 做速度级驱动;腿被锁死,无 gz_ros2_control。
  - `rl_effort`:加载 `gz_ros2_control`(gazebo.xacro:43-47)+ joint_state_broadcaster / effort controllers(go2_sim.launch.py:248-302),关节级力矩执行。

### 1.2 bridge.yaml(`src/simulator/Utils/go2_description/config/bridge.yaml`,由 go2_sim.launch.py:183、321 装载)
| ROS 话题 | gz 源 | 类型/方向 | 频率 |
|---|---|---|---|
| `/clock` | `/clock` | Clock, GZ→ROS | 仿真步频 |
| `/go2/imu` | `/go2/imu` | sensor_msgs/Imu, GZ→ROS | 200 Hz |
| `/tf` | `/model/go2/pose` | TFMessage, GZ→ROS | — |
| `/go2/{FL,FR,RL,RR}_foot_contact` | 同名 | Contacts, GZ→ROS | 100 Hz |
| `/quad_0/body_pose` | `/model/go2/odometry` | nav_msgs/Odometry, GZ→ROS | 100 Hz |
| `/quad_0/cmd_vel` | `/model/go2/cmd_vel` | Twist, **ROS→GZ** | — |
| `/quad_0/cloud` | `/quad_0/lidar/points/points` | PointCloud2, GZ→ROS | 10 Hz |

### 1.3 go2_tf_bridge.py(`src/planner/plan_manage/scripts/go2_tf_bridge.py`)
纯 RViz 辅助:一次性广播 3 条静态 TF——`world→go2/odom`(零位)、`go2/base_footprint→base`(零位)、`base→go2/base/exploration_lidar`(+0.10, 0, +0.12,:29-37)。这个 (0.10, 0, 0.12) 正是 run.launch.py:82-89 传给 grid_map 的 lidar 外参,供点云从机体系转世界系。

---

## 2. C++ 感知与局部规划

### 2.1 grid_map.cpp 融合管线(`src/planner/plan_env/src/grid_map.cpp`,map 由 planner_manager.cpp:70-72 创建)

**输入订阅**(initMap,:431-451,均在 `map_callback_group`):
- `sensor_pose`(remap 到 `/quad_0/body_pose`)→ `sensorPoseCallback` :1336-1397:把位姿写入 `lidar_pose_history_`(0.5 s 窗口,:314),更新滑动地图中心。
- `cloud`(remap `/quad_0/cloud`,SensorDataQoS,10 Hz)→ `cloudCallback` :1471-1605。
- `body_pose` → `slidingMapFrameCallback` :1399-1427:低通跟踪机体高度(tau 0.75 s、限速 0.5 m/s),作为 LocalMapPatch 的参考 z。
- `/robot/locomotion_state`(transient_local)→ `locomotionStateCallback` :1457-1462。

**cloudCallback 的门与插值**:
1. `locomotionReady()` :1473 → :1464-1469:要求 0.5 s 内有 `perception_pose_valid=true` 的 LocomotionState,否则**丢帧**(防止趴着/起身阶段的扫描用错误的平面位姿把自由空间"封死",与 frontier_explorer.py:800-833 同一契约)。
2. 位姿对时 :1498-1519:`lookupTimedPose`(:20-71)按云戳在位姿历史里做 EXACT/INTERPOLATED/NEAREST 查找(插值缝隙上限 0.05 s、最近邻年龄 0.02 s,planner.yaml:141-142),失败即丢云。
3. 外参 :1523-1529(Gazebo 模式 (0.10, 0, 0.12)),`cloud_is_world=false` 时逐点转世界系,按 `local_update_range`(±5 m)与 `max_ray_length`(5 m)过滤。

**20 Hz 占据更新**:`occ_timer_` 50 ms(:453-455)→ `updateOccupancyCallback` :1110-1188(只有 `occ_need_update_` 才干活):
- `raycastProcess()` :850-1061:滑窗地图平移 :621-719;逐条 ray 做自由空间 log-odds 衰减 + 端点命中投票(hit_wins 规则 :1013-1016;p_hit 0.85 / p_miss 0.30 / clamp 0.12–0.98 / 阈值 0.80,planner.yaml:162-166);**已占据体素需要连续 3 帧显式 free 证据才允许被 miss 削弱**(`occupied_miss_confirmation_frames`,:1024-1033);机体自过滤双圆柱(radius 0.35、offset 0.18,:87-110)。
- `map_revision_++` :1132、`last_map_update_ns_` :1133(供 FSM 的 map-age 安全检查)。
- 构建不可变快照 `latest_inflated_snapshot_` :1135-1136(thread_local 绑定消费,:207-221)——FSM/A*/安全检查全部只读快照,不拿写锁。
- **LocalMapPatch**:`buildLocalMapPatchLocked` :1190-1270,按 `local_patch_period=0.5 s` 节流(:1201-1205)+ revision 去重,0.2 m 分辨率切片(-1 unknown / 0 free / 100 occupied),发布到 `grid_map/local_map_patch`(best_effort,:467-468)。这是全局表示节点的唯一输入。

**输出**:`grid_map/occupancy`、`grid_map/occupancy_inflate`(4 Hz 可视化 `visCallback` :456-458, :1089-1108)、`grid_map/sliding_map_bbox`、`grid_map/unknown`、`grid_map/depth_cloud`、`grid_map/sensor_pose_extrinsic`、TF `world→sliding_map`。

**规划器怎么查地图**:`InflatedOccupancySnapshot::getInflateOccupancy(pos, yaw, clearance_offsets)` :135-179——先查膨胀体素缓冲,再按给定的 2D clearance offset 模板在前/后两个圆柱心(±0.18 m 沿 yaw)逐点采样。膨胀本体来自 `updateInflation` :555-596(对每个占据体素做 0.35 m 半径、z −0.4/+0.1 的柱状膨胀,planner.yaml:119-133)。

### 2.2 scan_planner_node.cpp 入口(`src/planner/plan_manage/src/scan_planner_node.cpp:7-32`)
单节点 `scan_planner_node`,构造 `SCANReplanFSM` 并 `init`,`MultiThreadedExecutor(6 线程)`(:18-19)。注释点明:规划、里程计、控制反馈、地图、可视化分属 6 个互斥回调组,可视化不得饿死传感器融合。

### 2.3 SCANReplanFSM:话题接线(`scan_replan_fsm.cpp init()` :20-331)
- 订阅:`body_pose`→`safetyOdometryCallback`(:271-274,odom 组,100 Hz);`planning/execution_state`(:279-282);`planning/trajectory_ack`(:283-286);`planning/go2_heading_stalled`(:287-290);`planning/go2_heading_error`(:291-294);模式相关:`move_base_simple/goal`(navi_mode=1)、`initial_path`(navi_mode=3,:320-324)、**`observation_task`(task_mode,:309-312)**。
- 发布:`planning/bspline`(:296)、`planning/data_display`、`self_inflation`、`planning/blocked_segment`(:300)、`planning/execution_command`(reliable,:302-303)、`planning/failure_evidence`(:304-305)、`planning/status`(:306)、**`planning/observation_feedback`(:307-308)**。
- 定时器:**`exec_timer_` 10 ms = 100 Hz**(:258-260,planning 组);**`safety_timer_` 50 ms = 20 Hz**(:261-263,safety 组)。

### 2.4 里程计侧(safetyOdometryCallback :982-1117)
body 坐标 twist 被旋转到世界系;优先用 0.35 s 位姿历史差分得到的平面速度/偏航角速度(`estimatePlanarMotionFromPoseHistory`),再经 LPF(速度 tau 0.20 s、yaw rate tau 0.15 s)平滑。注释解释:步态周期内 Gazebo twist 会瞬时归零/反向,直接采样会让每次重规划从静止重启(:1058-1063、:2660-2665)。规划线程用 `refreshPlanningOdomFromSafety()` :1119-1128 取该状态。

### 2.5 六态 FSM 主循环 `execFSMCallback()`(:1768-2369,100 Hz)
状态枚举 `INIT→WAIT_TARGET→GEN_NEW_TRAJ→REPLAN_TRAJ⇄EXEC_TRAJ→EMERGENCY_STOP`(scan_replan_fsm.h:47-55)。每拍流程(switch 之前的前置段,新逻辑最密集处):

1. **ACK 超时事务**(:1778-1857):`pending_handoff_` 有效且 `trajectory_ack_timeout_`(0.75 s)内没收到匹配 ACK → 发布 `ExecutionCommand COMMAND_HOLD reason=CONTROLLER_ACK_UNKNOWN`;再等 `trajectory_ack_reconciliation_timeout_`(0.75 s)仍无版本化 ExecutionState → 丢弃候选事务、保留 HOLD(`CONTROLLER_ACK_UNRESOLVED`)、置 `safety_stop_active_`、`stopped_local_repair_pending_`、`predictive_replan_requested_`。
2. **交接期间 FSM 全冻结**(:1859-1869):`pending_handoff_.valid` 时直接 return——不推进轨迹时间、不触发新规划、不完成请求(odom 与安全回调继续跑)。
3. `activateDeferredReferencePathIfReady()`(:847-870):恢复事务终结后激活被 defer 的 Explorer 路径。
4. **静止修复入口**(:1906-1929):`shouldStartStoppedLocalRepair`(replan_fsm_utils.h:374-381:`stopped_local_repair_pending_ && have_target_ && 速度≤0.05 && !pose_occupied && !temporal_fault`)→ `GEN_NEW_TRAJ "LOCAL_REPAIR_FROM_REST"`。
5. **安全闩锁门**(:1935-1943):`safety_stop_active_ && !safetyLatchAllowsPlanning(...)`(utils.h:365-371:仅 replacement-planning/emergency-pending/stopped-repair-planning 三种情况放行)→ 打印 `SAFETY_LATCHED` 并 return。
6. **结构化修复完成→rejoin**(:1949-1969):versioned FINISHED/STALLED 反馈 + `structured_local_repair_finished_` → `REPLAN_TRAJ "STRUCTURED_REPAIR_FINISHED"`(去 rejoin 原参考路径)。
7. **终端无进度**(:1977-1996):控制器 `TERMINAL_NO_PROGRESS` → `REPLAN_TRAJ "TERMINAL_NO_PROGRESS"`,保留全局承诺。
8. **预测性碰撞重规划**(:2002-2020):`ordinaryRollingReplanAllowed(structured_executing, predictive_replan_requested_)`(utils.h:565-568,**结构化修复执行期间禁止普通滚动重规划打断**)且到重试时刻 → `REPLAN_TRAJ "PREDICTIVE_COLLISION"`。
9. **航向冻结恢复**(:2022-2054):`go2_heading_stalled_` 且未处理 → `REPLAN_TRAJ "HEADING_FREEZE_STALLED"`,预算 `heading_freeze_max_recoveries_=2`,用尽 → `BLOCKED` 回 Explorer。

**switch 内各态**:
- WAIT_TARGET :2084-2105:每 0.5 s 重发 `WAIT_TARGET request_id=...`(Explorer 容错)。
- GEN_NEW_TRAJ :2108-2151:`stopped_local_repair_active_` 时走 `tryStructuredLocalRepair()`,否则 `callReboundReplan(true, 首次=false/重试=true 随机初始化)`;失败 `replan_fail_count_++`。**提交≠切换状态:ACK 回调才把 FSM 切到 EXEC_TRAJ**(:1504)。
- REPLAN_TRAJ :2153-2199:先查到点(goal_tolerance 0.25);`planFromCurrentTraj()`(:2459-2733,见 2.9);失败时 `shouldKeepExecutingAfterRollingReplanFailure`(utils.h:884-892:参考路径有效、无挂起更新、未冻结、剩余时间 > emergency_time 1.0 s)→ **继续执行旧轨迹,0.25 s 后重试**(`ROLLING_RETRY`,:2180-2188),否则 `requestExecutionStop("REPLANNING")`。
- EXEC_TRAJ :2201-2324:到点检查;spline 走完但未达全局终点 → 立即续规划 `LOCAL_SEGMENT_DONE`(:2249-2273);`next_rolling_replan_attempt_ns_` 退避窗口(:2297-2300);终点 0.1 m 内不再重规划;**滚动触发条件 `shouldTriggerRollingReplan(planned_progress, odometry_progress, replan_thresh_=1.0)`**(utils.h:1004-1012:规划进度与里程计进度**都** ≥ 1.0 m)→ `[ROLLING_REPLAN_TRIGGER]` → REPLAN_TRAJ。
- EMERGENCY_STOP :2326-2362:一次性 `callEmergencyStop`;速度 < 0.1 后按 `need_hover_stop_`/`emergency_path_pending_` 回 GEN_NEW_TRAJ 或 WAIT_TARGET。

**`finishProcess()`**(:2371-2457):`replan_fail_count_ ≥ 5` 时,若本地修复拥有失败 → 发布 `FailureEvidence(SCOPE_VIEWPOINT)` + `VIEWPOINT_LOCAL_REJECTED`(保留 HOLD,只把失败视点还给 Explorer);否则 EMERGENCY_STOP + `BLOCKED`/`TRACKING_RECOVERY_FAILED`。

### 2.6 安全实时检查 `checkCollisionCallback → checkCollisionRealtimeCallback`(:4085-4099 → :3304-4083,20 Hz)
逐项:
- 时间性故障:`map_age > 0.5 s` 或安全回调间隔 > 0.3 s → `LOCAL_PERCEPTION_STALE` HOLD(:3364-3384)。
- **物理位姿占据**:`getInflateOccupancy(odom_pos, actual_yaw)` ≠0 → `POSE_OCCUPIED`(:3395-3405),3 个连续 free 周期才释放(:3407-3426)。
- **制动扫掠**(速度>0.03 m/s,:3531-3628):`brakingSweepHorizon(speed, 0.32 m/s², 0.30 s 停令延迟, 上限 4.0 s)`;按实测速度方向+实测 yaw rate 前推预测足印并查 clearance 带;硬碰撞 → `PREDICTED_BRAKING_HOLD`(释放条件:静止 3 周期**且**当前位置 clearance 带自由,utils.h:392-401;否则转入 stopped repair,:3476-3495)。
- **轨迹余项 0.02 s 采样扫描**(:3883-4006):每点查 `planning_clearance_margin_`(0.10 m)带;命中且 `predictiveCollisionRequiresHardStop`(时间到命中 < 硬停时间 `predictiveHardStopTime(speed, 0.32, 0.30, min 0.35)`)→ `tripRealtimeSafety("BLOCKED")`(:3192-3263:置 safety_stop、BLOCKED 情形进入 stopped repair 并 HOLD 而不是终止请求);否则 `predictive_replan_requested_=true`(PREDICTIVE_COLLISION_WARNING)。同时把 [进入,退出,rejoin] 区间存为 `PredictedCollisionInterval` + `collision_interval_replan_requested_`(:4008-4069)。
- **preferred clearance 触发**(:4071-4082):MODE_NORMAL 轨迹余项进入 0.20 m preferred 带 → `predictive_replan_requested_=true`(`[PREFERRED_CLEARANCE_REPLAN]`,"requesting a wider or speed-limited replacement")——**这就是 clearance-limited 慢速模式的入口之一**。
- **跟踪偏差分类 `classifyTrackingDeviation`**(utils.h:578-620):连接器(里程计→轨迹最近匹配点,匹配窗 back 0.40 s/forward 0.80 s)≤0.10 m → TRACKING;恢复走廊被 clearance 带阻断或误差 > 0.30 m → LOCAL_REPAIR;普通轨迹中度偏差 → ROLLING_REPLAN;**MODE_CLEARANCE_LIMITED 轨迹:宽限期 1.0 s / 无进度超时 2.0 s / 最小进度 0.05 m** 内 KEEP_EXECUTING("clearance_limited_execution_owned"),超时且位移不足 → **`CLEARANCE_LIMITED_NO_PROGRESS`** → `stopped_local_repair_pending_` + HOLD(:3747-3771)。
- clearance escape 完成判定(:3810-3881):实际位姿出带 + violations 改善 + 位移/进度达标 + 3 个 free 周期。

### 2.7 `callReboundReplan()` 完整步骤链(:4100-4945)
1. 记录 `executing_trajectory`(提交前保留旧轨迹,失败即原子回滚,:4138-4147);取 `map_snapshot`(:4148-4151)。
2. 目标选择:override 或 `getLocalTarget()`(reference 模式走 `getReferencePathLocalTarget`,见 2.8;task_mode 走 `selectObservationTaskTarget()`)。
3. **两段式余量规划**(:4240-4250):先以 `preferred_clearance_margin_=0.20` 调 `planner_manager_->reboundReplan(...)`;失败则回退 `planning_clearance_margin_=0.10` 重试(记 `[PREFERRED_CLEARANCE_FALLBACK]`)。`reboundReplan`(planner_manager.cpp:87-410):多项式/随机初始化或旧轨迹后缀重采样 → `initControlPoints`(内部 **A\*** 修正穿障段)→ `BsplineOptimizeTrajRebound`(梯度优化)→ 可行性检查/时间重分配。
4. `safety_generation_` 与 `active_reference_request_id_` 复核(:4286-4303)——安全停或换请求即 `PLAN_RESULT_DISCARDED`。
5. seed(碰撞区间修补)情形:校验与旧轨迹的 p/v/a 接缝连续(:4306-4371)。
6. **0.02 s 全样最终复核**(:4376-4452):`footprintOccupiedWithMargin`(:3265-3290,0.10 m clearance 模板)逐样查;起始在带内允许"不恶化逃逸前缀"(`clearanceViolationAllowedDuringEscape`,utils.h:403-413,deadline 0.75 s);失败 → `FailureEvidence(FINAL_CLEARANCE)` + 拒绝。
7. **clearance-limited 降速重定时**(:4462-4487):候选需要受限剖面时,`scale = max(1, 速度控制点峰值 / min_forward_effective_speed_=0.165)`,对 knots 做时间缩放——**把样条参考速度压到实测步态 0.165 m/s 水平**;再做转向扫掠(±max_yaw_rate 0.50 与 0,:4522-4553)与 `min_forward_command_=0.30` 前向制动距离扫掠(:4554-4579),任何不可采纳违例即拒绝(`FORWARD_GAIT_STEERING_REJECTED` / `FORWARD_GAIT_SWEEP_REJECTED`)。
8. **交接扫掠**(:4586-4743):用实时 odom 匹配候选最近点;`speedAwareHandoffErrorLimit`(utils.h:901-918,0.30 m 静止→0.10 m 运动)超限 → `HANDOFF_PREDICTED_UNSAFE` 停车(:4632-4646);随后按"先原地转(5° 步进)、再走连接器(0.05 m 步进)"模拟 Controller 的执行顺序做 clearance 扫掠(:4717-4742)。
9. 发布事务(:4809-4913):填 `Bspline`(order 3、knots、pos_pts、request_id/task_id/version/candidate_id、**execution_mode**:MODE_RECOVERY_PRIMITIVE / MODE_CLEARANCE_LIMITED / MODE_NORMAL,及 max_forward/lateral/yaw/tracking 与 min_forward_command/effective_speed 字段)→ 写 `pending_handoff_`(:4869-4912)→ `bspline_pub_->publish` → `[CANDIDATE_TRAJECTORY_SUBMITTED]` + `PLAN_TIMING`(毫秒级耗时)。
10. ACK 路径:`trajectoryAckCallback`(:1265-1511)——ACCEPTED 才把候选装入 `local_data_`/`execution_snapshot_`、清 pending、按 segment/rejoin 分支推进/终结恢复事务、可能 `PATH_TRAJECTORY_READY`、切换 EXEC_TRAJ;REJECTED 回滚旧轨迹并请求重规划;LATE ACK/超时对账(:1130-1195, :1307-1318)。
11. `callEmergencyStop`(:4947-4986):6 个重合控制点的静止样条(时长 1.0 s)**同步发布**,不等 ACK。

### 2.8 `getReferencePathLocalTarget()`(当前版本,:5099-5181)
- task_mode → `selectObservationTaskTarget()`。
- 否则 `projectReferencePathLookahead(reference_path_, start_pt_, progress_idx, ratio, lookahead=max(0.5, fsm.reference_path_lookahead=4.5))`:沿 Explorer 折线做**弧长前视**取目标点+切向(utils.h:273-358),同步维护进度游标。
- 目标被占据:**只在原折线上缩短前视**(0.2 m 步进降到 0.4 m,:5136-5156)——绝不横向偏出全局走廊;全被堵则目标=当前点、切向置零。
- 终端速度:`remaining_after_target ≤ max_vel²/(2·max_acc)`(0.75²/(2·0.5)=0.56 m)或切向为零 → 目标速度置零,否则 `tangent * 0.75`(:5174-5180)。
- 非参考路径模式(RViz goal/预设点)用 `getLocalTarget()`(:4988-5097):沿全局样条按 planning_horizon 7.5 m 截取局部目标并做占据点前/后调整。

### 2.9 `planFromCurrentTraj()`(:2459-2733)滚动重规划的主体
- 若有 `collision_interval_replan_requested_` 且区间可用(`localPatchSeedIsUsable`:地图 revision 未变、时间窗有效)→ 用旧轨迹 + 直线补丁拼 seed,`callReboundReplan(seed_path=..., seed_interval=...)` 做**局部区间修补**(`[LOCAL_INTERVAL_REPAIR_SUBMITTED]`)。
- 否则常规路径:`shouldReuseCurrentTrajectorySuffix`(tracking_error ≤ 0.30 且无挂起换路)→ `callReboundReplan(false,false)`(旧后缀初始化);失败退 `callReboundReplan(true,false)`;再失败退随机初始化;最后在 predictive/tracking/stop 任何激活态下 `tryStructuredLocalRepair()`。
- 结构化修复 `tryStructuredLocalRepair()`(:2735-3021):要求速度≤0.05(否则挂 pending 等停,:2782-2810);沿参考路径切向的一组方向(turn-then-drive,`enable_holonomic_lateral_repair=false` 默认禁横移,:2829-2838)生成 0.45–0.90 m 锚点(步进 0.15、回退上限 0.60、候选 ≤6,:2845-2964),每个锚点须通过"旋转+平移+rejoin 连接器"三段 clearance 扫掠且违例不恶化;`callReboundReplan(local_target_override=anchor, low_speed_local_repair=true, max_speed 0.30)`。事务级预算 `local_repair_max_segments_per_transaction_=3`(:2765-2780),防止"每接受 0.45 m 短段就重开一个候选家族"的无穷左右爬行。

---

## 3. 执行侧

### 3.1 closed_loop_controller.cpp(`src/planner/plan_manage/src/closed_loop_controller.cpp`)
订阅 `planning/bspline`(:70-72)、`body_pose`(:73-75)、`planning/execution_command`(:85-87)、`/robot/locomotion_state`(:88-90)、`simulation/collision`(:91-93);发布 `cmd_vel`(remap→`/planning/cmd_vel_raw`,run.launch.py:169)、`planning/execution_state`、`planning/trajectory_ack`、`planning/go2_heading_error`、`planning/go2_heading_stalled`、`planning/local_execution_event`。**`cmd_timer_` 10 ms = 100 Hz**(:94-95)。

**bsplineCallback(:210-473,接收→ACK 事务)**验证链:task retirement → 样条结构 → 版本新旧(`shouldAcceptTrajectoryVersion`)→ 执行剖面合法性(min/max speed 组合,:246-268)→ 非有限值;静止样条(所有控制点间距 <1e-4)→ 记为版本并 `soft_hold_ "STATIONARY_HOLD_TRAJECTORY"`(:311-336);否则在 1.0 s 内 0.02 s 采样与 odom 做最近**前向**匹配(:345-364),`matched_error > speedAwareHandoffErrorLimit(0.50, 0.25, 0.10→0.30)` → ACK REJECTED `HANDOFF_POSITION_ERROR`(保当前执行版本);yaw 突变**不拒绝**——接受并冻结平移直到对齐(:386-416)。安装后 ACK ACCEPTED(含 matched_time/position_error/yaw_error),发布 STATE_RUNNING。

**executionCommandCallback(:475-550)**:COMMAND_HOLD(level-triggered)→ `soft_hold_=true` + STATE_SOFT_HOLD;RESUME;RETIRE_TASK(任务栅栏,之后任何该 task 的旧样条/命令全部拒收)。

**cmdCallback(:617-851,100 Hz 每拍做什么)**:
1. `locomotionReady()` 不满足(0.5 s 内无 actuation_ready 的 LocomotionState)→ `publishStop()`(:620-625)。
2. `simulation_collision_latched_` → STATE_HARD_STOP + `publishStop`(:626-634)。
3. `soft_hold_` → STATE_SOFT_HOLD + `publishStop`(:636-644)。
4. 无轨迹/无 odom → STATE_IDLE + `publishStop`(:646-652)。
5. **spatial gait 进度推进**(`active_min_forward_command_>0` 即受限/恢复模式,:664-695):不用墙钟,而是在 `[exec_time_, +2.0 s]` 内以 0.05 s 步找离 odom 最近的样条点作为实际进度,`forwardGaitLookaheadTime` 取前视点;`spatial_target_time ≥ duration && endpoint_error < finish_dist 0.15` → STATE_FINISHED + `publishStop`。
6. **航向**:期望 yaw = 指向前视点方向(spatial)或 `estimateDesiredYaw`(time_forward 0.8 s);发 `planning/go2_heading_error`;**`|yaw_error| > heading_error_threshold_ 0.8 rad` → HEADING_FREEZE**:只发 `publishStop(yaw_command)`(原地转,:747-753),STATE_HEADING_FROZEN;3.0 s 无改进 → `publishHeadingStalled(true)`(:737-746)→ 触发 SCAN 的 HEADING_FREEZE_RECOVERY;SCAN 侧 2 次预算用尽 → BLOCKED。
7. **正常跟踪**:`vel_world = vel_des + kp_pos(0.8)·p_error`(限 `max_tracking_correction`),转机体系 clamp(vx≤0.75, vy≤0.35, vyaw≤1.0);**`calibratedForwardGaitCommand`(utils.h:621-630)**:期望有效速度按 `cmd = v_eff · 0.30 / 0.165` 折算并夹在 [0.30, max_vx]——把"有效 0.165 m/s"映射回"命令 ≥0.30";**spatial gait 下直接 `command.linear.x = 0.30, linear.y = 0`**(:780-786,前向专用步态、无倒退/横移原语)。
8. **终端收敛**:`exec_time_ ≥ duration` 且误差 ≥0.15 → 超过 `max(1.0, time_forward_)` 无改进 → STATE_STALLED `TERMINAL_NO_PROGRESS` + `LOCAL_TRAJECTORY_STALLED` 事件,且**此后命令置零 Twist**(:845-846,防止 RL 低速死区里的反向/横移指令)。
9. 发布 `cmd_vel`(:850)。

`publishStop()`(:147-153):零 Twist(可带 clamp 后的 yaw)。

### 3.2 go2_locomotion_supervisor.py(`scripts/go2_locomotion_supervisor.py`,Go2 运动唯一属主)
- 订阅:`/joint_states`(:187)、**`/planning/cmd_vel_raw`**(:189,这是 Controller 输出的去处)、`body_pose`(:190-191)、`/go2/imu`(rl_effort,:215)。发布:`/robot/locomotion_state`(transient_local,:183-184);kinematic 模式发 `/quad_0/cmd_vel`(:197),rl_effort 发 `/joint_group_effort_controller/commands` + `/robot/locomotion_diagnostics`(:216-219)。服务 `/robot/enter_navigation`(:185)。
- **双循环**:`control_dt=0.002` → **PD 200 Hz**;`policy_dt=0.02` → **ONNX 50 Hz**(decimation 10,:49-56)。ONNX 推理在独立线程(policy_worker :455-477),只保留最新请求,200 Hz 回调永不阻塞。
- **kinematic 模式**(control_loop :575-604):每 2 ms 发布默认站姿关节目标(纯装饰,legs locked)+ 把最近 0.5 s 内收到的 raw_cmd 原样转成 Twist 发 `/quad_0/cmd_vel`(超时发零);位姿无效 40 周期 → STATE_FAULT。**这是"cmd_vel → 车动"的直接一跳**。
- **rl_effort 模式**:未 arm 前被动;`/robot/enter_navigation` 后零命令 ONNX get-up,姿态/高度/倾角/关节速度 25 个策略周期(0.5 s)连续稳定 → `STAND_VERIFIED` → STATE_RUN(:634-682);RUN 态 50 Hz:`filtered_cmd = advance_limited_command`(斜率 [0.50, 0.40, 1.00]/s,:700-705)→ `evaluate_policy`(45 维 obs + 450 维历史)→ 目标关节 `DEFAULT_DOF_POS + 0.25·action` → **200 Hz PD(kp 40/kd 1.0)→ 12 关节 effort**(:707-711)。
- **能力契约**:`capabilities()`(:310-332)从同一处派生 `actuation_ready`(可动)与 `perception_pose_valid`(可融合地图)两个布尔,随 LocomotionState 发布——grid_map 与 Explorer 的融合门、Controller 的 `locomotionReady` 全部读它。

### 3.3 planar_cmd_vel_adapter.cpp(可选,wrench_actuator=true 时启动,run.launch.py:274-301)
订阅 `/quad_0/cmd_vel` + `body_pose`,**100 Hz timer**(:44-46)做 PD 力控:目标速度转世界系,`F = 150·Δv − 30·v`(clamp 200 N)、`τz = 20·Δω − 5·ω`(clamp 30 N·m),发 gz `/world/{world}/wrench/persistent` ApplyLinkWrench;0.5 s 无 cmd_vel 清零(:92-97)。注释明确:力控基座在此机器人上没有稳定工作点(易打滑/翻倒),所以默认走 VelocityControl,此节点是备选实验通道。

---

## 4. Python 探索侧

### 4.1 frontier_explorer.py 一个 0.25 s 决策拍的完整时序(3953 行)
**接线**:`planning_period=0.25` 的 `exploration_timer`(:722-724,planning 组)+ 2.0 s `metrics_timer`(:725-727);回调组三组:sensor(cloud/body_pose)、locomotion(LocomotionState 独立,防被长规划饿死)、planning。发布 `initial_path`(:705,非 task 模式)或 `observation_task`(:706);大量 `explorer/*` 可视化与 `map_updates` 增量。

**传感器回调**:
- `odom_callback`(:1051-1102):更新 `position/body_z/body_yaw`、`pose_history`(200 帧时间戳队列),随后 `drain_pending_clouds()`。
- `cloud_callback`(:1180-1279):frame 校验 → `map_fusion_allowed()`(perception_pose_valid,:800-817)→ 位姿对时(同 grid_map 参数)→ **0.5 s 节流**(`map_update_period`,:1211-1212)→ 720 bin 平面射线分类(z 带 0.08–0.85 m,命中膨胀 3 bin)→ 锁内 `integrate_ranges` 更新 `_live_grid`,`map_content_revision++` → `publish_maps()`(explorer/map、explorer/planning_map、map_updates)。

**exploration_timer(每 0.25 s)**,`exploration_timer()` :2327-2411:
1. `locomotion_ready()` 不满足 → `PAUSED_ROBOT_UNHEALTHY`(:2331-2333)。
2. **等待 SCAN 的请求在飞**:`pending_path_publish_ns` 未清且未超 `path_request_timeout=5.0 s` → `PATH_REQUEST_IN_FLIGHT` return;超时 → 自我判定 `REFERENCE_PATH_REJECTED reason=TIMEOUT`(:2338-2364)。
3. 有活动目标:`validate_and_repair_active_path()`(受阻边/拓扑约束/`planning/blocked_segment` 回调记录的临时封锁);goal_timeout 120 s 拉黑;观察任务进度评估(0.60 预备 / 0.80 完成),达标则 `handoff_satisfied_observation`,否则预选下一观察点。
4. 无目标 → **`plan_from_current_position()`**(:2413-2507):
   - `ReplanGate`(explorer_core/replan_control.py:20-33:同一上下文=机器人格+`map_content_revision`+`route_constraint_revision`+重试纪元 `map_update_count//10`,最多 2 次)不允许 → `WAITING_FOR_PLANNING_INPUT_CHANGE`。
   - 膨胀障碍(0.65 m)+ 起点逃逸走廊 → `frontier_cells` → `cluster_frontiers`(≥6 格)→ `choose_frontier`(:2509+,分层策略:区域承诺门 `ResidualCommitmentGate`、稀疏路由图快速估价、密集 A* 终验、终端 clearance/观察单元集、黑名单/冷却/排除目标)。
   - `simplify_known_path` + 逐段 `segment_known_free` 终验(:2495-2505)→ `activate_candidate → publish_path → publish_reference_path`(:3476-3508):**request_id 取 header.stamp = 单调纳秒**,发 `/initial_path`,状态 `PATH_PUBLISHED`,进入"等待 SCAN"态。

**planning_status_callback**(:1958-2233)解析 SCAN 的 `planning/status` 字符串:`PATH_ACCEPTED request_id=` / `PATH_TRAJECTORY_READY`(轨迹就绪)/ `REACHED` / `BLOCKED` / `VIEWPOINT_LOCAL_REJECTED` / `REFERENCE_PATH_REJECTED ...` / `WAIT_TARGET` 等——终结态清 `pending_path_publish_ns` 并按需 reroute。`local_execution_event_callback`(:1893+)解析 Controller 的 `LOCAL_TRAJECTORY_FINISHED/STALLED` 驱动段间交接。`planning/blocked_segment`(:1153-1178)把 SCAN 安全停时发的阻断段转成 30 s TTL 的临时有向封锁边。

### 4.2 global_representation_node.py 的 0.5 s patch 处理
无定时器,**完全由 0.5 s 一帧的 `grid_map/local_map_patch`(best_effort)驱动**。`patch_callback`(:247-529):revision 去重 → clearance 场 → `extract_topology`(骨架细化→节点/边)→ 显式封锁确认(连续 3 个 patch,`blocked_confirmation_patches`)→ 每 10 个图 revision 做一次 safe-region 提取+持久身份匹配 → 发布 `global_representation/topology_delta`(每帧)与 `topology_snapshot`(每 10 帧,transient_local)+ `safe_region_snapshot`(JSON)。同时订阅 `planning/failure_evidence`(:111-113):SCOPE_VIEWPOINT 的证据固化为图级 `viewpoint_constraints` 并转发到 `global_representation/failure_evidence`(:183-245)→ Explorer 立即冷却该视点(frontier_explorer.py:745-778、:861-891)。

### 4.3 global_astar_planner.py(仅 benchmark)
事件驱动、无定时器:一次性从 `/map_generator/global_cloud`(transient_local)建 2D 栅格(0.20 m、膨胀 0.65、z 带 0.08–0.85),`goal`(`/move_base_simple/goal`)触发 A* → 0.5 m 重采样 → `/initial_path`;状态发 `global_planning/status`。只在 `quad_benchmark.launch.py:45-60` 里启动(RViz 点目标的对照管线),探索运行用的是 frontier_explorer 而非它。

---

## 5. 辅助节点清单(细节后续单独成文)

| 节点 | 文件 | 用途 |
|---|---|---|
| odom_visualization | `src/simulator/Utils/odom_visualization/src/odom_visualization.cpp` | body_pose → RViz 位姿/速度/轨迹/机器人 mesh marker + 可选 TF |
| map_generator (map_pub) | `src/simulator/map_generator/src/map_publisher.cpp` | PCD 全局云 → `/map_generator/global_cloud`(0.2 Hz),供 local_sensing 采样 |
| mockamap | `src/simulator/mockamap/src/mockamap.cpp` | 程序化地图 → 同一全局云话题 |
| local_sensing_node | `src/simulator/local_sensing/src/pointcloud_render_node.cpp` / `opengl_render_node.cpp` | 从全局云+body_pose 做 FOV 仿真 → `/quad_0/cloud`、`/quad_0/depth`(非 Gazebo 物理路径的传感器源) |
| waypoint_generator | `src/simulator/Utils/waypoint_generator/src/waypoint_generator.cpp` | RViz goal → `waypoints` Path(配合 navi_mode) |
| pose_utils | `src/simulator/Utils/pose_utils/` | SE(3)/欧拉工具库(odom_visualization 用) |
| sdf_map_publisher | `scripts/sdf_map_publisher.py` | SDF 碰撞几何采样为全局云(planar/3D 模式),transient_local |
| sdf_lidar_simulator | `scripts/sdf_lidar_simulator.py` | 直接在 SDF 体素结构上射线仿真本地 LiDAR + `simulation/collision` 碰撞布尔(轻量无 Gazebo 模式) |
| keypoint_recorder | `scripts/keypoint_recorder.py` | 交互录 odom 关键点 → keypoints.yaml(navi_mode=2 预设航点) |
| go2_gait_publisher | `src/go2_gait_publisher.cpp` | 由 odom 速度驱动 60 Hz 关节**反馈**动画(非 Gazebo 路径;默认禁用) |
| go2_kinematic_sim | `src/go2_kinematic_sim.cpp` | 最简运动学仿真:`/quad_0/cmd_vel` 积分 → `/quad_0/body_pose`(100 Hz,非 Gazebo、非真机路径) |
| open_loop_controller | `src/open_loop_controller.cpp` | 旧开环样条回放控制器(对照/调试) |
| go2_policy_node | `scripts/go2_policy_node.py` | 独立 TorchScript/HIM 策略(config policy_key go2/himloco)→ joint_effort_command;独立于 supervisor 的旧方案 |
| go2_onnx_policy_node | `scripts/go2_onnx_policy_node.py` | 独立 ONNX(parkour_moe_full_model.onnx)50 Hz 控制环 → 关节命令;同样是被 supervisor 取代的旧方案 |
| **go2_locomotion_supervisor** | `scripts/go2_locomotion_supervisor.py` | **当前 run.launch.py 实际启动的唯一位移属主**:内嵌同一个 ONNX(rl_effort)或 VelocityControl 代理(planning_kinematic),外加 get-up/状态机/LocomotionState 门控。三者关系:supervisor = 两个 policy 节点的"接管+安全封装" |
| observation_task_* | `src/observation_task_adapter.cpp` + `scripts/explorer_core/observation_*.py` | task_mode 的 ObservationTask→SCAN 事务、反馈与退休协议 |

关键消息字段(`src/planner/scan_planner_msgs/msg/`):`Bspline`(order/traj_id/**request_id**/task_id/version/candidate_id/start_time/knots/pos_pts/yaw_pts/**execution_mode**/max_forward_speed/max_lateral_speed/max_yaw_rate/max_tracking_correction/**min_forward_command/min_forward_effective_speed**);`TrajectoryAck`(request_id/trajectory_id/status/reason/matched_time/position_error/yaw_error);`ExecutionState`(STATE_IDLE/RUNNING/HEADING_FROZEN/SOFT_HOLD/HARD_STOP/FINISHED/STALLED + execution_time/duration/terminal_error);`ExecutionCommand`(HOLD/RESUME/RETIRE_TASK);`LocalMapPatch`(map_revision/origin/resolution/width/height/occupancy/generation_ms);`LocomotionState`(epoch/mode/state/**actuation_ready/perception_pose_valid**/reason)。

---

## 6. 一次"从看到 frontier 到轮子转动"的端到端时间线

以 Gazebo + planning_kinematic + 探索模式为例:

| t(相对) | 事件 | 环节 | 延迟 |
|---|---|---|---|
| 0 | gpu_lidar 打出新扫描(10 Hz) | gazebo.xacro:73 | — |
| ~0–20 ms | bridge → `/quad_0/cloud`;grid_map `cloudCallback` 收下(若 0.5 s 节流窗口未开则**本帧直接丢弃**,等下一帧) | grid_map.cpp:1471, :1476 | ≤100 ms |
| ≤50 ms | grid_map `updateOccupancyCallback` raycast 融合,`map_revision++` | grid_map.cpp:1110-1133 | 毫秒级 |
| ≤500 ms | **LocalMapPatch 节流**到 0.5 s 一发 → global_representation 更新拓扑图(对探索决策无强依赖,只是影子) | grid_map.cpp:1201-1205 | 0.5 s |
| ≤500 ms | Explorer `cloud_callback`(0.5 s 节流)把同一扫描并进 explorer 栅格,`map_content_revision++` | frontier_explorer.py:1211-1277 | 0.5 s |
| ≤250 ms | 下一个 `exploration_timer`:ReplanGate→frontier 聚类→估价→A*→终验→`/initial_path`(request_id=now_ns),状态 PATH_PUBLISHED | frontier_explorer.py:2327, :2413-2507 | ≤0.25 s + 搜索几十~几百 ms |
| ~几 ms | SCAN `pathCallback`(planning 组):defer 检查→重建 reference_path→全局样条→`PATH_ACCEPTED`→GEN_NEW_TRAJ | scan_replan_fsm.cpp:645-827 | 毫秒级 |
| ≤10 ms | exec_timer 拍:`callReboundReplan`:A* 修正+梯度优化+0.02 s 终验+交接扫掠→`/planning/bspline`(PLAN_TIMING 通常几~几十 ms) | scan_replan_fsm.cpp:4100-4933 | 毫秒级 |
| ~1 ms | Controller `bsplineCallback` 匹配→ACK ACCEPTED→`/planning/trajectory_ack`;SCAN `trajectoryAckCallback` 提交快照、GEN_NEW_TRAJ→EXEC_TRAJ、`RUNNING` | closed_loop_controller.cpp:210-473;scan_replan_fsm.cpp:1265-1511 | 毫秒级 |
| ≤100 ms | 下一个 100 Hz `cmdCallback`:跟踪 P 律→`/planning/cmd_vel_raw`(首帧通常带 0.30 前向或纯转向) | closed_loop_controller.cpp:617-851 | ≤10 ms |
| ≤8 ms | Supervisor control_loop(200 Hz):kinematic 直转 `/quad_0/cmd_vel`(→ gz VelocityControl 立即推动模型);rl_effort:≤20 ms 后 ONNX 动作→2 ms PD→关节 effort | supervisor :575-604 / :700-711 | ≤20 ms |
| 合计 | **frontier 出现 → 轮子动** | | 典型 **≈0.9–1.4 s**(主要消耗两处 0.5 s 融合节流 + 0.25 s 决策拍;规划/ACK/控制合计 <100 ms)。若 SCAN 在恢复事务中,Explorer 的 path 会被 defer(`REFERENCE_PATH_DEFERRED`),链路再加一个事务终结边界 |

---

## 7. 两个常见现象的代码级映射

### 7.1 "滚动重规划"由什么触发/被什么抑制

**触发(进入 REPLAN_TRAJ 的全部入口)**:
1. 进度触发:`shouldTriggerRollingReplan`(utils.h:1004-1012,planned 与 odometry 进度均 ≥ `thresh_replan=1.0 m`)——EXEC_TRAJ 每 100 Hz 检查(scan_replan_fsm.cpp:2309-2322)。
2. 预测性碰撞:安全回调 0.02 s 扫描发现余项硬命中但未到硬停窗口 → `predictive_replan_requested_`(:3996-4003),execFSMCallback :2002-2020 消费;或 `PredictedCollisionInterval` → `collision_interval_replan_requested_` → `planFromCurrentTraj` 走**区间修补** seed 路径(:2475-2626)。
3. preferred-clearance 侵入:0.20 m 带冲突 → `predictive_replan_requested_`(:4074-4082)。
4. 跟踪偏差:`classifyTrackingDeviation → ROLLING_REPLAN`(utils.h:604、618)。
5. 样条走完未到终点 `LOCAL_SEGMENT_DONE`(:2249-2273);航向冻结(:2022-2040);终端无进度(:1977-1996);ACK 对账失败后的强制恢复(:1824-1857、:1130-1195)。

**抑制/门(事务化分支新加的 gate)**:
- **pending_handoff 事务门**:候选已发未 ACK 时 FSM 整体 return(:1859-1869)——任何 100 Hz 拍都不推进、不重规划;ACK 超时两段窗(0.75+0.75 s)走 HOLD/对账。
- **结构化修复所有权**:`ordinaryRollingReplanAllowed = !structured_executing && requested`(utils.h:565-568);`referenceRequestMustBeDeferred`(utils.h:456-464)让新 Explorer 路径在修复事务期间被 defer(:668-696);rejoin 未 ACK 前不允许新逃逸(utils.h:35-38)。
- **安全闩锁**:`safety_stop_active_` + `safetyLatchAllowsPlanning`(utils.h:365-371)三条件白名单;`newPlanMayReleaseSafetyStop` 要求 pose_occupied/predictive_hold/temporal 三闩全清才能靠新计划放行(utils.h:383-390)。
- **ROLLING_RETRY 退避**:重规划失败且剩余时间 >1 s → 继续执行旧轨迹、`next_rolling_replan_attempt_ns_ = now+0.25 s`(:2180-2188;EXEC_TRAJ 里先查退避 :2297-2300)。
- **request/revision 一致性**:规划期间换请求或 `safety_generation_` 变化即 `PLAN_RESULT_DISCARDED`(:4260-4303、:4745-4760);区间修补还要 `map_revision` 一致(:4761-4805)。
- **速度感知交接失败**:`HANDOFF_PREDICTED_UNSAFE` → 不发布、转停车+本地修复(:4632-4646)。
- **finishProcess 预算**:5 次失败 → VIEWPOINT_LOCAL_REJECTED 或 EMERGENCY_STOP(:2371-2457)。

### 7.2 窄通道里"小幅度前进-停止循环"的机制

相关参数(planner.yaml:34-47):`min_forward_command=0.30`、`min_forward_effective_speed=0.165`、`clearance_limited_max_forward_speed=0.30`、`clearance_limited_execution_grace=1.0`、`clearance_limited_no_progress_timeout=2.0`、`clearance_limited_min_progress=0.05`。`tools/min_speed_probe/` 标定的结论:**0.30 不是速度而是策略可执行的最小命令档,其真实位移速率只有 0.165 m/s(平地开阔场地)**。可导致走走停停的具体回路:

1. **死区边缘爬行**:clearance-limited 候选发布时 `max_forward_speed=0.30`(scan_replan_fsm.cpp:4826-4828),spatial gait 下命令恒为 0.30(closed_loop_controller.cpp:780-786)——机器人以 0.165 m/s 爬行;同时样条已被重定时到峰值 0.165 m/s(:4476-4487)。任何额外阻力/转弯都会让实际速度跌进策略死区(<0.30 命令档不动),出现"命令发了、车没动"。
2. **HEADING_FREEZE 停走**:窄通道转弯时 |yaw_error|>0.8 rad → 平移冻结、只发 yaw 命令(closed_loop_controller.cpp:708-753);每 3 s 无改进上报 STALLED,SCAN 重规划(最多 2 次)后 BLOCKED。视觉表现即"走到拐角→原地转/停→再走"。
3. **CLEARANCE_LIMITED_NO_PROGRESS 循环**:clearance-limited 执行 2.0 s 内位移 <0.05 m → 安全回调直接 HOLD + `stopped_local_repair_pending_`(scan_replan_fsm.cpp:3747-3771)→ 停稳(≤0.05 m/s × 3 周期)后 `LOCAL_REPAIR_FROM_REST` → `tryStructuredLocalRepair` 生成 0.45–0.90 m、≤0.30 m/s 的静止-静止逃逸段;ACK 后执行该短段(同样 0.165 m/s 水平),完成→rejoin 参考路径→可能再次进入同样的受限执行→再次无进度。事务预算 3 段/6 候选用尽才 `VIEWPOINT_LOCAL_REJECTED` 交给 Explorer 换路。**这就是典型的窄通道小幅前进-停-挪-停循环**。
4. **PREDICTED_BRAKING_HOLD**:0.10 m clearance 带 + 制动扫掠(0.30 s 反应 + 0.32 m/s²,扫掠上限 4 m)在窄通道里极易命中预测点 → HOLD;释放要求静止 3 周期**且**当前位置出带(utils.h:392-401),出不了带就转 3 的修复回路(scan_replan_fsm.cpp:3476-3495)。
5. **soft hold 家族**:任何 `requestExecutionStop`(:1598-1660)都让 Controller `soft_hold_` → 每拍 `publishStop()`(零 Twist);终端收敛超时后命令直接置零(closed_loop_controller.cpp:845-846)。
6. **进度触发与重定时的相互作用**:`thresh_replan=1.0 m` 的滚动触发在 0.165 m/s 下约 6 s 才到一次;每次 REPLAN 都可能产生新的 clearance-limited 版本并重置 Controller 的进度游标——`classifyTrackingDeviation` 的 KEEP_EXECUTING 分支(utils.h:616-617)与 `clearance_limited_execution_owned`(scan_replan_fsm.cpp:3784-3799)专门压制"用等价 fallback 反复重置时钟"的退化;若仍反复触发,就形成"爬 0.05 m→HOLD→修复段→再爬"的低频循环。

**排查日志关键字**(按出现顺序读):`[CLEARANCE_LIMITED_EXECUTION_OWNED]`(正常爬行)→ `[CLEARANCE_LIMITED_NO_PROGRESS]`(进入停止修复)→ `[LOCAL_REPAIR_FROM_REST]`/`[STRUCTURED_LOCAL_REPAIR_*]`(修复段事务)→ `[LOCAL_SAFETY_HOLD] reason=actual_braking_sweep`(制动扫掠 HOLD)→ `[HEADING_FREEZE_*]`(拐角冻结)→ `[TRAJECTORY_HANDOFF_REJECTED]`(交接太远被拒)→ `[PREFERRED_CLEARANCE_FALLBACK]`(总是退到硬带 → 通道实际窄于 ~0.9 m)。

**三个最直接的旋钮**:`fsm.min_forward_command / min_forward_effective_speed`(死区标定,注意平地标定值是最优情形,转弯/窄通道下有效速度更低)、`heading_error_threshold`(0.8 rad 偏宽,窄通道易频繁冻结)、`fsm.planning_clearance_margin`(0.10 m 带叠加 0.35 m 双圆柱膨胀后,等效要求通道宽 ~0.9 m)。
