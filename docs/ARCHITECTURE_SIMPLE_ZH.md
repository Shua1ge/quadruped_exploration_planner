# SCAN-Planner 当前结构（简明版）

> 更新日期：2026-09-13，对应提交 `56f4b44`。
> 系统仍然是二维俯视探索加 SCAN 局部规划；`roboterrain_inspection` 场景把机器人放进起伏地形，但 Explorer 的决策表示仍是二维占据栅格，没有地形可通行性模型。

## 一句话理解

Explorer 决定"接下来往哪里探索、走哪条全局路线"，SCAN 决定"眼前这一小段怎样安全、平滑地走"，控制器负责让 GO2 跟随轨迹，安全回调负责在执行期间随时否决已经变得不安全的轨迹。Explorer 的长线路径估计来自一张由 SCAN 局部地图派生的稀疏拓扑图，但最终下发的每一段路径都会在 Explorer 自己的稠密栅格上做最终验证。

```text
点云 + GO2 位姿
   │
   ├────────────────────────────┐
   │                            │
   │  SCAN 局部三维滚动地图       │
   │        │                   │
   │        ├─> 轨迹安全检查       │
   │        └─> LocalMapPatch    │
   │              │              │
   │    global_representation    │
   │      骨架化 → 稀疏拓扑图      │
   │      TopoGraphDelta 增量发布 │
   │              │              │
   └─> Explorer 二维持久栅格 <───┘
         │  frontier → 簇 → 拓扑挂接区域 → 滚动区域选择
         │  稀疏路由估价（不可权威）→ 稠密 A* 终验
         v
      /initial_path（全局参考路径）
         │
         v
   SCAN 前向截取 + A* 修正 + B 样条优化
         │
         v
      /planning/bspline
         │
         v
   闭环控制器 /cmd_vel → GO2
```

## 五个主要部分

### 1. Explorer：选择探索目标和全局路线

入口是 `src/planner/plan_manage/scripts/frontier_explorer.py`（单节点 `frontier_explorer`，约 2200 行）。它按 `map_update_period`（0.5 s）节流地读取点云，把 z 带内的障碍点压成 720 个方位射线 bin，维护一张 `0.20 m` 分辨率的二维持久占据栅格（`explorer_core/grid.py`）。物理仿真模式下 z 带相对机体（`obstacle_z_relative_to_body`）。

每个规划周期（`planning_period` 0.25 s）它执行：frontier 提取 → 8 连通聚类 → 每簇选代表视点并挂接到稀疏拓扑（component/branch）→ 按拓扑走廊划分持久区域（`explorer_core/frontier_regions.py`）→ 滚动单步区域选择（1.35 迟滞）→ 候选路径估价 → 下发 `/initial_path`。候选路径长度优先用稀疏路由估计（`explorer_core/sparse_routing.py`），稀疏结果只用于估价和取回折线；真正发布前必须逐段通过稠密栅格验证，失败就回退完整 A*（`materialize_sparse_candidate`，`frontier_explorer.py:1414`）。

区域承诺由两层门控组成：frontier 消失/持续不可达的防抖释放（`update_region_commitment`），以及按地图 revision 计时的残值承诺门（`explorer_core/region_commitment.py` 的 `ResidualCommitmentGate`，残值耗尽或停滞才释放）。开放 Held-Karp 全区域游序现在只是背景预测，首项被强制等于滚动决策，不直接控制行为。

Explorer 的主要输出是 `/initial_path`。这是一条全局参考路线，不是直接发送给电机的轨迹。

### 2. global_representation：从局部地图提取稀疏拓扑

入口是 `src/planner/plan_manage/scripts/global_representation_node.py`。它订阅 SCAN 发布的 `grid_map/local_map_patch`（低分辨率二维切片），对自由空间做 Zhang-Suen 骨架化（`explorer_core/topology.py`），提取端点/交叉点为节点、度 2 链压缩为带折线的边，以 `TopoGraphDelta` 增量发布，并周期性发全量 snapshot。节点 ID 由世界坐标量化而来，跨滑动窗口保持稳定。

Explorer 订阅 delta 维护自己的 `SparseRouteGraph` 镜像，用它回答"从这里到每个候选视点大约多远"的一对多估价查询。该节点还计算 dense A* oracle 指标（连通率、代价误差），用于监控拓扑图质量。

### 3. SCAN：把全局路线变成可执行的局部轨迹

入口是 `src/planner/plan_manage/src/scan_planner_node.cpp`，状态机位于 `scan_replan_fsm.cpp`（INIT → WAIT_TARGET → GEN_NEW_TRAJ → REPLAN_TRAJ ⇄ EXEC_TRAJ → EMERGENCY_STOP），具体规划由 `planner_manager.cpp` 组织。

SCAN 收到 `/initial_path` 后，按 request id 校验请求（旧请求的迟到状态会被丢弃），从机器人当前位置沿路径弧长向前取一段局部目标，用 A* 修正受影响部分，再生成 B 样条并优化，发布 `/planning/bspline`。旧轨迹剩余部分仍安全时优先延续。

### 4. 控制器：跟随 B 样条

`closed_loop_controller.cpp` 根据当前位姿和 `/planning/bspline` 计算 `/cmd_vel`（P 控制位置和航向；航向偏差过大时冻结轨迹时间、原地转向）。仿真中由 `go2_kinematic_sim.cpp` 更新位姿，或在 `use_gazebo_physics` 模式下由 Gazebo 物理仿真驱动；实际机器人部署时，`run.launch.py` 将位姿、点云和速度命令重映射到外部 LIO 与底盘驱动。

### 5. 地图与安全回调：执行期间持续检查

`plan_env` 维护 SCAN 的 0.05 m 三维滚动占据图。规划、地图融合和安全检查分别位于不同 callback group：规划读取固定快照，地图线程继续融合新点云，安全线程用最新实时地图检查真实位姿、位姿到轨迹的连接段以及剩余轨迹。真正的安全停车会打印 `[REALTIME_SAFETY_STOP]`；`[SAFETY_MAP_STALE]` 只是地图更新年龄告警，不等于已经停车。

## 代码目录怎么找

```text
src/planner/plan_manage/scripts     Explorer、global_representation 与 explorer_core 逻辑库
src/planner/plan_manage/src         SCAN 主节点、状态机、控制器、运动学仿真
src/planner/plan_manage/launch      run / quad_exploration / roboterrain_inspection 等 launch
src/planner/plan_env                SCAN 局部占据地图、LocalMapPatch 发布
src/planner/path_searching          2.5D A*（B 样条穿障重连用）
src/planner/bspline_opt             B 样条 rebound 优化
src/planner/traj_utils              轨迹表示和可视化
src/planner/scan_planner_msgs       Bspline / LocalMapPatch / TopoGraphDelta 等消息
src/simulator                       点云传感器模拟、地图发布、Go2 模型与 Gazebo 仿真
docs                                架构、交接和版本说明
tools                               地图转换脚本（PCD 生成）
```

## 一次完整运行

平地迷宫自主探索：

```bash
cd /home/t1an/ros2_ws/scan_planner_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch scan_planner quad_exploration.launch.py
```

起伏地形场景（`physics:=true` 时使用 Gazebo 物理仿真和 heightmap 世界）：

```bash
ros2 launch scan_planner roboterrain_inspection_exploration.launch.py physics:=true
```

正常的数据闭环是：Explorer 发布参考路径，SCAN 接收并生成局部轨迹，控制器开始执行；接近局部段末端时 SCAN 沿原参考路线滚动生成下一段。Explorer 在任务信息增益达到 60% 时准备下一个观测任务，达到 80% 且连续 3 次地图更新确认后交接；路径失效时，稀疏路由与区域承诺保证替换目标稳定收敛。

## 卡住时先看什么

如果 SCAN 一直显示 `WAIT_TARGET`，先看 Explorer 是否持续打印 `NO_REACHABLE_FRONTIER`、`REGION_COMMITMENT_RETAINED` 或 `WAITING_FOR_PLANNING_INPUT_CHANGE`。这表示上层没有提供目标，不是 B 样条优化卡死。

如果全局蓝线变化很快但局部红线没有更新，检查 `/planning/status` 中的 request id、`[PATH_REQUEST_IN_FLIGHT]`、`[STALE_PLANNING_STATUS_IGNORED]` 和 `[PLAN_RESULT_DISCARDED]`。

稀疏路由质量看 `[GLOBAL_PLAN_BASELINE]` 中的 `sparse_candidate_hits / fallbacks` 和 miss 原因分布；拓扑图本身的质量看 `[TOPOLOGY_SHADOW]` 的 recall 和 cost_error。

如果出现 `[REALTIME_SAFETY_STOP]`，再根据 `pose_occupied`、`connector_blocked` 或 `trajectory_blocked` 判断具体碰撞原因。`[PHYSICAL_COLLISION]` 表示仿真接触守卫已锁存，Explorer 在本次运行中不可恢复。
