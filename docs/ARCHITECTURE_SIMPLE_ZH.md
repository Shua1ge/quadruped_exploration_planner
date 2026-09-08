# SCAN-Planner 当前结构（简明版）

本文描述 `v0.2.0` 的实际运行结构。系统仍然是二维平地探索加 SCAN 局部规划，没有加入三维地形决策、强化学习或额外安全层。

## 一句话理解

Explorer 决定“接下来往哪里走”，SCAN 决定“眼前这一小段怎样安全、平滑地走”，控制器负责让 GO2 跟随轨迹，安全回调负责在执行期间随时否决已经变得不安全的轨迹。

```text
点云 + GO2 位姿
       │
       ├──> Explorer 二维地图 ──> Frontier/区域排序 ──> 全局参考路径 /initial_path
       │                                                    │
       └──> SCAN 局部地图 ──────────────────────────────────┤
                                                            v
                                         前向截取 + A* 修正 + B 样条
                                                            │
                                                            v
                                              /planning/bspline
                                                            │
                                                            v
                                                 闭环控制器 /cmd_vel
                                                            │
                                                            v
                                                           GO2

执行期间：最新位姿 + 最新局部地图 ──> 安全回调 ──> 继续执行或立即停止并要求新路径
```

## 四个主要部分

### 1. Explorer：选择探索目标和全局路线

入口是 `src/planner/plan_manage/scripts/frontier_explorer.py`。它读取点云和机器人位姿，维护一张 `0.20 m` 分辨率的二维占据图，提取“已知自由空间和未知空间交界处”的 Frontier，再为 Frontier 寻找安全且 A* 可达的观察点。

区域分块只用于确定探索顺序。它不会直接驱动 GO2，也不是碰撞检测地图。当前区域短暂没有可达观察点时会保留承诺；持续不可达约 `5 s` 时会释放承诺，防止 Explorer 与 SCAN 一起无限停在 `WAIT_TARGET`。

Explorer 的主要输出是 `/initial_path`。这是一条全局参考路线，不是直接发送给电机的轨迹。

### 2. SCAN：把全局路线变成可执行的局部轨迹

入口是 `src/planner/plan_manage/src/scan_planner_node.cpp`，状态机位于 `scan_replan_fsm.cpp`，具体规划由 `planner_manager.cpp` 组织。

SCAN 收到 `/initial_path` 后，从机器人当前位置向前取一段可靠感知范围内的路线。旧轨迹剩余部分仍然安全时优先延续；局部地图发生变化时，用 A* 修正受影响的部分，再生成平滑 B 样条；只有旧路径不能继续使用时才完整重算。生成的局部轨迹发布到 `/planning/bspline`。

常见状态含义如下：

```text
WAIT_TARGET    等待 Explorer 或人工目标
GEN_NEW_TRAJ   第一次生成局部轨迹
REPLAN_TRAJ    执行中滚动更新轨迹
EXEC_TRAJ      控制器正在执行轨迹
EMERGENCY_STOP 安全回调已经否决当前轨迹
```

### 3. 控制器：跟随 B 样条

`closed_loop_controller.cpp` 根据当前位姿和 `/planning/bspline` 计算 `/cmd_vel`。仿真中由 `go2_kinematic_sim.cpp` 更新 GO2 位姿；实际机器人部署时，`run.launch.py` 将位姿、点云和速度命令重映射到外部 LIO 与底盘驱动。

控制器只执行当前已接受的轨迹。新轨迹交接时使用机器人最新前向位置，不应回放旧路径起点。

### 4. 地图与安全回调：执行期间持续检查

`plan_env` 维护 SCAN 的高分辨率局部占据图。点云预处理、地图融合、规划读取和安全检查通过快照与独立回调协作，规划期间地图 revision 仍应持续增加。

安全回调以最新 GO2 位姿检查当前位置、当前位置到轨迹的连接段，以及剩余轨迹。真正的安全停车会打印 `[REALTIME_SAFETY_STOP]`；`[SAFETY_MAP_STALE]` 只是地图更新年龄超过阈值的性能告警，本身不等于已经触发停车。

## 代码目录怎么找

```text
src/planner/plan_manage     主节点、状态机、滚动规划、控制器、Explorer 和 launch
src/planner/plan_env        SCAN 局部占据地图与射线融合
src/planner/path_searching  A* 路径搜索
src/planner/bspline_opt     B 样条优化
src/planner/traj_utils      轨迹表示和可视化
src/planner/scan_planner_msgs  自定义 ROS 2 消息
src/simulator               点云传感器、地图和 GO2 仿真辅助节点
docs                        架构、交接和版本说明
```

日常调试通常先看 `plan_manage` 和 `plan_env`，不需要在所有包之间来回搜索。

## 一次完整运行

`quad_exploration.launch.py` 组合仿真器、SCAN、闭环控制器、Explorer 和 RViz。启动命令为：

```bash
cd /home/t1an/ros2_ws/scan_planner_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch scan_planner quad_exploration.launch.py
```

正常的数据闭环是：Explorer 发布路径，SCAN 接收并生成局部轨迹，控制器开始执行；快到局部段末端时 SCAN 沿原全局路线滚动生成下一段。如果路径失效，Explorer 或 SCAN 只修改受影响部分；安全回调发现真实风险时才停止并等待新路径。

## 卡住时先看什么

如果 SCAN 一直显示 `WAIT_TARGET`，先看 Explorer 是否持续打印 `NO_REACHABLE_FRONTIER` 或 `REGION_COMMITMENT_RETAINED`。这表示上层没有提供目标，不是 B 样条优化卡死。

如果出现 `[REALTIME_SAFETY_STOP]`，再根据 `pose_occupied`、`connector_blocked` 或 `trajectory_blocked` 判断具体碰撞原因。只有 `[SAFETY_MAP_STALE]` 而没有安全停车日志时，应把它当成地图融合性能问题，而不是停车结论。

如果全局蓝线变化很快但局部红线没有更新，检查 `/planning/status` 中的 request id、`[PATH_REQUEST_IN_FLIGHT]` 和 `[PLAN_RESULT_DISCARDED]`，确认旧请求的迟到结果是否已被丢弃。
