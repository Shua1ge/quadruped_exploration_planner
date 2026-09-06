# SCAN-Planner + Explorer 项目交接说明

> 更新日期：2026-09-06  
> 工作空间：`/home/t1an/ros2_ws/scan_planner_ws`  
> 主仓库：`/home/t1an/ros2_ws/scan_planner_ws/src/SCAN-Planner`  
> 当前分支：`ros2-community`  
> 当前代码状态提交：`e874635`  
> 修改前回滚点：`027af12`

## 1. 项目现在要做什么

本项目不是重新实现一个完整导航栈，而是在 ROS 2 Humble 下形成一套“全局自主探索 + 可替换局部规划器 + Go2 执行”的研究平台。

当前阶段只研究平地迷宫中的自主探索。Explorer 负责维护已知/未知空间、选择值得观测的区域和观测位姿，并生成已知自由空间内的全局参考路径；SCAN 负责把参考路径转成满足动力学约束的局部 B 样条，并利用局部稠密三维点云否决碰撞轨迹；闭环控制器负责执行轨迹。

第一阶段的核心研究问题是：在 `quad` 这类大平面迷宫里，持久区域、开放式 ATSP、区域承诺、信息增益完成条件和局部实时修正，能否减少折返、重复路线和目标切换，同时保持安全到达与较高探索效率。

`quad` 是平地迷宫，不是三维起伏地形。它适合先隔离全局探索策略问题，但不能证明系统具备楼层切换、楼梯、山地或三维 traversability 能力。

## 2. 当前系统边界

当前版本明确包含：二维持久探索栅格、frontier 检测、候选观测位姿、已知自由区 A*、区域级开放式 ATSP、区域承诺、观测收益完成判据、SCAN 局部三维碰撞感知、B 样条轨迹、闭环 Go2 仿真执行和探索指标记录。

当前版本明确不包含：多层地图、楼梯和坡面通行性、地形代价、足端落脚规划、能耗模型、真实机器人 LIO/驱动集成验证，以及 RL 局部规划器。不要为了“架构完整”提前加入这些模块。

SCAN 被视为当前局部规划后端，不应让 Explorer 依赖 SCAN 内部的 B 样条或 rebound optimizer 细节。后续替换 RL 局部规划器时，应保留“参考路径输入、轨迹就绪、局部阻塞、停止”这一层接口，而不是重写 Explorer。

## 3. 地图到底是不是“有图”

仿真器加载完整 `quad_flat.pcd`，但完整 PCD 主要用于生成局部传感器观测；Explorer 和 SCAN 不应直接把全部真值障碍当作已知地图使用。

系统中实际存在两张用途不同的在线地图：

1. Explorer 从局部 LiDAR 点云逐步积累二维持久占据栅格，用于 unknown/free/occupied、frontier、区域划分和全局 A*。未观测空间继续保持 unknown。
2. SCAN 从局部点云维护三维滚动占据图和膨胀层，用于局部轨迹碰撞检查。它更密、更局部，也会随机器人移动更新窗口。

因此当前实验是“仿真器知道完整世界，但规划系统只使用传感器逐步揭示的地图”，不是传统意义上把完整真值图直接交给导航器的静态有图导航。

## 4. 当前数据流与模块职责

```text
quad_flat.pcd
    ↓ 仿真局部感知
cloud + body_pose
    ├──────────────→ Explorer 持久二维地图
    │                    ↓ frontier / region / observation
    │                    ↓ 已知自由区 A* 参考路径
    │                 initial_path（蓝色参考路径）
    │                    ↓
    └──────────────→ SCAN 三维滚动地图 + 膨胀层
                         ↓ 路径跟随局部目标
                         ↓ rebound/B-spline 优化
                      planning/bspline（红黄局部轨迹）
                         ↓
                      闭环控制器 → Go2

SCAN → planning/status / planning/blocked_segment / emergency_stop → Explorer
```

Explorer 对“去哪里、区域顺序、探索任务是否已经获得足够信息”负责。SCAN 对“眼前这段轨迹现在能不能安全执行”负责。控制器对“尽量跟上已发布轨迹并报告执行冻结状态”负责。

全局层和局部层都检查路径并不等于两套相互竞争的守卫。全局层只保证路径在其较粗、逐步建立的地图里成立；局部层使用最新稠密地图作最终执行否决。两者不能用不同规则同时反复重规划同一件事，也不能让安全回调自己同步跑一轮耗时 A*。

## 5. Explorer 的全局策略

### 5.1 Frontier 与安全观测位姿

frontier 是已知自由空间与未知空间的边界。机器人不再被要求踩到 frontier 格子上，而是在 frontier 的已知自由侧选择一个有安全退距的观测位姿。这样墙体在后续点云中显现时，旧 frontier 消失不会立刻把机器人目标变成障碍物内部的点。

当前 `viewpoint_standoff` 是 `1.0 m`。候选位姿和到 frontier 的视线段必须位于 Explorer 当前认为的已知自由空间中，并避开其规划膨胀集合。

### 5.2 不是“到点即完成”，而是“信息任务完成”

这部分受 FUEL 思路启发。选择一个观测任务时，Explorer 冻结该 frontier 周围当前仍为 unknown 的目标单元集合。运行中统计这些单元有多少已经变成 free 或 occupied。

当前默认在有效观测完成度达到 `60%` 时准备下一个候选，在达到 `80%` 且连续 `3` 次地图更新都满足时完成当前任务。机器人可以在真正走到观测位姿之前切换任务；如果到达位姿但信息提升不足，任务也不能被误判为完成。

这比固定“距离目标 X 米开始下一次规划”更适应长短不同的局部轨迹。

### 5.3 区域级 ATSP 与区域内选择

当前策略是两层结构。第一层把 frontier clusters 组织成带持久 ID 的区域，并使用实际已知自由区路径成本构建非对称代价矩阵，再求开放式 ATSP 路线；开放式表示不要求从最后一个区域返回起点。第二层在当前承诺区域内选择具体 frontier 和观测位姿。

区域顺序不是每个定时周期全部推翻。只要承诺区域还有可用候选，ATSP 的第一个区域就被强制保留。区域短暂消失时还要经过连续若干次更新才能释放，以吸收 frontier 因点云抖动产生的分裂、合并和暂时消失。

区域间和候选间路径成本使用一次单源最短路径树复用，不再为每个候选独立运行一次 A*。准备阶段保存候选路径；交接时从机器人最新位置拼接到仍有效的路径后缀，而不是重新枚举所有 frontier。

### 5.4 当前区域承诺还没有被证明做好

现有机制已经解决“区域 ID 每帧改变”和“每次全局更新都推翻首区域”的基础问题，但实测仍出现附近有未完成探索内容却跳到远区域的情况。

当前最重要的假设是：`region_size=8.0 m` 的区域构造可能过粗，把几何位置接近但需要绕不同门口才能到达的 frontier 合并到同一 ATSP 节点，导致区域内部的通路结构被压扁。这样 ATSP 看起来承诺了同一区域，具体候选却可能位于另一个走廊入口；也可能把本应独立排序的两个局部区域当成一个区域。

这仍是待验证问题，不应直接通过加入“禁止远跳 X 米”掩盖。下一步应先把区域 ID、区域包含的 frontier clusters、区域内候选路径成本和当前承诺区域在 RViz 中可视化，再比较更细分区或基于自由空间连通/门户的区域构造。

## 6. 连续准备与路径交接

当前“提前准备”发生在 Explorer 层。它可以提前选出下一个观测任务并保存对应全局 A* 路径，但不会提前生成下一条 SCAN B 样条。原因是 A2、A3 可能还在 SCAN 当前滚动窗口或清晰感知范围之外，远端路径只能被当作全局意图，不能被当作已经验证的局部可执行轨迹。

交接时会重新检查准备路径的终点和可复用后缀，并从最新机器人位置拼接；失效前缀可以丢弃，失效终点或无法安全拼接则放弃准备结果。这样能减少重算，但不能承诺 A2/A3 的整条局部轨迹提前平滑且永远可达。新障碍被观察到时，局部否决和重新选路仍然是正常行为。

Explorer 现在一次只允许一条 `initial_path` 请求处于等待 SCAN 响应的状态。收到 `PATH_TRAJECTORY_READY`、`BLOCKED`、`REFERENCE_PATH_REJECTED` 或 `INVALID_REFERENCE_PATH` 后才结束本次握手，避免定时器重复发布路径、SCAN 不断追逐新请求和规划延迟随运行时间增长。

## 7. SCAN 当前在做什么

SCAN 接收 Explorer 给出的折线路径，并沿路径弧长向前选择局部目标。当前 `planning_horizon=7.5 m`，`reference_path_lookahead=4.5 m`。使用弧长而不是到终点的直线距离，可以避免在相邻迷宫走廊之间跨墙跳目标。

SCAN 使用 A* 或已有路径信息产生初值，再用三阶 B 样条平滑轨迹。图中局部轨迹出现适当圆弧是正常的：B 样条在减少速度、加速度和转向突变。但如果圆弧明显偏离参考走廊、绕出大圈或突然回头，就不是“平滑必然如此”，而应检查参考路径稀疏程度、起点速度方向、局部目标、fitness/collision/smooth 权重以及地图是否在优化期间发生了版本变化。

后续如果 RL 局部规划器能够承担局部避障与运动连续性，SCAN 可以替换。Explorer 不应因此改变区域策略和信息任务定义。

## 8. 最近解决的系统级碰撞问题

此前 SCAN 使用单线程执行器。一次 rebound/B-spline 规划可能耗时 `7～10 s`，这段时间内点云融合和名义上每 `50 ms` 执行的安全检查都无法运行。所有碰撞检查读到同一张过时地图，再增加终点检查或轨迹采样也只会重复得到“安全”。这是多次出现 GO2 走进膨胀障碍区的关键系统原因。

提交 `e874635` 将规划、地图和安全回调放入三个独立 callback group，并使用三线程 executor。规划读取一份固定的膨胀地图快照，地图线程继续融合新点云，安全线程使用最新实时地图检查真实位姿、真实位姿到理论轨迹位置的连接段，以及剩余 B 样条。

规划完成后不会直接发布。系统先用最新地图复查完整 B 样条和从最新里程计位置到新轨迹起点的连接段。如果规划期间安全线程已经否决执行，该规划结果会因 safety generation 改变而丢弃。

实时否决是一个锁存状态。否决旧轨迹后，状态机不能继续拿旧 reference path 自动恢复；只有新目标或新 reference path 被明确接收后才释放。这样关闭了“安全线程停止旧轨迹，但旧规划稍后成功并重新启动”的竞态窗口。

这次修改提高的是规划期间的感知和安全响应能力，不会直接把 SCAN 的 `7～10 s` 优化变成毫秒级。红色局部轨迹出现前仍可能等待较久；未来 RL 局部规划器可以从性能上替换这一段。

## 9. 局部阻塞反馈的边界

当 SCAN 根据稠密地图否决轨迹时，它发布 `BLOCKED` 和可选的 `planning/blocked_segment`。Explorer 把阻塞反馈映射到当前全局路径的一条有方向的边，并在有限 TTL 内禁止这条边。

临时阻塞不应屏蔽整个目标、整个区域或一大片膨胀侧空间。否则窄走廊会被过度封死，探索通行性下降。当前 `blocked_edge_ttl` 默认 `30 s`，屏蔽对象是小段有向路径边；反方向不自动永久屏蔽。

如果 SCAN 只知道“当前位置或轨迹已碰撞”而没有可靠阻塞段，Explorer 应换候选或重算路径，但不能伪造大面积不可通行区域。

## 10. 已讨论问题与当前结论

### 10.1 全局路径绕路、回头和重复

可能来源包括区域过粗、区域内部仍采用局部候选决策、地图增长导致代价变化、prepared path 拼接、路径简化，以及当前候选完成/失效后的切换。区域承诺只能减少随意切换，不能自动保证区域内部访问顺序最优。

应该用总行程、折返距离、区域切换次数、覆盖率随时间、重复经过栅格比例和规划耗时判断改进，而不是只看一条轨迹是否美观。

### 10.2 GO2 走走停停

曾经有两类停顿。Explorer 每次到点后等待下一轮低频全局规划，造成约十秒空窗；SCAN 收到蓝色参考路径后同步优化，又造成第二段等待。Explorer 的单源搜索、候选路径缓存、信息完成条件和请求握手已经减少第一类停顿。SCAN 多线程修复保证等待期间地图与安全不中断，但第二类计算时延仍存在。

### 10.3 SCAN 的局部轨迹看起来很短

滚动地图窗口是 `10 × 10 × 5 m`，以机器人附近滑动；局部规划 horizon 是 `7.5 m`。实际 B 样条长度还受障碍物、当前参考路径形状、局部目标、最大速度、终点速度和优化成功范围限制，所以可视化轨迹可能短于窗口尺寸。

### 10.4 蓝色点和不同颜色轨迹

RViz 中青蓝色方块通常来自 Explorer frontier 或地图边界类 Marker；蓝/青折线是 Explorer 或全局 A* 给出的参考路径；红黄渐变粗线是 SCAN 的局部 B 样条或轨迹时间/速度着色；绿色线通常是历史/候选/其他规划可视化，具体应以 RViz Display 名称和话题为准。彩虹色立方体是 SCAN 三维膨胀占据层，不代表机器人实际已经碰撞；需要结合 Go2 碰撞体中心、朝向和双圆柱查询判断。

### 10.5 为什么不能只相信全局点选得好

Explorer 的栅格更粗、更新更慢，而且不知道 SCAN 当前滚动窗口里的完整三维占据细节。全局点选得合理只能降低局部失败率，不能替代执行前和执行中的稠密碰撞否决。局部层的必要性在于处理地图时差、传感器新信息、轨迹平滑偏移和跟踪误差，而不是再做一套全局探索。

## 11. 当前关键参数

当前 `quad_exploration.launch.py` 的主要参数如下：

```text
Explorer resolution             0.20 m
Explorer mapping_range          7.5 m
Explorer no_return_range        3.0 m
Explorer inflation_radius       0.65 m
viewpoint_standoff              1.0 m
region_size                     8.0 m
region_match_distance           6.0 m
region_release_updates          3
observation_prepare_ratio       0.60
observation_done_ratio          0.80
observation_done_updates        3
planning_period                 0.25 s

SCAN map resolution             0.05 m
SCAN rolling window             10 × 10 × 5 m
SCAN local update range         5 × 5 × 2.5 m
SCAN double cylinder radius     0.35 m
SCAN double cylinder offset     0.18 m
SCAN planning horizon           7.5 m
SCAN reference lookahead        4.5 m
SCAN max velocity               0.75 m/s
SCAN max acceleration           0.5 m/s²
```

Explorer 的 `0.65 m` 膨胀用于较粗全局路径安全余量，SCAN 的 `0.35 m + 0.18 m` 双圆柱用于更接近 Go2 身体形状的局部检查。不要继续无条件叠加更多固定膨胀层；窄空间通行性已经是已知权衡。

## 12. Debug 消息的读法

`[PATH_REQUEST_IN_FLIGHT]` 表示 Explorer 已发布参考路径，正在等待 SCAN，不会重复发新请求。

`[LOCAL_TRAJECTORY_READY]` 给出从全局路径发布到局部轨迹准备完成的 pipeline latency。

`[PLAN_TIMING]` 给出 SCAN 本轮耗时、规划快照 revision、完成时实时地图 revision、地图年龄和结果。规划期间 live revision 持续增长，才能证明地图没有被优化阻塞。

`[SAFETY_SCHEDULER_LAG]` 表示安全回调实际间隔超过 `150 ms`。如果它频繁出现，应先检查 CPU、点云处理和 RViz 发布，而不是继续添加碰撞判断条件。

`[SAFETY_MAP_STALE]` 表示局部地图超过 `250 ms` 没有完成融合。

`[REALTIME_SAFETY_STOP]` 会区分 `pose_occupied`、`connector_blocked` 和 `trajectory_blocked`。

`[PLAN_RESULT_DISCARDED]` 表示优化得到了结果，但新地图、最新 odom connector 或规划期间发生的安全否决使结果失效，因此没有发布给控制器。

`[SAFETY_LATCHED]` 表示旧路线已经被否决，等待新路线；`[SAFETY_RELEASE]` 表示新路线已经被 SCAN 接收并允许恢复规划。

## 13. Benchmark 与评价方法

第一阶段继续使用确定性的 `quad` 平地迷宫。固定初始位姿或固定若干起点，至少比较 `hierarchical` 与现有简单策略，并记录相同地图、相同终止条件和相同最大运行时间下的结果。

建议主要指标是最终覆盖率、达到给定覆盖率的时间、总行程、折返距离、重复经过率、区域切换次数、局部 `BLOCKED` 次数、物理碰撞次数、Explorer 规划耗时和蓝线到红线的 pipeline latency。规划成功率不能代替物理碰撞统计。

随机生成地图可用于大量统计和消融，但不能取代人工结构化场景。后续可增加走廊、环路、死胡同、多个门口和狭窄通道的场景；开放世界只适合在第一阶段逻辑稳定后验证尺度扩展。

## 14. 下一步优先级

下一步先实测提交 `e874635` 的线程修复，确认慢规划期间 `[PLAN_TIMING]` 的 live map revision 持续增加，安全回调没有长时间缺席，并验证一次阻塞事件形成：

```text
REALTIME_SAFETY_STOP
    → SAFETY_LATCHED
    → Explorer 新 initial_path
    → PATH_ACCEPTED / SAFETY_RELEASE
    → PATH_TRAJECTORY_READY
```

确认系统级安全数据流后，再处理区域承诺。优先增加区域和承诺状态可视化，并用实际 A* 连通关系检查 `8 m` 分区是否合并了拓扑上不同的走廊。不要先加固定远跳距离保护，也不要同时改 SCAN 优化器权重，否则无法判断区域策略的真实影响。

连续滚动的下一阶段应先定义稳定的局部规划器接口和“当前轨迹剩余可执行时间/距离”，再考虑后台预生成下一条局部轨迹。A2、A3 超出可靠感知范围时只能预备全局路径，不能提前承诺局部可执行性。

## 15. 构建、测试和启动

构建受影响包：

```bash
cd /home/t1an/ros2_ws/scan_planner_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select plan_env scan_planner --symlink-install
source install/setup.bash
```

启动 quad 自主探索：

```bash
cd /home/t1an/ros2_ws/scan_planner_ws
source install/setup.bash
ros2 launch scan_planner quad_exploration.launch.py
```

当前验证记录：`plan_env` 和 `scan_planner` 编译通过；3 个 C++ 测试和 2 个启动测试通过；禁用用户目录中不兼容的第三方 pytest 插件后，37 个 Python 测试通过：

```bash
cd /home/t1an/ros2_ws/scan_planner_ws/src/SCAN-Planner
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
python3 -m pytest -q \
  src/planner/plan_manage/test/test_global_astar.py \
  src/planner/plan_manage/test/test_frontier_explorer.py \
  -o cache_dir=/tmp/scan_planner_pytest_cache
```

直接运行 `colcon test` 时，用户目录里的新版 `anyio` pytest 插件与 ROS/Ubuntu 自带 pytest 不兼容，可能在收集阶段报 `No module named '_pytest.scope'`。这是测试环境插件冲突，不是项目测试失败。

## 16. Git 与仓库状态

```text
e874635  fix: keep mapping and safety responsive during planning
027af12  checkpoint: exploration and collision handling baseline
d0b921c  Update simulation configuration and map asset
```

只撤销最新线程与安全修改时使用：

```bash
cd /home/t1an/ros2_ws/scan_planner_ws/src/SCAN-Planner
git revert e874635
```

外层仓库仍显示未跟踪目录 `quadruped_exploration_planner/`。它本身是独立 Git 仓库，当前内部提交为 `d3c41e1`，没有被强行加入 SCAN-Planner 外层提交。上传 GitHub 前需要决定保留为独立仓库、正式 submodule，还是删除嵌套 `.git` 后纳入主仓库；不要在未决定前直接 `git add .`。

## 17. 给新会话的建议开场

可以直接把本文件交给新会话，并发送：

> 请完整阅读 `/home/t1an/ros2_ws/scan_planner_ws/src/SCAN-Planner/docs/PROJECT_HANDOFF_ZH.md`，再检查当前仓库和最近提交。不要重做已完成修改，也不要扩大到三维地形、RL 或新的安全层。先基于 `e874635` 的实测日志验证规划期间地图与安全回调是否持续运行，然后继续分析区域分块是否过粗、是否丢失自由空间连通性。任何代码修改前先解释数据流和预期影响。
