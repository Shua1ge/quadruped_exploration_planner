# SCAN-Planner + Explorer 项目交接说明

> 更新日期：2026-09-13  
> 工作空间：`/home/t1an/ros2_ws/scan_planner_ws`  
> 主仓库：`/home/t1an/ros2_ws/scan_planner_ws/src/SCAN-Planner`  
> 当前分支：`ros2-community`  
> 当前代码状态提交：`56f4b44`（feat: execute sparse routes with dense connectors）  
> 注意：工作树中存在未提交修改（plan_manage 的 launch 与 explorer 脚本、go2_description 仿真配置等），本文描述以工作树实际代码为准。

本文取代 2026-09-06 版交接说明（对应 `e874635`）。那一版描述的"区域级 ATSP 驱动全局顺序"已被"稀疏拓扑路由 + 滚动区域承诺"取代；SCAN 局部规划与安全链路的设计没有变化。

## 1. 项目现在要做什么

本项目不是重新实现一个完整导航栈，而是在 ROS 2 Humble 下形成一套"全局自主探索 + 可替换局部规划器 + Go2 执行"的研究平台。

第一阶段（平地迷宫）的核心问题——持久区域、区域承诺、信息增益完成条件和局部实时修正能否减少折返、重复路线和目标切换——已经有了一套完整实现，当前重点转向两件事：

1. 全局路由的稀疏化：SCAN 局部地图被提炼成稀疏拓扑图，Explorer 在图上做长程估价，避免每次全局规划都在整张稠密栅格上搜索；
2. 起伏地形场景（`roboterrain_inspection`）的接入：目前只完成了仿真环境、PCD 地图和参数适配，Explorer 仍是二维表示，尚无地形可通行性模型。

明确不包含：多层地图、楼梯和坡面通行性、地形代价、足端落脚规划、能耗模型、真实机器人 LIO/驱动集成验证、RL 局部规划器。不要为了"架构完整"提前加入这些模块。

## 2. 当前系统边界

当前版本明确包含：二维持久探索栅格、frontier 检测、候选观测位姿、拓扑挂接的持久区域、滚动单步区域选择、残值区域承诺、开放 Held-Karp 预测游序、稀疏拓扑路由与稠密终验、观测收益完成判据、连续准备与路径交接、SCAN 局部三维碰撞感知、B 样条轨迹、闭环 Go2 仿真执行（运动学或 Gazebo 物理）和探索指标记录。

SCAN 被视为当前局部规划后端，不应让 Explorer 依赖 SCAN 内部的 B 样条或 rebound optimizer 细节。后续替换 RL 局部规划器时，应保留"参考路径输入、轨迹就绪、局部阻塞、停止"这一层接口，而不是重写 Explorer。

## 3. 系统里现在有三张"地图"

1. **Explorer 二维持久栅格**（`explorer_core/grid.py`）：0.20 m，从局部点云逐步积累 unknown/free/occupied，用于 frontier、区域划分和稠密 A*。占据证据是粘性的（sticky），未知空间保持未知。
2. **SCAN 三维滚动占据图**（`plan_env`）：0.05 m，10 × 10 × 5 m 滑动窗口，用于局部轨迹碰撞检查，并派生 `LocalMapPatch`。
3. **稀疏拓扑图**（`global_representation_node` → `SparseRouteGraph`）：由 LocalMapPatch 骨架化而来，只表示已知自由空间的走廊结构与 clearance，用于长程路径估价。它不是独立感知来源，最终发布路径前必须在第 1 张图上做稠密验证。

仿真器加载完整 PCD 只用于生成局部传感器观测；规划系统不直接使用真值地图。

## 4. 当前数据流与模块职责

```text
quad_flat.pcd / roboterrain_inspection_*.pcd
    ↓ 仿真局部感知
cloud + body_pose
    ├──────────────→ Explorer 持久二维地图
    │                    ↓ frontier / 簇 / 拓扑挂接区域 / 滚动选择
    │                    ↓ 稀疏路由估价 → 稠密 A* 终验
    │                 initial_path（蓝色参考路径，含 request id）
    │                    ↓
    └──────────────→ SCAN 三维滚动地图 + 膨胀层
                         │            ↓ LocalMapPatch（0.2 m 切片）
                         │      global_representation → 骨架化 → TopoGraphDelta
                         │            ↓
                         │      Explorer 的 SparseRouteGraph 镜像（估价用）
                         ↓ 沿弧长前向局部目标
                         ↓ rebound/B-spline 优化
                      planning/bspline（红黄局部轨迹）
                         ↓
                      闭环控制器 → Go2

SCAN → planning/status（含 request_id）/ planning/blocked_segment / emergency_stop → Explorer
```

Explorer 对"去哪里、区域顺序、探索任务是否已经获得足够信息"负责。SCAN 对"眼前这段轨迹现在能不能安全执行"负责。控制器对"尽量跟上已发布轨迹并报告执行冻结状态"负责。全局层和局部层都检查路径并不是两套竞争守卫：全局层保证路径在较粗、逐步建立的地图里成立；局部层使用最新稠密地图作最终执行否决。

## 5. Explorer 的全局策略

### 5.1 Frontier 与安全观测位姿

frontier 是已知自由空间与未知空间的边界（4 邻域判定，`grid.py:154`）。先做 8 连通洪泛聚类（`min_frontier_size=6` 过滤碎簇），机器人被引导到 frontier 已知自由侧一个有 `viewpoint_standoff=1.0 m` 退距的观测位姿，位姿和视线段都必须在规划膨胀集合之外。到点不等于完成——见 5.4。

### 5.2 拓扑挂接的区域划分

每个簇选一个代表视点，用 `sparse_router.topology_attachment` 把它挂到稀疏拓扑图上，得到 `(component_id, branch_id)`（连通分量 + 走廊分支）。挂接成功的簇**按拓扑键分组**形成区域；未挂接的簇保留旧的纯几何分块回退（`partition_frontier_clusters_by_topology`，`frontier_regions.py:297`）。这样区域天然沿走廊切开，不再依赖 8 m 网格近似。

`PersistentRegionTracker` 用 IoU 重叠 + 质心距离做全局最优匹配维持持久区域 ID：承诺区域优先挑选后继，拆分/合并不再轻易换 ID。挂接失败的旧簇对保留成对稠密 A* 连通性检查作为回退，拓扑不完整时不会凭空合并区域。

### 5.3 稀疏路由：估价可以稀疏，发布必须稠密

`SparseRouteGraph`（`explorer_core/sparse_routing.py`）消费 `TopoGraphDelta`，维护节点 + 边折线的空间桶索引。查询点按 `sparse_attachment_radius=1.5 m` 挂到附近节点或边中段采样点；一次多源 Dijkstra 出全部候选的距离和转弯代价（`batch_estimates_with_reasons`），未挂上或断连的候选带 miss 原因（`start_unattached / target_unattached / disconnected / connector_rejected / blocked_disabled`），回退一次稠密最短路树。

关键约束：稀疏结果"从不权威"（`frontier_explorer.py:1441`）。发布前把稀疏折线逐段 bresenham 化并整段验证（`materialize_sparse_candidate`），失败回退完整稠密 A*；某区域全部候选在稠密图上失败时，该区域在本 revision 被标记为稠密不可达（`dense_invalid_region_revisions`），若是承诺区域则直接释放承诺。

### 5.4 区域承诺：滚动单步决策 + 残值门控

当前区域选择是**滚动单步**的：用 `region_information_efficiency`（信息/行程比）给每个区域打分，当前承诺区域除非被超过 `region_switch_ratio=1.35` 倍的挑战者超越，否则保持（`select_rolling_region`）。开放 Held-Karp（≤ `max_global_regions=10` 时）只做背景预测游序，用于连续准备；它的重算由输入签名（区域集合、量化机器人位置、路由约束 revision、拓扑结构）触发，且首项被强制对齐滚动决策（`frontier_explorer.py:1777`）。

承诺释放有两条独立路径：

1. **存在性防抖**（`update_region_commitment`）：frontier 连续 `region_release_updates=3` 拍消失，或连续 `region_unreachable_timeout=5.0 s` 不可达，释放；
2. **残值门控**（`ResidualCommitmentGate`，`region_commitment.py`）：以地图 content revision 为时钟（同一 revision 重复评估幂等），剩余观测单元 ≤ `region_min_remaining_cells=24` 持续 `region_exhausted_revisions=3` 个 revision（残值耗尽），或剩余单元 > 24 但 `region_stagnation_revisions=20` 个 revision 无进展（残值停滞），释放。

观测目标单元集合在任务激活时冻结，此后分母不变，进度比例才有意义（`observation_target_cells` 的文档注释）。frontier 增长被明确视为新工作而不是停滞证据（`region_commitment.py:71-74`）。

### 5.5 信息任务完成与连续准备

选择观测任务时冻结 frontier 周围 `observation_radius=3.0 m` 内的 unknown 单元集合。有效观测完成度达到 `observation_prepare_ratio=0.60` 时准备下一个候选（不发布路径）；达到 `observation_done_ratio=0.80` 且连续 `observation_done_updates=3` 次地图更新满足时完成当前任务并交接。机器人可以在真正走到观测位姿之前切换任务；到达位姿但信息提升不足时任务不能被误判为完成——SCAN 先到会报 `REACHED`，Explorer 只在增益确认后交接，否则拉黑该目标并原子激活预备路径。

交接时从机器人最新位置拼接预备路径仍有效的后缀（`splice_prepared_path`），失效则退化为一次点到点稠密搜索。任务完成后允许跨区域转移承诺（`activate_prepared_observation(allow_commitment_transfer=True)`，日志 `[REGION_COMMITMENT_TRANSFER]`）；任务未完成时跨区域激活仍被拒绝（`[PREPARED_HANDOFF_DEFERRED]`）。

### 5.6 与 SCAN 的请求握手

Explorer 一次只允许一个 `initial_path` 请求处于在途状态。request id（纳秒时间戳）编码在路径 header.stamp 中，SCAN 侧校验单调性，旧请求的迟到状态被丢弃（`[STALE_PLANNING_STATUS_IGNORED]`）。Explorer 收到 `PATH_TRAJECTORY_READY`、`BLOCKED`、`REFERENCE_PATH_REJECTED` 或 `INVALID_REFERENCE_PATH` 后才结束本次握手（`[PATH_REQUEST_IN_FLIGHT]` 期间不发新请求）。

### 5.7 全局重规划节流

`ReplanGate`（`explorer_core/replan_control.py`）以上下文 `(robot_cell, map_content_revision, route_constraint_revision, retry_epoch)` 为键，同一上下文最多 `max_replans_per_context=2` 次，超出打印 `[GLOBAL_REPLAN_SUPPRESSED]`。retry_epoch 每 `replan_retry_updates=10` 次地图更新递增，保证长时间无进展时仍会重试。

## 6. SCAN 当前在做什么

SCAN 接收 Explorer 给出的折线路径，按路径弧长向前选择局部目标（`planning_horizon=7.5 m`，`reference_path_lookahead=4.5 m`；弧长可以避免相邻走廊之间跨墙跳目标）。A* 或已有路径信息产生初值，三阶 B 样条 rebound 优化（平滑 + 碰撞 + 可行性，L-BFGS），时间重分配后发布。

闭环控制器是 cmd_vel 层 P 控制（位置 + 航向）；航向偏差 > 0.8 rad 时发布 `go2_execution_frozen=true` 冻结轨迹时间、原地转向。新轨迹交接时使用机器人最新前向位置做匹配校验，不回放旧路径起点。

一次 rebound 优化的耗时量级仍是秒级（约 1–10 s，视场景），多线程化提高的是规划期间的感知与安全响应，不是优化本身的速度。

## 7. 系统级安全链路（自 e874635 起保持稳定）

规划、地图融合和安全检查分别位于独立 callback group（`scan_planner_node.cpp`，4 线程 MultiThreadedExecutor）。规划读取固定的膨胀地图快照；地图线程持续融合；安全线程以最新实时地图检查真实位姿、位姿到轨迹的连接段和剩余 B 样条。

规划完成后先用最新地图复查整条 B 样条和连接段，规划期间发生安全否决则结果被丢弃（`[PLAN_RESULT_DISCARDED]`）。实时否决是锁存状态：只有新目标或新参考路径被明确接收后才释放（`[SAFETY_LATCHED]` → `[SAFETY_RELEASE]`）。5 次连续重规划失败进入 EMERGENCY_STOP。

## 8. 局部阻塞反馈的边界

SCAN 否决轨迹时发布 `BLOCKED` 和可选 `planning/blocked_segment`。Explorer 把阻塞映射到当前全局路径的一条有向边，`blocked_edge_ttl=30 s` 内禁止该边；反方向不自动永久屏蔽。若 Explorer 自己能立刻验证并重算出仍指向当前观测位姿的新路径，会走 `reroute_active_goal`（不重置信息增益，状态 `ACTIVE_GOAL_REROUTED`）。临时阻塞不应屏蔽整个目标、区域或大片侧空间，否则窄走廊会被过度封死。

## 9. 关键参数（与 `quad_exploration.launch.py` 一致）

```text
Explorer resolution             0.20 m
Explorer mapping_range          7.5 m
Explorer no_return_range        3.0 m
Explorer ray_count / dilation   720 / 3 bins
Explorer obstacle z band        [0.08, 0.85] m（planar，绝对高度）
                                [-0.30, 0.50] m（physics，相对机体）
Explorer inflation_radius       0.65 m
viewpoint_standoff              1.0 m
region_size                     8.0 m（仅未挂接簇的几何回退用）
region_switch_ratio             1.35
region_release_updates          3
region_unreachable_timeout      5.0 s
region_min_remaining_cells      24
region_exhausted_revisions      3
region_stagnation_revisions     20
max_global_regions              10
sparse_attachment_radius        1.5 m（limit 8 个候选）
observation_radius              3.0 m
observation_prepare_ratio       0.60
observation_done_ratio          0.80
observation_done_updates        3
map_update_period               0.5 s
planning_period                 0.25 s
goal_timeout                    120 s
blocked_edge_ttl                30 s
max_replans_per_context         2

SCAN map resolution             0.05 m
SCAN rolling window             10 × 10 × 5 m
SCAN local patch                0.20 m 切片，0.5 s 周期，z=0.3
SCAN planning horizon           7.5 m
SCAN reference lookahead        4.5 m
SCAN max velocity               0.75 m/s
SCAN max acceleration           0.5 m/s²
```

地形场景（`roboterrain_inspection_exploration.launch.py`）地图为 68 × 104 m，`physics:=true` 时 z 向 14 m、使用 Gazebo 物理仿真和 heightmap 世界，`physics:=false` 时使用 planar PCD 的确定性仿真。

## 10. Debug 日志读法

Explorer 侧：

- `[GLOBAL_PLAN_BASELINE]`：每次全局规划的完整耗时分解（map_preprocess / region_partition / sparse_candidate / candidate_tree / sparse_region / region_sequence / total）以及稀疏命中、回退和 miss 原因计数。调优先看这里。
- `[ROLLING_REGION_CHOICE]`、`Region sequence: [...]`：滚动选择与预测游序。
- `[REGION_COMMITMENT_RELEASED]`（附 reason）/`[REGION_COMMITMENT_RETAINED]`/`[REGION_COMMITMENT_TRANSFER]`：承诺生命周期。
- `[SPARSE_ROUTE_REJECTED]`：稀疏候选未通过稠密终验。
- `[PATH_REQUEST_IN_FLIGHT]`、`[STALE_PLANNING_STATUS_IGNORED]`、`[LOCAL_TRAJECTORY_READY]`（含 pipeline_latency 与 EWMA）：握手与延迟。
- `EXPLORATION_COMPLETE` / `NO_REACHABLE_FRONTIER` / `WAITING_FOR_PLANNING_INPUT_CHANGE`：无目标的三种原因。
- `[GOAL_FAILURE_COOLDOWN]`、`[PHYSICAL_COLLISION]`（锁存，不可恢复）。

SCAN 侧：

- `[PLAN_TIMING]`：本轮耗时、快照 revision、完成时 live revision、地图年龄。规划期间 live revision 持续增长才证明地图未被优化阻塞。
- `[SAFETY_SCHEDULER_LAG]`（>150 ms）、`[SAFETY_MAP_STALE]`（>250 ms）：性能告警，不是停车结论。
- `[REALTIME_SAFETY_STOP]`：真正的停车，区分 `pose_occupied` / `connector_blocked` / `trajectory_blocked`。
- `[SAFETY_LATCHED]` / `[SAFETY_RELEASE]`：否决锁存与解除。
- `[REFERENCE_PATH_ACCEPTED]`、`PATH_ACCEPTED` / `PATH_TRAJECTORY_READY` / `REACHED` / `BLOCKED` / `REFERENCE_PATH_REJECTED` / `INVALID_REFERENCE_PATH`（均带 request_id）。

global_representation 侧：`[TOPOLOGY_SHADOW]`（recall、cost_error、节点压缩率、处理耗时）、`[SAFE_REGION_SHADOW]`（仅指标，见下）。

## 11. 已知问题与边界（2026-09-13）

1. **双/三地图表示只在发布时刻对齐**。Explorer 稠密图与 SCAN 派生的拓扑图在更新时机、分辨率和膨胀上均不同，靠"稀疏只估价、稠密终验"的规则兜底。分析"蓝线对、红线撞墙"类问题时先怀疑两图时差。
2. **safe_region 影子是纯开销**。`safe_region_graph.py` 的区域/门户提取与 oracle 只产出指标（`[SAFE_REGION_SHADOW]`），没有任何规划消费者。默认 `safe_region_shadow_enabled=true`，做长时间实验时可关闭。
3. **性能热点在 Python 逐像素循环**：骨架化与 clearance 场每 patch 全量重算（`topology.py:45-111`）；`inflated_obstacles` 每规划拍重建全图膨胀集；未挂接簇回退的成对 A* 在拓扑覆盖差时会放大。诊断工具 `sparse_graph_diagnostics.py`（未入库）可从 CSV 指标出图。
4. **地形能力缺失是表示问题不是参数问题**。Explorer 的 z 带投影把斜坡、台阶和墙变成同一种占据格；物理仿真模式下相对机体的 z 带可以适配缓坡，但全局路由没有可通行代价。不要期望靠调 frontier 权重"调出"地形能力。
5. 到达观测位姿但增益不足时目标会被无条件拉黑（`REACHED` 分支），超长走廊场景可能误杀，尚未验证。

## 12. Benchmark 与评价方法

固定地图与起点，比较不同 `selection_strategy`（`greedy` / `explorer/hierarchical`）或参数下的运行结果。`metrics_timer`（2 s 周期）发布 `/explorer/metrics` 并写 CSV（`metrics_file`），字段包括 coverage、known_cells、frontier/region 数、total_distance、revisit_distance、region_switches、commitment 状态与释放原因、goals_reached、paths_published、short_paths_published、sparse 命中统计等。规划耗时看 `[GLOBAL_PLAN_BASELINE]`，蓝线到红线延迟看 `[LOCAL_TRAJECTORY_READY]`。

主要指标：最终覆盖率、达到给定覆盖率的时间、总行程、折返距离、重复经过率、区域切换次数、局部 `BLOCKED` 次数、物理碰撞次数、规划耗时、pipeline latency。规划成功率不能代替物理碰撞统计。

## 13. 下一步优先级

1. 在 quad 迷宫对 `56f4b44` 形态做一次基线实测：确认稀疏命中率与 fallback 原因分布、`[GLOBAL_PLAN_BASELINE]` 各阶段耗时、长时运行的承诺释放频率。后续一切优化以此为对照。
2. 定义地形可通行性作为 Explorer 的新输入：在 2D 栅格上叠 per-cell 通行代价（点云 z 分布或高度图导出），A* 与稀疏拓扑边代价共用。先在缓坡 world 验证"路由主动选坡道绕台阶"。
3. 处理 Python 性能热点（骨架化/clearance 的 numpy 化、inflated set 复用），用基线数据决定优先级。
4. 清理：`safe_region_graph` 与 `global_astar_planner.py`（仅 quad_benchmark.launch.py 在用）二选一处置；决定 `quadruped_exploration_planner/` 嵌套仓库去留；将未跟踪文件（两份设计文档、`sparse_graph_diagnostics.py`、地形场景）提交或移除。

## 14. 构建、测试和启动

```bash
cd /home/t1an/ros2_ws/scan_planner_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select plan_env scan_planner --symlink-install
source install/setup.bash
```

平地自主探索：`ros2 launch scan_planner quad_exploration.launch.py`  
地形场景：`ros2 launch scan_planner roboterrain_inspection_exploration.launch.py physics:=true`  
确定性仿真 + 手动目标：`ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=1 sensor_type:=lidar controller_mode:=closed_loop use_gpu:=false`

Python 测试（8 个文件，在 `src/planner/plan_manage/test/`）：

```bash
cd /home/t1an/ros2_ws/scan_planner_ws/src/SCAN-Planner
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
python3 -m pytest -q src/planner/plan_manage/test \
  -o cache_dir=/tmp/scan_planner_pytest_cache
```

注意：`colcon test` 直跑可能因用户目录 `anyio` pytest 插件与系统 pytest 不兼容而在收集阶段报 `No module named '_pytest.scope'`，这是环境插件冲突，不是项目测试失败。

## 15. Git 与仓库状态

```text
56f4b44  feat: execute sparse routes with dense connectors   ← HEAD
68e1f2a  feat: report sparse routing fallback reasons
50ebd45  fix: transfer region commitment at completed handoff
1260e27  fix: keep global handoff off sparse false positives
d979724  feat: attach frontier regions to sparse topology
c3c63fc  feat: gate region handoff on residual exploration value
fdf9484  feat: route global exploration over sparse topology
e874635  fix: keep mapping and safety responsive during planning（旧交接文档基线）
```

未跟踪内容（提交前需逐项决定）：`docs/ATOMIC_HANDOFF_DESIGN.md`（已被取代的设计提案，见其状态声明）、`docs/OPTIMIZATION_PROPOSAL_EXPLORER.md`（未实现的优化 backlog）、`docs/LOCAL_MAP_TOPOLOGY_INTERFACE_ZH.md` 与本文档的重写版、`sparse_graph_diagnostics.py`、`roboterrain_inspection` 场景文件、`quadruped_exploration_planner/`。

`quadruped_exploration_planner/` 是嵌套独立 Git 仓库（RAEM 思路、ATSP+Greedy 全局规划器脚手架），目前未被任何 launch 或代码引用。上传前必须决定保留为独立仓库、正式 submodule，或删除嵌套 `.git` 后纳入主仓库；不要在未决定前直接 `git add .`。

## 16. 给新会话的建议开场

> 请完整阅读 `/home/t1an/ros2_ws/scan_planner_ws/src/SCAN-Planner/docs/PROJECT_HANDOFF_ZH.md` 和 `docs/ARCHITECTURE_SIMPLE_ZH.md`，再检查当前仓库与最近提交。当前形态是"稀疏拓扑路由 + 滚动区域承诺"。不要重做已完成修改，不要扩大到三维地形、RL 或新的安全层。先做基线实测（稀疏命中率、GLOBAL_PLAN_BASELINE 耗时分解），再考虑地形可通行性输入或性能热点。任何代码修改前先解释数据流和预期影响。
