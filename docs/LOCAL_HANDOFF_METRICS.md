# Local 接续修改与验收（2026-11-02）

本批以回退后的普通滚动为基线，速度上限、普通跟踪公式、物理恢复和安全制动模型保持原配置。需要验证的是正常接续的有效安装率与行走连续性。隔离进程回归通过不等于 Campus 实机理仿真已改善，仍需同配置长测。

## 接续边界

FSM 的执行快照只在精确 ACK 后继承候选的执行合同。合同包含模式、实测进度、速度/修正上限、最低步态命令及对应实测速度；端点制动约束按保守方向合并。曲线相同不代表机身命令相同。

普通→普通沿用原 C2 公共前缀。受限→受限沿用前缀和步态合同，仅新增尾段使用受限参考速度生成/必要时精化；禁止为降低尾段速度而拉长旧前缀的时间。完整足迹、转向扫掠和制动认证继续执行。若模式需要改变，使用当前实测入口重新认证，不声称继承另一种执行方式的前缀。

Controller 保留前任身份、存活状态、C2 重合、动态边界及证书检查，并比较安装时旧/新实际普通跟踪命令。合同变化拒绝为 `HANDOFF_EXECUTION_CONTRACT_CHANGED`；实际指令变化拒绝为 `HANDOFF_PREFIX_COMMAND_CHANGED`。拒绝候选后旧轨迹能否继续，仍由现有安全前缀与制动余量裁决，不直接把接续失败写成站位不可达。

跨 Goal 的未来入口最多比较三处；后两处沿旧轨迹按约 0.5m、1.25m 弧长前移，而非固定增加 0.2s 参数时间。每处仍受原几何、速度、加速度和域约束认证。新候选没有获准安装时，旧轨迹不会被候选覆盖。大转角或证据不足仍可能必须停车。

## 记录

沿用现有启动命令，并给本轮单独的 `--actual-route-csv`、`--metrics-csv`、`--log` 路径。另开终端运行现有只读记录器：

```bash
cd /home/t1an/ros2_ws/scan_planner_dyn_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
RUN_DIR='/你的本轮记录目录'
python3 src/SCAN-Planner/tools/record_region_growth.py \
  --domain 148 --output "$RUN_DIR/region_record" --duration 0
```

记录器新增保存 `/planning/handoff_metrics`、`/planning/execution_state`、`/planning/bspline`，并保留 ACK 的精确版本和拒绝原因。前者是低频 String JSON 事件，无消息 ABI 修改。记录器不发运动、暂停或取消命令。

`handoff_metrics` 的 `ATTEMPT`、`SUBMITTED`、`ACK`、`TIMEOUT`、`LATE_ACK` 区分生成、已发送、真实安装和未确认结果；`JOIN_SELECTION` 表示有界入口选择尚未找到可行连接。精确身份为 `(request_id, attempt_id, trajectory_id)`。每个记录目录对应一个 Planner 进程生命周期；重启后应新建目录。保留同任务/跨任务、来源/候选模式和公共前缀长度。生成与安装延迟使用 steady wall clock，运动数据使用仿真时钟。

## 分析

```bash
cd /home/t1an/ros2_ws/scan_planner_dyn_ws
RUN_DIR='/你的本轮记录目录'
python3 src/SCAN-Planner/tools/analyze_local_motion.py \
  --record "$RUN_DIR/region_record" \
  --route "$RUN_DIR/actual_route.csv" \
  --start 10 --output "$RUN_DIR/local_analysis"
```

结果为 `summary.json`、逐候选 `attempts.csv` 和逐停顿 `stops.csv`。`--start`、`--end` 是轨迹 CSV 的 elapsed_s，可固定相同时段比较；建议排除起身阶段。旧记录缺少 handoff 事件时仍可计算物理运动指标，安装延迟和安装率返回缺失值，不填写零。

重点比较以下口径：

| 指标 | 定义 |
| --- | --- |
| 有效安装率 | 精确 ACK 激活数 / 实际提交数；同时报告未结束提交，避免截断误读 |
| 安装样本 P95 | 只对实际激活的同一 attempt 统计；准备耗时与准备至 ACK 总墙钟耗时分别提供 |
| 实际平均 XY 速度 | 有效连续采样段 XY 路程 / 对应仿真时长 |
| 每百米长停 | 1s 窗口 XY 弧长速度低于 0.04m/s 且持续至少 3s；不把 FSM HOLD 直接当物理停车 |
| 停顿原因证据 | 停顿期间收到的执行状态 reason 分布，不自动宣称是根因 |
| 断流及截断 | 超过 0.75s 的仿真采样缺口或时钟回退切段；边界停车标记 censored |

由于一秒观测窗口，停车识别存在检测延迟；阈值和采样频率必须在对比中保持一致。只有一端截断的停车仍计数但单独标记。时间分布另外保留全部提交及失败样本，防止大量快速拒绝拉低总体 P50，却掩盖实际安装的 P95。

## 回归与范围

隔离测试使用 ROS domain 232，合成点云/里程计驱动真实 Planner、Controller 进程，不连接运行中的 Gazebo。覆盖普通滚动、同/跨任务拒绝保留旧轨迹、初始转向所有权、完整受限合同继承、模式转换需要实测入口，以及精确 ACK 指标。C++ 覆盖 C2 前缀、直行/缓弯/无证据反转入口和原跟踪命令等价性。

本批未实现新转向控制律、移动中观测合同、速度上限调整、物理恢复重构或 Global 接入。方向变化无法在剩余安全窗口内完成时，仍保留受控停止的退路。

## 2026-11-03：跨任务边界统一版

普通、允许局部几何调整的跨任务接续，先从 T1 的最早可用未来入口选择一次 G2 局部路线和终点意图，再比较至多三个未来入口。比较时只调整到同一终点的接入，使用已选终点速度；不再先假设零终速、再由另一次选择器改成通行终速。`PreparedRouteEntry` 保存请求、前任轨迹、来源/接入时刻、起始位置/速度/加速度、路线、目标、终速、域和地图 revision。生成直接消费该结果，最终认证仍使用新地图。

六条短五次曲线的连接探测仅提供初始化。探测失败但存在路线接入时，使用原 polynomial/A* 与 B 样条生成路径，起始导数、保留前缀、域代价及全部最终认证仍有效；不扩展速度/加速度阈值。没有路线接入、任务或前任变化时仍拒绝该候选。同任务滚动不经过新增目标预选流程。

新增日志 `TASK_JOIN_TARGET` 表示本次已确定的目标和终速，`TASK_BOUNDARY_CONSUMED` 表示生成消费同一边界；`TASK_ENTRY_GUIDE result=SPLINE_INITIALIZATION` 表示探测无初值、正在完整生成。`TASK_JOIN_OPTION.feasible` 仍表示短连接探测通过，`generation_eligible` 单独表示可进入完整生成。`TASK_JOIN_PROBE.feasible` 表示存在可生成的入口，必须结合 `guide` 解释，不能把它当最终可行或安装成功率。真实结果仍看 `SUBMITTED`、精确 `ACK` 和实际运动。

目标预选失败记为 `JOIN_TARGET:NO_ROUTE_TARGET`，无路线附着记为 `JOIN_SELECTION:NO_ROUTE_ATTACHMENT`，边界过期和路线改变分别记为 `BOUNDARY:STALE_PREPARATION`、`BOUNDARY:APPROACH_CHANGED`。目标预选时间计入总准备时长及 selector 计时。既有记录器和分析命令继续使用，验收仍比较全体准备尝试、跨任务提交/激活、安装样本 P95、每百米长停与实际平均速度。

隔离 ROS 回归增加“短连接探测无解但完整曲线获真实 Controller 激活”的用例；该用例仅在测试进程设置小的名义加速度以稳定触发无初值分支，不修改生产参数。保留共同前缀逐点相等、伪造前任证明被拒、受限模式继承和拒绝候选不停止安全前任的回归。Campus 的停顿与性能收益需要重启新节点实测。
