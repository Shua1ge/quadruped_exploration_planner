# Frontier 延续归属：只读离线对照工具

工具为 `tools/region_growth_audit.py`，测试为 `tools/test_region_growth_audit.py`。不需要ROS节点或Gazebo，无话题发布、服务调用、运动命令和生产地图写入。只读取输入JSON和dyn源码，按命令指定的位置写JSON／Markdown报告；导入时禁止Python写pycache。

另提供独立现场记录器 `tools/record_region_growth.py`：Campus正常运行后，在第二个终端订阅既有话题，保存真实发布的Region／Goal／任务／反馈／地图过程。记录器不重新融合点云、不运行分组和规划，也不向现场发布消息。它与离线演示工具用途不同。

## Campus 运行中，另开终端记录

```bash
cd /home/t1an/ros2_ws/scan_planner_dyn_ws/src/SCAN-Planner
source /opt/ros/humble/setup.bash
source /home/t1an/ros2_ws/scan_planner_dyn_ws/install/setup.bash

region_capture_dir="/home/t1an/ros2_ws/scan_planner_dyn_ws/run_records/region_$(date +%Y%m%d_%H%M%S)"
env PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  nice -n 10 python3 tools/record_region_growth.py \
  --domain 148 --duration 0 --output "$region_capture_dir"
```

`--domain`必须与Campus启动命令相同；不传时沿用环境ROS_DOMAIN_ID。`--duration 0`持续到Ctrl+C，也可设成300记录五分钟。结束这个记录器只关闭它自己的订阅和文件，不控制仿真。输出目录必须是新目录，避免覆盖旧记录；父目录由脚本建立。每五秒打印收到的地图增量、Region帧数和实际发布ID；一直是零时应先核对Domain和话题。

记录内容为 `bag/`（标准rosbag2 sqlite3，保存原消息）、`events.jsonl`（小型时间线摘要）和 `session.json`（计数、主题类型、采样及数据缺口）。原消息的capture/header时间保留，bag按记录器接收的墙钟纳秒排序，events另存观察到的/clock和本机monotonic时间。默认位姿最多5Hz、clock最多2Hz，其他已收到的话题原样保存；主动采样计入deliberately_skipped，不能当作DDS丢包数。

默认记录：`/explorer/map`、`/map_updates`、`/explorer/frontiers`、`/explorer/region_overlay`、`/explorer/region_sequence`、`/explorer/goal`、`/explorer/path`、`/explorer/status`、`/explorer/metrics`、`/observation_task`、`/planning/observation_feedback`、`/planning/status`、`/planning/trajectory_ack`、`/planning/local_execution_event`、`/quad_0/body_pose`、`/clock`。没有默认记录大点云或膨胀的planning_map。

实际Region ID来自已发布的information_regions Marker；ObservationTask只说明发布意图，不自行认定已安装／执行，必须结合ACK和反馈。Marker不是严格与某次融合revision同步的内部tracker快照，因此也不假装具有这样的同步关系。

现场记录的数据边界必须保留：原始完整地图目前只首次发布，后续是增量，晚加入可能漏过早期增量；当前公开话题也没有逐格实测FREE与原点补FREE的完整区分。记录器把raw_map_history_complete和measured_free_provenance_available标为false。可以立即采集真实Region变化和任务交接，但不能仅靠这些话题声称已经获得严格的实测生长证明。

这份bag不是离线 `--input` 所要求的富来源快照JSON，不能直接把session.json传给它。精确延续回放仍需源端提供实测自由来源和完整一致地图；记录器不从点云另造一套融合来补出这些事实。

当前用途是实施第一步：可靠提取快照间的变化，并检验“旧工作→实测新增FREE→新Frontier”的延续依据。它没有实现新RegionRegistry、正式分裂合并、工作完成判定或新的Goal选择策略。也不把这份证据当成Local的机身可行性认证。

## 运行

从仓库根目录执行：

```bash
cd /home/t1an/ros2_ws/scan_planner_dyn_ws/src/SCAN-Planner

env PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  nice -n 10 python3 tools/region_growth_audit.py --demo \
  --output /tmp/region_growth_audit/result.json \
  --write-demo-inputs /tmp/region_growth_audit/inputs
```

同时生成 `/tmp/region_growth_audit/result.md`。只运行某一个输入：

```bash
python3 tools/region_growth_audit.py \
  --input /tmp/region_growth_audit/inputs/open_advance_8m.json \
  --output /tmp/region_growth_audit/open_result.json
```

运行回归：

```bash
env PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 -m unittest discover -s tools -p test_region_growth_audit.py -v
```

从其他目录保存的工具副本运行时，用 `--source-root /path/to/SCAN-Planner` 明确指向生产源码。依赖只使用仓库已有的NumPy／SciPy和Python标准库。单次调用只比较一个源码根目录。

## 两侧分别计算什么

基线直接导入当前仓库的 `ExplorationGrid.frontier_cells`、`cluster_frontiers`、`partition_frontier_clusters_by_topology` 和 `PersistentRegionTracker`。所有topology key传None，等价于当前Global shadow下的分组入口；未加载ROS节点，不会执行候选站位、任务发布和Local规划。

基线按原规则逐帧更新。实验侧只查询上一帧基线Region的边界归属，给出当前簇的候选来源；建议不反灌基线，也不维护另一套会影响现场的身份状态。因此多帧回放是在逐次对照真实tracker历史，不是完整新Registry生命周期的反事实仿真。

实验侧过程：

1. 对同一固定坐标网格的连续快照生成before／after变化，保留Frontier出现、消失、实测UNKNOWN→FREE和没有实测来源声明的UNKNOWN→FREE。
2. 只在输入明确声明为本轮实测的UNKNOWN→FREE格中做四邻接连通传播。关联仍为FREE的旧Region边界，保留所有可能来源；输出完整自由格路径见证。已有边界重叠也作为单独依据。
3. 多个来源接入同一新簇时输出歧义。一个来源出现多个当前簇时，用现有 `KnownFreeSearch` 做有限的短连接核对；找到短路线只支持连续访问，没找到不证明空间分裂。
4. 输出当前tracker的归属、实验候选父级、依据、预算、工作跨度提示和分歧。活动需求只提供哪个子簇可能优先保留身份的只读提示，不更改任务。

## 输入格式

`--write-demo-inputs` 会导出完整样例，可以直接复制调整。结构为：

```json
{
  "schema_version": 1,
  "name": "recorded_sequence",
  "grid": {
    "width": 75,
    "height": 24,
    "resolution": 0.2,
    "origin": [0.0, 0.0]
  },
  "parameters": {
    "region_size_m": 8.0,
    "region_match_distance_m": 6.0,
    "min_frontier_size": 6,
    "growth_budget_cells": 20000,
    "continuity_budget_cells": 2000,
    "continuity_max_m": 12.0,
    "work_span_m": 12.0
  },
  "frames": [
    {
      "revision": 1,
      "epoch": 0,
      "capture_stamp_ns": 1000000000,
      "evidence_source": "capture-time measured-free export",
      "occupancy": [],
      "measured_free_cells": [[14, 3]],
      "active_region_id": null,
      "active_demand_cells": []
    }
  ]
}
```

上面的occupancy空数组只是格式占位，不能作为有效输入运行；使用导出样例获得完整数组。occupancy接受height×width二维数组或按行展平数组，取值限定为-1/0/100。坐标使用[x,y]，occupancy访问为[y][x]。所有快照使用相同分辨率、尺寸和世界原点；不支持把移动窗口的相同索引直接当相同世界位置。

`revision`是本次回放的快照序号，在同一epoch中严格增加；不是要求每个新传感器快照都改变occupancy。epoch变化会清除上一世界的关联，不能跨仿真重置继承旧来源。

`measured_free_cells`是本轮或两个快照间实测自由证据的显式来源声明，不是“所有目前为FREE的格”。工具只将其中确实发生UNKNOWN→FREE的部分用于生长传播；未提供或为空时不猜测。工具信任该输入声明，不能自行证实它来自真实传感器。原点附近补FREE、地图重置、坐标重新投影等必须排除。多个融合帧合为一个回放快照时，需要保留这段时间真正对应的实测来源，不能把最终所有FREE都填进去。

当前OccupancyGrid以及一份LocalMapPatch本身不能提供完整的时间延续与实测来源。独立现场记录器保存现有公开话题，但没有改融合节点以导出该字段。已有旧日志若没有这些信息，只能作为缺少来源的对照，不能伪造成有实测证明的回放。

`active_demand_cells`可填活动任务关联的边界／目标见证；只输出当前簇与它们重叠或四邻接的子簇编号。没有匹配时不按最近距离强行选择继承者。

## 输出判读

| 状态 | 含义 |
|---|---|
| NEW_WORK_CANDIDATE | 冷启动／没有旧工作时，可进入新工作创建评估；工具不分配正式新ID |
| CONTINUITY_SUPPORTED | 有重叠或实测自由路径支持的候选父级；不授权合并／执行 |
| AMBIGUOUS_MULTIPLE_PARENTS | 同一新簇有多个旧工作来源，不按遍历顺序抢占 |
| REVIEW_MULTIPLE_CONTINUATIONS | 同一来源有多支延续，未证明能短距离连续访问；不等于已证明T字分叉 |
| NO_PARENT_EVIDENCE | 没有足够的旧来源依据；不表示不可达，也不自动永久等待 |
| DEFERRED_GROWTH_BUDGET | 新FREE传播预算耗尽，没有把不完整结果当唯一归属 |
| DEFERRED_CONTINUITY_BUDGET | 短连接搜索容量不足，没有把它当分裂／无路依据 |

`baseline_identity_disagreement`只在有单一、未被多支检查挂起的候选父级，而当前tracker给出不同ID时为true。其他歧义不强行算“旧实现错误”。`workload_review`是当前单簇代表边界点间最大直线距离超过实验阈值的提示，是工作范围的下界，不是完整Region路线直径，也不自动拆分。12m等实验阈值可改，不是正式能力或世界尺度定义。

`delta.cell_transitions`保留每个变化格的before／after与输入实测声明；`witnesses`保留候选来源及路径；`continuity_checks`保留短连接的实际检查结果。基线与实验侧计时覆盖范围不同，不能据此给出整个规划器的加速倍数。

## 八个构造回归场景

开放边界推进8m：当前tracker因匹配距离换ID，实验侧能否保存有路径证明的旧来源；柱子断开Frontier：当前实现已经能保留归属，实验侧应同样保留，不能虚构基线失败；T字长支路：同一生长父级不能使两支被自动合并；两旧工作汇合：保留多来源歧义；冷启动：明确新工作候选；未实测FREE：不给生长证明；两种预算耗尽：输出各自待完成状态。

演示场景为了容纳窄分支使用min_frontier_size=2，其余默认接近当前源码参数。它们是构造测试，不是NewMine或Campus现场回放。回归还检查输入不变、证据路径不穿未知／占据、旧边界变占据后不得继续供证、相同地图重复扫描不制造生长、epoch隔离、输入遍历顺序以及基线不受实验预算影响。

本工具的已完成范围是“可靠变化记录+当前tracker与延续证据的只读对照”。真实分支确认、长期身份继承、需求账与调度替换留给下一阶段，不通过更多状态名称冒充已经实现。
