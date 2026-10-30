# Global 空间记忆观察模式（2026-10-29）

2026-10-30 已分离凸支撑域与空间身份，并补齐净空来源、连接压缩及被压缩域的接入。具体实现、接口字段及表达限制见 [Global 几何语义](GLOBAL_GEOMETRY_SEMANTICS_ZH.md)。[无 Vision 的 Global 设计](GLOBAL_NO_VISION_DESIGN_ZH.md) 中的全链路增量发布与 Explorer 替代仍未完成。

默认继续使用 `shadow`：Global 独立建图和显示，Explorer 不订阅它的 topology delta/snapshot、safe-region snapshot 或转发 failure evidence。Local、消息定义、控制器、恢复、步态与模型均不在本次修改范围内。

```bash
cd ~/ros2_ws/scan_planner_dyn_ws
./run_newmine_rl.py --duration 0 --gui --show-rviz --global-mode shadow
```

原指令的世界、出生位姿、地图大小、domain 参数继续保留。旧指令默认也是 shadow。`off` 不启动 Global，可作为资源开销对照；`route_cost` 是显式接回消费者的实验模式，本轮不默认启用。逻辑隔离不能保证共享 CPU 不受影响。

## 公开节点不再等同于计算 tile

内部保留世界坐标下 2m tile 的 FREE 连通分量与波前见证。`SpatialDomainView` 是这份细图的派生查询视图，不拥有另一张 occupancy。相邻分量只有在合并的整数格凸包已完全由它们的 FREE 格覆盖时才合并，不填 UNKNOWN 或占据洞。

机身 Z 起伏、实际转向和重复经过只更新经验，不移动细图种子、不强制保留拓扑节点。公开锚点按覆盖、几何分叉、瓶颈或接口保留；准入原因随 `spatial_supports.domains[].reason` 发布。这些是几何保守依据，不是语义厅室或完备路口识别标签。单次合并在 X/Y 方向以 20m 为计算范围界限，大空间仍可有多个必要覆盖锚点。

相关地图 tile 改变时，仅重建其空间域及邻域、相关边与依赖；无关域保留原对象和身份。边保留实际 FREE 路径、长度和最小净空，局部可替代的自由空间连线可移除。障碍/未知洞两侧的真实替代路线不能被压成无法命名的同端点平行边。被剪边的替代依据变化时会重新审查。

`SparseRouteGraph` 通过已知空间域接入锚点，取代“必须在固定 3m 内找到骨架”的覆盖要求。远距离接入必须由查询方再次检查实际地图；域归属不授权运动。一个空平台可以只有一个节点、没有边，域内可见查询走直接连接而不绕锚点。旧的近节点/边采样接入继续支持缺少域数据时的情况；预算或证据不足不会被当成已知可达。

压缩掉的锚点通过 `edge_attachment` 对应 Edge 折点，单 tile 非凸域可携带原有波前父树见证；接入不新增栅格搜索。域的世界格行区间和身份通过现有 `safe_region_snapshot` JSON 的 `spatial_supports` 字段传输，没有修改 ROS 消息或 Local 接口。快照可先于图到达，使用时必须匹配当前节点的锚点；旧 map epoch 的域不能复活。Local 地图重启强制发布完整图快照。

## 稳定地面颜色与动态信息边界分开

`/global_representation/graph_markers` 仍是 MarkerArray，RViz 显示名改为 **Global Spatial Memory (Observation)**。新增 `spatial_domains` namespace，表示稳定空间地面填色；`spatial_nodes`、`spatial_edges` 保留锚点与粗连接。颜色绑定独立的 `place_id`，相同空间可以非凸，多个路线支撑域可以共享同一地面颜色；覆盖范围或新分叉要求增加锚点时不重染旧平台。扩展保留旧身份，真实连通分裂最大重叠的一支继承旧身份，其他部分分配新身份。Frontier 移动、满足、换 ID、换 Goal 不能改变空间颜色。

填色按连续矩形合并，复用未改变的三角形几何；矩形只覆盖实际 FREE 格。它仍是固定高度的 XY 投影，不是世界地面高程，也不是身体可通行认证。没有测量支持的地方不绘制虚构地面。

`/explorer/region_overlay` 改为 **Explorer Information Decision**。`information_regions` 只画动态信息边界点，继续保留候选、准备/活动目标和访问序列；删除按当前 Frontier 对全片 FREE 地面做 BFS 的填色入口。信息任务 ID、完成账、评分及承诺不因显示稳定性而修改。若使用自定义 RViz 配置，启用新 namespace；当前节点不支持热替换 Python，正常结束本轮后重新启动。

## 资源与边界

每 0.5s 处理一个最新 patch、相同内容去重、每 2s 的属性年龄刷新和低频恢复快照继续保留。默认仍关闭 topology oracle 与旧 safe-region shadow 重建。Global 不额外膨胀 raw occupancy，也不复制原始点云。

地形继续消费已有 Local 地面摘要，通过已有属性链给节点/边附着软估价。合并域的节点聚合其覆盖的地形 tile，避免只看锚点周围而漏掉域内事实。本轮没有增加 IMU 建点阈值、重新实现通行能力或按坡度硬删边，也没有新增返航、历史轨迹和瓶颈等级显示。

验收包括平地机身起伏不增点、平台远角接图、障碍两侧路线、墙体分裂/重新连通、局部更新保留远处身份、快照乱序及 map epoch、Frontier/Goal 不影响地面颜色。性能比较使用同机离线长图和一份已录制 Campus patch；它不能替代长程 Gazebo 或边缘设备运行验收。此次修改不承诺解决 Local 既有停车或恢复问题。
