# 局部地图 → 稀疏拓扑接口说明

> 更新日期：2026-09-13，对应提交 `56f4b44`。
> 本接口已从"影子验证"阶段转正：`global_representation` 维护的稀疏拓扑图现在是 Explorer 全局路由估价的正式输入。仅 safe_region 影子指标仍处于"只采集、不消费"状态。

## 设计目的

把 SCAN 的局部高分辨率地图信息以一个轻量、可版本化的二维接口送到全局层。Explorer 的稠密栅格适合验证和局部搜索，但每次全局规划都在整张栅格上做多对多搜索代价高；稀疏拓扑图只保留自由空间的走廊结构（节点 + 带折线的边），让"从这里到每个候选大约多远"变成图上一次多源 Dijkstra。

## 数据流

```text
SCAN 点云融合
    │
    ├─ 原有膨胀地图 / B 样条安全检查（行为不变）
    │
    └─ LocalMapPatch（滑动地图的 0.20 m 二维切片，带 map_revision）
            │
            v
      global_representation（独立节点）
            ├─ clearance 场（8 邻域 Dijkstra，全 patch 复用一份）
            ├─ Zhang-Suen 骨架化自由空间
            ├─ 端点/交叉点为节点，度 2 链压缩为带 polyline 的边
            ├─ 与上一版图做增量对比 → TopoGraphDelta
            ├─ dense A* oracle 指标（recall / cost error）
            └─ safe_region 影子（tile 区域 + 门户，仅指标）
            │
            v
      Explorer 的 SparseRouteGraph 镜像
            └─ 点挂接（半径 1.5 m，节点或边中段采样）
            └─ batch_estimates_with_reasons 一对多估价
```

## ROS 接口

`/grid_map/local_map_patch`（`scan_planner_msgs/msg/LocalMapPatch`，BEST_EFFORT，depth 1）：栅格值 `-1=unknown`、`0=free`、`100=occupied/inflated`，携带 map_revision、原点、分辨率和生成耗时。参数在 `planner.yaml`：`grid_map.local_patch_enabled / local_patch_period (0.5 s) / local_patch_resolution (0.2) / local_patch_z (0.3)`。只测试原系统时可关闭 `local_patch_enabled`；降负载优先增大 period 或 resolution，不要加执行线程。

`/global_representation/topology_delta`（`TopoGraphDelta`，depth 10）：当前 patch 引起的节点/边增删改。节点 ID 由世界坐标量化打包成 uint64，边 ID 是无向节点对的 FNV-1a，滑动窗口移动后保持稳定。

`/global_representation/topology_snapshot`（`TopoGraphDelta`，RELIABLE + TRANSIENT_LOCAL）：每 `snapshot_period_revisions=10` 个图 revision 或首个 revision 发布全量图，用于 Explorer 启动竞态或重启后的原子重建（Explorer 侧日志 `[SPARSE_GRAPH_SNAPSHOT]`）。

`/global_representation/oracle_metrics`（JSON 字符串）：`connectivity_recall`（dense A* 能连通的抽样节点对中拓扑图仍能连通的比例）、`mean_cost_error` / `max_cost_error`（拓扑代价相对 dense A* 的误差）、`patch_generation_ms` / `processing_ms`、`dense_free_cells`、`local_nodes` / `local_edges`、`node_compression_ratio`，以及 safe_region 影子的采样指标。

## 负载控制

默认 patch 是 10 m × 10 m 滑动地图上的 0.20 m 栅格（约 50 × 50 单元），最多每 0.5 s 一个，QoS 队列深度 1。patch 在地图融合完成、地图处于一致状态时抽样，发布在地图锁释放后执行。实测 patch 抽样约 0.8–1.7 ms；骨架化、clearance 场、delta 和最多 8 组 oracle 查询在独立节点中运行（量级几十毫秒），不占用 SCAN 的地图互斥回调。

已知热点：骨架化与 clearance 场是逐像素 Python 循环，每个 patch 全量重算（`explorer_core/topology.py:45-111`）。地图变大或 patch 频率提高时先量化这两个函数的耗时。

## 当前边界

- 拓扑图只表示已知自由空间的几何连通性与 clearance，不包含 elevation、slope、roughness、地形风险。滑动窗口内看不到的长走廊在 patch 移出后由 Explorer 侧镜像保留（图是全局持久增量，不是单 patch 覆盖）。
- 稀疏路由结果不是执行保证：Explorer 发布任何路径前都会在自己的稠密栅格上整段验证，失败回退完整 A*（`frontier_explorer.py` 的 `materialize_sparse_candidate`）。
- safe_region 影子（区域 tile、门户、瓶颈识别、持久 ID、oracle recall）目前只产出指标，没有规划消费者；长时间实验可用 `safe_region_shadow_enabled:=false` 关闭。它是最接近"基于门户的区域构造"候选方向的现有代码，但决定启用前应先看其 recall/false-positive 指标是否稳定。
