这样设计的目的，是把 SCAN 的局部高分辨率地图信息以一个轻量、可版本化的二维接口送到全局层，同时先保持现有 Explorer 和路径控制完全不变。

## 数据流

```text
SCAN 点云融合
    │
    ├─ 原有膨胀地图 / B 样条安全检查（行为不变）
    │
    └─ LocalMapPatch（低分辨率二维切片，带 map_revision）

            global_representation（独立影子节点）
                    │
                    ├─ 自由空间骨架化
                    ├─ 端点/交叉点与连通边提取
                    ├─ TopoGraphDelta（节点和边的增删改）
                    └─ dense A* oracle 指标
```

`global_representation` 不发布 `initial_path`，也不订阅或修改规划状态，所以本阶段不会改变 GO2 的目标、轨迹或安全回调。它的作用是建立并测量 Local Dense Map 到 Global Graph 的接口，等信息保持指标稳定后，才有依据决定是否让 Explorer 消费这张图。

## ROS 接口

`/grid_map/local_map_patch` 使用 `scan_planner_msgs/msg/LocalMapPatch`。栅格值为 `-1=unknown`、`0=free`、`100=occupied/inflated`，并携带地图 revision、原点、分辨率和生成耗时。

`/global_representation/topology_delta` 使用 `scan_planner_msgs/msg/TopoGraphDelta`。消息只发送当前 Patch 引起的节点和边的增删改，节点 ID 基于世界栅格坐标，滑动窗口移动后仍能保持一致。

`/global_representation/oracle_metrics` 是 JSON 字符串，关键字段包括：

- `connectivity_recall`：dense A* 能连通的抽样节点对中，拓扑图仍能连通的比例；
- `mean_cost_error` / `max_cost_error`：拓扑路径相对 dense A* 路径的代价误差；
- `patch_generation_ms` / `processing_ms`：SCAN 生成 Patch 和影子节点处理 Patch 的耗时；
- `dense_free_cells`、`local_nodes`、`local_edges`、`node_compression_ratio`：空间压缩规模。

## 负载控制

默认 Patch 是 10 m × 10 m 滑动地图上的 0.20 m 栅格，即约 50×50 单元；最多每 0.5 s 生成一次，QoS 队列深度为 1。Patch 在一次地图融合完成、地图仍处于一致状态时抽样，发布动作在地图锁释放后执行。实测 Patch 抽样约 0.8–1.7 ms；骨架、Delta 和最多 8 组 dense A* oracle 查询在独立节点中约 42–92 ms，不占用 SCAN 的地图互斥回调。

参数位于 `planner.yaml`：`grid_map.local_patch_enabled`、`grid_map.local_patch_period`、`grid_map.local_patch_resolution`、`grid_map.local_patch_z`。如果只测试原系统，可关闭 `grid_map.local_patch_enabled`；如果需要进一步降低负载，应先增大 period 或 resolution，而不是增加执行线程。

## 当前边界

这是最小二维接口，不包含 elevation、slope、roughness、terrain risk，也没有引入 RL 或新的安全层。当前 TopoGraph 仅表示已知自由空间的几何连通性和 clearance，dense A* 只作为影子 oracle。它暂时不会直接修复区域选择或短路径问题；下一阶段应先收集不同场景下的 recall、cost error、压缩率和耗时，再决定如何把 portal/connectivity 信息接入 Frontier Region 的分块与评分。
