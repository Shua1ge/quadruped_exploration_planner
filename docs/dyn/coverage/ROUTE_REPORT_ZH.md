# Campus 室外观测路线（离线对照）

- 世界：`resources/worlds/campus_fortress.sdf`（collision mesh 1,929,134 三角形）
- 目标：从 spawn `(0,0,0.45)` 出发，用 MID360 观测**室外可达可行走地面**；离线生成近优覆盖路线，作对照基线
- 传感器：360° 方位、−7…+52° 俯仰、0.1–40 m、10 Hz；安装 `(0.187, 0, 0.0803)` m（相对 trunk/IMU）、pitch `+13°`（前下倾）；传感器离地约 0.530 m
- 机体约束：Go2 trunk `0.376×0.0935×0.114` m；净空 0.35 m（与现有 Local 自膨胀一致）、坡度 ≤30°、邻格高差 ≤0.20 m

## 结果

| 指标 | 数值 |
|------|------|
| 可行走面积（spawn 连通、室外地面） | 25,488 m²（0.3 m 格，283,201 格） |
| 路线长度（`route_on_graph.csv`） | **4,704 m**，3,884 点，起点 `(0.02,0.64)`，终点 `(−95.98,53.44)`（开环） |
| 严格覆盖（格心被 MID360 命中） | **95.53%**（270,550 / 283,201 格） |
| 0.3 m 容差覆盖 | **98.59%** |
| 0.6 m 容差覆盖 | **99.25%** |
| 未覆盖（严格） | 1,139 m²，2,261 个碎片，最大单簇 29.5 m² |
| 可覆盖上界（2 m 候选 × 360×32 射线） | 97.01%（更密射线会略高，故这是保守下界） |
| 路线采样点落在非可行走细格 | 0.68%（107 / 15,680，0.3 m 采样；2 段细网格 A* 未连通造成） |

对比：初始“视点 TSP”路线 3.55 km 覆盖 76.9%；120 个覆盖引导绕行点后 4.92 km（32 线口径 91.9%）；细网格重连后 4.70 km（64 线独立复验 95.53%）。

## 方法

1. **地面层**：每 0.3 m 格取近水平三角形（≤35°）高度的 10% 分位为地面，0.6 m 内缺格用最近地面补齐，3×3 中值抑噪；屋顶/树冠因高度不连续自然断开。
2. **可行走图**：以 spawn 为种子，8 邻接、高差/坡度限制、墙体在机体带（地面 +0.15…0.90 m）视为障碍；障碍与无地面格按 0.35 m 净空膨胀；保留 spawn 连通分量。
3. **MID360 模型**：按 `mid360.yaml` 生成 360×32（规划）/ 720×64（复验）射线，做 Embree BVH 求交；命中点落在可行走格且与地面高度差 ≤0.15 m 记为观测。近场盲区实测约 1.5 m，与下倾 13°、离地 0.53 m 一致。
4. **初始路线**：2 m 候选视点 → 贪心集合覆盖 → 1.2 m 粗网格测地距离 → 开环 TSP（最近邻 + 2-opt）。
5. **覆盖引导插入**：沿路线连续累计覆盖，反复插入“新增覆盖 / 绕行长度”最优的候选点（120 次）。
6. **清理与复验**：贴到最近可行走细格；细网格 A*（禁止穿角）重连每 4 m 航点；用 720×64 射线、1 m 采样独立重算覆盖（不复用生成阶段的 32 线结果）。

## 文件

| 文件 | 内容 |
|------|------|
| `docs/dyn/coverage/route/route_on_graph.csv` | 最终路线 `x,y,z`（推荐使用） |
| `docs/dyn/coverage/route/route_final.csv` | 贴格后、A* 前的路线 |
| `docs/dyn/coverage/route/route_improved.csv` | 120 次插入后的粗网格路线 |
| `docs/dyn/coverage/route/final_route.png` | 覆盖图：绿=已覆盖、红=未覆盖、白=路线、黄=spawn |
| `docs/dyn/coverage/route/final_metrics.json` | 参数、覆盖率、未覆盖簇、起终点 |
| `docs/dyn/coverage/route/validation.json` | 720×64 独立复验结果（含 0/0.3/0.6/0.9 m 容差） |
| `docs/dyn/coverage/route/clean_report.json` | 贴格与占用审计 |
| `docs/dyn/coverage/walkable.npz` | 地面高度、可行走/障碍掩膜 |
| `docs/dyn/coverage/achievable.npz` | 可覆盖上界掩膜 |
| `docs/dyn/coverage/visibility_spawn.json/png` | 单视点核验 |

## 复现

```bash
cd ~/ros2_ws/scan_planner_dyn_ws/src/SCAN-Planner
python3 tools/worlds/prepare_campus.py --download        # 官方 ZIP → STL/OBJ（只做一次）
python3 tools/coverage_route/mesh_probe.py               # 网格体检
python3 tools/coverage_route/build_walkable.py           # 可行走图
python3 tools/coverage_route/mid360_visibility.py        # 单视点核验
python3 tools/coverage_route/achievable_coverage.py      # 可覆盖上界 + 候选可见集
python3 tools/coverage_route/plan_coverage.py --spacing 3.0 --max-candidates 300 --target 0.90
python3 tools/coverage_route/improve_route.py --target 0.99 --max-inserts 120
python3 tools/coverage_route/clean_route.py
python3 tools/coverage_route/astar_clean_route.py
python3 tools/coverage_route/validate_route.py --route route_on_graph.csv --az 720 --el 64
```

依赖：`numpy scipy trimesh embreex`（本机已装）；Mesh 转换需 `assimp`、`ignition-common4-graphics`。

## 限制（会影响解释）

1. **离线 oracle**：路线假设 Campus 网格已知；不是在线探索策略，覆盖数是该路线的上界，不代表 Explorer 实际能做到。
2. **网格真值**：Campus 是扫描重建，存在孔洞、植被与薄结构；不可见区中有一部分是网格缺失而非真实遮挡。
3. **覆盖定义**：格心被命中为“严格覆盖”；0.3 m 容差更接近建图实际（一个点云点会覆盖邻格）。两种口径都已给出。
4. **射线模型**：规划用 360×32 规则栅格近似 MID360，真实传感器为非重复扫描；复验用 720×64，仍不是真实点云时序。尚未与 Gazebo 实采点云逐射线对拍。
5. **非动态认证**：路线在 30° 坡度、0.2 m 台阶、0.35 m 净空的几何图上生成，未经过 Local 规划器/控制器、转弯半径、速度剖面的动态验证；0.68% 采样点仍在非可行走格（2 段 A* 失败）。
6. **开环**：终点不在 spawn，未加返程；如需闭环对照可追加返程段（会增加长度、几乎不增加覆盖）。
7. **无最优性证书**：启发式近优路线，未给 TSP/覆盖下界；4.7 km 不是可证明最短。

## 建议的下一步

1. 在 Gazebo 里按 `route_on_graph.csv` 回放（kinematic 或 open-loop），累计 `/quad_0/cloud`，与网格真值比较覆盖率；这是对射线模型和路线可执行性的第一道实证。
2. 用同一路线与 `frontier_explorer` 的实际探索对比：单位路程获得的信息量、覆盖率-时间曲线、重复经过比例。
3. 若要闭环/更平滑，加入返程与曲率约束，再重算覆盖（预计长度增加 5–10%，覆盖基本不变）。
