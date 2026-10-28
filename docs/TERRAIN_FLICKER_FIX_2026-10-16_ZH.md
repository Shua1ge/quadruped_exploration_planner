# 地面假占据与 Z 轴标签闪动修复（2026-10-16）

现象：`/grid_map/body_obstacles`/占位表征即便允许修正，地面上的黑色占据块依然存在或不能及时
修正；Local 频繁规划短线、前方不允许通行。下面按"定位 → 改动 → 验证 → 残留"记录，所有数字
都来自实际运行或读取的日志。

## 定位：三层缺陷，不是雷达反射或 Explorer 快照

1. **分类器判据把粗糙地面判成凸起**。`observed_ground.hpp` 的 type 5 判据是
   `error < .50 && (sharp_rise(p) || error > .04)`：固定 4 cm 加上"邻居更低就算抬升"的
   `sharp_rise`，本帧局部支撑面拟合一直在动（slope 实测 0.006–0.047），矿洞地面的自身起伏
   正好在这个量级上反复踩线。实测（12:10，机器人静止，10 帧）：**14.2%（25/176）的 0.25 m
   网格在 type 3/5 之间来回跳**；凸起点集中在 |lat| 0.86–1.57 m 的墙根。
2. **Local 的滞后是不对称的锁存**（`plan_env/src/grid_map.cpp`）：危险帧 `++hits` 且把
   `support_frames` 清零，2 帧即投影到机体高度；清除要**连续 3 帧支撑**，抖动格的
   `support_frames` 被反复清零，`hits` 只增不减，`terrain_projections_` 没有任何时间衰减。
   一旦某两帧连续危险 → 机体高度上的投影**永久锁存**。Explorer 侧 `_apply_evidence` 是净值
   计数（+1/−1），对抖动本来就鲁棒，不是主因；但它吃同一批标签。
3. **观测盲区**：`[OBSERVED_GROUND]` 只打 ground/drop，最大的 type 5 没有计数；
   `[RAY_ENDPOINT_CHANNELS]` 没有活动投影数。这两类问题此前无法从日志直接看到。

## 改动

| 文件 | 改动 |
| --- | --- |
| `plan_manage/include/plan_manage/observed_ground.hpp` | 分类改两遍：先收集候选 `error`，用逐帧 MAD 估计粗糙度，抬升阈值 `rise = clamp(3σ, 0.06, 0.12)`；type 5 判据变为 `rise < error < .50`（`sharp_rise` 不再单独触发）；新增 `ObservedGround::rise/errors` 与 `applyTwoFrameRiseGate`（同一 0.25 m 世界格**连续两帧**才算危险证据；未确认且靠支撑面（≤ rise+0.04）的降为支撑，更大的降回普通波段，绝不降为清除） |
| `plan_manage/src/mid360_ray_bridge.cpp` | 接两帧门；日志加 `rise=/unconfirmed=/threshold=` |
| `plan_env/include/plan_env/grid_map.h` | `TerrainProjection{id, hits, last_scan}`；扫描计数器 `terrain_scan_`（`raycast_num_` 是 char，不能用于有序比较） |
| `plan_env/src/grid_map.cpp` | 有界对称泄漏：危险 +1、支撑 −1，上限 `occupied_miss_confirmation_frames_`；**`hits==0` 即释放**；`terrain_scan_` 超过 `terrain_projection_timeout_frames_` 未再观测到危险即衰减释放；日志加 `terrain_proj=/terrain_active=` |
| `plan_manage/config/planner.yaml` | 新增 `grid_map.terrain_projection_timeout_frames: 20`（约 2 s @10 Hz） |

## 验证

- **单元**（`test_observed_ground.cpp`，14 项全绿）：新增浅浮雕不成为凸起、粗糙地面抬高阈值
  但仍保留 20 cm 岩石、孤立抬升须连续两帧确认、被打断的抬升永不确认。
- **运行时**（`test_mid360_mapping_runtime.py`，10 项全绿，真实 `scan_planner_node` + 隔离域）：
  新增 ①危险只占 1/3 帧的格永不投影到机体高度（同帧中的持续危险必须投影，作为正对照）
  ②停止观测后投影必须在超时内释放。
- **反向消融**：把两个循环临时还原为旧语义（`hits` 单调、无衰减、无支撑回退）重建后，
  上述两个运行时用例**都失败**；恢复修复版后全绿。证明用例确实抓着缺陷本身。
- **全量 oracle**：`plan_env` ctest 1/1 ✓；`scan_planner` ctest 11/11 ✓（含 119 s 链路运行时
  套件）；pytest 25 个文件 527 项通过（`test_planner_startup.py`/`test_waypoint_parameters.py`
  需 launch_test harness，在 ctest 内为 Passed）。
- **实机**（无头 Gazebo 物理链路，隔离域 217，隧道姿势 (11.85,−0.28)）：分类器 `valid=1`，
  实测 `threshold=0.074–0.105`（旧代码固定 0.04），`rise=1519–1630/帧`、
  `unconfirmed=0–364/帧`（两帧门在实机生效）。闪动格 30/122，**全部在 |lat| 0.76–1.12 m 的
  墙根**（z−body −0.195..−0.042），走廊中央 |lat|<0.6：**0 格**。Local `/grid_map/occupancy`
  在走廊中央（|lat|<0.6、前方 0.5–4 m、|z−body|<0.15）**0 格**；Explorer 地图同区域 8 格，
  都在 |lat|≈0.5 的走廊边缘/真实几何处（前方 3.3–3.9 m 有 4 格，与修复前探针在同位置量到的
  16 格 inflate 是同一处）。

## 残留与限制（如实记录）

1. 墙根的 3/5 标签仍会抖动（含门控后）。它不影响走廊：中央 0 格，且墙体本身由 kind-1 原始
   占位挡住；泄漏滞后 + 衰减保证它不会锁存。若要把墙根标签也稳定下来，需要按"墙面"几何
   单独处理，不再属于本次范围。
2. 停留在出生平台时分类器 `valid=0`（平台面小、地面比机体低 0.15 m，支撑面锚定带排除），
   此时没有任何 Z 摘要标签，全部回落到波段逻辑；走进隧道后恢复。
3. 无头复测无法完成"行走中的 A/B"：出生平台处 Explorer 报
   `FRONTIERS_PRESENT_BUT_UNREACHABLE`（147 个前沿不可达、候选 0 → 机器人不出发）；把出生点
   直接放进隧道则触发既有身高契约 fault（body 0.361 超出平面映射契约、
   `KINEMATIC_POSE_INVALID_MAP_RESTART_REQUIRED`），感知链约 15 s 后 HOLD。两者都与本次改动
   无关，但使边走边测的实机对照无法在无头环境里快速完成；锁存本身已由运行时用例 + 反向
   消融覆盖。
4. 下一次 GO 正常运行的验收点：`[OBSERVED_GROUND]` 的 `threshold` 应随地面粗糙度落在
   0.06–0.12、`unconfirmed` 反映门控；`[RAY_ENDPOINT_CHANNELS]` 的 `terrain_active` 应只跟随
   机器人附近的真实凸起/落差，走廊中央不再出现持续黑块；同一路线下 `HOLD/REJECTED` 事件
   每分钟应少于 12:06 那轮。

## 追加：Explorer 不再用"地形"语义形成障碍（2026-10-16 晚）

架构判断：Explorer 是表征层，不应代替底层策略裁决"GO 能不能过去"，更不能因此在地图里造出
障碍。据此把 kind 4/5（落差/凸起 = 地形语义）从 Explorer 的障碍路径中彻底移除：

- `explorer_core/grid.py`：`_project_identified_rays` 删除 `hazards` 参数与"hazard 强制障碍"
  分支（死代码一并清掉）；`classify_spatial_rays` 不再把 kinds 4/5 交给投影。4/5 仍算 hits
  （射线清除到自己的实测端点），但不再进 `nearest`（障碍），也不再产生占用。
- `frontier_explorer.py`：Explorer 的障碍只剩"实测到达机体波段
  `[obstacle_min_z, obstacle_max_z]` 的 kind 1"；kind 4/5 进新增的稀疏地形注解
  `terrain_rises`/`terrain_drops`（每格有界计数，只做表征与诊断，`[FUSION_STATS]` 增
  `terrain_rise_cells`/`terrain_drop_cells`），不参与占据、不产生 0.65 m 膨胀。跨不跨台阶
  由执行层决定；底层能力接入时，这一层就是它的输入。

保留边界（有意不改）：Local 仍把 kind 4/5 投影到机体高度（执行层的保守裁决，`LocalMapPatch`
→ `global_representation` 也仍然继承它）；Explorer 的带内 kind 1 仍占据——那是"平面规划必须
避开的几何"，不是地形判断。

验证：`test_mid360_mapping_runtime.py::test_typed_ground_keeps_low_rocks_and_drop_hazards`
的 Explorer 断言改为"落差/凸起不得成为占据，带内 kind 1 仍占据（正对照）"；
`test_grid_vectorized.py` 对应用例改为锁定新语义（含 miss 不成障碍、带内几何仍障碍）。
ctest 全量 11 项通过（其中 `test_observation_task_runtime` 首跑 1 例已知时序抖动、重跑与
直跑均 64/64 通过），pytest 527/527。

这一改动在同一姿势实机数据上的量级：前方 2–6 m 的占据来源里 kind 5 有 517 个点
（|lat|<0.45 m 有 74 个，中位离地 0.31 m），带内 kind 1 只有 73 个点（|lat|<0.30 m 17 个）
——远端走廊的"黑块"大部分由地形标签及其膨胀造成，移除后应显著收缩；但真实到达机体高度的
立面仍会占据（当前姿势 4 m 处就有一面），在底层能力未接入阶段这是正确行为。

## 追加二：分类器降级为"地面/高度估计器"（2026-10-16 晚）

架构判断：语义判决（"凸起 = 危险"）不该待在感知层，而且它的唯一消费者 Local 只拿它做"挡不挡
机体"的判决——正是假障碍的来源。同时实测：用分类器自己的判据复算，一帧 1483 个实测回波里只有
**31% 算得出离地高度，69% 是真未知**；而代码里没有"未知"这个状态，未知点静默落到 type 1 由机体
带兜底（新加的 `errors` 数组还把它们记成 0.0，等于假装贴地）。因此把这一层改成只做估计：

- `observed_ground.hpp`：`estimateObservedGroundHeights()` 只输出每点"离地高度或 NaN"。删掉
  `rise` 阈值、MAD 估计、3/4/5 标签、两帧门（`RiseGateMemory`）及其一切判决。
- **线上仍只有一个字节**（复用 `return_type`，不加字段、不加话题）：`plan_env/ground_height.hpp`
  是唯一契约来源——0 无效；1 实测但高度未知；2 实测无回波；**3..252 = 离地高度，1 cm 分辨率、
  范围 [−0.50, +1.99] m**；253..255 保留（按未知处理）。bridge 编码并打印
  `measured/known/unknown/h50`；回调不再按"1..5"过滤字节。
- Local（`grid_map.cpp`）：有高度的点按**地面相对的净空体积**判——`|h| ≤ 0.035` 是地面（清除 +
  support）；`0.035 < |h| < 0.22` 是腿部地形（本格清除，**不挡机体平面**）；`0.22 ≤ h ≤ 0.55`
  或 `h ≤ −0.22` 是可能撞躯干的几何（投影到机体高度 + 膨胀，泄漏/衰减机制不变）；`h > 0.55`
  是头顶结构（清除）。**没有高度的点回退到原来的机体带**（行为不变）。
- Explorer（`grid.py` + `frontier_explorer.py`）：同样的净空体积判障碍（与 Local 一致，避免两层
  打架），`|h| ≤ 0.035` 给 free 证据，其余进**稀疏地形注解——现在存的是高度**（每格最大抬升 /
  最深落差），仍然不占据、不膨胀。
- 新增三个参数（两侧同名同默认，机器人几何 + 能力声明，不是调参）：`terrain_floor_tolerance
  0.035`、`obstacle_min_height 0.22`、`obstacle_max_height 0.55`。**未接入 60 cm 能力前，
  0.22–0.60 m 的地形仍然算挡**；接入时把 `obstacle_min_height` 提到 ≈0.82（能力 + 躯干高度）。

实测（同一隧道姿势，无头）：`[OBSERVED_GROUND] valid=1 measured=16413 known=6239 unknown=10174
h50=0.60m`（known 38%，与离线复算同量级）；`[RAY_ENDPOINT_CHANNELS] terrain_proj=200
terrain_active=200`——**机体高度投影从同姿势旧规则的 486 降到 200**，剩下的都是 ≥22 cm 的真实
几何。验证：单测 10 项（含编码往返 ±1 cm、未知/保留值解码为 NaN）、运行时 90 项、`plan_env`
ctest 1/1、`scan_planner` ctest 11/11、pytest 527 项、链路运行时套件 64/64。

仍未做的：脚触地零点（62% 未知点目前仍靠写死零点的机体带）；Explorer 的 0.65 m 膨胀半径在
2 m 走廊里仍偏大（Goal 落点校验会继续偏紧），这两件是下一步要单独决定的。

