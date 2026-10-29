# coverage_route — Campus 室外观测路线（离线）

从 Campus 碰撞网格生成“用 MID360 观测全部室外可达可行走地面”的近优路线，供对照/基准使用。
不属于在线探索策略；不做动态可执行认证。

## 依赖

```bash
/usr/bin/python3 -m pip install --user trimesh embreex plotly   # numpy/scipy 已有
# 网格转换：assimp、ignition-common4-graphics（本机已有）
```

## 流程（按顺序）

```bash
cd src/SCAN-Planner
python3 tools/worlds/prepare_campus.py --download    # 输出 assets/runtime_worlds/campus（仅一次）
python3 tools/coverage_route/mesh_probe.py           # 网格体检
python3 tools/coverage_route/build_walkable.py       # 0.3 m 可行走图 → walkable.npz
python3 tools/coverage_route/mid360_visibility.py    # 单视点核验
python3 tools/coverage_route/achievable_coverage.py --spacing 2.0
python3 tools/coverage_route/plan_coverage.py --spacing 3.0 --max-candidates 300 --target 0.90
python3 tools/coverage_route/improve_route.py --target 0.99 --max-inserts 120
python3 tools/coverage_route/clean_route.py
python3 tools/coverage_route/astar_clean_route.py
python3 tools/coverage_route/validate_route.py --route route_on_graph.csv --az 720 --el 64
```

输出在 `docs/dyn/coverage/`；最终路线 `docs/dyn/coverage/route/route_on_graph.csv`，
报告 `docs/dyn/coverage/ROUTE_REPORT_ZH.md`。

## 关键假设

- 传感器：`go2_description/config/mid360.yaml`（360°×−7…52°，0.1–40 m，pitch +13° 前下倾），
  传感器离地约 0.530 m（Go2 spawn 0.45 m + mount 0.0803 m）。
- 可行走：0.3 m 格、坡度 ≤30°、邻格高差 ≤0.20 m、0.35 m 净空、spawn 连通分量。
- 覆盖：格心被射线命中为严格覆盖；0.3/0.6 m 容差口径另列。

## 调整

- `build_walkable.py --slope/--step/--footprint`：机体与地形约束。
- `achievable_coverage.py --spacing`：候选密度（默认 2 m）。
- `plan_coverage.py --spacing/--target`：初始路线密度与目标。
- `improve_route.py --target/--max-inserts/--min-gain`：插入预算。
- `validate_route.py --az/--el/--step`：独立复验精度。

## 限制

- 离线 oracle，不是在线探索；覆盖数是该路线的上界，不代表 Explorer 实际能力。
- Campus 网格含扫描孔洞/植被；未覆盖区有一部分是网格缺失。
- 规划射线 360×32、复验 720×64；尚未与 Gazebo 实采点云逐射线对拍。
- 未做 Local 规划器/控制器层面的动态可行性认证；路线为开环，未加返程。


## 记录 GO 实际走过的路线（2026-10-29）

`run_newmine_rl.py` 默认另外启动一个只订阅 `/quad_0/body_pose` 的轻量记录进程，
最多每墙钟秒采样 5 次。原来的启动指令不用增加参数；终端会打印 `actual_route=...`。
默认与 metrics 同名，增加 `_actual_route.csv` 后缀，例如
`/tmp/scan_newmine_rl_20261029_160000_actual_route.csv`。
旁边的 `.json` 记录 world 文件、来源话题、采样率和最终里程。
CSV 每秒 flush，Ctrl-C 后保存最后一帧并关闭；异常强杀可能损失最后一秒缓冲。
使用 `--actual-route /你的目录/run01.csv` 指定位置，`--no-record-route` 关闭。
已有 CSV/JSON 会拒绝覆盖。该记录器不依赖 Explorer 的规划定时器，不发布任何运动命令。

已经在运行的仿真，可以另外开终端开始记录，无需重启：

```bash
cd ~/ros2_ws/scan_planner_dyn_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=148 python3 src/SCAN-Planner/tools/record_actual_route.py \
  --output /tmp/go_actual_$(date +%Y%m%d_%H%M%S).csv
```

这里 `148` 必须与正在运行的仿真一致。需要时附加 `--world /实际使用的世界.sdf`；
独立记录器不猜测当前世界。停止这个记录终端只需 Ctrl-C，不会停止仿真。
`/quad_0/path`（`nav_msgs/msg/Path`）仍可在 RViz 中显示近期实走轨迹；它有历史点数上限，
完整的长程记录以新增 CSV 为准。不要用 `/explorer/path` 等规划路线充当实走。

打开原来的 `docs/dyn/coverage/route/route_3d.html`，右下角“导入记录”选择 CSV。
橙色显示实走，保留原有青色离线路线供比较，起终点单独标记；显示样本数、平面里程、
三维里程和有效记录时长。“聚焦实走”放大到实际经过的范围。
文件只在浏览器中解析，不上传。也可把 CSV 放到页面同目录，用“同源 CSV 地址”加载，
或打开 `route_3d.html?actual=run01.csv`。同一路径再次导入会替换显示，不累加旧 trace。

页面的绿色/红色覆盖率仍属于离线 oracle，并未计算真实传感器覆盖率。
实走 Z 是未经地面投影的机身里程计高度，含真实机身起伏；不能把机身 Z 与离线地面路线
Z 的差当成定位误差。只有同一世界、同一 `world` 坐标才适合叠加 Campus 背景。
NewMine 等其他世界请选择“仅显示实走”；当前页面拒绝直接导入未转换的 `odom`/`map` 坐标。
时钟回退、坐标系切换、超过 2 秒的输入间断会分段，不跨段画虚假的连线或累计跳跃里程。
采样会保留停车样本，但不会把重复时间戳（仿真暂停）重复写入。
页面最多约 5 万个显示样本，CSV 保留全部采样，显示降采样不改变统计口径。
