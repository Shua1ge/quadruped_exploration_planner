# 跨 Goal 接入、尾段走廊、轻量路线记忆（batch6）

已完成代码修改并构建安装，未启动现场Gazebo长测。范围为约定的1、2、5；普通失败停车处置、物理恢复独立入口及搜索容量循环修复不在本批。

## Local 改动

跨Goal保留公共段前，最多探测3个未来接点。各点使用旧轨迹对应p/v/a，对G2粗路线做有界几何/动态探测；可行后只对选定点调用原完整选择器和生成器。每次连接至多3锚点×2时长×25探测，不新增A*或完整选择器轮次。同任务路径不使用该探测。

选定task_approach后，新尾段用带完整速度、加速度的五次入口曲线形成种子，之后按原接近路线延伸；不再仅以沿新路线的速度投影描述跨任务起始状态。入口遵守现有任务域，速度/加速度按现有配置逐分量探测。探测不是安全认证；最终原有轨迹动态、任务域、足迹/扫掠和Controller认证继续执行。

跨任务尾段优化对实际三次B样条每个span采样3点，增加同一taskDomain的越界代价，同时保留原guide、避障与平滑。该代价是生成约束引导，不代替硬终验。公共段仍精确拼接；同任务候选不启用新corridor callback。

新日志TASK_JOIN_PROBE记录可行性、接点和probe_ms；TASK_ENTRY_GUIDE记录入口时长、接入弧长和样本数。探测位于CANDIDATE_ATTEMPT计时之前，分析总准备开销需合并probe_ms；入口种子本身计入原seed_ms。探测失败是候选不可用，由现有FSM决定旧轨迹延续或停车，本批不更改该决定链。

## 最小 Global

CLI新增--global-mode memory，并成为run_newmine_rl.py默认。直接ros2 launch的原shadow默认保留，使用轻量模式必须传global_usage_mode:=memory。memory模式不启动完整global_representation节点，复用Explorer既有BoundedVisitRoutes.paths作为唯一历史路线缓存，无第二张地图或定时构图。

RouteMemory在现有48条缓存上增加总采样点上限16000和小型空间采样索引。按请求尝试接入已有路线内部，返回前检查当前raw FREE及有方向的blocked edge；无缓存连接则继续原有有界搜索。已测实际里程计每约12个不同格形成一个小历史段，跳变不记为经过。历史段仍须在当前地图下复核，不能作为机身/地形认证。更新远处地图不主动重算所有路线。

经验第一版只保存实际经过的几何见证和计数，不增加风险模型、Region或信息收益策略。缓存返回可行粗路线，不保证当前最短路线。GLOBAL_ROUTE_MEMORY日志记录records/cells/hits/path_checks/motion_records和本轮搜索扩展数。

原/global_representation/graph_markers在此模式下不会继续发布完整图。Explorer现有区域显示仍按原逻辑发布。保留--global-mode shadow作完整Global对照；--global-mode off可关闭历史路线增强。

根工作区run_newmine_rl.py已改为调用版本库中的同名入口，解决原根脚本落后而不支持--actual-route的问题。策略模型内容已核对相同，没有切换RL模型。

## 验证

Release构建bspline_opt与scan_planner通过。68项LocalTaskPlanner、102项FSM工具、4项B样条测试通过；263项Python（Region访问、Explorer、任务桥）通过；8项独立ROS domain232测试通过，包括真实跨Goal公共段安装、新入口日志、速度坐标一致性、速度跳变拒绝、转向owner及无效证书。memory模式在独立domain231启动确认，launch条件测试确认memory/off不启动完整Global、shadow/route_cost启动。

其中测试夹具补齐WorkMetrics和点云宽高；原严格停车转向用例显式关闭可选moving-alignment，与其断言及生产配置一致，Controller生产代码未改。

合成64×64绕行图6个相邻目标：旧缓存扩展15558格，memory2674格（后5次0搜索扩展）；单次本机墙钟约233.7ms/39.2ms。缓存粗路线略长，不是最优路线对照，也不能外推为整机CPU收益。详见route_memory_benchmark.json。

本轮未解决两条既有长停出口，长测仍应分开统计到站等后继与pose_occupied安全锁，不能把它们当作新尾段优化失败。

## Campus 长测

终端一：

```bash
cd /home/t1an/ros2_ws/scan_planner_dyn_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
RUN_DIR="$PWD/run_records/campus_batch6_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
printf '%s\n' "$RUN_DIR" > /tmp/scan_dyn_batch6_run_dir

git -C src/SCAN-Planner rev-parse HEAD > "$RUN_DIR/git_head.txt"
git -C src/SCAN-Planner status --short > "$RUN_DIR/git_status.txt"
git -C src/SCAN-Planner diff --binary > "$RUN_DIR/worktree.patch"
tar --exclude='__pycache__' --exclude='*.pyc' -czf "$RUN_DIR/planner_source.tar.gz"   -C src/SCAN-Planner run_newmine_rl.py src/planner/plan_manage src/planner/bspline_opt

./run_newmine_rl.py --duration 0 --no-sparse-routing --global-mode memory   --gui --show-rviz --domain 148   --world "$PWD/src/SCAN-Planner/assets/runtime_worlds/campus/campus_fortress_runtime.sdf"   --map-size-x 460 --map-size-y 400   --log "$RUN_DIR/runtime.log" --metrics "$RUN_DIR/metrics.csv"   --actual-route "$RUN_DIR/actual_route.csv"
```

终端二在启动阶段尽早运行，持续记录到整个长测结束，再Ctrl-C：

```bash
cd /home/t1an/ros2_ws/scan_planner_dyn_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=148
RUN_DIR="$(cat /tmp/scan_dyn_batch6_run_dir)"
python3 src/SCAN-Planner/tools/record_region_growth.py   --domain 148 --duration 0 --output "$RUN_DIR/region_record"
```

记录器默认duration0是墙钟无限，最后必须与仿真一起停止，不能只运行600秒。建议先完成约10分钟检查跨任务提交/安装和绕弯，再持续到原来后段场景；这不是重新改观测任务的完成规则。记录开始晚时历史地图仍可能不全。

参数记录可在第三个已source的终端执行：

```bash
export ROS_DOMAIN_ID=148
RUN_DIR="$(cat /tmp/scan_dyn_batch6_run_dir)"
ros2 param dump /scan_planner_node > "$RUN_DIR/local_params.yaml"
ros2 param dump /closed_loop_controller > "$RUN_DIR/controller_params.yaml"
ros2 param dump /frontier_explorer > "$RUN_DIR/explorer_params.yaml"
```

需要完整Global对照时，新建另一个RUN_DIR，仅把--global-mode memory改为shadow，其他配置保持相同。对照时Local仍是本批版本，因此可区分Global负担，不是本批Local前后对照。

## 备份

本轮差异、修改前后文件和日志位于/home/t1an/文档/Codex/2026-11-02/new-chat/local_global_batch6。before包含此前已有未提交改动，回退本批不能git reset清掉它们。changes.patch仅本轮范围，新文件列表new_files.json；工作区根入口另有workspace_run_newmine_rl.py备份。
