# 终端启动探索

本页对应分支 `dyn-20261029-214547`。推荐配置是 LiDAR＋本体感知 RL、Vision 关闭、Global shadow。NewMine 模型和策略权重随仓库提供；Campus 必须先准备外部资源。完整环境安装见 [SETUP_ZH.md](SETUP_ZH.md)，资源来源见 [WORLD_RESOURCES_ZH.md](WORLD_RESOURCES_ZH.md)。

## 1. 首次部署：安装依赖、克隆、构建

先完成环境说明中的 Ubuntu 22.04 / Humble / Fortress 与 Python 依赖安装，然后执行：

```bash
mkdir -p ~/ros2_ws/scan_planner_dyn_ws/src
cd ~/ros2_ws/scan_planner_dyn_ws/src
git clone --branch dyn-20261029-214547 --single-branch \
  https://github.com/Shua1ge/quadruped_exploration_planner.git SCAN-Planner
cd ..
source /opt/ros/humble/setup.bash
# 尚未初始化 rosdep 时执行一次：sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --parallel-workers 2 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DUSE_GPU=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

已有本次工作区时，无需重复 clone；更新代码后在工作区根目录重新构建。`USE_GPU=OFF` 不会禁用 Gazebo GPU LiDAR 的渲染需求。使用英文、无空格路径。

## 2. 启动 NewMine

```bash
cd ~/ros2_ws/scan_planner_dyn_ws/src/SCAN-Planner
./run_newmine_rl.py --dry-run --duration 0 --gui --show-rviz --global-mode shadow --domain 148
./run_newmine_rl.py --duration 0 --gui --show-rviz --global-mode shadow --domain 148
```

第一条只打印并检查命令；第二条实际运行并自动申请起身授权和探索。包装器会 source 系统 ROS 和所属工作区 install，启动 Gazebo/RViz，等待关节、IMU 与 odom。`navigation armed` 表示授权成功，实际站起结果仍需看 locomotion 状态。`--duration 0` 运行到 Ctrl-C；停止该包装器会清理它自己创建的进程组。

从 `src/SCAN-Planner` 执行仓库内脚本。工作区根目录若有历史同名脚本，不属于仓库交付入口，也可能缺少路径记录等新功能。换布局时加 `--workspace /path/to/colcon_ws`。

## 3. Campus：准备一次，再启动

第一次使用 Campus，三种资源输入任选一种：

```bash
cd ~/ros2_ws/scan_planner_dyn_ws/src/SCAN-Planner
python3 tools/worlds/prepare_campus.py --download
# 或使用已下载 ZIP：python3 tools/worlds/prepare_campus.py --archive /path/to/environments.zip
# 或使用已解压目录：python3 tools/worlds/prepare_campus.py --source /path/to/campus
```

自动下载受网络/下载确认页影响时使用官方 ZIP 的 `--archive` 入口。成功后使用生成的世界：

```bash
./run_newmine_rl.py --duration 0 --gui --show-rviz --global-mode shadow --domain 148 \
  --world "$PWD/assets/runtime_worlds/campus/campus_fortress_runtime.sdf" \
  --map-size-x 460 --map-size-y 400 \
  --init-x 0 --init-y 0 --init-z 0.45
```

该文件已存在且资源完整时直接执行启动命令，不需要每次转换。使用生成的 Z_UP OBJ＋STL 世界；不要将原始大型 DAE 直接替换到碰撞入口。

## 4. 输出与运行状态

终端打印 `log=...`、`metrics=...` 和 `actual_route=...`。默认输出在 `/tmp/scan_newmine_rl_时间戳*`；实际路径另存 `_actual_route.csv` 与同名 JSON，来自 `/quad_0/body_pose`，不是规划路线。

Global shadow 只观察和显示，不参与 Explorer 选路。`--global-mode off` 可用于计算开销对照；当前不推荐自行切为 route_cost。正常包装器已启用探索，通常无需另发开关。只有在同一实例已就绪、你曾主动关闭 Explorer 时，才在同一 ROS domain 的终端恢复：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/scan_planner_dyn_ws/install/setup.bash
ROS_DOMAIN_ID=148 ros2 topic pub --once /explorer/enabled std_msgs/msg/Bool '{data: true}'
```

这个开关不负责生成模型、恢复暂停物理或越过 Local 安全状态。仅看世界、不起身的启动方式见环境说明第 5 节。

实际路径 CSV 可导入 `docs/dyn/coverage/route/route_3d.html`；Campus 背景是离线对照，不代表本轮传感器覆盖。其他世界选择“仅显示实走”。可在仓库根目录执行 `python3 -m http.server 8765 --bind 127.0.0.1`，打开 `http://127.0.0.1:8765/docs/dyn/coverage/route/route_3d.html`。离线路线工具的额外依赖见 [coverage_route 说明](../../tools/coverage_route/README.md)。
