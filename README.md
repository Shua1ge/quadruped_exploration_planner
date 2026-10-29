# dyn：Go2 长程自主探索

Ubuntu 22.04 · ROS 2 Humble · Gazebo Fortress · Go2 RL。当前推荐使用 LiDAR 和本体感知，Vision 不启用；Global 保持 `shadow`，先独立验证空间表示和计算开销。

本页对应包含本页的提交。历史分支快照 `7d48e6f` 的参数和行为不同，不包含后续 Global 观察模式等改动；部署和复现实验时请记录完整 commit。

从终端开始请按 [启动探索步骤](docs/dyn/START_EXPLORATION_ZH.md) 操作；新部署使用本页指定的时间分支。

## 环境与构建

先按 [环境安装说明](docs/dyn/SETUP_ZH.md#1-系统与依赖) 安装 Humble、Fortress、控制插件及系统 Python 的策略依赖。运行推理无需 Isaac Lab。使用英文、无空格的工作区路径。

```bash
mkdir -p ~/ros2_ws/scan_planner_dyn_ws/src
cd ~/ros2_ws/scan_planner_dyn_ws/src
git clone --branch dyn-20261029-214547 \
  https://github.com/Shua1ge/quadruped_exploration_planner.git SCAN-Planner
cd ..
source /opt/ros/humble/setup.bash
# 首次使用 rosdep 时执行一次：sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --parallel-workers 2 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DUSE_GPU=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

`USE_GPU=OFF` 是规划器感知后端选项；Gazebo GPU LiDAR 仍需可用的 OpenGL/EGL 驱动。完整环境和无界面运行要求见 [SETUP_ZH.md](docs/dyn/SETUP_ZH.md)。

## NewMine：推荐启动入口

NewMine 的世界、所需模型和纹理，以及当前 TorchScript 权重已随仓库保存。

```bash
cd ~/ros2_ws/scan_planner_dyn_ws/src/SCAN-Planner
./run_newmine_rl.py --dry-run --duration 0 --gui --show-rviz --global-mode shadow
./run_newmine_rl.py --duration 0 --gui --show-rviz --global-mode shadow --domain 148
```

第一条只检查并打印命令。第二条启动 Gazebo、策略和探索，传感器就绪后会自动调用起身授权；`--duration 0` 持续运行到 Ctrl-C。仅检查世界和传感器、暂不起身时，使用 [不自动起身的启动方式](docs/dyn/SETUP_ZH.md#5-不自动起身的检查方式)。

从仓库根目录运行包装器；其他布局通过 `--workspace /path/to/colcon_ws` 指定已构建工作区。日志和 metrics 默认写入 `/tmp/scan_newmine_rl_时间戳.*`，当前包装器还记录实际里程计路径 CSV。参数以本提交的 `./run_newmine_rl.py --help` 为准。

`shadow` 中 Global 计算和显示，但 Explorer 不消费其路线或空间划分。`--global-mode off` 用于开销对照。`route_cost` 是待单独验收的规划模式，不是当前推荐启动项。

## Campus：先准备外部资源

Campus 大型原始资源不随 Git 分发。按 [来源与资源说明](docs/dyn/WORLD_RESOURCES_ZH.md) 获取官方 environments ZIP，再执行：

```bash
python3 tools/worlds/prepare_campus.py --archive /path/to/environments.zip
# 也支持 --download 或 --source /path/to/campus，三者选一
./run_newmine_rl.py --duration 0 --gui --show-rviz --global-mode shadow --domain 148 \
  --world "$PWD/assets/runtime_worlds/campus/campus_fortress_runtime.sdf" \
  --map-size-x 460 --map-size-y 400 \
  --init-x 0 --init-y 0 --init-z 0.45
```

使用转换后的 Z_UP OBJ 视觉与 STL 碰撞世界；不要直接将原始大型 DAE 当作 Fortress 碰撞体。下载、转换、资源路径和坐标说明见 [Campus 接入](docs/dyn/SETUP_ZH.md#4-campus下载导入与-fortress-适配)。

## 发布验收

发布候选先形成明确提交，再验证该提交导出的干净源码；不要用当前机器的旧 `install/` 证明新提交可运行。

```bash
# 无仿真、无运动：资源与文档检查，独立导出 HEAD
python3 tools/validate_release.py --ref HEAD --output /tmp/dyn-release-source
# 独立源码 + 全量编译 + 安装入口 + 离线策略检查
python3 tools/validate_release.py --ref HEAD --build --output /tmp/dyn-release-build
```

报告记录实际 commit、未包含的工作树改动、逐项结果和日志。`--build` 也不会启动 Gazebo 或起身；仿真就绪和长程探索分别验收，不能用离线通过替代。详见 [发布验收流程](docs/dyn/RELEASE_CHECK_ZH.md)。历史结果保留在 [快照验证记录](docs/dyn/SNAPSHOT_VALIDATION.md)。

## 当前开发范围

[无 Vision 的 Global 设计与替代计划](docs/dyn/GLOBAL_NO_VISION_DESIGN_ZH.md) 定义空间身份、增量维护、候选接入及批量路线服务。它是后续实现与验收约束，不表示已完成；现有观察模式见 [GLOBAL_OBSERVATION_ZH.md](docs/dyn/GLOBAL_OBSERVATION_ZH.md)。历史停车与地图问题见 [已知问题记录](docs/dyn/KNOWN_ISSUES_20261028_ZH.md)，其中的现场状态对应记录日期。

本项目衍生自 [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner)，原始算法与研究署名保留于 [NOTICE](NOTICE)。许可证见 [LICENSE](LICENSE)；世界资源和策略的来源及许可单独记录于资源说明和 `models/`。
