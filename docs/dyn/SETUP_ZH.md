# dyn 时间快照：环境、构建与运行

推荐入口见 [仓库首页](../../README.md)。本页区分 2026-10-28 历史快照与后续工作树；实际参数以所部署提交的 `run_newmine_rl.py --help` 为准。历史恢复、地面与显示问题见 [日期化记录](KNOWN_ISSUES_20261028_ZH.md)，不能将历史审计状态当作当前已验收状态。当前默认 Global shadow；无 Vision 的后续设计见 [空间记忆与计算替代计划](GLOBAL_NO_VISION_DESIGN_ZH.md)，发布步骤见 [发布验收](RELEASE_CHECK_ZH.md)。

## 1. 系统与依赖

本机验证环境是 Ubuntu 22.04.5 LTS、ROS 2 Humble、Python 3.10、Gazebo Fortress 6.18、ignition-transport11 / msgs8、ros_gz 与 gz_ros2_control。不是 Gazebo Classic，也不是新版 Gazebo Harmonic；包中出现的 `ign` CLI 是 Fortress 的正常命名。Go2 使用力矩控制和 TorchScript 推理，运行不需要 Isaac Sim / Isaac Lab 训练环境。

先按 [ROS 2 Humble Ubuntu 官方安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 配置 ROS apt 源并安装 Humble；再安装：

```bash
sudo apt update
sudo apt install build-essential cmake pkg-config ros-humble-desktop python3-colcon-common-extensions python3-rosdep \
  ros-humble-ros-gz ros-humble-gz-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-joint-state-publisher-gui \
  ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
  libarmadillo-dev libeigen3-dev libpcl-dev libassimp-dev \
  libglfw3-dev libglew-dev libgl1-mesa-dev libglu1-mesa-dev \
  libignition-gazebo6-dev libignition-common4-graphics-dev \
  libignition-transport11-dev libignition-msgs8-dev \
  python3-numpy python3-scipy python3-pil python3-yaml python3-pip python3-pytest
```

若当前 apt 源没有 Fortress 开发包，按 [Gazebo Fortress 官方 Ubuntu 安装说明](https://gazebosim.org/docs/fortress/install_ubuntu/) 配置 OSRF stable 源，然后安装 `ignition-fortress` 和上述开发包。不要混装另一代 Gazebo 的控制插件来满足名称相近的依赖。

策略节点通过系统 `/usr/bin/python3` 运行，Python 包也必须能被这个解释器导入：

```bash
/usr/bin/python3 -m pip install --user --index-url https://download.pytorch.org/whl/cpu 'torch>=2.1'
/usr/bin/python3 -m pip install --user 'onnxruntime>=1.18,<2' 'numpy>=1.23,<2'
/usr/bin/python3 -c 'import rclpy, torch, onnxruntime, scipy; print(torch.__version__)'
```

当前宿主环境版本记录见 `environment_observed.json`。这里使用兼容范围而不宣称所有未来依赖组合均已验证。RViz/Gazebo GUI 和 GPU LiDAR 都需要可用的 OpenGL/EGL 驱动；规划器的 `USE_GPU=OFF` 不会把 Gazebo 的 GPU LiDAR 改成 CPU LiDAR。可用 `glxinfo -B` 检查显卡驱动；无界面运行仍需 ogre2/EGL 能建立渲染上下文。

## 2. 创建独立工作区并构建

```bash
mkdir -p ~/ros2_ws/scan_planner_dyn_ws/src
cd ~/ros2_ws/scan_planner_dyn_ws/src
git clone --branch dyn-20261029-214547 https://github.com/Shua1ge/quadruped_exploration_planner.git SCAN-Planner
cd ..
source /opt/ros/humble/setup.bash
# rosdep 未初始化时仅执行一次 sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --parallel-workers 2 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DUSE_GPU=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

使用英文、无空格的工作区路径；ROSIDL 在包含中文路径的本轮隔离构建中失败，改用 ASCII 路径后通过。

用 `-DUSE_GPU=ON` 构建本地感知 OpenGL 后端也是现有选项；先用 OFF 建立可重复的 CPU 规划器基线。仓库的模型和 Python 脚本通过现有 ament 安装规则提供。根目录 `run_newmine_rl.py` 会向上查找 `install/setup.bash`；自定义布局用 `--workspace /path/to/colcon_ws` 或 `SCAN_DYN_WS` 指定，不要求用户名是 t1an。

## 3. NewMine：克隆后直接使用

此分支包含 `resources/newmine/NewMine.sdf` 及其六个 Edgar 模型和纹理，引用已换成可移动的 `model://` URI。资源来源、CC0 声明和文件校验见同目录 `SOURCES.json`、`NOTICE.md`、`MANIFEST.json`。只包含 NewMine 实际引用的模型，不依赖本机原来的 mine_tunnel_world 目录。

```bash
cd ~/ros2_ws/scan_planner_dyn_ws/src/SCAN-Planner
./run_newmine_rl.py --dry-run --duration 0 --global-mode shadow --gui --show-rviz --domain 148
./run_newmine_rl.py --duration 0 --global-mode shadow --gui --show-rviz --domain 148
```

`--dry-run` 只打印 launch 命令，不启动仿真或起身。正式运行时，包装器隔离 ROS domain / IGN partition，等待关节、IMU 和 odom 后调用 `/robot/enter_navigation`，开始策略起身与探索。`--duration 0` 持续到 Ctrl-C；`--duration 90` 是起身授权成功后的 90 秒墙钟运行。Ctrl-C 只清理该包装器创建的进程组。

当前默认 `--global-mode shadow`：Global 计算并显示粗连通图，Explorer 不消费其路线、safe-region 或转发失败约束。`--global-mode off` 则不启动 Global，可用于资源开销对照。原 `--no-sparse-routing` 保留兼容；省略它也不会越过 shadow 隔离。只有显式 `--global-mode route_cost` 才接回既有 Global 规划入口，本轮先不启用。说明和验证边界见 [Global 观察模式](GLOBAL_OBSERVATION_ZH.md)。

## 4. Campus：下载／导入与 Fortress 适配

Campus 原始资源入口是 [CMU development environment](https://www.cmu-exploration.com/development-environment)，下载脚本来源仓库为 [HongbiaoZ/autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment)，本轮审计固定源提交 `bf0cba71365271ebff09831a05afd78578150300`。资源包是约458MB的 environments ZIP，Campus DAE 约382MiB。它们不随代码重新上传，也不把第三方场景自动改授本仓库的 Apache-2.0 许可。来源与本机资产摘要见 [资源说明](WORLD_RESOURCES_ZH.md)。

下面三种输入任选一种，输出统一为仓库内的忽略目录 `assets/runtime_worlds/campus`：

```bash
python3 tools/worlds/prepare_campus.py --download
# 已有官方 ZIP 时，避免重复下载；扩展名为 .download 也可以
python3 tools/worlds/prepare_campus.py --archive /path/to/environments.zip
# 已经解压，且有 campus/meshes/campus.dae
python3 tools/worlds/prepare_campus.py --source /path/to/campus
```

下载服务可能需要浏览器确认或受地区网络限制；自动下载若取得 HTML 而非 ZIP 会明确失败。可在官方入口手动下载后用 `--archive`，无需更换转换步骤。

脚本先用 Gazebo MeshManager 提取原始三角形供 STL 碰撞使用，再用保留 Z_UP 的 Assimp 导出 OBJ 视觉网格；每个三角形的位置必须与原始参考在1毫米内一致，才生成部署世界。纹理降到最长边2048，保留9个材质；原点支撑高度按此前几何求交作0.000157343m平移。不是简单给 GO 或重力旋转90度，也不改变场景大小。

```bash
./run_newmine_rl.py --duration 0 --global-mode shadow --gui --show-rviz --domain 148 \
  --world "$PWD/assets/runtime_worlds/campus/campus_fortress_runtime.sdf" \
  --map-size-x 460 --map-size-y 400 \
  --init-x 0 --init-y 0 --init-z 0.45
```

包装器自动把世界旁的 `models/` 加入 Gazebo 资源路径，并显式传入仓库的 RL 权重路径。直接使用 `ros2 launch` 时则必须自己设置这两个值。转换前的原始 DAE 不应直接作为 Fortress 的碰撞 mesh；此前 DART/ODE 对它崩溃，且原始视觉加载很慢。使用生成的 STL+Z_UP OBJ 世界。

## 5. 不自动起身的检查方式

若只检查世界、时钟或传感器，不使用会自动起身的包装器：

```bash
source /opt/ros/humble/setup.bash
source ../../install/setup.bash  # 从工作区 src/SCAN-Planner 执行
export ROS_DOMAIN_ID=149
export IGN_PARTITION=scan_dyn_check_149
ros2 launch scan_planner newmine_exploration.launch.py \
  world_file:="$PWD/assets/runtime_worlds/campus/campus_fortress_runtime.sdf" \
  gazebo_resource_path:="$PWD/assets/runtime_worlds/campus/models" \
  policy_model_path:="$PWD/models/go2_rl_lab_policy.pt" \
  locomotion_mode:=rl_effort locomotion_policy:=go2_rl_lab \
  auto_start:=false headless:=true show_rviz:=false \
  map_size_x:=460 map_size_y:=400 init_x:=0 init_y:=0 init_z:=0.45
```

控制器激活并不等于已经起身。启动链应依次出现 `STARTUP_STEP_DRAINED`、`SIMULATION_RUNNING_VERIFIED`，随后 launch 才报告持续运行已验证。`/clock` 必须实际递增，服务 `data:true` 本身不算恢复运行完成。需要起身时，在同一个 domain 的另一个终端调用 `ros2 service call /robot/enter_navigation std_srvs/srv/Trigger '{}'`；Explorer 开关是 `/explorer/enabled`。

## 6. RViz 与排错

当前工作树的 `/global_representation/graph_markers` 显示 Global 空间支撑与投影地面，`/explorer/region_overlay` 显示 Explorer 信息边界、候选和任务。二者分别表示空间结构与信息工作集。Global 的 `place_id` 划分仍存在非凸空间合并及观测顺序依赖，不是已验收的语义房间；固定 XY 投影也不能作为地面高程或通行认证。计划与边界见 [无 Vision 的 Global 设计](GLOBAL_NO_VISION_DESIGN_ZH.md)。

历史 `7d48e6f` 的显示仍是 Frontier 分组向地面扩色，参数也没有 `--global-mode`。部署历史快照时使用该提交文档，不将新工作树显示说明套回旧版。

离线代码下钻和终端原因码反查使用 `docs/CONTRACT_STACK_INTERACTIVE_ZH.html`；生成器与说明在 `tools/code_audit/`。旧页面是历史生成产物；修改代码后用 `python3 tools/code_audit/generate.py` 和 `python3 tools/code_audit/test_index.py` 更新并检查索引。

## 7. 验证范围

已有启动与坐标修复测试证明 Campus / NewMine 控制器启动后时钟持续推进，Campus visual/collision全部1,929,134三角形一致。Campus 完整栈只读加载验证获得正常地面支撑和渲染，未以此宣称长期探索稳定。当前长程运行暴露的恢复死锁与地面拟合问题仍在此快照中。

历史归档结果见 [SNAPSHOT_VALIDATION.md](SNAPSHOT_VALIDATION.md)。发布候选先按 [RELEASE_CHECK_ZH.md](RELEASE_CHECK_ZH.md) 从指定提交导出并验证；以下是开发工作树的回归命令，不代表远端提交已包含这些文件：

```bash
source /opt/ros/humble/setup.bash
source ../../install/setup.bash
python3 -m unittest discover -s src/simulator/Utils/go2_description/test -p 'test_*.py' -v
python3 tools/test_snapshot_portability.py
# 纯 Python/静态合同回归；ROS 进程测试另用已构建工作区中的 colcon test
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/planner/plan_manage/test \
  --ignore=src/planner/plan_manage/test/test_observation_task_runtime.py \
  --ignore=src/planner/plan_manage/test/test_mid360_mapping_runtime.py \
  --ignore=src/planner/plan_manage/test/test_planner_startup.py \
  --ignore=src/planner/plan_manage/test/test_waypoint_parameters.py
```
