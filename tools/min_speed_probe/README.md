# ONNX 策略最低有效速度标定

在开阔平地(Gazebo Fortress)给一系列恒定前进指令,测每档是否产生持续位移、
姿态是否稳定,确定 `parkour_moe_full_model.onnx` 的最低有效速度。

## 命令通路(全部已在代码中核实)

- **策略运行时是 `go2_locomotion_supervisor.py`**(内嵌 `go2_onnx_50hz` 推理
  线程,力矩 getup + 站立校验),不是独立的 `go2_onnx_policy_node.py`。
- 命令正门是 **`/planning/cmd_vel_raw`**:supervisor 收到后裁剪到
  [±2, ±1, ±2],经健康门控(摔倒/未站立自动归零)再喂给策略观测
  (`obs[6:9] = cmd × [2, 2, 0.25]`);`cmd_timeout = 0.5 s`,探针以 50 Hz 持续发布。
- 位姿真值:`/quad_0/body_pose`,由 go2 桥接自 Gazebo `/model/go2/odometry`。
- **绝不能与规划器同时发命令**:`closed_loop_controller.cpp` 的 `cmdCallback`
  在无轨迹时每 10 ms 无条件 `publishStop()`(零速),100 Hz 零速会淹没探针。
  因此用"最小链路":Gazebo + supervisor,不起规划器。

## 步骤

```bash
# 终端 1:平地物理仿真 + rl_effort 控制器(必须 source 外层工作空间,
# 仓库内的 SCAN-Planner/install 是残留的残缺安装树,只有 exploration_planner)
source ~/ros2_ws/scan_planner_ws/install/setup.bash
ros2 launch go2_description go2_sim.launch.py \
  world:=$HOME/ros2_ws/scan_planner_ws/src/SCAN-Planner/tools/min_speed_probe/flat_ground.sdf \
  locomotion_mode:=rl_effort

# 终端 2:supervisor(自动力矩 getup → 站立校验 → 待命)
ros2 run scan_planner go2_locomotion_supervisor.py --ros-args \
  -r body_pose:=/quad_0/body_pose \
  -p locomotion_mode:=rl_effort \
  -p model_path:=/home/t1an/ros2_ws/scan_planner_ws/parkour_moe_full_model.onnx

# 等 supervisor 日志报告站立稳定后,打开导航使能(rl_effort 模式默认要求):
ros2 service call /robot/enter_navigation std_srvs/srv/Trigger '{}'

# 终端 3:探针(默认 0.05–0.6 m/s 八档,每档 10 s,档间 2 s)
python3 $HOME/ros2_ws/scan_planner_ws/src/SCAN-Planner/tools/min_speed_probe/probe_run.py \
  --levels 0.05,0.1,0.15,0.2,0.3,0.4,0.5,0.6 \
  --drive 10 --out /tmp/min_speed_probe.csv

# 终端 4(或探针结束后):分析
python3 $HOME/ros2_ws/scan_planner_ws/src/SCAN-Planner/tools/min_speed_probe/probe_analyze.py \
  /tmp/min_speed_probe.csv --metrics-out /tmp/min_speed_metrics.csv
```

## 判定标准(probe_analyze.py 默认值,可调)

每档取去掉前 2 s 起步、末尾 0.5 s 的稳态窗口:

- **持续位移**:航向投影 x 的最小二乘斜率 ≥ 0.5×cmd,且 1 s 滚动窗位移
  < 0.02 m 的"停滞窗"占比 ≤ 10%(抓 step-stall 式走走停停);
- **姿态稳定**:roll RMS ≤ 0.12 rad、pitch RMS ≤ 0.15 rad,峰值
  ≤ 0.3 / 0.4 rad,偏航漂移 ≤ 10°/s;
- **摔倒判据**:base z < 0.15 m 或 |roll| > 0.6 或 |pitch| > 0.8
  (supervisor 门控此时也会自动归零命令)。

最低有效速度 = 通过全部判定的最低档位;输出同时给出相邻下一档的失败项,
"最低有效"只有连同余量一起看才有意义。判定阈值全部可用命令行参数覆盖。

## 注意

- 建议先用单档 `--levels 0.4` 冒烟,确认链路能正常行走,再扫低档。
- 0.05 m/s 对应观测仅 0.1(`cmd × [2, 2, 0.25]`),可能落在训练分布外;
  若表现为原地小碎步或周期性走走停停,停滞窗指标会把它抓出来。
- `go2_sim.launch.py` 的 bridge.yaml 已含 `/quad_0/body_pose` 桥接,最小链路
  无需额外节点。
- 本套件面向 Gazebo rl_effort 链路;真机标定需另确认部署栈接口,从最低档开始。
