# ONNX 策略最低有效速度标定

在开阔平地(Gazebo Fortress)给一系列恒定前进指令,测每档是否产生持续位移、
姿态是否稳定,确定 `parkour_moe_full_model.onnx` 的最低有效速度。

## 新的 stateful Parkour MoE 策略（独立仿真测试入口）

新策略不能直接交给旧四输入加载器。本目录新增 `probe_supervisor.py` 和
`stateful_policy.py`，复用现有 supervisor 的站立校验、限矩和摔倒门控，仅替换
推理接口。普通探索入口和旧模型默认值不变；这组脚本不需要重新 colcon build。

此次暂时沿用现有 45 维观测/关节顺序、默认姿态、动作缩放 0.25、kp=40、kd=1。
训练侧配置尚未提供，因此结果表示“新策略在现有合同假设下”的仿真表现。
只跑盲走：一帧零深度、vision_flag=0。循环状态跨推理保存，零速/档间休息不清零。
启动新 policy episode 才重置；不能为了站起来而放宽现有健康门限。

默认 Actor 50 Hz，估计器每 5 个已处理的 Actor 拍刷新（约 10 Hz）。刷新拍调用新
`parkour_moe_full_stateful.onnx`，得到动作、MCP 和新状态；中间拍使用压缩包中原始
`actor.onnx`、缓存 MCP 和最新观测。没有把整个策略降到 10 Hz。
`--refresh-every 5` 是 use.txt 提示的暂定值；训练侧节拍确认前不是已认证配置。

为避免混到正在运行的探索，下面每个终端都使用独立的域与 Gazebo partition：

```bash
source /home/t1an/ros2_ws/scan_planner_ws/install/setup.bash
export ROS_DOMAIN_ID=44
export IGN_PARTITION=parkour_moe_probe_44
```

终端 1 启动独立平地物理仿真（不启动 SCAN、Explorer 或 closed_loop_controller）：

```bash
ros2 launch go2_description go2_sim.launch.py \
  world:=/home/t1an/ros2_ws/scan_planner_ws/src/SCAN-Planner/tools/min_speed_probe/flat_ground.sdf \
  locomotion_mode:=rl_effort headless:=false
```

终端 2 启动新策略测试 supervisor：

```bash
python3 /home/t1an/ros2_ws/scan_planner_ws/src/SCAN-Planner/tools/min_speed_probe/probe_supervisor.py \
  --model /home/t1an/文档/Codex/2026-10-09/new-chat/parkour_moe_onnx_20261010/parkour_moe_full_stateful.onnx \
  --archive /home/t1an/文档/xwechat_files/wxid_kzl51q4s1y6s22_7a9f/msg/file/2026-10/policies.zip \
  --refresh-every 5
```

终端 3 在关节、IMU 和 odometry 初始化后开启策略；服务若返回初始化未完成，稍后再调用。
“允许起身”发生在站立验证之前，不要等 STAND_VERIFIED 后才开服务：

```bash
ros2 service call /robot/enter_navigation std_srvs/srv/Trigger '{}'
```

先跑单档冒烟，探针会等 actuation_ready，60 秒内没有通过验证则中止，不生成假的零速档：

```bash
python3 /home/t1an/ros2_ws/scan_planner_ws/src/SCAN-Planner/tools/min_speed_probe/probe_run.py \
  --use-sim-time --require-actuation-ready --wait-timeout 60 \
  --levels 0.30 --drive 10 --out /tmp/new_policy_smoke.csv
```

确认单档没有摔倒/持续摇摆后再扫档。这里的 0.30 仅为测试命令，不是沿用旧策略的速度下限：

```bash
python3 /home/t1an/ros2_ws/scan_planner_ws/src/SCAN-Planner/tools/min_speed_probe/probe_run.py \
  --use-sim-time --require-actuation-ready \
  --levels 0.05,0.10,0.15,0.20,0.25,0.30,0.35,0.40,0.50,0.60 \
  --drive 10 --out /tmp/new_policy_min_speed.csv
python3 /home/t1an/ros2_ws/scan_planner_ws/src/SCAN-Planner/tools/min_speed_probe/probe_analyze.py \
  /tmp/new_policy_min_speed.csv --metrics-out /tmp/new_policy_min_speed_metrics.csv
```

`--use-sim-time` 让档位时长和速度统计都按 Gazebo /clock 计算。旧测试默认用真实时间，
如果仿真实时因子不足 1，会把仿真运行慢混进策略速度；旧 0.30 阈值应按同样时间口径重测。
`--require-actuation-ready` 防止“机身高度够了、控制器还没放行”被误判为策略死区。
回调由单调真实时钟唤醒，避免 /clock 暂停时连健康/启动超时检查也一起暂停；采样时间仍取仿真时间。
原命令未加这两个参数时保持原行为，便于对照。

测试结束分别 Ctrl+C 探针、测试 supervisor、仿真 launch，不使用批量 pkill。
若在新策略下站立验证无法通过，先记录日志和姿态；它是接入合同/零速稳定性问题，
不是“某个 vx 档不够大”。

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
  -p locomotion_policy:=legacy \
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
