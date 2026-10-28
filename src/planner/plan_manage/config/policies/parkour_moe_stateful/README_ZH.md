# 可选策略：stateful Parkour MoE（普通探索已回退 legacy）

2026-10-12：普通探索、run.launch.py 和直接启动的运动监督器默认改为
`locomotion_policy=legacy`，读取原文件
`/home/t1an/ros2_ws/scan_planner_ws/parkour_moe_full_model.onnx`。
新模型及其配套 actor 保留；需要测试时显式选择 `parkour_moe_stateful`。
这是策略选择回退，不是机器人动力学、控制循环或整套代码的历史回滚。
运动合同检查见 `docs/LOCOMOTION_CONTRACT_AUDIT_ZH.md`。

`parkour_moe_full_stateful.onnx` 是用户提供的 estimator 与 actor 的组合图；
`actor.onnx` 原样取自用户的 `policies.zip`。它们是一对资源，不可独立换版本。
两者随 scan_planner 的 config 目录安装，不再依赖微信附件位置或测试输出目录。

显式选择 `locomotion_policy=parkour_moe_stateful` 时，Go2LocomotionSupervisor
使用 go2_stateful_policy.py 的运行时。actor 按原 policy_dt=0.02 s 运行，
完整模型每五个已处理的 actor 步刷新 estimator、MCP 与 64 维状态；
其余步用缓存 MCP 和当前 obs 调用配套 actor。推理放在原来的工作线程，
关节 PD 仍按 control_dt=0.002 s 运行。

状态只在 begin_policy_phase（初次起身/进入策略控制）重置。
零速度、HOLD、轨迹替换和 Explorer 换任务不会重置。
模型输入/输出的维度、float32 与有限性检查失败会沿现有路径进入 FAULT，
不静默回退到旧策略，也不放宽站立/姿态/传感器健康门控。

当前接入是已在 Gazebo 平地测试过的盲走方式：vision_flag=0、深度为零，
沿用已有 45 维观测、关节顺序、action_scale=0.25、kp=40、kd=1。
用户后来提供了部署 YAML 和脚本；原 MuJoCo 物理模型仍需对照。
此版本不宣称已经接通视觉地形输入。
0.15 m/s 是此前平地测试最低通过的命令档，不是矿洞可靠运动下限。
选择新策略时，受限/恢复轨迹保留 0.30 m/s 命令档；入口把该档的有效速度标定由旧策略的
0.165 更新为新策略实测约 0.252 的 0.25 m/s，用于样条重定时和跟踪换算。
制动认证仍使用更保守的 0.30 命令档；不修改 Explorer/Global/SCAN 算法。
legacy 回退继续读取原 YAML 的 0.165 标定。

原探索命令继续使用，rl_effort 默认选择 legacy。显式选择新策略或旧策略：

```bash
SCAN_ROS_DOMAIN_ID=43 /home/t1an/ros2_ws/scan_planner_ws/launch_exploration.sh \
  headless:=false locomotion_mode:=rl_effort auto_start:=true task_mode:=true \
  locomotion_policy:=parkour_moe_stateful metrics_file:=/tmp/new_policy_exploration.csv

SCAN_ROS_DOMAIN_ID=43 /home/t1an/ros2_ws/scan_planner_ws/launch_exploration.sh \
  headless:=false locomotion_mode:=rl_effort auto_start:=true task_mode:=true \
  locomotion_policy:=legacy metrics_file:=/tmp/legacy_policy_exploration.csv
```

旧模型保留在 /home/t1an/ros2_ws/scan_planner_ws/parkour_moe_full_model.onnx。
可用 policy_model_path 与 policy_actor_path 覆盖路径；留空即选择相应默认资源。
estimator_refresh_every 默认 5，按实际处理步数计，不是独立的墙钟定时器。
切换策略必须结束旧 launch 后重新启动，不能在机器人运动时热切换。

## 本次正式接入验证（2026-10-10）

colcon build 完成 12 个依赖包。60 项测试通过：原运动逻辑 36 项、
共享 adapter 11 项（含原始 estimator/actor 100 步数值对照）、
探针时钟/健康门控 6 项、正式加载与 launch 参数 7 项。
CTest 注册的 test_locomotion_supervisor 与 test_go2_policy_loading 均通过。

隔离 ROS_DOMAIN_ID=44、IGN_PARTITION=parkour_moe_replacement_44，
使用普通 launch_exploration.sh 运行 NewMine，rl_effort + task_mode。
日志实际选择 parkour_moe_stateful；读取 SCAN 参数得到 0.25。
调用既有 /robot/enter_navigation 后通过 STAND_VERIFIED，机身高度 0.306 m。
一次 60 秒只读观察窗口内没有 locomotion FAULT，观测结束时仍 actuation_ready。
停止前 Explorer 记录 6 次任务/路线发布、2 次观测满足，累计路径约 25.08 m。
这些是短时集成结果，不是对长程路径质量的结论；碰撞真值检测未开启。

启动入口仍保留显式使能：auto_start 控制 Explorer，不代替起身授权。
若日志停在 WAITING_FOR_NAVIGATION_ENABLE，在已 source 工作区的另一终端执行：

```bash
ROS_DOMAIN_ID=43 ros2 service call /robot/enter_navigation std_srvs/srv/Trigger '{}'
```

原始运行指标另存到
/home/t1an/文档/Codex/2026-10-09/new-chat/parkour_moe_onnx_20261010/speed_probe_results/new_policy_exploration_integration_20261010.csv。
测试 launch 已结束，检查域 44 无残留节点。退出时原有 go2_tf_bridge 的重复
rclpy.shutdown 报错仍出现，但 Gazebo 与控制器已退出；本次不扩展修改该模块。
