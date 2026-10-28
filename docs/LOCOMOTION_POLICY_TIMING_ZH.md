# Go2 策略时序诊断：状态是什么时候采样的，动作是什么时候装上的

这份诊断用于区分“模型输入拼错了”和“输入内容正确但时间关系错了”。当前仍然盲走；它不改变 PD、速度限制、HOLD、估计器刷新规则或动作接收规则，不会因为测到了延迟就自动丢弃动作。

## 一次请求的流转

```text
JointState / IMU / Odometry 的 header.stamp：源端采样时间
        ↓ 回调同时保留源时间和原有的接收时间
evaluate_policy：构造观测和历史，分配单调递增 sequence
        ↓ 携带该次观测的传感器时间、command、previous_action_sequence
一个最新请求槽：被替换的尚未取走请求计入 request_overwrites
        ↓ 后台线程取走请求，记录排队时间
ONNX / StatefulPolicyAdapter：记录推理耗时及实际估计器刷新
        ↓ 一个最新结果槽；未被安装就被替换的结果计入 result_overwrites
consume_policy_result：原样安装关节目标，并记录对应 sequence 与安装时间
        ↓ 每秒汇总，异常状态的原有 fault_trace 也带上最近安装记录
```

`previous_action_sequence` 指观测中的 last_action 对应哪个已安装结果。`sequence` 是这次观测的编号，`episode` 是本次策略启动的编号，`epoch` 标识控制器实例。普通停车、换轨迹或换 Goal 不产生新策略 episode；显式进入新的策略阶段才推进它。编号不会因为一秒统计窗口结束而归零。

“安装”在这里的边界是 `desired_target` 已经赋值，接下来的 PD 会读取它。它不是 Gazebo 或真实执行器已收到力矩、也不是机器人已完成动作；本次没有测量 ROS 力矩输出到执行器的传输时间。

## 两套时间不能混算

`*_ros_ms` 使用节点的 ROS 时间。启用 use_sim_time 时，它是仿真时间，用来比较 header.stamp、观测时刻与安装时刻。`*_wall_ms` 使用单调时钟测量真实耗时，不是日期时间。两者分别相减，不互相相减。

比如仿真暂停了 80ms，推理或等待仍可以耗费 80ms 的墙钟时间，ROS 时间却完全不变。仿真慢于实时也会导致墙钟间隔大于 ROS 间隔，这不等于观测语义一定错了。因此不能单看一个墙钟数字就下“必须停车”的结论。

没有 header、header 全零，或者还没有有效接收过传感器状态时，相应年龄是 JSON 的 `null`，不是 0ms。源时间看起来来自未来或 ROS 时间回退时，保留负值，提醒检查时钟；不把它截成“新鲜的 0ms”。源时间年龄的解释以消息 header 和节点处于同一 ROS 时间域为前提。

## 怎么读取

控制器在 rl_effort 模式下每秒输出一条 `[POLICY_TIMING]` 日志，并发布一份 JSON 到 `/robot/locomotion_policy_timing`。汇总定时器使用单调时钟，仿真暂停或没有新动作时仍可报告；它仍需要 ROS executor 有机会运行，不能绕过 executor 本身的阻塞。

```bash
source /opt/ros/humble/setup.bash
source /home/t1an/ros2_ws/scan_planner_ws/install/setup.bash
ROS_DOMAIN_ID=43 ros2 topic echo /robot/locomotion_policy_timing --field data
```

`latest` 是最近一次安装的完整关联记录；`summary` 是报告窗口内的均值、最小值、最大值和样本数。推理和估计器统计包含那些推理完成、但尚未安装就被覆盖的结果，避免只统计幸存结果。窗口结束后只清理窗口样本，保留累计计数和最近一次安装记录。

| 字段 | 回答的问题 |
|---|---|
| `window_counts.requested / inferred / installed` | 这一报告窗口产生、算完、安装了多少次？窗口边界可能切开一次请求，不能只凭这一秒数量不等认定丢请求。 |
| `totals` | 从启动开始的累计计数，配合 pending/inflight 看长期差异。 |
| `request_overwrites / result_overwrites` | 最新请求槽和最新结果槽分别覆盖了多少次？这是保留最新值的策略发生了多少次，不是 DDS 丢包计数。 |
| `observation_interval_ros_ms` | 相邻入队观测的实际 ROS 时间间隔，是否仍约为 20ms？ |
| `queue_wall_ms` | 请求排队到推理开始耗费多久？ |
| `inference_wall_ms` | 本次模型调用真实耗费多久？ |
| `result_wait_wall_ms` | 推理结果准备好之后，等目标安装又耗费多久？ |
| `observation_to_install_wall_ms / observation_to_install_ros_ms` | 观测构造到目标安装的真实耗时和 ROS 时间跨度。 |
| `joint / imu / odom_sample_age_ms` | **这次观测所用**传感器状态，在动作安装时已多旧？不是当前最新传感器状态的年龄。 |
| `joint / imu / odom_receipt_age_ms` | 同一份输入距回调接收时间有多旧？与源采样年龄对比可暴露晚到的旧消息。 |
| `estimator_refreshed / estimator_refresh_count` | 本次是否真的更新了估计器，累计更新了多少次？旧策略不暴露这些计数，标为 null。 |
| `estimator_observation_interval_ros_ms` | 两次真实估计器刷新采用的观测时刻相隔多久？通常目标是约 100ms。 |
| `estimator_execution_interval_ros_ms / estimator_interval_wall_ms` | 两次真实估计器刷新调用开始时，ROS 时间和单调时间分别相隔多久？ |
| `estimator_sequence_gap` | 两次真实刷新之间产生了多少次观测？正常五拍通常是 5；跳过请求后可能变大。 |
| `newer_requests / episode_matches` | 安装这个结果时已有多少更新请求？它还属于当前策略 episode 吗？这里只报告，不改变原有接受行为。 |
| `last_install_age_ros_ms / last_install_age_wall_ms` | 最近一次目标安装距现在多久？没有新动作时仍增长，不会因窗口没有样本而假装延迟为零。 |
| `pending_sequence / pending_age_wall_ms` | 是否有尚未被线程取走的请求，已经等了多久？ |
| `inflight_sequence / inflight_observation_age_wall_ms` | 是否有正在推理的请求，其观测距现在多久？ |
| `current_sensor_sample_age_ms / current_sensor_receipt_age_ms` | 现在最新输入有多旧？与 latest 中旧动作对应的输入区分开。 |

估计器第一次刷新没有“上一次”，间隔为 null。刷新间隔只出现在真正刷新了估计器的推理记录上，不能把 Actor 的非刷新拍当成估计器的 0ms 间隔。

## 怎么看异常

如果 receipt 很小而 sample 很大，说明消息最近收到，但其中状态已经旧了，应检查源端发布、桥接或排队。原有安全检查仍按 receipt 运行，这份诊断不悄悄改变其阈值。

如果 inference 很小而 queue 或 result_wait 很大，瓶颈不在模型本身，而在请求等待、线程调度或结果消费。如果 inference 本身变大，才继续查推理负载。没有新结果时，要一起看 inflight、pending 和 last_install_age，不能只看上一条成功结果的推理耗时。

如果估计器的观测间隔从约 100ms 变成 300ms，并出现 sequence_gap=15、请求覆盖增长，那么五次“实际处理”已经不是五个名义 20ms 拍。此时再评估是否把估计器调度改为显式时间驱动，不能把名义 10Hz 当成实测 10Hz。本次仍保留现有调度规则。

这些是诊断方向，不是已经证明现场趴下由某一个指标造成。下一次异常要把时序与姿态、关节速度、力矩和状态转换放在同一段记录里看。当前 epoch 的 fault_trace 也会携带最近一次安装的 policy_timing，便于关联。

可在原有探索运行期间额外记录一份 bag。以下输出目录应为尚不存在的目录；已有目录时换一个名称，不要覆盖旧记录：

```bash
ROS_DOMAIN_ID=43 ros2 bag record -o /tmp/go2_policy_timing_01 \
  /robot/locomotion_policy_timing /robot/locomotion_fault_trace \
  /robot/locomotion_diagnostics /robot/locomotion_state \
  /planning/cmd_vel_raw /joint_states /go2/imu /quad_0/body_pose /clock
```

这不启动机器人。当前安装中的 supervisor 和 logic 是源码符号链接，新的控制器进程会直接使用修改后的 Python 文件，无需重编译 C++。已经运行的 Python 进程不会自动热更新；需要在下一次正常启动时启用这些记录。

## 本次验证边界

离线测试直接调用生产控制器的回调、请求入队、后台推理、结果安装和 JSON 汇总逻辑，并使用可控的模型与时钟复现 80ms 推理、晚到的传感器消息、请求覆盖、估计器 300ms 间隔和仿真暂停。另有真实打包 ONNX 经过后台线程安装结果的测试；这些测试不启动 Gazebo、不调用导航启用服务、不发送驱动机器人运动的命令。

没有正在运行的 Go2/Gazebo 实例可供本次现场采样，所以上述受控时序数值是回归测试结果，不是矿洞中的实测延迟。
