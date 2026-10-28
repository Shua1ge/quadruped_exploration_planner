# 运动合同核查与旧策略回退（2026-10-12）

## 本次改动边界

普通探索入口 newmine_exploration.launch.py、run.launch.py，以及直接启动
go2_locomotion_supervisor.py，默认都选择 legacy。它加载原有
/home/t1an/ros2_ws/scan_planner_ws/parkour_moe_full_model.onnx。
没有覆盖或删除新旧模型。新策略用 locomotion_policy:=parkour_moe_stateful
显式选择，新策略速度探针也明确选择该后端。

legacy 不注入新策略的 fsm.min_forward_effective_speed=0.25 覆盖，继续读取
planner.yaml 的 0.30 命令档和 0.165 有效速度。这些是既有标定配置，不是本轮
重新验证过的矿洞安全下限。回退不改变关节阻尼、摩擦、PD 增益、控制周期、
命令平滑、地图膨胀或任务交接；因此也不等于完整恢复此前的运动表现。

## 现有运行链与已确认缺口

SCAN 发布 Bspline → Controller 生成 /planning/cmd_vel_raw → 监督器平滑
命令并生成观测 → ONNX 生成关节目标 → 监督器 PD 发布力矩 →
gz_ros2_control 在仿真中施加力矩。轨迹 ACK 表示 Controller 接收轨迹，
推理结果安装表示监督器赋值关节目标，两者都不是机器人已经起步或停稳。

第一处缺口在轨迹合同覆盖范围。scan_replan_fsm.cpp 发布候选时，只有
非 MODE_NORMAL 才写 min_forward_command 和 min_forward_effective_speed。
closed_loop_controller.cpp 用 min_forward_command>0 选择 spatial_gait；
否则按时间推进样条、允许连续小速度、倒退和横移。最低可动速度与可执行
运动方向是策略/机器人的属性，不应仅在贴障受限模式生效。普通轨迹缺少
同一套执行能力约束，但也不能把任意小速度直接抬到最低档，否则终点和
安全停车的运动可能被改变。

第二处缺口在命令语义。Controller 的 publishStop 发零平移，可同时发偏航；
监督器随后以每秒 [0.50,0.40,1.00] 的变化率再次改变命令。函数复现：从
[0.30,0,0] 要求 [0,0,0.80]，0.10s 后策略输入仍为 [0.25,0,0.10]，
0.30s 后为 [0.15,0,0.30]，0.60s 后才为 [0,0,0.60]。这是输入命令，
不是实际速度保证。上游的“平移冻结”并不等于下游已停止平移。检查安全
预测是否覆盖该过渡，必须结合实际速度、反馈和停车测量，不能只看零 Twist。

第三处缺口在支撑状态。初始 STAND_VERIFIED 检查稳定窗口；进入 RUN 后，
观测 HOLD 只令策略执行零命令，不重新验证驻足/重新起步的支撑状态。
本轮已修正低伸展的语义：0.15m 腿部伸展不再单独关闭 capabilities 或
触发 RUN 的姿态 FAULT。它不是离地高度，不能仅据此断言倒地或地图失效。
持续低伸展只报告 LEG_EXTENSION_RISK，零速度/HOLD 下仍保留策略推理和
PD 支撑；风险恢复不重置策略。新鲜反馈、有效姿态和倾斜保护仍保留。
这不等于已经具备真实触地/离地高度观测，也未补全驻足与重新起步认证。

第四处边界是内环执行。原部署每个 MuJoCo 物理步直接计算并施加 PD，
当前关节反馈和力矩通过 ROS 往返。500Hz 配置和正常 50Hz 推理不证明
每个物理步的力矩时序等价。现场模型还存在额外阻尼 0.5、关节摩擦 0.05；
其与训练模型的一致性需原 MuJoCo XML 核验。此轮不调整它们。

## 证据与结论边界

此前运行在 trajectory=25 普通轨迹起步后身体逐渐下沉。故障前 100 个
采样没有命令力矩削顶，机身倾斜小；零速度请求发生在腿部伸展已低于
0.15m 后。推理 50Hz、估计器 100ms 刷新正常，无请求/结果覆盖。
这支持“支撑闭环失稳”，不证明以上某一处是唯一原因，也不证明模型训练
必然有错。旧策略切换后的物理稳定性仍需现场运行验证。

后续应让运动监督器单独持有策略执行能力和起步/减速/驻足事实，Controller
使用同一合同并消费实际执行状态；Explorer 继续独占观测任务完成权。
不在 Explorer 新建运动账本，不在停车时无条件重置循环状态，不简单放宽
安全门。本轮未实施整套合同重构，仅修正低伸展误升级为故障的入口。

## 旧策略停车支撑修复与证据保存

对照 GitHub 分支 codex/transactional-clearance-handoff-20261001 的提交
8edf6ede54320f5d017e5240ccf83fcc356b5574，旧版普通停车继续以零速度运行
ONNX 并输出 PD 力矩。当前应保持这个行为，不能把零 Twist 当成零关节力矩。
此前新增的 FK 低伸展判据会把运行切到 FAULT，随后停止策略并进入仅阻尼，
这是模型回退未恢复的控制语义，不是旧模型普通停车的固有动作。

真正的姿态异常、无效反馈和推理错误仍按既有故障逻辑处理；仅阻尼等故障
输出分支没有被放开成自动起身。初始站立验证仍检查腿部伸展和稳定窗口。
没有改 ONNX、PD 增益、动作缩放、控制频率、命令平滑或任务交接。

首次持续低伸展和首次 FAULT 分别保存独立 JSON，默认目录为
/tmp/scan_locomotion_traces，可用监督器参数 trace_directory 指定目录。
文件名为 go2_<epoch>_low_extension_risk.json 和 go2_<epoch>_fault.json。
每次运行最多两份，独立写线程执行 JSON 编码与落盘，不在控制回调里写盘；
正常退出等待记录写完。两个事件都继续发布 /robot/locomotion_fault_trace，
通过 event 字段区分风险与故障，不能收到该 topic 就认定已经 FAULT。

记录含事件时刻、策略后端、当前关节/动作/目标/力矩、原始与平滑命令、
重力方向、基座速度、传感器采样时刻、推理时间关联，以及故障前最多
100 个采样（标称约 2 秒）。LEG_EXTENSION_RISK / LEG_EXTENSION_RECOVERED
是风险日志；LOCOMOTION_TRACE_SAVED 给出实际保存文件；写盘失败显式报告
LOCOMOTION_TRACE_FAILED，不改变执行状态，并在可编码时保留 topic 证据。
默认 /tmp 路径能跨进程退出保留，不保证跨系统重启保留。
