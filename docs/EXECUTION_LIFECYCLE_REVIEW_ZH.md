# 执行异常与生命周期门控：合入审核

日期：2026-10-05。主线工作分支：codex/observation-task-local-planner-20261003。

## 审核修正

FSM 不再根据 RUNNING/STALLED 状态跳过整个 tracking geometry 分类。转向、HOLD、未开始执行时，仅中和无进度输入；恢复走廊占据和严重轨迹偏差仍参与分类。独立碰撞/制动安全回调和 Controller 的健康、碰撞锁存、soft HOLD 优先级保持不变。

ObservationFeedback 增加 PATH_REJECTED。局部路径族耗尽与参考路线不能激活走路径失败分支；无新站位与站位预算耗尽走站位/任务失败分支；真实执行无进度和不安全出发仍走 EXECUTION_BLOCKED。Explorer 对 PATH_REJECTED 先在同一任务内尝试避开失败路径证据的重路由，拒绝原样重发同一条离散路线；无替代路线则进入已有 sibling Viewpoint 仲裁。回调按 task/version/request 验证，并按反馈身份去重。

无进度替换预算按任务版本持有，非任务模式按参考请求持有。等价候选 ACK 不购买新预算。未匹配实际 execution snapshot 的执行反馈不能改变活跃任务的预算。

perf 计算门位于完成检查、任务超时、请求等待/超时、执行阻塞处理和到站等待之后。计算上下文包含地图、路线约束、位置格、任务身份、请求与替换状态，并以时间重试桶驱动有界周期重试。主线的统一任务终止清理入口没有被 perf 分支的手工字段清理覆盖。

向量化融合在空扫描、全过滤扫描、所有过近扫描时回退参考实现，保留种子格与 sensor_revision 语义；负无穷回波按参考实现处理。新增四项退化扫描逐格等价性回归。

## 验证结果

- 合并后 scan_planner_msgs / scan_planner 编译通过，BUILD_TESTING=ON。
- C++ test_replan_fsm_utils：80 项通过；test_local_task_planner：15 项通过。
- 普通 Python 回归：280 项通过。命令显式排除两个 launch 测试、一个真实 ROS 合同测试，以及依赖隧道场景的 test_tunnel_grid_third_slice.py；不是声称仓库所有场景测试全绿。
- 两个 launch 测试通过显式加载 launch_testing 插件单独运行：2 项通过，ROS_DOMAIN_ID=219。
- test_observation_task_runtime.py 真实节点合同测试：1 项通过。测试使用自己的专用 ROS 域与合成里程计，不运行 Gazebo，不接入用户仿真域。
- perf 合并前的 Explorer / task bridge / vectorized 融合回归：171 项通过。

新增检查包含非行走状态的几何恢复、FSM 实际分类调用点、路径耗尽反馈接线、同任务重路由与反馈去重、无替代路线的 sibling 仲裁，以及静态占据下的任务完成、超时、传感器证据累计。

## 运行边界

这次审核认证代码与上述测试，不等同于完成 NewMine 中的 RL 步态/最小速度现场验证。没有启动或重启 Gazebo。编译已更新本地 install；下一次新启动会使用新版消息及节点。

perf 分支原有频率改动仅把 quad_exploration 的 map_update_period 改为 0.25 s；NewMine 启动文件仍保留原有 0.5 s，LocalMapPatch 周期没有改动。这里解决的是生命周期不能被计算门跳过，不是机械统一全部周期。
