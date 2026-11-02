# 普通滚动第一批：执行进度、失败处置与计时

本批建立在 dyn 回退后的 Local 上，只处理普通滚动；速度参数、ROS 消息、Controller 安装认证和跨任务停车换向协议保持现状。

FSM 把前任进度显式传入 Manager。Manager 后缀分支不再自行用 now-start_time 推断空间跟踪进度。当前空间执行版本没有新鲜反馈时，不授权后缀复用；空轨迹和时钟类型不匹配不会做时间相减。

普通候选的接入误差返回 ENTRY_MISMATCH，不在生成函数内直接发出 HANDOFF_PREDICTED_UNSAFE。REPLAN_TRAJ 和同任务普通安装拒绝使用 keepOrdinaryPredecessor 判断是否保留旧执行。跨任务、恢复和显式转向仍走既有路径。

保留旧执行要求：执行版本匹配、Controller RUNNING、反馈年龄不超过0.25s、地图未超过既有 stale warn 阈值；没有安全停车、预测风险、转向/恢复事务；旧段具有剩余时间和覆盖反应、重试及制动距离的几何余量。对旧段检查机身足迹，最多128个样条采样，检查前后地图revision与安全generation一致。任何条件不足都不授权继续；实时安全监测持续有效。这个检查只发生在失败路径，不在每个正常执行周期增加扫描。

删除了 task 模式伪后缀尝试：task_seed优先于Manager的后缀初始化，因此之前标为reuse的尝试仍然生成相同任务种子，失败后又可能重算一次。本批没有实现尾段局部优化，task模式仍使用任务几何种子，正常安全认证完整保留。

新增日志：

- CANDIDATE_ENTRY_MISMATCH：仅说明候选不能接入，不等于机器人已经停车。
- ROLLING_PREDECESSOR_RETAINED：记录保留的请求、轨迹、进度、距离余量、反馈年龄。
- CANDIDATE_ATTEMPT_TIMING：所有进入计算的尝试退出均记录结果及wall耗时，包括提前失败；seed_ms为planning_started到任务种子完成的准备耗时，generation_ms为之后生成/优化（含preferred/hard重试）的耗时，elapsed_ms覆盖候选函数。SUBMITTED只表示发布待ACK，不表示执行成功。原PLAN_TIMING仍保留，不能把两种计时日志混成两个候选。

验证：scan_planner构建通过；开启BUILD_TESTING后，test_replan_fsm_utils共100例、test_local_task_planner共55例通过。新用例覆盖版本不符、非RUNNING、过期/非有限反馈或地图、无效/耗尽后缀进度。没有进行新一轮Gazebo运动测试，没有实测停车或生成P95改善结论。

下一轮需要完整重启运行进程，记录runtime.log与actual_route.csv。先比较相同时间窗口的>=3s停顿/100m、ENTRY_MISMATCH后是否保留旧段、拒绝到实际安装的延迟，以及全部尝试的生成P95。保持速度不变。Explorer的120秒退役与连接等待属于后续批次，本批没有修复。
