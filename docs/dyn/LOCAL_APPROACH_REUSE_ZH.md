# 同任务接近路线复用

本批针对215225复测中局部目标/接近路线准备占主耗时的问题：普通task滚动在已有接近路线可用时，先裁剪、尝试延展并重验，不再清空后立即进入最多1024次扩展的完整候选搜索。

这是任务接近路线的几何复用，尚不是B样条控制点的尾段局部优化。Manager仍拟合生成候选，Local完整校验及Controller最终安装认证继续执行；速度配置不变。跨任务、转向阶段、预测安全风险、恢复和候选失败后的替代路线搜索保留原处理。

复用要求当前request/version、路线portal cursor和端点身份一致；机器人运动中、没有相应恢复/换向/安全事务、没有已计入的规划失败。消费旧路线时，投影到尚存路线并删除已走部分，在前方约0.25m处连接；偏离超过0.25m、路线无剩余或非法输入时回退完整搜索。路线最多128点。

中间transit目标进入原有提前准备距离后，尝试从剩余路线尾端直接延展到当前参考lookahead/portal checkpoint。只有向前延展且满足边、转向、任务域/portal、可见性、终端速度/制动等原有谓词，才采用。已有capture保持停车合同；新信息使transit成为capture时重新传播停车语义。

复用和完整搜索共享edge_safe、turn_safe、goal_allowed、visible、terminal_safe，不新增一套几何安全判据。复用的线段采样上限256，使用当前规划快照重验；不信任“上次走过/上次认证过”作为当前可执行证据。成功复用跳过完整候选搜索及偏好clearance场构建；需要preferred-margin escape时仍回退原完整分支。失败不会新增物理恢复记账。

新增日志TASK_APPROACH_REUSED mode=TRIMMED|EXTENDED，记录点数、检查量、耗时；TASK_APPROACH_REBUILD记录上下文/模式不适用或重验/延展失败。上一批CANDIDATE_ATTEMPT_TIMING继续记录包含局部准备的seed_ms。ROLLING_PREDECESSOR_NOT_RETAINED记录source、gate、反馈/地图年龄、剩余轨迹参数、所需距离以及安全/转接标志；没有放宽旧段继续执行门限。

验证：生产节点构建成功；test_replan_fsm_utils 100例、test_local_task_planner 59例通过。新增4例覆盖裁剪保留终点、偏离/无效/耗尽路线回退、新起点连接与弯道重验、延展段遇新障碍拒绝。单元测试不代表整机性能改善；本轮未运行Gazebo。复测关注复用命中率、重建原因、seed_ms全样本P95、接入失败、实际>=3s停顿/100m，不仅看某一停车原因是否变少。

当前运行节点需完整重启才能加载新产物。Explorer超时/连接等待与制动后的恢复事务没有在本批修改。
