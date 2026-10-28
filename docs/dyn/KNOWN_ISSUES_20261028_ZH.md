# Campus 现场停车与区域颜色审计

本轮只读审计，无生产代码修改，无运动、取消、启停或参数修改。现场进程为 domain 148、17:35:06 启动的 dyn Campus runtime 实例，日志 /tmp/scan_newmine_rl_20261028_173506.log。原生启动与坐标修复不重新归因为本轮停车原因。

## 当前状态与最后一次失败

只读订阅确认当前机身坐标约 (-32.11077, 82.42340, -9.40359)，roll=-1.036°、pitch=-8.406°，速度为零。策略日志维持 STAND_VERIFIED、约 50Hz；Local 地图 revision 仍持续增长。Explorer 状态在 FRONTIERS_PRESENT_BUT_UNREACHABLE / LOCAL_DEPARTURE_ACTIONS_EXHAUSTED 路径上，无活动 task。现场快照保存在原测试机，仓库只保存本审计结论。

最后成功安装轨迹为 18:36:24 的 request=3628606000000、trajectory=1698。18:36:25.844–25.880，trajectory=1699 的侧移恢复候选连续在 LATERAL_RECOVERY_BRAKING 阶段被拒绝，sample_t=1.500、sampled_distance=0.217m，未找到该候选的安装提交记录。日志当时 current_margin_blocked=0、current_physical_blocked=0、candidate_physical_blocked=0。不能把这条日志解释成机器人当前已经真实撞入障碍。

此拒绝来自 scan_replan_fsm.cpp:5427 附近的组合条件：taskDomainContains 失败，或者带 capture/纵向偏差/刹停延长的 footprint tube 检查失败。两者共用 LATERAL_SWEEP_REJECTED；hit_valid=0，没有提供确切命中来源。因此已确认其认证没有通过，尚不能仅凭现有日志确定是合同域越界、真实障碍还是余量/地形表示误判。

18:36:31 与 18:36:47，Explorer 又发两个新 request。Local 普通规划报 A-star 起止点失败/资源池边界问题，随后恢复入口直接报 LATERAL_RECOVERY_ALREADY_USED，发 LOCAL_EXECUTION_BLOCKED，进入 WAIT_TARGET。此后没有新的安装成功记录。原运行日志保存在原测试机。

## 为何等待不能自恢复

scan_replan_fsm.cpp:3281–3295 在恢复入口就把 task_lateral_repair_used_ 和 departure_recovery_memory_.used 置为 true，发生在恢复候选生成、制动认证和安装之前。因此本次搜索候选没有真正执行，却仍留下“物理恢复已经用过”的记录。

observation_task_adapter.cpp:902 的 refreshDepartureRecoveryBudget 只在物理位姿/退出条件变化后重新开放。Explorer observation_task_bridge.py:462 的 local_departure_candidate_allowed 又按此前失败的路线方向限制新候选；其 physical-change 判断包含移动 >=0.30m、显著转向或局部已知障碍释放，对这些 departure 失败没有普通时钟过期释放。

双方形成闭环：Local 等认证成功的替换轨迹才能释放 safety；恢复入口因 used 拒绝进一步尝试；Explorer 按失败方向进一步压候选；位姿不变又不足以释放这些记忆。这解释了没有新任务/轨迹时长时间静止，不能用延长等待或增加 Goal 切换解决。

修复应分别记录“搜索尝试预算”和“实际安装/执行的物理动作预算”。认证失败不等于物理动作已消耗；Local 自己拥有有界恢复候选搜索和终态，Explorer 不把容量、认证失败或 ALREADY_USED 当成永久方向不可达。物理碰撞和认证要求不应被直接关闭。扫掠失败还需要明确输出 DOMAIN_OUTSIDE / collision source / margin / terrain，不能继续共用不透明理由。

## 地面估计是另一个已发现的问题，不能直接认定为此次扫掠失败的唯一来源

当前 MID360 日志持续 valid=0、support_cells=0，约 13038 个实测返回的地面高度仍未知。用已验证坐标对齐的碰撞 mesh 在当前精确 XY 下做竖直求交，地面 z=-9.738978，输入机身 z=-9.403590，相差 0.335389m。当前位置确实存在地面支撑，且机器人没有持续掉落。几何求交在原测试机完成。

当前 dyn observed_ground.hpp 仍是旧实现：body-relative Z 取样 [-0.9,0.55]m，RANSAC 支撑参考固定 body_height=.30±.06m，max_slope=.35 用于平面梯度范数（对应 atan(.35)=19.29°，不是 .35rad 的角度判断）。这部分尚未吸收此前规划的支撑参考与识别能力改造。当前净高 .335m 本身仍在其允许范围，不能仅用 Z 高度变化解释为什么本次所有地面拟合失败；需要针对本次实际射线点进一步重放支撑拟合。

## RViz 的异常颜色有三条实证

只读 MarkerArray 记录 47 个 region、15000 个填色点。47 个区域中 36 个都是同一红色，其余 11 个各一种颜色；全部 alpha≈.38，没有 active 加深颜色。frontier_explorer.py:3945–3947 分配颜色时用完 12 色后，next(...,0) 将余下全部回退到 0 号红色。直接提取原函数，以 47 个区域运行也复现一个颜色占 36 区。因此大片同红不证明这些地方业务上已合为一个区域，红色也不是危险或承诺状态。

region_overlay.py:82 把所有 region cube 的高度固定到世界 z=.05；实测所有填色点均如此，机器人则在 z=-9.40359。颜色平面在机器人上方约 9.45m，无法贴合 Campus 起伏地形，遮挡/透视关系会误导观察。可选择明确标注的二维审计平面，或仅在有可靠支撑高度的地方贴地绘制；不能从 IMU/机身高度假造全图地面。

frontier_explorer.py:3982 将显示用多源 FREE-cell BFS 总标注数截断为 15000。现场 Marker 点数正好 15000；地图 known_cells 已约172209。截断后留下任意覆盖边缘/空白，并且这个“未完成填色”直接缓存。应使业务归属计算和显示预算分离，增量完成/缓存标注后再做有界抽样，不宜简单放大循环到全图。

现有色块是 Frontier region 到已知 FREE 格子的显示扩展，不是已验收的房间/庭院语义区域，也不是 region commitment 状态图。修显示不应修改 Explorer/Local 权限或替 Local 判断通行。

## 建议改动边界

优先联合修 Local 物理预算记账时机、恢复终态和 Explorer departure 反馈归类；保留正常执行 owner 和认证。并行可独立修区域配色扩展、显示高度与填色增量预算。地面支撑拟合则用本次实际输入做确定性重放，确认触发原因后修，不能因为一个 valid=0 就把所有墙/地面证据改成 free。
