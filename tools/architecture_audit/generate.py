#!/usr/bin/env python3
"""Generate docs/ARCHITECTURE_MAP_ZH.html — interactive architecture & audit map.

Single-file, zero-dependency, offline-viewable.  Re-run after code changes:
    python3 tools/architecture_audit/generate.py

Audit findings are aligned with the status taxonomy of 问题.txt:
  open = 🔴 待修(主线) | done-perf = ✅ 已在 perf/fusion-stage1-4
  designed = 🟡 已设计待实施 | verify = ⚪ 待实测验证
Drill-down: 节点(进程) → 内部结构/组件 → 组件流水线 → 文件/发现。
"""

import datetime
import json
import os
import subprocess

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SKIP = {'.git', 'build', 'install', 'log', '__pycache__', '.pytest_cache', 'Img', 'assets'}
BIN_EXT = {'.pcd', '.dae', '.gif', '.jpg', '.jpeg', '.png', '.bin', '.so', '.stl', '.obj'}
LANG = {'.cpp': 'C++', '.h': 'C++', '.hpp': 'C++', '.py': 'Python', '.yaml': 'YAML',
        '.msg': 'msg', '.md': 'md', '.txt': 'txt', '.xml': 'xml'}


def git(*args):
    try:
        return subprocess.run(['git', *args], cwd=ROOT, capture_output=True,
                              text=True, timeout=10).stdout.strip()
    except Exception:
        return ''


def walk(path, node):
    for name in sorted(os.listdir(path)):
        if name in SKIP:
            continue
        full = os.path.join(path, name)
        rel = os.path.relpath(full, ROOT)
        if os.path.isdir(full):
            sub = {'t': 'd', 'n': name, 'p': rel, 'c': {}}
            node['c'][name] = sub
            walk(full, sub)
        else:
            ext = os.path.splitext(name)[1].lower()
            loc, lang, binary = None, LANG.get(ext, ext.lstrip('.') or '?'), ext in BIN_EXT
            if not binary:
                try:
                    with open(full, 'rb') as f:
                        loc = sum(1 for _ in f)
                    if loc > 8000:
                        binary, loc = True, None
                except Exception:
                    binary = True
            node['c'][name] = {'t': 'f', 'n': name, 'p': rel,
                               'loc': loc, 'lang': 'bin' if binary else lang}
    return node


def summarize(node):
    if node['t'] in ('d', 'root'):
        files = loc = 0
        for c in node.get('c', {}).values():
            f, l = summarize(c)
            files += f
            loc += l
        node['files'], node['loc'] = files, loc
        return files, loc
    return 1, node.get('loc') or 0


tree = walk(ROOT, {'t': 'root', 'c': {}})
summarize(tree)


def inventory_paths(node):
    if node['t'] == 'f':
        return [node['p']]
    return [path for child in node.get('c', {}).values()
            for path in inventory_paths(child)]


FILE_PATHS = inventory_paths(tree)


def canonical_reference(reference):
    """Resolve only unambiguous inventory matches; never guess by longest path."""
    path, separator, location = reference.partition(':')
    candidates = [p for p in FILE_PATHS if p == path or p.endswith('/' + path)]
    if len(candidates) != 1:
        return reference
    return candidates[0] + (separator + location if separator else '')

# ---------------------------------------------------------------- runtime nodes
NODES = [
    {'id': 'gazebo', 'name': 'Gazebo Fortress / SDF 仿真', 'layer': 'sim',
     'pos': [26, 258], 'file': 'src/simulator/…; mine_tunnel_world',
     'role': '物理仿真与传感器源。gpu_lidar 10Hz(720 线,0.2–7.5m)、IMU 200Hz、odometry 100Hz、足端接触 100Hz(当前无消费者)。两种执行模式:planning_kinematic(VelocityControl 速度级)与 rl_effort(gz_ros2_control 力矩)。',
     'in': [], 'out': ['/clock', '/quad_0/cloud 10Hz', '/go2/imu 200Hz', '/quad_0/body_pose 100Hz', '/go2/*_foot_contact 100Hz'],
     'notes': ['world 插件名用裸名在 Fortress 6.18 会加载失败(SceneBroadcaster)——用 lib*.so 完整文件名(NewMine.sdf 与自建 world 均有此雷)']},
    {'id': 'bridge', 'name': 'ros_gz_bridge', 'layer': 'sim',
     'pos': [236, 258], 'file': 'src/simulator/Utils/go2_description/config/bridge.yaml',
     'role': 'gz↔ROS 桥。注意 gpu_lidar 实际话题在 /quad_0/lidar/points/points(gz-transport 追加 /points)。',
     'in': ['gz 各话题'], 'out': ['/clock', '/quad_0/cloud', '/go2/imu', '/quad_0/body_pose', '/go2/*_foot_contact', '/tf'],
     'notes': ['/quad_0/cmd_vel 为 ROS→GZ 反向桥(kinematic 模式用)']},
    {'id': 'explorer', 'name': 'frontier_explorer', 'layer': 'decision',
     'pos': [466, 84], 'file': 'src/planner/plan_manage/scripts/frontier_explorer.py (3953行)',
     'role': '探索任务的决策者，也是实际观测进度的唯一权威。点云融合与 0.25s 决策定时器是不同回调；当前主线融合节流为 0.5s。选择区域、统计目标格的真实观测证据，发布粗路线或观测任务。Local 可以调整任务内站位，但不能据预测可见性宣布任务完成。',
     'in': ['/quad_0/cloud', '/quad_0/body_pose', '/robot/locomotion_state', '/planning/status', '/planning/blocked_segment', '/planning/observation_feedback', '/planning/local_execution_event (task_mode 下非完成权威)', 'global_representation/topology_delta', 'global_representation/failure_evidence'],
     'out': ['/initial_path | /observation_task', '/explorer/map', '/explorer/metrics', 'map_updates'],
     'notes': ['证据制栅格(0.2m):可逆占据 + first-occluder 弱化', '决策拍已按 map_content_revision 门控(perf 分支)', 'planning map 全量 tolist() 每 0.5s ≈ 2M 元素(F4)'],
     'comps': ['grid2d', 'commitment', 'sparse', 'explorer', 'taskbridge'],
     'flow': [
         {'label': 'cloud_callback 融合', 'sub': '0.25/0.5s 节流 · 720 射线 · 证据制', 'file': 'explorer_core/grid.py'},
         {'label': 'exploration_timer 决策拍', 'sub': '0.25s · revision 门控', 'file': 'frontier_explorer.py:2327'},
         {'label': '观测完成仲裁', 'sub': '完成拥有最高优先级', 'file': 'frontier_explorer.py:2338'},
         {'label': 'choose_frontier', 'sub': '承诺门 → 稀疏估价 → A* 终验', 'file': 'frontier_explorer.py:2509'},
         {'label': 'publish 发布', 'sub': '/initial_path 或 /observation_task', 'file': 'frontier_explorer.py:3507'},
     ]},
    {'id': 'globalrep', 'name': 'global_representation', 'layer': 'decision',
     'pos': [466, 428], 'file': 'src/planner/plan_manage/scripts/global_representation_node.py (521行)',
     'role': '全局稀疏拓扑表示。每 0.5s 消费 LocalMapPatch:clearance 场(heapq Dijkstra)→ Zhang-Suen 骨架化 → 节点/边提取(世界量化稳定 ID)→ 块级增量 TopoGraphDelta;每 10 revision 全量快照;failure_evidence 固化为图级视点约束。单线程 rclpy.spin。',
     'in': ['grid_map/local_map_patch 0.5s', 'planning/failure_evidence'],
     'out': ['global_representation/topology_delta', 'topology_snapshot (10 rev)', 'safe_region_snapshot', 'failure_evidence(转发)'],
     'notes': ['纯 Python 逐像素热点(F7):Zhang-Suen + clearance Dijkstra 每 patch 全量重算', 'P2-③:增量 GVD + 波前骨架(双轨)'],
     'comps': ['sparse'],
     'flow': [
         {'label': 'patch_callback', 'sub': '0.5s 一帧 · revision 去重', 'file': 'global_representation_node.py:247'},
         {'label': 'clearance 场', 'sub': 'heapq 逐像素 Dijkstra(热点)', 'file': 'explorer_core/topology.py:88'},
         {'label': '骨架化 + 节点/边', 'sub': 'Zhang-Suen → 端点/交叉点', 'file': 'explorer_core/topology.py:45'},
         {'label': '显式封锁确认', 'sub': '连续 3 patch', 'file': 'explorer_core/topology_persistence.py'},
         {'label': 'TopoGraphDelta 发布', 'sub': '每帧增量 + 每 10 rev 快照', 'file': 'global_representation_node.py:358'},
     ]},
    {'id': 'scan', 'name': 'scan_planner_node(C++ FSM)', 'layer': 'plan',
     'pos': [726, 258], 'file': 'src/planner/plan_manage/src/scan_replan_fsm.cpp (5100+行)',
     'role': 'C++ 局部规划。六态 FSM(INIT→WAIT_TARGET→GEN_NEW_TRAJ→REPLAN_TRAJ⇄EXEC_TRAJ→EMERGENCY_STOP)+ 20Hz 安全回调;事务化交接(pending_handoff/版本化 ACK);结构化修复(锚点 ≤6 候选、3 段/事务);clearance-limited 受限执行;GridMap 20Hz 融合 + TLS 快照 + LocalMapPatch 在同进程。',
     'in': ['/quad_0/body_pose', '/initial_path | /observation_task', '/planning/execution_state', '/planning/trajectory_ack', '/robot/locomotion_state'],
     'out': ['/planning/bspline', '/planning/execution_command', '/planning/blocked_segment', '/planning/failure_evidence', '/planning/status', '/planning/observation_feedback', 'grid_map/local_map_patch 0.5s'],
     'notes': ['getReferencePathLocalTarget:reference 模式目标只沿折线缩短、不横移', 'task_mode → selectObservationTaskTarget(域内自选站位,见 LocalTaskPlanner)'],
     'comps': ['gridmap', 'fsm', 'ltp', 'optimizer', 'astar'],
     'flow': [
         {'label': 'GridMap 融合', 'sub': '20Hz raycast + TLS 快照 + 膨胀', 'file': 'plan_env/src/grid_map.cpp:1110'},
         {'label': 'FSM 六态', 'sub': '100Hz exec + 20Hz 安全', 'file': 'scan_replan_fsm.cpp:1768'},
         {'label': '目标选择', 'sub': 'reference 弧长前视 / task 域内站位', 'file': 'scan_replan_fsm.cpp:5194'},
         {'label': 'callReboundReplan', 'sub': 'A* 重连 → 软走廊 LBFGS → 终验 → 扫掠', 'file': 'scan_replan_fsm.cpp:4100'},
         {'label': '版本化 ACK', 'sub': 'pending_handoff → EXEC_TRAJ', 'file': 'scan_replan_fsm.cpp:1265'},
     ]},
    {'id': 'controller', 'name': 'closed_loop_controller', 'layer': 'exec',
     'pos': [956, 152], 'file': 'src/planner/plan_manage/src/closed_loop_controller.cpp (850+行)',
     'role': '按当前已接收的轨迹和执行合同生成速度命令，并回报版本化 ACK/执行状态。跟踪含 B 样条采样、位置反馈与航向门控；spatial gait 按轨迹携带的最低命令与有效速度校准，并非始终固定 0.30。HOLD 输出零命令，但实际停稳还依赖运动策略与物理动力学。100Hz 是定时器配置。',
     'in': ['/planning/bspline', '/quad_0/body_pose', '/planning/execution_command', '/robot/locomotion_state', 'simulation/collision'],
     'out': ['/planning/cmd_vel_raw 100Hz', '/planning/execution_state', '/planning/trajectory_ack', '/planning/go2_heading_error|stalled', '/planning/local_execution_event'],
     'notes': ['publishStop 调用点 10 处——空闲即零速,勿与探针/其他指令共存'],
     'comps': ['controller'],
     'flow': [
         {'label': 'cmdCallback 100Hz', 'sub': '门控:locomotion/碰撞/soft-hold', 'file': 'closed_loop_controller.cpp:617'},
         {'label': 'spatial gait 进度', 'sub': '0.05s 步找最近样条点', 'file': 'closed_loop_controller.cpp:664'},
         {'label': '航向判定', 'sub': '|err|>0.8rad → freeze 原地转', 'file': 'closed_loop_controller.cpp:747'},
         {'label': 'P 律跟踪', 'sub': 'vel_des + 0.8·p_error 限幅', 'file': 'closed_loop_controller.cpp:786'},
         {'label': '发布 cmd_vel_raw', 'sub': '→ supervisor 门控', 'file': 'closed_loop_controller.cpp:850'},
     ]},
    {'id': 'supervisor', 'name': 'go2_locomotion_supervisor', 'layer': 'motion',
     'pos': [956, 386], 'file': 'src/planner/plan_manage/scripts/go2_locomotion_supervisor.py',
     'role': 'Go2 运动唯一属主:力矩 getup → 站立校验(STAND_VERIFIED)→ 内嵌 ONNX 50Hz 策略(parkour_moe_full_model)+ 200Hz PD(kp40/kd1.0)→ 关节 effort;命令门控(/planning/cmd_vel_raw → 健康过滤 → 策略);capabilities() 派生 actuation_ready/perception_pose_valid 两个门控布尔,全栈消费。',
     'in': ['/planning/cmd_vel_raw', '/joint_states', '/quad_0/body_pose', '/go2/imu'],
     'out': ['/robot/locomotion_state (transient_local)', '/joint_group_effort_controller/commands', '/robot/locomotion_diagnostics', 'service: /robot/enter_navigation'],
     'notes': ['取代了 go2_policy_node/go2_onnx_policy_node 两个遗留执行器', '死区标定:命令 0.30 ↔ 有效速度 0.165 m/s(tools/min_speed_probe)'],
     'comps': ['supervisor'],
     'flow': [
         {'label': 'getup 状态机', 'sub': '力矩起身 → 25 周期稳定判据', 'file': 'go2_locomotion_supervisor.py:634'},
         {'label': 'STAND_VERIFIED', 'sub': '站立校验通过 → STATE_RUN', 'file': 'go2_locomotion_supervisor.py:667'},
         {'label': '命令门控', 'sub': 'cmd_vel_raw → 健康/斜率限幅', 'file': 'go2_locomotion_supervisor.py:700'},
         {'label': 'ONNX 50Hz', 'sub': '45 维 obs + 450 历史 → action', 'file': 'go2_locomotion_supervisor.py:455'},
         {'label': 'PD 200Hz', 'sub': 'kp40/kd1.0 → 关节 effort', 'file': 'go2_locomotion_supervisor.py:707'},
     ]},
    {'id': 'rviz', 'name': 'RViz2', 'layer': 'sim',
     'pos': [236, 80], 'file': 'src/planner/plan_manage/launch/rviz.launch.py',
     'role': '可视化:explorer/map、bspline 轨迹、拓扑图、目标标记。go2_tf_bridge 提供 3 条静态 TF(lidar 外参 +0.10,0,+0.12)。',
     'in': ['各可视化话题'], 'out': [],
     'notes': []},
]

EDGES = [
    ['gazebo', 'bridge', '点云/IMU/odom/clock', 0],
    ['bridge', 'explorer', '/quad_0/cloud 10Hz · body_pose', -14],
    ['bridge', 'scan', 'cloud · body_pose', 10],
    ['explorer', 'scan', 'initial_path | observation_task', -18],
    ['scan', 'globalrep', 'local_map_patch 0.5s', 14],
    ['globalrep', 'explorer', 'topology_delta · 失败证据', -14],
    ['scan', 'controller', 'bspline · execution_command', -12],
    ['controller', 'scan', 'trajectory_ack · execution_state', 12],
    ['scan', 'explorer', 'status · blocked_segment · task反馈', 32],
    ['scan', 'globalrep', 'failure_evidence', -22],
    ['controller', 'supervisor', '/planning/cmd_vel_raw 100Hz', -8],
    ['supervisor', 'scan', 'locomotion_state(门控)', 10, True],
    ['supervisor', 'explorer', 'locomotion_state(门控)', 16, True],
    ['supervisor', 'controller', 'locomotion_state(门控)', -18, True],
    ['supervisor', 'gazebo', 'effort → ros2_control → 关节', 24],
]

# ---------------------------------------------------------------- components
COMPONENTS = [
    {'id': 'gridmap', 'name': 'GridMap 融合', 'lang': 'C++',
     'files': ['src/planner/plan_env/src/grid_map.cpp', 'src/planner/plan_env/include/plan_env/grid_map.h'],
     'role': '0.05m 三维滑动窗(10×10×5m,~80MB 固定)log-odds 融合:位姿历史插值配准 → 20Hz raycast → 增量膨胀(0.35m 柱)→ 不可变 TLS 快照(读侧无锁)→ 0.5s LocalMapPatch(0.2m 切片)。',
     'in': ['/quad_0/cloud', 'sensor_pose', 'body_pose', '/robot/locomotion_state(门控)'],
     'out': ['膨胀快照(TLS)', 'grid_map/local_map_patch'],
     'mech': ['hit_wins 投票 + 3 帧 free 确认才允许削墙', 'occupied_miss_confirmation 防抖', '双圆柱机体自过滤(0.35m/0.18m)', 'locomotionReady 门:无状态心跳时静默丢帧(A3)'],
     'findings': ['A3'],
     'pipeline': ['cloudCallback 配准(位姿插值)', 'raycastProcess 20Hz(log-odds)', 'updateInflation(0.35m 柱状)', 'TLS 不可变快照', 'LocalMapPatch 0.5s']},
    {'id': 'fsm', 'name': 'SCANReplanFSM', 'lang': 'C++',
     'files': ['src/planner/plan_manage/src/scan_replan_fsm.cpp', 'src/planner/plan_manage/include/plan_manage/scan_replan_fsm.h', 'src/planner/plan_manage/include/plan_manage/replan_fsm_utils.h'],
     'role': '局部规划与安全交接的所有者。选局部目标、调用优化器、验证完整候选和交接扫掠，然后等待 Controller 的版本化 ACK 才替换执行轨迹。正常滚动、区间修补、后缀复用和静止恢复是按条件选择的分支，不是每次都顺序执行。紧急停车不等待候选 ACK。',
     'in': ['/initial_path 或 /observation_task', 'body_pose', 'ACK/状态'],
     'out': ['/planning/bspline', 'blocked_segment', 'failure_evidence', 'status'],
     'mech': ['pending_handoff 期间 FSM 整体冻结', '安全闩锁三条件白名单', '结构化修复期间禁止普通滚动重规划', 'task_mode:兄弟重规划(路线失败≠任务失败)'],
     'findings': ['A2', 'A6'],
     'pipeline': ['WAIT_TARGET(0.5s 重发)', 'GEN_NEW_TRAJ(callReboundReplan)', 'pending_handoff 冻结', 'ACK → EXEC_TRAJ', 'REPLAN(滚动/区间修补/结构化修复)', 'EMERGENCY_STOP']},
    {'id': 'ltp', 'name': 'LocalTaskPlanner [新]', 'lang': 'C++',
     'files': ['src/planner/plan_manage/include/plan_manage/local_task_planner.h', 'src/planner/plan_manage/src/observation_task_adapter.cpp'],
     'role': '运行在 SCAN 进程内的库，不是独立节点。输入任务范围、地图查询和当前位置，搜索局部站位与接近路线。当前代码混合了运动硬约束、预测可见性、已访问站位记忆和方向限制；这些局部代理不能替代 Explorer 的实际观测账本。路线形状还要由 FSM 在优化后检查。',
     'in': ['/observation_task', '地图快照', 'odom'],
     'out': ['local_target / approach 路线', 'observation_feedback', 'VIEWPOINT_LOCAL_REJECTED'],
     'mech': ['edge_safe：检查运动边是否可走', 'visible：预测能否看见代表目标格，不代表这些格已经被观测', 'goal_allowed：当前还含已访问/方向代理；弯曲路线使用全程方向向量存在误杀风险', 'guide tube 由优化器承接；实际任务完成由 Explorer 判定'],
     'findings': ['F12'],
     'pipeline': ['有效旧目标优先，减少站位跳变', '域内搜索，调用 edge_safe / visible / goal_allowed', '返回局部目标与接近路线，或无候选原因', 'FSM 优化后再检查路线形状', 'FSM 发布到站或失败反馈；Explorer 决定任务去留']},
    {'id': 'optimizer', 'name': 'BsplineOptimizer / Manager', 'lang': 'C++',
     'files': ['src/planner/bspline_opt/src/bspline_optimizer.cpp', 'src/planner/plan_manage/src/planner_manager.cpp'],
     'role': '优化器与 Manager 生成 B 样条候选：穿障段 A* 重连，L-BFGS 平衡平滑、避障、可行性与软走廊偏离。时间可行性可在 Manager 内调整；受限执行重定时、完整足迹检查、制动和交接扫掠由 FSM 负责，不属于优化器已认证的输出。',
     'in': ['控制点种子', '地图快照', 'guide path'],
     'out': ['待 FSM 完整验证的 B 样条候选（不是发布即执行）'],
     'mech': ['两段余量:preferred 0.20m 失败回退 0.10m', 'min_forward_command=0.30 / effective=0.165(死区标定)'],
     'findings': ['A6', 'F12', 'A5'],
     'pipeline': ['setGuidePath 种子注入', 'initControlPoints(A* 穿障重连)', '软走廊约束(0.20m 管)', 'L-BFGS rebound(平滑/避障/可行性)', '时间重分配 / clearance-limited 重定时', '0.02s 全样终验 + 交接扫掠']},
    {'id': 'astar', 'name': 'DynAStar', 'lang': 'C++',
     'files': ['src/planner/path_searching/src/dyn_a_star.cpp', 'src/planner/path_searching/include/path_searching/dyn_a_star.h'],
     'role': '2.5D A*(2D 8 连通 + z 平面插值),B 样条穿障重连用。0.2s 超时。',
     'in': ['起终点', '地图快照'], 'out': ['路径'],
     'mech': ['节点池 100³'],
     'findings': ['A1', 'A2'],
     'pipeline': ['起终点锚点与节点池检查', '邻域扩展与优先队列（inf 常量实际为 0，见 A1）', '路径回溯或失败原因']},
    {'id': 'controller', 'name': 'ClosedLoopController', 'lang': 'C++',
     'files': ['src/planner/plan_manage/src/closed_loop_controller.cpp'],
     'role': '见节点卡片。P 律 + spatial gait + heading freeze + soft-hold;publishStop 10 处调用点。',
     'in': ['bspline', 'odom', 'execution_command', 'locomotion_state'],
     'out': ['/planning/cmd_vel_raw', 'execution_state/ack/事件'],
     'mech': ['calibratedForwardGaitCommand:有效速度→命令折算,夹 [0.30, max]'],
     'findings': [],
     'pipeline': ['locomotionReady 门', 'spatial gait 进度推进', '航向 freeze 判定', 'P 律 vel+0.8·err(限幅)', '死区校准 calibratedForwardGaitCommand', '发布 cmd_vel_raw']},
    {'id': 'explorer', 'name': 'Explorer 决策拍', 'lang': 'Python',
     'files': ['src/planner/plan_manage/scripts/frontier_explorer.py'],
     'role': '0.25s 决策定时器先检查健康和启动条件，再判定实际观测完成；完成可优先发起下一任务，不必等旧局部请求结束。未完成时才处理在途请求、站位反馈和活动目标维护；需要新目标时选择 frontier 并发布。定时器周期不是一次全局规划的保证耗时。',
     'in': ['cloud/body_pose/locomotion', 'planning/status 族', 'topology_delta'],
     'out': ['/initial_path | /observation_task', 'explorer/map', 'metrics'],
     'mech': ['PlanningSnapshotCoordinator 快照绑定', 'ReplanGate:revision 上下文限 2 次', 'information completion owns handoff'],
     'findings': ['A7', 'A8', 'F4', 'F19'],
     'pipeline': ['健康/启动条件', '实际观测完成仲裁（优先于在途请求）', '在途请求与局部反馈', '目标维护(超时/冷却/封锁)', 'choose_frontier(承诺门→估价→终验)', '发布任务/路径，等待状态反馈']},
    {'id': 'grid2d', 'name': 'ExplorationGrid', 'lang': 'Python',
     'files': ['src/planner/plan_manage/scripts/explorer_core/grid.py'],
     'role': '0.2m 证据制三态栅格(int8 占据 + int16 证据):720 平面射线 bin → Bresenham 融合,可逆占据(3 帧 free 确认)、first-occluder 弱化、方位角膨胀堵缝。固定声明尺寸,越界=OCCUPIED。',
     'in': ['点云（当前主线 0.5s 节流）'], 'out': ['planning_free/frontier_cells/膨胀集'],
     'mech': ['perf/fusion-stage1-4 有向量化、掩膜和字节序列化实现；尚不能据此认定主线整拍耗时为 1.4ms，见 B1'],
     'findings': ['F16', 'B1'],
     'pipeline': ['720 方位 bin 分类(z 带)', 'Bresenham 融合（主线参考实现）', '证据更新与首个遮挡格处理', '生成膨胀集合', 'Explorer 组装并发布地图；perf 优化另见 B1']},
    {'id': 'commitment', 'name': '两级承诺系统', 'lang': 'Python',
     'files': ['src/planner/plan_manage/scripts/explorer_core/frontier_regions.py', 'src/planner/plan_manage/scripts/explorer_core/region_commitment.py'],
     'role': 'Explorer 的区域承诺：匹配区域身份、继承 lineage，控制何时继续当前区域、何时释放。它与 Local 的站位迟滞不是同一种进度账本。G1/G2 在任务交接讨论中表示先后两个目标，不应同时用作区域层/站位层的编号。',
     'in': ['frontier 簇', '观测目标格', 'map revision', '状态回流'],
     'out': ['active_scope/释放原因/方向轴', '指标计数'],
     'mech': ['frontier 增长视为新工作而非停滞', '完成驱动转移(仅此一条跨 scope 路径)'],
     'findings': ['A7', 'A8', 'E1'],
     'pipeline': ['簇 → 拓扑挂接 → 分区', 'IoU+质心全局匹配(承诺者优先)', 'scope=(lineage,id)/safe-region', '去抖(missing/unreachable)', '残值门(exhausted/stagnant)', '滚动迟滞 + 方向门']},
    {'id': 'sparse', 'name': 'SparseRouteGraph / 骨架', 'lang': 'Python',
     'files': ['src/planner/plan_manage/scripts/explorer_core/sparse_routing.py', 'src/planner/plan_manage/scripts/explorer_core/topology.py', 'src/planner/plan_manage/scripts/global_representation_node.py'],
     'role': '世界量化稳定节点 ID(uint64)+ FNV 边 ID;1m 桶索引;一次多源 Dijkstra 回答全部候选;显式封锁 3 patch 确认;逐折线点采样桶。稀疏从不权威:发布前逐段 Bresenham 稠密验证。',
     'in': ['local_map_patch'], 'out': ['topology_delta/snapshot', '挂接/估价'],
     'mech': ['component_id 按 revision 重建 → 只可做可达性,不可做承诺身份(scope docstring 自证)'],
     'findings': ['F7', 'G1'],
     'pipeline': ['patch → clearance 场', '骨架化(端点/交叉点)', '稳定 ID(世界量化 uint64)', '块级 delta + 显式封锁确认', '镜像估价(多源 Dijkstra)']},
    {'id': 'supervisor', 'name': 'LocomotionSupervisor', 'lang': 'Python',
     'files': ['src/planner/plan_manage/scripts/go2_locomotion_supervisor.py'],
     'role': '见节点卡片。ONNX obs:45 维(角速度×0.25/projected_gravity/cmd×[2,2,0.25]/关节偏差/关节速度×0.05/last_action)+ 450 维历史;动作=DEFAULT+0.25×action;cmd_timeout 0.5s。',
     'in': ['cmd_vel_raw', 'joint_states', 'imu', 'body_pose'],
     'out': ['effort', 'locomotion_state'],
     'mech': ['斜率限幅 [0.50,0.40,1.00]/s', 'policy 线程只保留最新请求'],
     'findings': [],
     'pipeline': ['命令门控(健康过滤)', '斜率限幅 [0.50,0.40,1.00]/s', 'ONNX 推理(独立线程)', 'PD 力矩(200Hz)', 'effort 发布 + 状态广播']},
    {'id': 'taskbridge', 'name': 'ObservationTask 协议 [新]', 'lang': 'C++/Python',
     'files': ['src/planner/scan_planner_msgs/msg/', 'src/planner/plan_manage/src/observation_task_adapter.cpp', 'src/planner/plan_manage/scripts/explorer_core/observation_task_bridge.py'],
     'role': 'ObservationTask 携带 task_id/version、request_id、region_id、粗路线和允许活动范围。target_cells 是剩余边界的代表格，不是完整完成判据。任务、请求、局部站位 candidate、执行 trajectory 标识各自解决不同关联问题；RETIRE_TASK 用于旧任务退役，不是停车标定。',
     'in': ['Explorer 任务'], 'out': ['FSM task_mode 状态机'],
     'mech': ['信息完成拥有 tick 最高优先级', '任务内换站位/换路线不换任务'],
     'findings': [],
     'pipeline': ['Explorer 组装任务与剩余代表格', 'FSM 选择站位 candidate，生成 trajectory', 'Controller ACK 关联执行版本', '到站/失败反馈回 Explorer；完成或替换时退役旧任务']},
]

# ---------------------------------------------------------------- logic trees
# 递归逻辑树:{l:名称, s:说明, f:源文件(叶标签), c:[子节点], find:[发现ID]}
GRAPHS = {
 'scan': {
  'title': 'scan_planner_node 内部结构', 'chain': False, 'desc': 'GridMap 供图,FSM 决策,优化器出轨迹,A* 做穿障重连,安全回调独立刹车。',
  'blocks': [
   {'id': 'gm', 'l': 'GridMap 融合', 's': '20Hz raycast · TLS 快照 · 膨胀', 'f': 'plan_env/src/grid_map.cpp', 'x': 40, 'y': 60, 'graph': 'gridmap'},
   {'id': 'fsm', 'l': 'FSM 六态', 's': '100Hz exec + 20Hz safety', 'f': 'scan_replan_fsm.cpp:1768', 'x': 430, 'y': 60, 'graph': 'fsm'},
   {'id': 'safety', 'l': '安全回调 20Hz', 's': '制动扫掠/余项扫描/偏差分类', 'f': 'scan_replan_fsm.cpp:3304', 'x': 820, 'y': 60},
   {'id': 'ltp', 'l': 'LocalTaskPlanner 库', 's': 'task_mode 内的局部站位选择', 'f': 'local_task_planner.h', 'x': 40, 'y': 340, 'graph': 'ltp'},
   {'id': 'opt', 'l': 'BsplineOptimizer', 's': 'A* 重连 + 软走廊 L-BFGS', 'f': 'bspline_optimizer.cpp', 'x': 430, 'y': 340, 'graph': 'optimizer'},
   {'id': 'astar', 'l': 'DynAStar', 's': '节点池世界范围取决于步长，见 A2', 'f': 'path_searching/src/dyn_a_star.cpp', 'x': 820, 'y': 340},
  ],
  'edges': [
   ['gm', 'fsm', '膨胀快照 TLS 供查询'],
   ['ltp', 'fsm', '局部目标/站位'],
   ['fsm', 'opt', '局部目标 + seed'],
   ['opt', 'astar', '穿障段查询'],
   ['astar', 'opt', '重连路径'],
   ['opt', 'fsm', '候选轨迹 → ACK 事务'],
   ['fsm', 'safety', '执行轨迹'],
   ['safety', 'fsm', 'HOLD / 重规划请求'],
  ]},
 'fsm': {
  'title': 'FSM 状态与候选交接', 'chain': False, 'desc': '箭头表达主要转移条件，不是每轮顺序。候选 ACK 与 FSM 状态不同；紧急停车可直接触发，恢复还需健康、安全和候选验证，不能只凭速度小就解除。',
  'blocks': [
   {'id': 'init', 'l': 'INIT', 's': '等待初始化与有效输入', 'f': 'scan_replan_fsm.cpp', 'x': 40, 'y': 340},
   {'id': 'wait', 'l': 'WAIT_TARGET', 's': '0.5s 重发等待认领', 'f': 'scan_replan_fsm.cpp', 'x': 40, 'y': 60},
   {'id': 'gen', 'l': 'GEN_NEW_TRAJ', 's': 'callReboundReplan 11 步链', 'f': 'scan_replan_fsm.cpp:2108', 'x': 430, 'y': 60, 'graph': 'gen'},
   {'id': 'exec', 'l': 'EXEC_TRAJ', 's': '滚动触发:双进度 ≥1.0m', 'f': 'scan_replan_fsm.cpp:2346', 'x': 820, 'y': 60, 'graph': 'exec'},
   {'id': 'estop', 'l': 'EMERGENCY_STOP', 's': '同步静止样条,不等 ACK', 'f': 'scan_replan_fsm.cpp:4947', 'x': 430, 'y': 340},
   {'id': 'replan', 'l': 'REPLAN_TRAJ', 's': '选择修补分支，验证替换候选', 'f': 'scan_replan_fsm.cpp', 'x': 820, 'y': 340},
  ],
  'edges': [
   ['init', 'wait', '初始化完成'],
   ['wait', 'gen', '目标/路径到达'],
   ['gen', 'exec', 'ACK ACCEPTED(版本化)'],
   ['exec', 'replan', '滚动触发/可修补冲突'],
   ['replan', 'exec', '有效候选完成 ACK'],
   ['gen', 'wait', '任务拒绝/等待新输入'],
   ['exec', 'estop', '预测硬碰撞'],
   ['estop', 'gen', '满足恢复条件，生成安全候选'],
  ]},
 'gen': {
  'title': 'callReboundReplan 规划链', 'chain': True,
  'blocks': [
   {'id': 's1', 'l': '快照 + 目标选择', 's': 'reference 弧长前视 / task 域内站位', 'f': 'scan_replan_fsm.cpp:5194'},
   {'id': 's2', 'l': '两段余量规划', 's': 'preferred 0.20m 失败回退 0.10m', 'f': 'planner_manager.cpp:87'},
   {'id': 's3', 'l': 'A* 穿障重连', 's': 'initControlPoints；范围见 A2', 'f': 'bspline_optimizer.cpp:69'},
   {'id': 's4', 'l': '软走廊 L-BFGS rebound', 's': 'guide tube 0.20m(F12)', 'f': 'bspline_optimizer.cpp:1179'},
   {'id': 's5', 'l': '0.02s 全样终验', 's': '足迹 + 余量带逐样检查', 'f': 'scan_replan_fsm.cpp:4376'},
   {'id': 's6', 'l': '重定时 + 交接扫掠', 's': '0.165m/s 受限剖面 → pending_handoff', 'f': 'scan_replan_fsm.cpp:4462'},
  ]},
 'exec': {
  'title': '执行中的替换候选来源', 'chain': False, 'desc': '这些是按冲突类型、跟踪偏差和恢复所有权选择的替代分支。每个分支产生候选后仍须完整验证与 ACK；不是滚动后必做区间修补、再逃逸。',
  'blocks': [
   {'id': 't1', 'l': '滚动触发', 's': 'planned+odom 双进度 ≥1.0m', 'f': 'scan_replan_fsm.cpp:2346'},
   {'id': 't2', 'l': '区间修补', 's': '预测碰撞区间 seed 修补', 'f': 'scan_replan_fsm.cpp:2475'},
   {'id': 't3', 'l': '后缀复用', 's': '跟踪误差 ≤0.30m', 'f': 'scan_replan_fsm.cpp:2650'},
   {'id': 't4', 'l': '结构化修复', 's': '锚点 0.45-0.9m · ≤6 候选 · 3 段/事务', 'f': 'scan_replan_fsm.cpp:2735'},
  ]},
 'optimizer': {
  'title': '优化候选到最终认证的责任边界', 'chain': True, 'desc': '前四块属于 Manager/Optimizer；最后两块回到 FSM。优化器输出的是候选，完整认证和交接不由优化器单独承担。',
  'blocks': [
   {'id': 'o1', 'l': 'setGuidePath 种子注入', 's': 'seed = 已验证几何引导', 'f': 'planner_manager.cpp:360'},
   {'id': 'o2', 'l': 'A* 穿障重连', 's': 'initControlPoints', 'f': 'bspline_optimizer.cpp:69'},
   {'id': 'o3', 'l': '软走廊约束', 's': 'guide tube 0.20m(F12)', 'f': 'bspline_optimizer.cpp:1203'},
   {'id': 'o4', 'l': 'L-BFGS rebound', 's': '平滑/避障/可行性梯度', 'f': 'bspline_optimizer.cpp:393'},
   {'id': 'o5', 'l': '重定时', 's': 'clearance-limited 0.165m/s 剖面', 'f': 'scan_replan_fsm.cpp:4462'},
   {'id': 'o6', 'l': '0.02s 终验 + 交接扫掠', 's': '足迹逐样 + 旋转平移扫掠', 'f': 'scan_replan_fsm.cpp:4376'},
  ]},
 'ltp': {
  'title': 'SCAN 内的局部目标选择与反馈', 'chain': True, 'desc': '这是库调用与 FSM 后处理的概念链，不是独立 ROS 节点。预测可见性仅用于选站位；真实观测进度和任务完成仍归 Explorer。',
  'blocks': [
   {'id': 'p1', 'l': 'retain_goal 迟滞', 's': '>0.6m 优先旧目标，仍须可行', 'f': 'observation_task_adapter.cpp:221'},
   {'id': 'p2', 'l': '域采样 + edge_safe', 's': '足迹/方向 cos0.8/进度单调', 'f': 'local_task_planner.h'},
   {'id': 'p3', 'l': '预测可见性与局部记忆', 's': '不是实际观测完成的账本', 'f': 'observation_task_adapter.cpp:267'},
   {'id': 'p4', 'l': 'FSM 调用 taskShapeResult', 's': '优化后的路线形状检查', 'f': 'scan_replan_fsm.cpp:4377'},
   {'id': 'p5', 'l': 'NO_NEW_OBSERVATION_STATION', 's': '耗尽 → 任务级升级', 'f': 'observation_task_adapter.cpp:241'},
  ]},
 'gridmap': {
  'title': 'GridMap 融合流水线', 'chain': True,
  'blocks': [
   {'id': 'g1', 'l': 'cloudCallback 配准', 's': '位姿历史 EXACT/插值/最近邻', 'f': 'grid_map.cpp:1471'},
   {'id': 'g2', 'l': 'raycastProcess', 's': 'log-odds · hit_wins · 3 帧确认', 'f': 'grid_map.cpp:850'},
   {'id': 'g3', 'l': 'updateInflation', 's': '0.35m 柱状增量膨胀', 'f': 'grid_map.cpp:555'},
   {'id': 'g4', 'l': 'TLS 不可变快照', 's': '读侧无锁', 'f': 'grid_map.cpp:207'},
   {'id': 'g5', 'l': 'LocalMapPatch', 's': '0.5s · 0.2m 切片', 'f': 'grid_map.cpp:1190'},
  ]},
 'timer': {
  'title': 'exploration_timer 决策拍', 'chain': True,
  'blocks': [
   {'id': 't1', 'l': 'locomotion 门', 's': 'actuation/perception 双布尔'},
   {'id': 't2', 'l': '观测完成仲裁', 's': '优先于旧请求 → 下一任务', 'f': 'frontier_explorer.py:2363'},
   {'id': 't3', 'l': '在飞请求检查', 's': '按参数超时，等待 SCAN 状态', 'f': 'frontier_explorer.py:2368'},
   {'id': 't4', 'l': '目标维护', 's': '超时 / 冷却 / 封锁边 TTL', 'f': 'frontier_explorer.py:2350'},
   {'id': 't5', 'l': 'choose_frontier', 's': '分层选择', 'f': 'frontier_explorer.py:2509', 'graph': 'choose'},
   {'id': 't6', 'l': '发布', 's': '等待 SCAN 状态；Controller ACK 给 FSM', 'f': 'frontier_explorer.py:3507'},
  ]},
 'choose': {
  'title': 'choose_frontier 分层选择', 'chain': True,
  'blocks': [
   {'id': 'c1', 'l': 'ReplanGate', 's': 'revision 上下文限 2 次', 'f': 'explorer_core/replan_control.py'},
   {'id': 'c2', 'l': 'frontier 检测/聚类', 's': 'FREE 格 4 邻域 + 8 连通', 'f': 'explorer_core/grid.py:369'},
   {'id': 'c3', 'l': '区域承诺 G1', 's': 'IoU 匹配 → scope → 残值门', 'f': 'explorer_core/frontier_regions.py:489', 'graph': 'commit1'},
   {'id': 'c4', 'l': '稀疏估价', 's': '多源 Dijkstra 批量 + miss 原因', 'f': 'explorer_core/sparse_routing.py:481'},
   {'id': 'c5', 'l': '稠密 A* 终验', 's': '逐段 Bresenham + 足印', 'f': 'explorer_core/path_planning.py:59'},
  ]},
 'commit1': {
  'title': '区域承诺 G1(一拍)', 'chain': True,
  'blocks': [
   {'id': 'm1', 'l': '簇 → 拓扑挂接 → 分区', 's': 'component/branch 键', 'f': 'frontier_explorer.py:2565'},
   {'id': 'm2', 'l': 'IoU+质心全局匹配', 's': '承诺者优先 · lineage 继承', 'f': 'frontier_regions.py:489'},
   {'id': 'm3', 'l': 'scope 确定', 's': '("lineage",id) / safe-region', 'f': 'frontier_regions.py:330'},
   {'id': 'm4', 'l': '去抖', 's': 'frontier_missing / unreachable', 'f': 'frontier_regions.py:582'},
   {'id': 'm5', 'l': '残值门', 's': 'exhausted / stagnant 半 MDP', 'f': 'region_commitment.py:87'},
   {'id': 'm6', 'l': '滚动迟滞 + 方向门', 's': '1.35 × · no-return 容差', 'f': 'frontier_regions.py:24'},
  ]},
 'grid2d': {
  'title': 'ExplorationGrid 主线融合', 'chain': True, 'desc': '此图描述主线参考积分与发布；perf 分支的向量化、掩膜和字节发布不是主线已启用的行为。性能结论另见 B1。',
  'blocks': [
   {'id': 'w1', 'l': '720 方位 bin 分类', 's': 'z 带 + NaN 过滤', 'f': 'explorer_core/grid.py:132'},
   {'id': 'w2', 'l': 'Bresenham 融合', 's': '射线逐格积分，perf 另有向量化实现', 'f': 'explorer_core/grid.py:281'},
   {'id': 'w3', 'l': '证据更新', 's': '可逆 + first-occluder', 'f': 'explorer_core/grid.py:281'},
   {'id': 'w4', 'l': '膨胀结果', 's': '主线集合；perf 掩膜缓存另见 B1', 'f': 'explorer_core/grid.py:351'},
   {'id': 'w5', 'l': 'Explorer 地图消息发布', 's': '主线全量序列化热点，见 F4', 'f': 'frontier_explorer.py:3898'},
  ]},
 'globalrep': {
  'title': 'global_representation 处理链', 'chain': True,
  'blocks': [
   {'id': 'r1', 'l': 'patch_callback', 's': '0.5s · revision 去重', 'f': 'global_representation_node.py:247'},
   {'id': 'r2', 'l': 'clearance 场', 's': 'heapq Dijkstra(F7 热点)', 'f': 'explorer_core/topology.py:88'},
   {'id': 'r3', 'l': '骨架化', 's': 'Zhang-Suen → 端点/交叉点', 'f': 'explorer_core/topology.py:45'},
   {'id': 'r4', 'l': '稳定 ID + 封锁确认', 's': 'uint64 量化 · 3 patch', 'f': 'explorer_core/topology.py:114'},
   {'id': 'r5', 'l': 'TopoGraphDelta', 's': '增量 + 每 10 rev 快照', 'f': 'global_representation_node.py:358'},
  ]},
 'controller': {
  'title': 'closed_loop_controller 执行决策', 'chain': True, 'desc': '图表示检查顺序，不表示所有步骤每拍都执行。门控失败会直接发零命令；航向冻结可只发转向命令；通过门控的正常跟踪才进入速度校准。阈值和速度以参数及当前轨迹合同为准。',
  'blocks': [
   {'id': 'd1', 'l': '门控', 's': 'locomotion/碰撞/soft-hold', 'f': 'closed_loop_controller.cpp:617'},
   {'id': 'd2', 'l': 'spatial gait 进度', 's': '0.05s 步找最近样条点', 'f': 'closed_loop_controller.cpp:664'},
   {'id': 'd3', 'l': '航向判定', 's': '|err|>0.8rad → freeze', 'f': 'closed_loop_controller.cpp:747'},
   {'id': 'd4', 'l': 'P 律跟踪', 's': 'vel_des + 0.8·p_error', 'f': 'closed_loop_controller.cpp:786'},
   {'id': 'd5', 'l': '死区校准', 's': '夹 [0.30, max]', 'f': 'closed_loop_controller.cpp:790'},
   {'id': 'd6', 'l': '发布 cmd_vel_raw', 's': '→ supervisor 门控', 'f': 'closed_loop_controller.cpp:850'},
  ]},
 'supervisor': {
  'title': 'go2_locomotion_supervisor', 'chain': True,
  'blocks': [
   {'id': 'v1', 'l': 'getup 状态机', 's': '力矩起身 → 稳定判据', 'f': 'go2_locomotion_supervisor.py:634'},
   {'id': 'v2', 'l': 'STAND_VERIFIED', 's': '→ STATE_RUN', 'f': 'go2_locomotion_supervisor.py:667'},
   {'id': 'v3', 'l': '命令门控', 's': '健康过滤 + 斜率限幅', 'f': 'go2_locomotion_supervisor.py:700'},
   {'id': 'v4', 'l': 'ONNX 50Hz', 's': '45 obs + 450 历史', 'f': 'go2_locomotion_supervisor.py:455'},
   {'id': 'v5', 'l': 'PD 200Hz', 's': 'kp40/kd1.0 → effort', 'f': 'go2_locomotion_supervisor.py:707'},
  ]},
}

# ---------------------------------------------------------------- findings
FINDINGS = [
    {'id': 'A1', 'sev': '高', 'status': 'open', 'sec': 'A', 'title': 'A* 的 inf 常量实际为 0，不是 1024',
     'detail': '当前源码为 constexpr double inf = 1 >> 20；整数右移结果是 0，不是 1024，也不是无穷。影响须结合节点初始化和搜索路径测试，不能由本常量直接断言“大地图必踩”。', 'ref': 'src/planner/path_searching/include/path_searching/dyn_a_star.h:10'},
    {'id': 'A2', 'sev': '高', 'status': 'verify', 'sec': 'A', 'title': 'A* 节点池范围需结合搜索步长核算',
     'detail': '池维度为 100³，但世界范围取决于该次 A* 的 step_size 与池中心。若步长是 0.05m，100 格是总宽约 5m、半宽约 2.5m，并非 ±5m。应检查运行时步长和端点映射后再判断超限。', 'ref': 'src/planner/plan_manage/src/planner_manager.cpp:78'},
    {'id': 'A3', 'sev': '高', 'status': 'verify', 'sec': 'A', 'title': '融合依赖有效 locomotion_state；Explorer 有门控日志',
     'detail': '启用运动健康门时，缺少有效状态会暂停融合。Explorer 已有 MAP_FUSION_GATED 日志，因此不能统称“静默丢弃”。部署应核对发布者、参数和心跳；暂停是安全策略，不宜直接取消门控。', 'ref': 'src/planner/plan_env/src/grid_map.cpp locomotionReady；frontier_explorer.py log_map_fusion_gate'},
    {'id': 'A4', 'sev': '中', 'status': 'open', 'sec': 'A', 'title': 'CMake 无构建类型/优化兜底',
     'detail': 'plan_env/bspline_opt/plan_manage 未设 CMAKE_BUILD_TYPE/-O3;换机直接 colcon build 退化为无优化;ARM 无 -march=native。', 'ref': '各包 CMakeLists.txt'},
    {'id': 'A5', 'sev': '中', 'status': 'open', 'sec': 'A', 'title': 'evaluateDeBoor 每调用堆分配',
     'detail': '每次构造 vector<Eigen::VectorXd>;被 100Hz 控制与验证链高频调用。改栈数组/复用缓冲。', 'ref': 'bspline_opt/src/uniform_bspline.cpp:75-80'},
    {'id': 'A6', 'sev': '中', 'status': 'open', 'sec': 'A', 'title': 'refine 只验证前 2/3 轨迹',
     'detail': '优化器内部 refine 检查范围与 rebound 不同。但 FSM 发布前另有完整候选验证，不能把内部检查范围直接等同于“未检查的轨迹已发布”。需区分优化阶段和最终安全边界。', 'ref': 'bspline_opt/src/bspline_optimizer.cpp (~1147)'},
    {'id': 'A7', 'sev': '中', 'status': 'open', 'sec': 'A', 'title': 'goal_timeout 不释放承诺 scope/方向门',
     'detail': '清 goal 但不清 active_commitment_scope/方向轴,后续候选被旧 no-return 门压制。', 'ref': 'frontier_explorer.py exploration_timer goal_timeout 分支'},
    {'id': 'A8', 'sev': '中', 'status': 'open', 'sec': 'A', 'title': '静态地图冷却永不释放',
     'detail': '冷却释放要求 map_content_revision 前进;死角停滞时永不失效。加时间兜底(与 revision 条件取或)。', 'ref': 'frontier_explorer.py 冷却释放段'},
    {'id': 'A9', 'sev': '低', 'status': 'open', 'sec': 'A', 'title': '仓库卫生',
     'detail': '死仓 quadruped_exploration_planner/(嵌套 .git)、git 跟踪 ~214MB 大文件(map.pcd 45.8MB 等)、部分文档滞后。', 'ref': '仓库根'},
    {'id': 'B1', 'sev': '高', 'status': 'verify', 'sec': 'B', 'title': 'perf 分支优化：有实现，整链路收益尚未确认',
     'detail': '0c20728 包含字节序列化、掩膜、向量化积分和 revision 门控，未合入本页主线。序列化加速不等于整个 ROS 回调加速。此前审查发现基准两路径共用同一 grid、射线位置越界，以及 revision 门可能跳过生命周期计时；全 NaN 扫描还出现参考/向量化不等价。须修复基准、补边界测试并 live 计时，不能宣称 1:1 同步或全链路 1.4ms。', 'ref': 'perf/fusion-stage1-4 @ 0c20728;tools/fusion_bench.py'},
    {'id': 'F4', 'sev': '高', 'status': 'done-perf', 'sec': 'B', 'title': 'planning map 全量 tolist 序列化(每 0.5s ≈ 2M 元素)',
     'detail': '主线仍为 .reshape().astype(int).tolist();perf 分支已改 tobytes+掩膜缓存。合回主线后本条关闭。', 'ref': 'frontier_explorer.py:3898(主线现状)'},
    {'id': 'F16', 'sev': '高', 'status': 'designed', 'sec': 'D', 'title': 'Explorer 栅格越界=墙(长程硬上限)',
     'detail': '固定声明尺寸，越界被规划层视作 OCCUPIED，限制可规划范围；这不构成物理围墙，也不能保证实际机器人不会被控制误差带出边界。chunk 化是后续方案。', 'ref': 'explorer_core/grid.py:261-263'},
    {'id': 'F19', 'sev': '中', 'status': 'designed', 'sec': 'D', 'title': 'blacklist/completed_goals 无界线性扫描',
     'detail': '逐候选线性扫描,长程下 O(候选×历史);D1 有界化一并处理。', 'ref': 'frontier_explorer.py:2272 is_blacklisted'},
    {'id': 'F7', 'sev': '中', 'status': 'designed', 'sec': 'G', 'title': '骨架/clearance 每 0.5s 纯 Python 全量重算',
     'detail': 'Zhang-Suen 逐像素收敛 + heapq Dijkstra,单线程 spin 内;G1 增量双轨的目标。', 'ref': 'explorer_core/topology.py:45-111'},
    {'id': 'C1', 'sev': '中', 'status': 'designed', 'sec': 'C', 'title': '规划拍剩余热点', 'detail': 'observation_target_cells 每候选全量重算;safe_viewpoint_cells 方位环预计算;稠密 A* 回退收走廊子图;静止融合空转门。先计时再动,目标:活跃期整拍 ≤100ms。', 'ref': 'docs/OPTIMIZATION_PROPOSAL_EXPLORER.md'},
    {'id': 'D1', 'sev': '高', 'status': 'designed', 'sec': 'D', 'title': '长程表示:固定栅格 → chunk 增长',
     'detail': '32×32 格(6.4m)chunk 世界坐标编址、惰性分配;增量 frontier 集;A* 回退收走廊子图;blacklist 有界化。内存/算力随探索面积增长;chunk 兼作开阔地带承诺单元。', 'ref': 'explorer_core/grid.py:261(现状:越界=墙)'},
    {'id': 'E1', 'sev': '中', 'status': 'designed', 'sec': 'E', 'title': '承诺系统升级:分量持久 tracker',
     'detail': '把 PersistentRegionTracker 的全局配对模式套到稀疏图分量(输入=stable_node_id)→ 承诺键升级 branch ID → IoU 层对挂接成功区域退役。立项依据:开阔场景 region churn 计数。', 'ref': 'frontier_regions.py:330-347(scope docstring 自证约束)'},
    {'id': 'F1', 'sev': '高', 'status': 'designed', 'sec': 'F', 'title': '地形代价通道(含负障碍)',
     'detail': '轻量三原则:融合同遍历累积 per-cell z 摘要、代价惰性求值、本体修正搭便车(supervisor 速度比/能耗→visited 格)。C++ 侧:z 锚定地形(替换端点线性插值)、机体带检查、terrain 代价项。验证:roboterrain A/B + 合成坡道/坑洞;指标不动即停。', 'ref': 'plan_env/include/plan_env/grid_map.h:484-521(单 z 圆盘检查现状)'},
    {'id': 'F12', 'sev': '中', 'status': 'open', 'sec': 'A', 'title': '软走廊权重复用 lambda4_ 参数',
     'detail': 'guide tube 的 weight=max(1.0,lambda4_) 与 refine 阶段 fitness 权重共用同一变量,调参互相牵连;建议拆为独立参数。', 'ref': 'bspline_opt/src/bspline_optimizer.cpp:1202(当前工作树)'},
    {'id': 'G1', 'sev': '中', 'status': 'designed', 'sec': 'G', 'title': '增量骨架(双轨)',
     'detail': '拟研究增量距离场/GVD与波前骨架，减少全量重算。尚未实现；代码规模、CPU 收益、与融合事件同拍以及骨架质量均需实验验证，不能承诺“约 200 行”或“免费 ESDF”。借鉴外部实现前还须单独核对许可证与论文算法假设。', 'ref': 'explorer_core/topology.py:45-111(现状每 0.5s 全量重算)'},
    {'id': 'Q1', 'sev': '中', 'status': 'verify', 'sec': '修复', 'title': '疑似:启发式强停过度(徘徊主因之一)',
     'detail': 'CLEARANCE_LIMITED_NO_PROGRESS(2s/<0.05m)在滚动重规划预算未用尽时也强制停车。判据:每次 stopped_repair 触发时记录"此前 1s 滚动是否已失败"的占比;占比高则停止判据应收敛为 制动不可行/位置违例/预算耗尽 三条。', 'ref': 'scan_replan_fsm.cpp CLEARANCE_LIMITED_NO_PROGRESS 分支'},
    {'id': 'Q2', 'sev': '中', 'status': 'verify', 'sec': '修复', 'title': '疑似:切向种子族太窄(横向修复禁用)',
     'detail': '锚点族=切向前向+有限后退 0.60m,enable_holonomic_lateral_repair=false。堵点挂切向正前方时 6 锚点大概率全灭 → 升级空转。判据:VIEWPOINT_LOCAL_REJECTED 频次 + 耗尽后 Explorer 换路成功率。', 'ref': 'scan_replan_fsm.cpp:2829(enable_holonomic_lateral_repair=false)'},
    {'id': 'Q3', 'sev': '中', 'status': 'verify', 'sec': '回流', 'title': '疑似:状态回流通道忽略率 >80%',
     'detail': 'planning/status 混装 局部遥测/世界知识/任务事件 三种内容,Explorer 大部分消息只记日志不改变决策。判据:planning_status_callback 分支计数(改变决策 / 忽略)。', 'ref': 'frontier_explorer.py:1958 planning_status_callback'},
    {'id': 'Q4', 'sev': '中', 'status': 'designed', 'sec': '回流', 'title': 'blocked_segment 是 TTL 注记而非证据层条目',
     'detail': '"这条道实测不通"是世界知识,应走证据固化路径(failure_evidence → 拓扑 ExplicitBlockageTracker 同类机制),让 Local/Global 的重规划自动消费;30s TTL 计划注记是错位存放。', 'ref': 'frontier_explorer.py blocked_segment 回调(30s TTL)'},
    {'id': 'Q5', 'sev': '中', 'status': 'verify', 'sec': '执行', 'title': '疑似:死区标定的工况局限',
     'detail': '0.30 命令 ↔ 0.165 m/s 为平地直线最优值;窄通道转弯实际有效速度更低、死区余量更薄。判据:转弯工况重跑 tools/min_speed_probe。', 'ref': 'tools/min_speed_probe;closed_loop_controller.cpp:790'},
    {'id': 'Q6', 'sev': '低', 'status': 'verify', 'sec': '执行', 'title': '疑似:planned/odometry 进度分叉',
     'detail': '执行层偏离(时间参数化虚构、P 修正、gait 重映射)的直接观测;metrics 已同时记录两个进度,分叉曲线即诊断量。', 'ref': 'frontier_explorer.py metrics(planned_progress / odometry_progress)'},
    {'id': 'H1', 'sev': '低', 'status': 'designed', 'sec': 'H', 'title': '实验与论文阵型', 'detail': 'LRAE(地形轴,可跑)+ DARE/ARiADNE(Humble 原生)+ TARE(经典锚)+ 消融;RAEM 代码放出即加;差异指标:per-tick CPU / 内存曲线 / time-to-information。', 'ref': '问题.txt H 节'},
]

STRENGTHS = [
    'GridMap 快照让一次规划/安全查询读取一致地图；不等于证明全系统并发安全，Controller 不直接消费该地图',
    '版本化握手(request_id/trajectory_id/ACK)——状态回流可追溯,迟到消息可丢弃',
    'Explorer 保留完整观测目标集和完成判据；Local 负责任务内运动目标，但当前局部进度代理仍需与权威账本分开',
    '事务化 clearance 交接:pending_handoff 冻结 + 安全闩锁白名单',
    '证据制栅格:可逆占据 + first-occluder 弱化,不为单帧噪声付永久代价',
    '测试密度:explorer 状态机 120+ 用例(行为锁)+ FSM utils gtest + perf 等价性属性测试 33 项',
    '测试和显式协议便于定位回归；用例数量和 TODO 数量本身不证明闭环正确',
]

FILE_NOTES = {
    'src/planner/plan_manage/scripts/frontier_explorer.py': '探索任务与真实观测进度的所有者。融合、任务生命周期、区域承诺和候选发布在不同回调中衔接。0.25s 决策不等于 0.25s 融合；perf revision 门控也不是本主线的既成行为。',
    'src/planner/plan_manage/src/scan_replan_fsm.cpp': 'C++ 巨石(5100+ 行):六态+事务+修复。replan_fsm_utils.h 已抽出纯函数(有 gtest),新逻辑勿再堆入 FSM。',
    'src/planner/plan_env/src/grid_map.cpp': '融合核心,并发设计好(TLS 快照);locomotion 静默依赖是部署陷阱(A3)。',
    'src/planner/plan_manage/scripts/explorer_core/grid.py': '证据制栅格;perf 分支已向量化;越界=墙限制长程(D1)。',
    'src/planner/plan_manage/scripts/explorer_core/topology.py': '骨架+clearance 纯 Python 热点(G1 目标)。',
    'src/planner/plan_manage/scripts/explorer_core/region_commitment.py': '半 MDP 残值门,side-effect-free 设计,可测性极佳。',
    'src/planner/plan_manage/scripts/explorer_core/frontier_regions.py': '区域身份/匹配/分区;scope docstring 是 E1 的路线图。',
    'src/planner/plan_manage/scripts/explorer_core/sparse_routing.py': '稀疏路由镜像;component_id 无跨版本身份(E1)。',
    'src/planner/plan_manage/scripts/explorer_core/observation_task_bridge.py': '任务消息桥;剩余边界过滤已实现。',
    'src/planner/plan_manage/include/plan_manage/local_task_planner.h': 'SCAN 内的 header-only 搜索库。运动可行性由 edge_safe 回调提供；可见性与目标许可由 adapter 提供。不是 ROS 服务，也不是实际观测完成的裁判。',
    'src/planner/plan_manage/src/observation_task_adapter.cpp': '任务消息接入与局部目标选择。visible/goal_allowed 目前还用 visited stations 和全程 route 方向代理任务推进：预测可见不等于实际观测，弯路不能由单一方向完整描述。',
    'src/planner/plan_manage/src/closed_loop_controller.cpp': '执行控制;spatial gait/死区校准在此。',
    'src/planner/plan_manage/scripts/go2_locomotion_supervisor.py': '运动属主+ONNX;capabilities 门控契约。',
    'src/planner/plan_manage/scripts/go2_onnx_policy_node.py': '遗留独立执行器(supervisor 取代)。',
    'src/planner/plan_manage/scripts/go2_policy_node.py': '遗留力矩接口执行器(supervisor 取代)。',
    'src/planner/plan_manage/scripts/global_astar_planner.py': '仅 benchmark launch 使用的真值 A* 适配器。',
    'quadruped_exploration_planner': '死代码脚手架(嵌套 .git,仅 70 行 OR-Tools ATSP)——待处置(A9)。',
    'src/planner/plan_manage/scripts/explorer_core/observation_lifecycle.py': '观测生命周期:completion_reason 区分 visible_information_resolved / frontier_closed。',
    'src/planner/plan_manage/scripts/explorer_core/execution_protocol.py': 'SCAN↔Explorer 状态协议解析(单号血缘)。',
    'tools/min_speed_probe': '策略最低有效速度标定套件(0.30 命令 ↔ 0.165 m/s)。',
    'tools/architecture_audit/generate.py': '本页生成器:重跑即刷新。',
}

LAYER_COLORS = {'sim': '#5a5a5a', 'decision': '#0f62fe', 'plan': '#007d79',
                'exec': '#ba4e00', 'motion': '#198038'}
LAYER_NAMES = {'sim': '仿真/感知', 'decision': '决策(Python)', 'plan': '局部规划(C++)',
               'exec': '执行', 'motion': '运动/RL'}
STATUS = {'open': ['🔴', '原审计待处理（不代表本次逐项复现）'], 'done-perf': ['🟦', '仅 perf 分支有实现，主线未合入'],
          'designed': ['🟡', '已设计待实施'], 'verify': ['⚪', '待实测验证']}

meta = {
    'branch': git('branch', '--show-current') or '?',
    'head': git('rev-parse', '--short', 'HEAD') or '?',
    'modified': [l[3:] for l in git('status', '--short').splitlines() if l],
    'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M'),
    'total_files': tree['files'], 'total_loc': tree['loc'],
    'align': '审计状态与 问题.txt 的 A–H 分区及 🔴✅🟡⚪ 标记对齐;行号以当前工作树为准,你正在并发修改的部分标注为函数级定位。',
}

DATA = {'meta': meta, 'tree': tree, 'nodes': NODES, 'edges': EDGES,
        'graphs': GRAPHS,
        'components': COMPONENTS, 'findings': FINDINGS, 'strengths': STRENGTHS,
        'fileNotes': FILE_NOTES, 'layers': LAYER_COLORS, 'layerNames': LAYER_NAMES,
        'status': STATUS}

# Short labels remain readable, but file navigation uses actual repo paths.
for component in COMPONENTS:
    component['files'] = [canonical_reference(p) for p in component['files']]
for node in NODES:
    for step in node.get('flow', []):
        step['file'] = canonical_reference(step['file'])
for graph in GRAPHS.values():
    for block in graph['blocks']:
        if 'f' in block:
            block['f'] = canonical_reference(block['f'])
DATA['filePaths'] = FILE_PATHS

TEMPLATE = r'''<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SCAN-Planner 架构与审计地图</title>
<style>
:root{--bg:#f6f6f4;--card:#fff;--line:#e3e3e0;--ink:#1c1c1c;--muted:#6b6b6b;--accent:#0f62fe;
--hi:#d32f2f;--mid:#ef6c00;--lo:#2e7d32;--mono:ui-monospace,Menlo,Consolas,monospace}
*{box-sizing:border-box} html,body{margin:0;height:100%}
body{background:var(--bg);color:var(--ink);font:14px/1.55 system-ui,-apple-system,"Segoe UI","Noto Sans SC",sans-serif;display:flex;flex-direction:column;height:100vh}
header{display:flex;flex-wrap:wrap;gap:10px;align-items:center;padding:10px 16px;background:var(--card);border-bottom:1px solid var(--line)}
header h1{font-size:15px;margin:0;font-weight:600}
.chip{font-size:11px;color:var(--muted);border:1px solid var(--line);border-radius:4px;padding:2px 7px;background:var(--bg)}
.chip b{color:var(--ink);font-weight:600}
.tabs{display:flex;gap:4px;margin-left:auto}
.tabs button{border:1px solid var(--line);background:var(--bg);padding:5px 14px;border-radius:6px;cursor:pointer;font-size:13px;color:var(--muted)}
.tabs button[aria-selected="true"]{background:var(--accent);color:#fff;border-color:var(--accent);font-weight:600}
#search{border:1px solid var(--line);border-radius:6px;padding:5px 10px;font-size:13px;width:200px;background:var(--bg)}
main{flex:1;display:flex;min-height:0}
#view{flex:1;overflow:auto;padding:14px 18px}
#drawer{width:0;transition:width .18s ease;overflow:hidden;background:var(--card);border-left:1px solid var(--line);flex-shrink:0}
#drawer.open{width:420px}
#drawer .inner{width:420px;padding:16px;overflow-y:auto;height:100%}
#drawer h2{font-size:15px;margin:2px 0 10px}
#drawer .close{float:right;border:none;background:none;font-size:16px;cursor:pointer;color:var(--muted)}
.kv{font-size:12px;color:var(--muted);margin:2px 0}
.kv b{color:var(--ink)}
.sec{font-size:11px;letter-spacing:.08em;color:var(--muted);text-transform:uppercase;margin:14px 0 4px}
ul.tight{margin:4px 0;padding-left:18px} ul.tight li{margin:3px 0;font-size:13px}
.badge{display:inline-block;font-size:10px;padding:1px 6px;border-radius:3px;margin-right:4px;color:#fff}
.sev-高{background:var(--hi)} .sev-中{background:var(--mid)} .sev-低{background:var(--lo)}
.tag{font-family:var(--mono);font-size:11px;color:var(--accent)}
.node{cursor:pointer}
.node rect{fill:var(--card);stroke:var(--line);stroke-width:1.2;rx:8}
.node.sel rect{stroke:var(--accent);stroke-width:2}
.node:hover rect{stroke:#9a9a9a}
.node text{font-size:12px;fill:var(--ink);font-weight:600;pointer-events:none}
.node .sub{font-size:10px;fill:var(--muted);font-weight:400}
.edge{stroke:#9aa1a9;fill:none;stroke-width:1.3}
.edge.dash{stroke-dasharray:4 4}
.elabel{font-size:10px;fill:#505964;paint-order:stroke;stroke:var(--bg);stroke-width:4px}
.legend{position:absolute;right:16px;top:12px;font-size:11px;color:var(--muted);background:rgba(255,255,255,.85);padding:4px 8px;border:1px solid var(--line);border-radius:6px}
.tree ul{list-style:none;margin:0;padding-left:18px}
.tree li{margin:1px 0}
.row{display:flex;gap:8px;align-items:baseline;padding:2px 6px;border-radius:5px;cursor:pointer}
.row:hover{background:#ececf8}
.row.sel{background:#e0eaff}
.row .n{font-size:13px}
.row .d{font-size:11px;color:var(--muted)}
.row .l{font-size:10px;color:#fff;border-radius:3px;padding:0 5px;margin-left:auto;flex-shrink:0}
.dir>.row>.n{font-weight:600}
.finding{border:1px solid var(--line);border-left:4px solid var(--line);border-radius:6px;background:var(--card);padding:9px 12px;margin:8px 0;cursor:pointer}
.finding:hover{border-color:#9a9a9a}
.finding .t{font-weight:600;font-size:13.5px}
.finding .m{font-size:11.5px;color:var(--muted);margin-top:2px}
.finding.f-高{border-left-color:var(--hi)} .finding.f-中{border-left-color:var(--mid)} .finding.f-低{border-left-color:var(--lo)}
.fchips{display:flex;gap:6px;margin:0 0 10px}
.fchips button{border:1px solid var(--line);background:var(--card);border-radius:14px;padding:3px 12px;font-size:12px;cursor:pointer;color:var(--muted)}
.fchips button[aria-pressed="true"]{border-color:var(--accent);color:var(--accent);font-weight:600}
.compgrid{display:grid;grid-template-columns:repeat(auto-fill,minmax(250px,1fr));gap:10px}
.compcard{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:11px 13px;cursor:pointer}
.compcard:hover{border-color:#9a9a9a}
.compcard .cn{font-weight:600;font-size:13.5px}
.compcard .cd{font-size:11.5px;color:var(--muted);margin-top:3px;display:-webkit-box;-webkit-line-clamp:2;-webkit-box-orient:vertical;overflow:hidden}
.note{font-size:12px;color:var(--muted);background:var(--bg);border:1px solid var(--line);border-radius:6px;padding:8px 11px;margin:10px 0}
.strengths li{margin:4px 0;font-size:13px}
.flowv{display:flex;flex-direction:column;margin:6px 0}
.fstep{position:relative;background:var(--bg);border:1px solid var(--line);border-radius:6px;padding:7px 11px;margin:0 0 14px;cursor:pointer}
.fstep:hover{border-color:var(--accent)}
.fstep:not(:last-child)::after{content:"▼";position:absolute;left:50%;transform:translateX(-50%);bottom:-13px;font-size:9px;color:#b5b5b0}
.fstep .fl{font-size:12.5px;font-weight:600}
.fstep .fs{font-size:11px;color:var(--muted)}
.fstep .ff{font-family:var(--mono);font-size:10px;color:var(--accent)}
.chips{display:flex;flex-wrap:wrap;gap:6px}
.chips button{border:1px solid var(--line);background:var(--bg);border-radius:12px;padding:3px 11px;font-size:12px;cursor:pointer;color:var(--ink)}
.chips button:hover{border-color:var(--accent);color:var(--accent)}
.pipe{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin:6px 0}
.pstep{background:var(--bg);border:1px solid var(--line);border-radius:6px;padding:4px 9px;font-size:11.5px}
.parrow{color:#b5b5b0;font-size:11px}
/* 递归逻辑树(XML 式父子展开) */
.ltree{font-size:13px}
.lrow{display:flex;gap:8px;align-items:baseline;padding:4px 8px;border-radius:6px;cursor:pointer;flex-wrap:wrap}
.lrow:hover{background:#ececf8}
.lrow .chev{width:14px;flex-shrink:0;color:var(--muted);font-size:10px;text-align:center}
.lrow .ll{font-weight:600}
.lrow .ls{color:var(--muted);font-size:12px}
.lrow .ltag{font-family:var(--mono);font-size:10px;color:var(--accent);border:1px solid var(--line);border-radius:4px;padding:0 5px;flex-shrink:0}
.lrow .ltag:hover{background:var(--accent);color:#fff}
.lkids{padding-left:22px;border-left:1px dashed var(--line);margin-left:13px}
.crumb{display:flex;gap:6px;align-items:center;font-size:12.5px;color:var(--muted);margin-bottom:10px}
.crumb a{color:var(--accent);cursor:pointer;text-decoration:none}
@media(max-width:900px){#drawer.open{width:100%;position:absolute;right:0;top:0;bottom:0;z-index:5;box-shadow:-4px 0 16px rgba(0,0,0,.12)}}
</style>
</head>
<body>
<header>
  <h1>SCAN-Planner 架构与审计地图</h1>
  <span class="chip">branch <b id="m-branch"></b></span>
  <span class="chip">HEAD <b id="m-head"></b></span>
  <span class="chip">未提交 <b id="m-mod"></b></span>
  <span class="chip"><b id="m-loc"></b> LOC / <b id="m-files"></b> 文件</span>
  <span class="chip" title="若与 generate.py 最后修改时间不符,请强制刷新(Ctrl+Shift+R)">版本 <b id="m-date"></b></span>
  <input id="search" type="search" placeholder="搜索文件/组件/问题 ( / )" aria-label="搜索">
  <nav class="tabs" role="tablist">
    <button role="tab" aria-selected="true" data-v="runtime">运行时视图</button>
    <button role="tab" aria-selected="false" data-v="tree">目录树</button>
    <button role="tab" aria-selected="false" data-v="comp">组件</button>
    <button role="tab" aria-selected="false" data-v="audit">审计清单</button>
  </nav>
</header>
<main>
  <div id="view" tabindex="-1"></div>
  <aside id="drawer" aria-label="详情"><div class="inner" id="drawer-body"></div></aside>
</main>
<script>
const DATA = __DATA__;
const $ = s => document.querySelector(s);
const view = $('#view'), drawer = $('#drawer'), dbody = $('#drawer-body');
let curView = 'runtime', curSel = null, sevFilter = 'all', query = '';
let logicNode = null, logicOpen = new Set();
let gstack = [{path: "root", label: "系统总览"}];   // 图下钻栈:root → 节点 → 子结构

const esc = s => String(s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
function sevBadge(sev){return `<span class="badge sev-${sev}">${sev}</span>`}
function stBadge(st){const s=DATA.status[st]||['',''];return `<span title="${esc(s[1])}">${s[0]}</span>`}
function openDrawer(html){dbody.innerHTML = `<button class="close" aria-label="关闭" onclick="closeDrawer()">✕</button>`+html;drawer.classList.add('open')}
function closeDrawer(){drawer.classList.remove('open')}
document.addEventListener('keydown',e=>{if(e.key==='Escape')closeDrawer();if(e.key==='/'&&document.activeElement!==$('#search')){e.preventDefault();$('#search').focus()}});

function findingCard(f,showSec){
  if(!f) return '';
  return `<div class="finding f-${f.sev}" onclick="openFinding('${f.id}')">
    <div class="t">${sevBadge(f.sev)}${stBadge(f.status)} ${esc(f.title)}</div>
    <div class="m">${showSec?`[${f.sec}] `:''}${esc(f.ref)}</div></div>`;
}
function openFinding(id){
  const f = DATA.findings.find(x=>x.id===id); if(!f) return;
  openDrawer(`<h2>${sevBadge(f.sev)} ${stBadge(f.status)} ${esc(f.title)}</h2>
    <div class="kv">分区 <b>${f.sec}</b> · 状态 <b>${(DATA.status[f.status]||[])[1]||f.status}</b></div>
    <div class="sec">说明</div><p style="font-size:13px;margin:4px 0">${esc(f.detail)}</p>
    <div class="sec">定位</div><div class="kv"><span class="tag">${esc(f.ref)}</span></div>
    <div class="sec">关联组件</div><div id="fcomps"></div>`);
  const rel = DATA.components.filter(c=>c.findings.includes(f.id));
  $('#fcomps').innerHTML = rel.length ? rel.map(c=>`<div class="row" onclick="showComponent('${c.id}')"><span class="n">${esc(c.name)}</span><span class="d">${c.lang}</span></div>`).join('') : '<div class="kv">—</div>';
}
function flowVertical(flow){
  return `<div class="flowv">` + flow.map(s =>
    `<div class="fstep" onclick="showRef('${esc(s.file)}')" title="点击跳转到目录树中的该文件">
       <div class="fl">${esc(s.label)}</div>
       <div class="fs">${esc(s.sub)}</div>
       <div class="ff">${esc(s.file)}</div></div>`).join('') + `</div>`;
}
function pipeHorizontal(pipe){
  return `<div class="pipe">` + pipe.map((s,i) =>
    (i?`<span class="parrow">→</span>`:'') + `<span class="pstep" title="${esc(s)}">${esc(s)}</span>`).join('') + `</div>`;
}
function showRef(fileHint){
  const path = fileHint.split(':')[0];
  const base = path.split('/').pop().toLowerCase();
  const candidates = DATA.filePaths.filter(p=>p===path || p.endsWith('/'+path));
  if (candidates.length===1) { showFile(candidates[0]); return; }
  query = base;
  $('#search').value = query;
  document.querySelectorAll('.tabs button').forEach(x=>x.setAttribute('aria-selected', x.dataset.v==='tree'?'true':'false'));
  curView = 'tree'; render();
  closeDrawer();
}

function arrowDefs(){return '<defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto" markerUnits="strokeWidth"><path d="M0,0 L8,4 L0,8" fill="none" stroke="#8b929b"/></marker></defs>'}
function edgeEndpoints(p,q,w,h){
  const a=[p[0]+w/2,p[1]+h/2],b=[q[0]+w/2,q[1]+h/2];
  const dx=b[0]-a[0],dy=b[1]-a[1];
  const t=Math.min(dx? (w/2+3)/Math.abs(dx):Infinity,dy? (h/2+3)/Math.abs(dy):Infinity);
  return [a[0]+dx*t,a[1]+dy*t,b[0]-dx*t,b[1]-dy*t];
}

/* ---------- runtime view ---------- */
function nodeById(id){return DATA.nodes.find(n=>n.id===id)}
function showNode(id){
  const n = nodeById(id); if(!n) return;
  const comps = (n.comps||[]).map(cid => DATA.components.find(c=>c.id===cid)).filter(Boolean);
  openDrawer(`<h2>${esc(n.name)}</h2>
    <div class="kv">层 <b>${DATA.layerNames[n.layer]}</b> · ${esc(n.file)}</div>
    <div class="sec">职责</div><p style="font-size:13px;margin:4px 0">${esc(n.role)}</p>
    ${n.flow?`<div class="sec">内部结构(运行链条 · 点击步跳转源文件)</div>${flowVertical(n.flow)}`:''}
    <div class="sec">输入</div><ul class="tight">${n.in.map(t=>`<li><span class="tag">${esc(t)}</span></li>`).join('')||'<li>—</li>'}</ul>
    <div class="sec">输出</div><ul class="tight">${n.out.map(t=>`<li><span class="tag">${esc(t)}</span></li>`).join('')||'<li>—</li>'}</ul>
    ${comps.length?`<div class="sec">内部组件(点击继续下钻)</div><div class="chips">${comps.map(c=>`<button onclick="showComponent('${c.id}')">${esc(c.name)}</button>`).join('')}</div>`:''}
    <div class="sec">要点</div><ul class="tight">${n.notes.map(x=>`<li>${esc(x)}</li>`).join('')||'<li>—</li>'}</ul>`);
}
function selNode(id){
  const n = nodeById(id);
  if (DATA.graphs[id]) { gstack.push({path:id, label:n.name}); renderGraphView(); }
  else showNode(id);
}
function graphTop(){ return gstack[gstack.length-1]; }
function crumbHtml(){
  return gstack.map((g,i) =>
    (i?`<span>›</span>`:'') +
    (i===gstack.length-1
      ? `<b style="color:var(--ink)">${esc(g.label)}</b>`
      : `<a onclick="gstack=gstack.slice(0,${i+1});renderGraphView()">${esc(g.label)}</a>`)).join(' ');
}
function renderGraphView(){
  if (graphTop().path === 'root') { renderRuntime(); return; }
  renderSubGraph(graphTop());
}
function renderSubGraph(top){
  const g = DATA.graphs[top.path];
  g.blocks.forEach((b,i)=>{ if(b.x==null){ b.x = 40+(i%3)*320; b.y = 60+Math.floor(i/3)*250; } });
  const W = 1180;
  const H = Math.max(...g.blocks.map(b=>b.y+72+90), 360);
  const byId = {}; g.blocks.forEach(b=>byId[b.id]=b);
  let edges='',labels='',blocks='';
  const seen = new Set();
  const drawEdge=(a,b,label)=>{
    const key=a+'|'+b+'|'+label; if(seen.has(key) || !byId[a] || !byId[b])return; seen.add(key);
    const p=byId[a], q=byId[b];
    const [x1,y1,x2,y2]=edgeEndpoints([p.x,p.y],[q.x,q.y],260,72);
    const mx=(x1+x2)/2,my=(y1+y2)/2,dx=x2-x1,dy=y2-y1,len=Math.hypot(dx,dy)||1;
    const nx=-dy/len,ny=dx/len,off=len<230?36:(len<430?22:10);
    const cx=mx+nx*off,cy=my+ny*off;
    edges+=`<path class="edge" marker-end="url(#arrow)" d="M${x1},${y1} Q${cx},${cy} ${x2},${y2}"><title>${esc(label||'概念流程顺序')}</title></path>`;
    if(label) labels+=`<text class="elabel" x="${cx}" y="${cy}">${esc(label)}</text>`;
  };
  if (g.chain !== false)
    for(let i=0;i+1<g.blocks.length;i++) drawEdge(g.blocks[i].id, g.blocks[i+1].id, '');
  (g.edges||[]).forEach(e=>drawEdge(e[0],e[1],e[2]));
  g.blocks.forEach(b=>{
    blocks+=`<g class="node" id="blk-${b.id}" onclick="blockClick('${top.path}','${b.id}')" onkeydown="if(event.key==='Enter')blockClick('${top.path}','${b.id}')" tabindex="0" role="button" aria-label="${esc(b.l)}"><title>${esc(b.l+'：'+(b.s||''))}</title>
      <rect x="${b.x}" y="${b.y}" width="260" height="72" rx="8"/>
      <rect x="${b.x}" y="${b.y}" width="4" height="72" rx="2" fill="${b.graph?'var(--accent)':'#b5b5b0'}"/>
      <text x="${b.x+14}" y="${b.y+25}">${esc(b.l.length>22?b.l.slice(0,21)+'…':b.l)}</text>
      <text class="sub" x="${b.x+14}" y="${b.y+44}">${esc((b.s||'').slice(0,38))}</text>
      ${b.graph?`<text class="sub" x="${b.x+14}" y="${b.y+61}" style="fill:var(--accent)">▸ 点击下钻内部结构</text>`
               :(b.f?`<text class="sub" x="${b.x+14}" y="${b.y+61}" style="font-family:var(--mono);font-size:9.5px;fill:var(--accent)">${esc(b.f)}</text>`:'')}
    </g>`;
  });
  view.innerHTML = `<div class="crumb">${crumbHtml()}
     <span style="font-size:11px;margin-left:auto">蓝边块 = 还有内部结构,点击继续下钻;灰边 = 叶子,点击看详情</span></div>
    <svg viewBox="0 0 ${W} ${H}" width="100%" style="max-width:${W}px;display:block;margin:0 auto" role="img" aria-label="${esc(g.title)}">
      ${arrowDefs()}${edges}${labels}${blocks}</svg>
    <div class="note"><b>${esc(g.title)}</b>:${esc(g.desc||'块与块之间为逻辑/数据先后关系。')}</div>`;
}
function blockClick(path,id){
  const g = DATA.graphs[path];
  const b = g.blocks.find(x=>x.id===id); if(!b) return;
  if (b.graph) { gstack.push({path:b.graph, label:b.l}); renderGraphView(); return; }
  openDrawer(`<h2>${esc(b.l)}</h2>
    ${b.s?`<div class="kv">${esc(b.s)}</div>`:''}
    ${b.f?`<div class="kv"><span class="tag" style="cursor:pointer" onclick="showRef('${esc(b.f)}')">${esc(b.f)}</span></div>`:''}
    <div class="sec">说明</div><p style="font-size:13px;margin:4px 0">${esc(g.desc||'此块是当前文档的展开终点，不表示函数内部没有更多逻辑。')} 文件行号为人工定位提示，代码变动后需按函数核对；点击路径查看文件职责与关联批注。</p>`);
}

function renderRuntime(){
  const W=1180,H=580;
  const pts = {}; DATA.nodes.forEach(n=>pts[n.id]=n.pos);
  const L = DATA.layers;
  let edges='',labels='',nodes='';
  for(const [a,b,label,bend,dash] of DATA.edges){
    const [x1,y1,x2,y2]=edgeEndpoints(pts[a],pts[b],150,60);
    const mx=(x1+x2)/2,my=(y1+y2)/2;
    const dx=x2-x1,dy=y2-y1,len=Math.hypot(dx,dy)||1;
    const nx=-dy/len,ny=dx/len,off=(bend||0);
    const cx=mx+nx*off,cy=my+ny*off;
    edges+=`<path class="edge${dash?' dash':''}" marker-end="url(#arrow)" d="M${x1},${y1} Q${cx},${cy} ${x2},${y2}"><title>${esc(nodeById(a).name+' → '+nodeById(b).name+'：'+label)}</title></path>`;
    labels+=`<text class="elabel" x="${cx}" y="${cy}">${esc(label)}</text>`;
  }
  for(const n of DATA.nodes){
    const [x,y]=n.pos,c=L[n.layer];
    nodes+=`<g class="node" id="node-${n.id}" onclick="selNode('${n.id}')" tabindex="0" role="button" aria-label="${esc(n.name)}"
      onkeydown="if(event.key==='Enter')selNode('${n.id}')"><title>${esc(n.name)}</title>
      <rect x="${x}" y="${y}" width="150" height="60" rx="8"/>
      <rect x="${x}" y="${y}" width="4" height="60" rx="2" fill="${c}"/>
      <text x="${x+12}" y="${y+25}">${esc(n.name.length>17?n.name.slice(0,16)+'…':n.name)}</text>
      <text class="sub" x="${x+12}" y="${y+43}">${DATA.layerNames[n.layer]}${(n.comps||[]).length?' · '+n.comps.length+' 组件':''}</text></g>`;
  }
  view.innerHTML = `<div style="position:relative">
    <div class="legend">${Object.entries(L).map(([k,c])=>`<span style="color:${c}">■</span> ${DATA.layerNames[k]}`).join('　')}</div>
    <svg viewBox="0 0 ${W} ${H}" width="100%" style="max-width:${W}px" role="img" aria-label="运行时数据流图">
      ${arrowDefs()}${edges}${labels}${nodes}</svg></div>
    <div class="note"><b>读图范围：</b>主图以 NewMine 的 rl_effort 启动链为主；kinematic 模式由速度桥执行，不经过同一力矩链。实线表示数据/命令，虚线表示运动健康门控；RViz 只消费显示话题，省略其大量显示连线。节点是进程级概览，下钻组件可能是同进程库，不是独立服务。</div>
    <div class="note"><b>时序不是一条同步流水线：</b>主线 Explorer 融合节流 0.5s，决策定时器 0.25s；LocalMapPatch 0.5s。C++ 融合、安全、FSM、Controller 有各自回调。标称 20/100Hz 是定时器配置，不保证每拍产生新地图或准时完成；perf 分支的 0.25s 改动未作为主线现状展示。</div>
    <details class="note"><summary>术语与责任：任务、站位、轨迹、ACK、HOLD</summary><p>任务是“在哪个区域解决哪些未知信息”，Explorer 持有完整目标格与真实观测完成判据。站位是 Local 为当前任务选的具体位置；可见性只是预测，不是完成证据。轨迹是从当前位置到局部目标的 B 样条候选，FSM 完整验证后交给 Controller。ACK 表示 Controller 对该执行版本的接收结果，不表示已到达或已完成观测。HOLD 是停止执行命令，不等于物理速度瞬间变为零；handoff 是版本化轨迹交接，不是停车标定。</p><p>这是生成时的静态工作树快照，不是在线 ROS 图。目录树只表示文件归属，不证明文件在当前 launch 被运行。审计项包含既有记录和待实施方案，本页核对不等于逐项复现；行号可能随代码编辑漂移。LOC 排除跳过目录、二进制和超过 8000 行的文件。</p></details>`;
}

/* ---------- tree view ---------- */
function matchTree(node,q){
  if(node.t==='f') return !q || node.n.toLowerCase().includes(q) || node.p.toLowerCase().includes(q) || (DATA.fileNotes[node.p]||'').toLowerCase().includes(q);
  return Object.values(node.c||{}).some(c=>matchTree(c,q));
}
function renderTreeNode(node,depth){
  const open = depth<2 || !!query;
  if(node.t==='f'){
    if(query && !matchTree(node,query)) return '';
    const note=DATA.fileNotes[node.p];
    const find = DATA.findings.filter(f=>note&&(f.ref||'').includes(node.n));
    return `<li><div class="row" onclick="showFile('${node.p}')" tabindex="0" onkeydown="if(event.key==='Enter')showFile('${node.p}')">
      <span class="n">${esc(node.n)}</span>
      ${note&&find.length?find.map(f=>stBadge(f.status)).join(''):''}
      <span class="l" style="background:${node.lang==='Python'?'#3572A5':node.lang==='C++'?'#f34b7d':node.lang==='bin'?'#999':'#8a8a8a'}">${node.lang}${node.loc!=null?' · '+node.loc:''}</span></div></li>`;
  }
  const hit = matchTree(node,query);
  if(query && !hit) return '';
  const kids = Object.values(node.c||{}).map(c=>renderTreeNode(c,depth+1)).join('');
  return `<li class="dir"><div class="row" onclick="this.parentElement.querySelector('ul').hidden=!this.parentElement.querySelector('ul').hidden" tabindex="0">
      <span class="n">${node===DATA.tree?'SCAN-Planner':esc(node.n)+'/'} </span>
      <span class="d">${node.files||0} 文件 · ${node.loc||0} LOC</span></div>
    <ul ${open?'':'hidden'}>${kids}</ul></li>`;
}
function renderTree(){
  view.innerHTML = `<div class="tree"><ul>${renderTreeNode(DATA.tree,0)}</ul></div>
   <div class="note">含审计批注的文件带状态标记(对齐 问题.txt);点击文件查看批注与相关发现;搜索可跨层展开匹配路径。</div>`;
}
function showFile(path){
  const note=DATA.fileNotes[path];
  const base=path.split('/').pop();
  const finds=DATA.findings.filter(f=>(f.ref||'').includes(base));
  const rel=DATA.components.filter(c=>c.files.some(f=>f===path||path.includes(f)||f.includes(path)));
  openDrawer(`<h2>${esc(base)}</h2>
    <div class="kv"><span class="tag">${esc(path)}</span></div>
    ${note?`<div class="sec">审计批注</div><p style="font-size:13px;margin:4px 0">${esc(note)}</p>`:''}
    ${rel.length?`<div class="sec">所属组件(点击下钻)</div><div class="chips">${rel.map(c=>`<button onclick="showComponent('${c.id}')">${esc(c.name)}</button>`).join('')}</div>`:''}
    ${finds.length?`<div class="sec">相关发现</div>`+finds.map(f=>findingCard(f)).join(''):''}`);
}

/* ---------- component view ---------- */
function renderComponents(){
  const cs = DATA.components.filter(c=>!query||c.name.toLowerCase().includes(query)||(c.role||'').toLowerCase().includes(query));
  view.innerHTML = cs.length? `<div class="compgrid">`+cs.map(c=>
    `<div class="compcard" onclick="showComponent('${c.id}')" tabindex="0" onkeydown="if(event.key==='Enter')showComponent('${c.id}')">
      <div class="cn">${esc(c.name)} <span class="d" style="font-weight:400">· ${c.lang}</span></div>
      <div class="cd">${esc(c.role)}</div>
      ${c.findings.length?`<div style="margin-top:6px">${c.findings.map(id=>{const f=DATA.findings.find(x=>x.id===id);return f?stBadge(f.status):''}).join('')}</div>`:''}
    </div>`).join('')+`</div>` : `<div class="empty">无匹配组件</div>`;
}
function showComponent(id){
  const c = DATA.components.find(x=>x.id===id); if(!c) return;
  openDrawer(`<h2>${esc(c.name)}</h2>
    <div class="kv">语言 <b>${c.lang}</b>${c.pipeline?` · 流水线 <b>${c.pipeline.length} 步</b>`:''}</div>
    <div class="sec">职责</div><p style="font-size:13px;margin:4px 0">${esc(c.role)}</p>
    ${c.pipeline?`<div class="sec">内部流水线(逻辑结构)</div>${pipeHorizontal(c.pipeline)}`:''}
    <div class="sec">输入</div><ul class="tight">${c.in.map(t=>`<li><span class="tag">${esc(t)}</span></li>`).join('')}</ul>
    <div class="sec">输出</div><ul class="tight">${c.out.map(t=>`<li><span class="tag">${esc(t)}</span></li>`).join('')}</ul>
    <div class="sec">关键机制</div><ul class="tight">${c.mech.map(x=>`<li>${esc(x)}</li>`).join('')}</ul>
    <div class="sec">文件</div>${c.files.map(f=>`<div class="kv"><span class="tag" style="cursor:pointer" onclick="showRef('${esc(f)}')">${esc(f)}</span></div>`).join('')}
    <div class="sec">相关发现</div>${c.findings.length?c.findings.map(id=>findingCard(DATA.findings.find(x=>x.id===id))).join(''):'<div class="kv">—</div>'}`);
}

/* ---------- audit view ---------- */
function renderAudit(){
  const fs = DATA.findings.filter(f=>(sevFilter==='all'||f.sev===sevFilter)&&(!query||f.title.toLowerCase().includes(query)||f.detail.toLowerCase().includes(query)||f.ref.toLowerCase().includes(query)));
  view.innerHTML = `<div class="fchips" role="group" aria-label="严重性过滤">
      ${['all','高','中','低'].map(s=>`<button aria-pressed="${sevFilter===s}" onclick="setSev('${s}')">${s==='all'?'全部':s}</button>`).join('')}</div>
    ${fs.length?fs.map(f=>findingCard(f,true)).join(''):'<div class="empty">无匹配发现(清除搜索可见全部)</div>'}
    <div class="sec">已有设计机制（不等同于整链路验收）</div>
    <ul class="strengths">${DATA.strengths.map(s=>`<li>${esc(s)}</li>`).join('')}</ul>
    <div class="note">状态图例：🔴 原审计待处理（本次不代表逐项复现） · 🟦 仅 perf 分支有实现 · 🟡 方案未实施 · ⚪ 尚待验证。A–H 沿用原审计分类；具体判定以条目说明为准，不能把提案当成当前能力。</div>`;
}
function setSev(s){sevFilter=s;renderAudit()}

/* ---------- shell ---------- */
function render(){
  if(curView==='runtime')renderRuntime();
  else if(curView==='tree')renderTree();
  else if(curView==='comp')renderComponents();
  else renderAudit();
}
document.querySelectorAll('.tabs button').forEach(b=>b.addEventListener('click',()=>{
  document.querySelectorAll('.tabs button').forEach(x=>x.setAttribute('aria-selected','false'));
  b.setAttribute('aria-selected','true');curView=b.dataset.v;if(b.dataset.v==='runtime')gstack=[{path:'root',label:'系统总览'}];render();
}));
$('#search').addEventListener('input',e=>{query=e.target.value.trim().toLowerCase();
  if(curView==='runtime'&&query){curView='tree';document.querySelectorAll('.tabs button').forEach(x=>x.setAttribute('aria-selected',x.dataset.v==='tree'?'true':'false'));}
  render();});
$('#m-branch').textContent=DATA.meta.branch;$('#m-head').textContent=DATA.meta.head;
$('#m-mod').textContent=DATA.meta.modified.length;$('#m-loc').textContent=DATA.meta.total_loc.toLocaleString();
$('#m-files').textContent=DATA.meta.total_files;$('#m-date').textContent='@'+DATA.meta.date;
render();
</script>
</body>
</html>'''

html = TEMPLATE.replace('__DATA__', json.dumps(DATA, ensure_ascii=False))
out_path = os.path.join(ROOT, 'docs', 'ARCHITECTURE_MAP_ZH.html')
os.makedirs(os.path.dirname(out_path), exist_ok=True)
with open(out_path, 'w', encoding='utf-8') as f:
    f.write(html)
print(f"written: {out_path} ({len(html)//1024} KB)")
print(f"meta: branch={meta['branch']} head={meta['head']} modified={len(meta['modified'])}")
