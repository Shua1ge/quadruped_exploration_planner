# dyn 代码审计索引：验收记录

- 工作区：`/home/t1an/ros2_ws/scan_planner_dyn_ws/src/SCAN-Planner`
- 源码快照：分支 `codex/observation-task-local-planner-20261003`，HEAD `a461e29`，工作树未提交改动 99 个（生成时刻）
- 页面：`docs/CONTRACT_STACK_INTERACTIVE_ZH.html`（单文件，8.8 MB，全部数据内嵌）
- 索引中间产物：`tools/code_audit/audit_data.json`
- 浏览器：ZCode 内置浏览器（IAB），页面经 `http://127.0.0.1:8765` 打开；`file://` 用本机 headless Chrome 单独验证
- 生成器：`tools/code_audit/generate.py` + `tools/code_audit/template.py`

本文件区分三类证据：**自动测试**（`test_index.py`）、**浏览器实际操作**（IAB 点击/输入/后退）、**源码人工核对**（回到源码逐行确认）。

## 一、必选验收项与结果

| # | 验收项 | 结果 | 实际执行的操作与观察 |
|---|--------|------|----------------------|
| 1 | 文件树展开到文件、类、函数 | 通过 | 点击 `src` → `planner` → `plan_manage` → `src` → `scan_replan_fsm.cpp`；文件面板列出 50 个函数（行号+签名），类/结构体单列。 |
| 2 | 函数继续展开调用、调用者、状态读写、回调、消息 | 通过 | 打开 `pathCallback`（L649），点击内部调用 `refreshPlanningOdomFromSafety` → hash `#f=f1056>f1061`，面包屑两级；该函数入边 16 条；状态命中 52 字段；`publishLocalMapPatch` 的「消息发送」行点击进入 `LocalMapPatch` 消息节点。 |
| 3 | 状态节点找到设置、读取、清除、保留位置 | 通过 | `replan_fail_count_`：3 设置 / 13 清除 / 5 修改 / 12 读取，每类有文件:行+代码；`task_lateral_repair_used_`：5 设置、0 清除，页面标注「保留(未见清除)」并说明是启发式结论。 |
| 4 | 消息节点找到定义、生产者、消费者、接线依据 | 通过 | `ObservationFeedback`：生产 `scan_replan_fsm.cpp:311`，消费 `frontier_explorer.py:822 → observation_feedback_callback`，发布点 `observation_task_adapter.cpp:65/76/281`（跨文件解析）；`LocalMapPatch`：生产 `grid_map.cpp:683`，发布 `:1778 publishLocalMapPatch`，消费 `global_representation_node.py:150 patch_callback`。 |
| 5 | 粘贴终端报错定位源码并继续追数据流 | 通过 | 粘贴真实格式日志 `[TASK_MOTION_FAILURE_TRACE] task=42 reason=LATERAL_RECOVERY_ALREADY_USED ...`；首条命中 `observation_task_adapter.cpp:45`（score 10.5）；原因码 → `scan_replan_fsm.cpp:3288`（`tryStructuredLocalRepair`）→ 字段 `task_lateral_repair_used_` 设置点 3169/3291 与 adapter 392/912/961 → 反馈发布点 → `ObservationFeedback` → Explorer 消费 `observation_task_bridge.py:500`。 |
| 6 | 导航历史、深链接、循环关系、离线 | 通过 | 浏览器后退/前进在 `#tok` ↔ `#f` 间恢复；刷新保留 hash 并恢复函数；新标签打开复制出的 `#f=f1056%3Ef1061` 恢复两级面包屑；`intbound` 自调用行标注「已在路径」，点击只弹提示不入栈；`file://` 直接打开渲染正常（截图 `08`），页面运行时 0 个外部资源请求。 |
| 7 | 静态无法确定的关系明确标注 | 通过 | 调用边四级徽标：精确 1985 / 名称匹配 1223 / 同名歧义 102 / 未解析 3622；函数面板单列未解析调用；成员调用仅在接收者类型可由声明推断时入图，否则不建边；字段分类标注为模式匹配启发式。 |

## 二、压力与稳健性检查（浏览器内实际执行）

- 大文件：`scan_replan_fsm.cpp`（6370 行）文件视图正常，50 个函数。
- 长函数：`callReboundReplan`（4825–6132，1307 行）→ 84 出边 / 34 去重、100 未解析、32 日志点；片段截断提示「完整函数体见文件视图」。
- 连续渲染 200 个函数 83 ms；2005 字段列表 10 ms；300 个字段详情 35 ms；1307 行函数 3 ms。
- 200 行日志粘贴 + 分析 43 ms，结果 15 行，无异常。
- 空搜索：全局搜索 `zzzz_no_such_symbol` → 「无结果」；字段过滤同理。
- 同名符号：全局搜索 `reset` → 46 条（含多个类同名方法），可区分文件/行。
- 循环：仅剩 9 条自环，逐一核对为真实递归（`intbound`、`recursiveDivision`×4、`recursizeDivisionMaze`×4）。
- 无空白面板、无横向溢出（各视图 `scrollWidth-clientWidth = 0`）。
- 页面异常：对全部视图 + 200 函数 + 300 字段 + 长日志做异常捕获，无页面异常（仅测试脚本自身误调不存在的 `renderTree`，页面树视图由 `renderSide` 渲染）。

## 三、发现并已修复的问题

| 问题 | 表现 | 根因 | 修正位置 |
|------|------|------|----------|
| 二进制文件计入 LOC | 总 LOC 显示 17,585,106 | 对 `map.pcd` 等按行计数 | `generate.py` `walk_tree`：BIN 扩展/超 2MB 不计 LOC |
| 嵌套 `struct` 劫持类作用域 | `local_recovery_state_` 等字段丢失 | 单变量 `cur_class` 无法表达嵌套 | `parse_cpp` 改为类作用域区间栈 + 最内层归属 |
| 跨行签名漏提函数 | `closed_loop_controller.cpp` 方法、构造函数缺失 | 正则要求 `)` 同行 | `FUNC_RE` + 括号平衡拼接；允许无返回类型构造 |
| 跨行/原子字段漏提 | `std::atomic<uint8_t> x_{...}` 缺失；`.store()` 写成读取 | 单行声明假设；未识别 store/exchange | 字段声明跨行拼接；`store/exchange/fetch_*` 分类 |
| 原因码消失 | `LATERAL_RECOVERY_ALREADY_USED` 只剩测试文件 | 提取前 `strip_noise` 删掉了字符串 | `strip_noise(keep_strings=True)` 用于原因码 |
| 跨文件发布点缺失 | ObservationFeedback publish 站点 0 | 文件字母序导致 publisher 声明晚于 publish 扫描 | 两遍扫描（先声明，再 publish） |
| 订阅回调解析成 `bind` | `cloud` 回调显示「bind」 | 正则先命中 `std::bind` | 提取 `&Class::method` 的 method |
| Python 日志重复 | 同一行出现两次 | 单行/多行两个正则重复收录 | 删除重复分支（日志点 359→313） |
| 函数定义行变自环 | `pathCallback` 调用自己 | 扫描从签名行开始 | 函数体从首个 `{` 之后扫描 |
| 成员调用假边 | `.load()` → Python 测试 `load`；`points_.resize()` 自环 | 按唯一名字猜成员调用，且不分语言 | 按语言隔离；成员调用需接收者类型推断（`member_types`），推断不出不建边 |
| 字段子串误匹配 | `max_replan_fail_count_` 混入 `replan_fail_count_` | 未加词边界 | 全部站点匹配加 `(?<![A-Za-z0-9_]) ... (?![A-Za-z0-9_])` |
| 顶部快照 chips 为空 | 只显示“分支/HEAD”标签 | 新模板未赋值 | boot 段填充 `#c-*` |
| 返回日志页丢输入/结果 | 后退后日志框和结果清空 | 视图重渲染重建 DOM | `logQuery/logHtml` + `sessionStorage` 持久化；字段过滤同样保留 |
| 搜索下拉/徽标排版 | 函数名截断、徽标换行、路径被大写 | CSS | `#gsres .item .n` 加宽、`.badge` nowrap、取消 `.sec` 大写 |

## 四、仍存在的限制（会影响使用，页面「说明」中同样列出）

1. C++ 解析基于文本正则 + 花括号匹配：宏生成函数、模板特化、预处理分支中的怪异换行可能漏提；不是编译器级语义分析。
2. 对象成员调用只在成员声明类型可推断接收者时入图；局部变量/返回值上的方法调用不建边（宁缺毋假）。
3. `name`（唯一同名）与 `ambig`（多个同名）边仍需人工确认；未解析调用不构成链路。
4. 字段的 设置/清除/修改/读取 是模式匹配；读取计数完整但列表上限 60 行、单字段读取记录上限 120 处（面板注明截断）。
5. 「保留(未见清除)」= 静态索引未见清除点，不是运行时证明。
6. 回调仅覆盖 `create_subscription`；定时器、服务、action 回调未索引；lambda 回调记为 `<lambda>`。
7. `simulator/local_sensing` 第三方代码、`tools/`、`docs/` 只在文件树出现，不提取符号；二进制不计 LOC。
8. 未解析调用列表最多保留 4000 条（当前 3622 条，未截断）。
9. IAB 不能直接导航 `file:`，因此交互验收经本地 HTTP；离线能力以「单文件 + 0 外部请求 + headless Chrome `file://` 渲染」三项证据确认。
10. 行号对应生成时刻工作树；源码变动后需重新生成。

## 五、浏览器实际走过的链路：回源码核对

| 页面结论 | 源码证据 | 核对结果 |
|----------|----------|----------|
| `pathCallback` 位于 scan_replan_fsm.cpp:649–840 | `void SCANReplanFSM::pathCallback(...)` L649 | 一致 |
| `refreshPlanningOdomFromSafety` L1085–1094 | 同文件 L1085 | 一致 |
| 原因码 `LATERAL_RECOVERY_ALREADY_USED` 产生于 L3288 | L3288 `observationTaskRejected("LATERAL_RECOVERY_ALREADY_USED");`（位于 `tryStructuredLocalRepair` 3274–3601） | 一致 |
| `task_lateral_repair_used_` 设置 3169/3291、adapter 392/912/961；无清除 | `store(true)`/`store(blocked)` 逐行核对；头文件 L174 注释说明由 physical departure memory 围栏 | 一致；「保留」有源码依据 |
| 反馈发布点 adapter 65/76/281 | L65/L76 `task_feedback_pub_->publish(feedback);`；L281 同样 | 一致 |
| `[TASK_MOTION_FAILURE_TRACE]` 日志点 adapter:45 | L45–46 `RCLCPP_WARN(... "[TASK_MOTION_FAILURE_TRACE] task=%llu reason=%s ...")` | 一致 |
| `ObservationFeedback` 消费 `frontier_explorer.py:822` | L822 `self.create_subscription(ObservationFeedback, "planning/observation_feedback", self.observation_feedback_callback, ...)` | 一致（回调实现位于 `observation_task_bridge.py:302`，Mixin 继承） |
| Explorer 消费失败原因 bridge.py:500 | L500 `if reason.startswith("LATERAL_RECOVERY_") or pose_wide or ...` | 一致 |
| `LocalMapPatch` 生产 `grid_map.cpp:683`、发布 `publishLocalMapPatch` L1765–1778、消费 `global_representation_node.py:150` | L683–684 `create_publisher<LocalMapPatch>("grid_map/local_map_patch", ...)`；L1778 `local_patch_pub_->publish(patch)`；L150–151 `create_subscription(LocalMapPatch, "grid_map/local_map_patch", self.patch_callback, ...)` | 一致 |
| `callReboundReplan` 4825–6132 | 同文件 L4825 定义，下一函数 L6134 | 一致 |

## 六、审计发现（生产代码）

本次未在运动、地图、控制逻辑中发现需要记录的新缺陷；**未修改任何生产代码**（改动仅 `docs/CONTRACT_STACK_INTERACTIVE_ZH.html` 与新增 `tools/code_audit/`）。

静态事实记录（非缺陷判断）：`task_lateral_repair_used_` 设计为不自动清除，源码注释明确其由物理离开记忆围栏；页面以「保留(未见清除)」呈现，与设计一致。`execution_progress_budget_` 的修改通过引用别名发生，字段级静态索引只能看到读取——这是静态分析边界，页面已按限制说明，不应解读为“从未修改”。

## 七、证据清单

- `evidence/01-function-panel.png`：函数面板（源码、调用分级、自环拦截）
- `evidence/02-state-field-sites.png`：`replan_fail_count_` 设置/清除/修改/读取
- `evidence/03-message-wiring.png`：`ObservationFeedback` 定义与常量
- `evidence/04-recovery-budget-sites.png`：恢复预算字段 5 处写入、0 清除、「保留」提示
- `evidence/05-explorer-consumption.png`：Explorer 消费函数源码第 500 行
- `evidence/06-same-name-search.png`：`reset` 同名符号搜索结果与循环徽标
- `evidence/07-long-function.png`：1307 行长函数、片段截断提示、调用分级
- `evidence/08-offline-file-scheme.png`：`file://` 离线打开渲染

## 八、复现命令

```bash
cd /home/t1an/ros2_ws/scan_planner_dyn_ws/src/SCAN-Planner
python3 tools/code_audit/generate.py     # 重新生成 HTML 与索引（首次会备份旧页为 *.pre-audit.html）
python3 tools/code_audit/test_index.py   # 40 项索引回归断言
```

浏览器验收（本地静态服务）：

```bash
cd docs && python3 -m http.server 8765 --bind 127.0.0.1
# 打开 http://127.0.0.1:8765/CONTRACT_STACK_INTERACTIVE_ZH.html
```

离线直开验证：

```bash
google-chrome --headless=new --disable-gpu --no-sandbox \
  --screenshot=tools/code_audit/evidence/08-offline-file-scheme.png \
  --window-size=1280,900 --virtual-time-budget=8000 \
  "file://$PWD/docs/CONTRACT_STACK_INTERACTIVE_ZH.html"
```
