# code_audit — dyn 工作区离线代码审计索引

从 SCAN-Planner 源码直接生成单文件、可离线的交互式审计页面，用于回答：
“这个函数调用了谁 / 谁调用它 / 它读写哪些状态 / 它发什么消息 / 终端这条报错从哪来”。

## 使用

```bash
cd /home/t1an/ros2_ws/scan_planner_dyn_ws/src/SCAN-Planner
python3 tools/code_audit/generate.py     # 生成页面 + 索引
python3 tools/code_audit/test_index.py   # 索引回归断言（40 项）
```

产物：

| 文件 | 说明 |
|------|------|
| `docs/CONTRACT_STACK_INTERACTIVE_ZH.html` | 唯一交付页面，内嵌全部索引与源码片段，无外部请求 |
| `tools/code_audit/audit_data.json` | 去掉内嵌源码的索引（约 6 MB），便于 diff / 断言 / 二次加工 |
| `docs/CONTRACT_STACK_INTERACTIVE_ZH.pre-audit.html` | 首次生成时自动备份的旧页面 |
| `tools/code_audit/evidence/*.png` | 浏览器验收截图 |

生成器直接读取工作树，不依赖编译数据库；`meta` 中记录工作区路径、分支、HEAD、未提交数量与生成时间。
行号与结论只对应生成时刻的源码快照。

## 页面功能

- **文件树**：目录 → 文件 → 类/结构体 → 函数（行号+签名）；二进制只显示 `bin`；长路径悬停显示完整 `title`。
- **函数面板**：源码片段（命中行高亮）、调用（出边）、被调用（入边）、未解析调用、状态读写、消息发送、
  回调接线、日志点。调用可继续展开，面包屑与 URL hash 同步。
- **状态节点**：按 类@文件 分组，字段详情给出 设置/清除/修改/读取 四类站点；无清除点时标注「保留(未见清除)」。
- **消息节点**：`.msg` 字段与常量、`create_publisher` 生产者、`create_subscription` 消费者（含回调名与注册行）、
  `publish()` 调用点（支持跨文件）。
- **日志反查**：粘贴终端日志 → 按 大写标记/键值对/词重叠 打分定位日志点；同时给出原因码（UPPER_CASE 字符串）
  的全部出现位置，可继续追到函数、状态与消息。输入与结果在返回/刷新后保留（sessionStorage）。
- **导航**：hash 深链接（`#f=fid>fid`、`#v=vid`、`#m=Msg`、`#tok=CODE`、`#file=path`、`#st`、`#log`）；
  浏览器后退/前进有效；循环调用在展开路径中命中时只提示、不重复入栈。

## 证据分级（不制造确定链路）

调用边 `conf`：

| 级别 | 含义 |
|------|------|
| `exact` | 限定名 `Class::method` 或同类/`this->` 命中，目标唯一 |
| `name` | 全工作区唯一同名（可能仍是不同类） |
| `ambig` | 存在多个同名符号，页面列出目标但要求人工确认 |
| unresolved | 目标不在工作区索引内；单独列出，不进入链路 |

成员调用（`obj->method()`）只有在接收者成员声明类型可推断时入图；推断不出则不建边。
字段分类是模式匹配（RHS 字面量、`.clear()/.reset()/.store()` 等），不是语义分析。
所有限制同时写进页面「说明」页签。

## 实现结构

- `generate.py`：遍历文件树 → 解析 C++（花括号匹配）/ Python（`ast`）→ 构建调用边、字段站点、ROS 接线、
  日志点、原因码 → 交给模板渲染 → 写 HTML + JSON。
- `template.py`：HTML/CSS/JS 单页应用（无框架、无 CDN），`render(DATA)` 注入 JSON。
- `test_index.py`：对恢复链、消息接线、非恢复链（LocalMapPatch）、词边界、自环、快照元数据等做断言，
  防止提取逻辑回退。
- 排除范围：`tools/`、`docs/`、`simulator/local_sensing/` 仅出现在文件树，不做符号提取；二进制与 >2MB 文件不计 LOC。

## 重新生成后的建议

1. `test_index.py` 若出现 FAIL，先看断言里的 `[detail]`，再回 `generate.py` 对应阶段排查；
2. 新增关键链路时，在 `test_index.py` 增加一条源码事实断言（行号 + 符号），把“页面正确”固化成回归；
3. 页面变大时优先压缩字段站点（读取点上限 120/字段、列表 60/类）与源码内嵌上限（700 KB/文件）。
