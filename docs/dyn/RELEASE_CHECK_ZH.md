# dyn 发布验收

发布单元是完整 Git commit。工作树里存在某个文件、原机器能运行、旧 install 能找到节点，都不能替代对应提交的验收。本流程不要求把 Campus 大资产提交进 Git，也不会自动推送、安装依赖或发送运动命令。

## 1. 固定候选与环境

记录 `git rev-parse HEAD` 和 `git status --short`。将拟发布的代码、脚本和文档纳入候选提交；不要为通过检查混入无关工作树改动。依赖按 [环境说明](SETUP_ZH.md) 安装。当前基线是 Ubuntu 22.04、Humble、Fortress、LiDAR＋本体感知 RL，Vision 关闭，Global shadow。

先从仓库根目录运行：

```bash
python3 tools/validate_release.py --ref HEAD --output /tmp/dyn-release-source
python3 tools/validate_release.py --ref HEAD --build --output /tmp/dyn-release-build
```

输出目录须不存在，避免覆盖旧证据。验证器使用 `git archive` 导出指定提交到 `/tmp` 下新建的 ASCII 路径工作区，不读取原工作区 build/install，也不带入未跟踪文件。`--ref` 默认 HEAD，可指定完整 SHA。报告会记录原工作树中被排除的改动，因此验证昨天的 HEAD 不代表今天未提交内容已经验证。

输出 `report.json` 和分项日志。源码检查包含 NewMine/策略资源闭包与哈希、主文档本地链接、包装器 dry-run。`--build` 再使用系统 ROS、独立 build/install 编译全部包，检查安装前缀、节点入口、launch 参数、Go2 URDF、启动流程单元测试、策略离线推理与 reset。编译与各阶段设置有界超时，失败返回非零；具体失败见对应日志。

`SOURCE_CHECKS_PASSED` 只代表源码静态与资源检查通过；`BUILD_AND_OFFLINE_CHECKS_PASSED` 才包含独立编译与安装后检查。两者均明确记录 `simulation_tested=false`、`campus_download_tested=false`、`long_run_tested=false`，不表示允许用旧运行记录替代这些步骤。验证器不会启动 ROS 节点或 Gazebo；`--show-args` 和 xacro 只生成描述。

## 2. 仿真就绪：NewMine 与 Campus 分开记录

在独立部署机器或隔离仿真实例上，使用该候选的 install、资源和启动命令。先采用 [不自动起身的方式](SETUP_ZH.md#5-不自动起身的检查方式)，保留日志与下列证据：控制器 active、有限步进已结束、原生 stats 显示持续运行、`/clock` 在一个连续观察窗口内递增、关节/IMU/odom/LiDAR 实际有消息。服务 ACK 或单帧时钟均不能替代时间推进。

NewMine 使用随提交保存的资源，必要时使用单独的空 Fuel cache 检查是否有隐式下载。Campus 从官方 ZIP 或 source 重新准备，保存输入哈希、preparation_manifest.json、转换日志和 visual/collision 比对结果；自动下载与 archive 输入分别标注。Campus 尚未准备或未测试，就在报告写未验收，不影响单独发布明确标为 NewMine 已验收的候选。

## 3. 自主运行与性能验收

只有在明确开始仿真自主运行后才调用起身授权。保存 commit、世界资源哈希、启动命令、配置、ROS/IGN 隔离信息、metrics、日志及实际路径 CSV。短启动验证、策略站起、持续探索分别记录，不使用一个“通过”涵盖三者。

Global 维持 shadow；比较 Global off 与 shadow 的实际覆盖、任务/安装统计、Global＋Explorer CPU 和尾延迟，确认表示改动没有引入调度退化。后续 route_cost 的启用必须另列实验与回退条件，不能在 README 默认命令中悄悄开启。

## 4. 现有证据边界

2026-10-29 已对远端 `7d48e6f77f6ea7f3ab3f01ba556c13c04ae3af3d` 独立导出并完成 13 包编译、7 项便携性检查、15 项启动流程单元测试、launch/xacro 解析及 10 次策略离线推理。该验证依赖本机已安装的系统环境，未使用全新 OS 容器，未在本轮启动 Gazebo，也不覆盖后续未提交修改。

历史 2026-10-28 的世界与运行证据保留在 [SNAPSHOT_VALIDATION.md](SNAPSHOT_VALIDATION.md)。新候选应生成新的 JSON 和日志，不覆盖历史记录或把历史成功写成当前提交的实测。
