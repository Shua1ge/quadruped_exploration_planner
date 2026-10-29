# 2026-10-28 dyn 快照验证

本文保留历史结果。后续提交使用 [发布验收流程](RELEASE_CHECK_ZH.md) 和 `tools/validate_release.py` 生成新的 commit 对应 JSON；不得将下列结果沿用为新工作树的通过记录。2026-10-29 对 `7d48e6f` 的独立编译补验范围也见该流程。

分支 `dyn-20261028-211234`，基于本地 `a461e29cbeaff7c95bc57b85aaddc2acdf655575`，保存归档前现有开发改动。此分支是可复现开发快照，不宣称已解决全部长程探索问题。

本轮新增整理：仓库根目录便携 run_newmine_rl.py（显式权重与资源路径）；NewMine模型及11份共享纹理离线闭包；固定当前TorchScript权重及来源；Campus本地适配工具与SDF模板；环境/构建/启动和已知问题文档。仅同步了一个 legacy 策略时序测试夹具缺失的 policy_command_scale 字段，没有在归档中修改运动策略或恢复逻辑。

## 已完成

- go2_description 启动回归：15项通过。
- 新增资源/便携性回归：7项通过，包含模型/纹理引用、本地资源清单和权重SHA-256。
- plan_manage纯Python/静态合同回归：574项通过，38.78秒。使用 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1，排除需要ROS进程/launch_testing的三个文件，命令见 SETUP_ZH.md。
- Campus准备脚本从原官方 environments ZIP 重新提取、编译转换工具、导出OBJ/STL并比对所有1,929,134三角形；最大顶点误差0.0000482902米，Z_UP一致，纹理引用均为本地。默认自动下载分支未在本轮重新下载458MB远端文件；网络若不支持Drive确认，可用已测试的 --archive 输入。
- 新分支内 NewMine 世界在隔离 IGN partition、ROS domain 184 与存在的空 Fuel cache 下完成控制器激活与时钟推进：原生RTF约0.997994，ROS RTF约0.999577。未起身、Explorer禁用。该测试验证资源/启动，不代表长程导航无缺陷。首次测试将Fuel cache指向不存在目录导致启动异常；创建目录后复测通过。

## 保留的限制

完整直接 pytest 初跑结果为626通过、15失败、2错误：其中5项是测试夹具缺少已有策略的 command scale 字段，补齐后已在574项回归中通过；10项ROS进程场景在沙箱中无法创建DDS socket；2项launch test不能直接通过禁用插件的pytest获得proc_info等fixture。未把这些环境失败伪装为通过；需要在正常主机通信和launch_testing环境用colcon test复验。

本轮没有重建全部C++，也没有改变当前现场进程。现有已构建dyn工作区用于启动检查，新机器应按说明重新colcon构建，不能复制本机install/目录。部分已有文件保持CRLF或历史空白格式，未作无关全仓格式化。

已知恢复预算/方向抑制闭环、区域配色耗尽/固定高度/填色截断和地面拟合限制，完整记录在 KNOWN_ISSUES_20261028_ZH.md。本次提交保留这些问题供后续可审计修复。
