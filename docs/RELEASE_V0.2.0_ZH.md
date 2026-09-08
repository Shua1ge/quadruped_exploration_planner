# v0.2.0 版本说明

`v0.2.0` 是当前 ROS 2 平地自主探索版本。它保留 SCAN-Planner 的局部 B 样条规划，在其上接入二维 Frontier Explorer、区域顺序规划、GO2 闭环控制和执行期安全检查。

相较于早期 ROS 2 移植版本，本版本主要完成了四项工作：规划期间点云融合与安全回调继续运行；滚动规划优先延续仍然安全的旧路径，并从机器人当前前向位置交接新轨迹；路径请求使用递增 request id，旧请求的迟到状态不能抢回控制权；区域承诺短暂不可达时保持稳定，但持续不可达时自动释放，不再无限停在 `WAIT_TARGET`。

本版本已通过 `scan_planner` 构建、46 项 Explorer/全局 A* Python 测试、11 项滚动规划 C++ 测试和 2 项 ROS 启动测试。尚需在完整 quad 地图上继续做长时间实测，重点观察区域切换平滑度、`SAFETY_MAP_STALE` 频率和极小残留 Frontier。这个版本不包含三维地形规划、强化学习或新的安全层。

简明架构与故障定位方法见 [ARCHITECTURE_SIMPLE_ZH.md](ARCHITECTURE_SIMPLE_ZH.md)，详细历史与参数见 [PROJECT_HANDOFF_ZH.md](PROJECT_HANDOFF_ZH.md)。
