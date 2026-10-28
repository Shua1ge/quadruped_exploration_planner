# 世界资源与来源

NewMine 的 SDF、实际引用的 Edgar 1–5 与 staging 模型、纹理、原作者 metadata 已纳入 resources/newmine。六个模型官方 Fuel 元数据均声明 CC0；SOURCES.json 保留完整来源链接和查询时版本，MANIFEST.json 保留本地适配后文件的校验值。部署不需要额外下载 Fuel 资源。

Campus 来源为 https://www.cmu-exploration.com/development-environment ，对应场景使用 HongbiaoZ/autonomous_exploration_development_environment 的 environments 包。官方代码固定提交 bf0cba71365271ebff09831a05afd78578150300；下载及源 DAE 哈希见 ../../resources/asset_manifest.json。场景包与代码仓库的许可应分别确认，不因本项目代码为 Apache-2.0 就将场景改授该许可。因此只提供官方获取入口和本地转换流程，不把约382MiB DAE、273MiB OBJ 和92MiB STL直接重新上传进 Git。

转换入口为 tools/worlds/prepare_campus.py，支持 --download、--archive、--source。输出 visual Z_UP 与碰撞同轴，纹理最长边2048，全三角形匹配容差1毫米；输出目录含资源、可启动SDF、构建日志和 preparation_manifest.json。resources/worlds/campus_fortress.sdf 是不含本机绝对路径的模板。

当前坐标参考下 Campus XY bounds 约 [-219.126,131.905]×[-181.310,162.899]m，建议 Explorer 460×400m；场景本身有显著高差、建筑和扫描重建孔洞，并非任意位置都可通行。推荐出生点 (0,0,0.45)，visual/collision 在原点地面 z≈0；运行时仍由 Local 做实际运动认证。图像 campus_overview.png 为本机验证过的正确Z_UP渲染。

RL 权重保存于 models/go2_rl_lab_policy.pt，6,399,130字节；SHA-256、输入/输出形状和来源见同目录 JSON。它是原 dyn 在用的同一份 TorchScript，不下载最新版替换。原本地工作区没有记录精确训练变体，因此明确保留这一来源缺口；公开 checkpoint 库 https://huggingface.co/wty-yy/go2_rl_gym_data 声明 MIT，完整上游实现许可证及依赖署名保存在 models/go2_rl_lab.LICENSE。
