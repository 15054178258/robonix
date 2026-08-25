# Unitree Go2 接入 Robonix：组会讲稿

建议时长：约 7 分钟。方括号内容是演示提示，不需要念。

## 1. 开场：先讲结论

大家好，我这周主要做 Unitree Go2 EDU 接入 Robonix。

最新实机结果是：Robonix 的 10 个组件已经全部启动，真实相机、二维雷达、
三维点云、地图、浏览器语音入口和任务状态可以在同一个 UI 中显示；Nav2 managed
nodes 已进入 ACTIVE，Mapping 也已经产生 ICP 和 local map。

目前还不能说自主导航闭环完成，因为 canonical `/odom` 缺失，`map -> base_link`
TF 尚未成立，也还没有在实地图保存自动售货机前方 Pose。本轮一直使用无运动配置，
没有给 Go2 发送真实运动命令。

[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：Go2 本体能力、部署、UI 和安全门禁的总入口。

## 2. 最终目标

最终目标是：用户在 Robonix UI 说“走到前面自动售货机那里”，系统完成中文 ASR，
将“自动售货机”映射到已保存地图中的安全 Pose，调用 Robonix Navigation 和 Nav2，
规划并避障，再通过 Go2 chassis adapter 输出速度，到达后自动停止。

语音不会直接变成运动命令。地图、定位、TF、语义 Pose、Nav2 lifecycle、唯一控制器、
命令超时和停止反馈必须依次通过。

## 3. 已完成工作

第一，完成 Go2 本体框架，包括 chassis、相机、MID-360、IMU、URDF/TF、部署、UI 和
运动门禁。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：当前 Draft 主 PR。

第二，完成真实硬件接入。设备是 Go2 EDU，背部是 Unitree 100 TOPS NX 和 MID-360；
笔记本通过独立机器人网段读取相机、点云、IMU 和本体状态。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：传感器 primitive 与部署配置。

第三，完成真实 Dashboard 和中文入口。最新页面能同时显示相机、雷达、地图、任务状态
和浏览器录音按钮；FunASR 中文流式 ASR 与“罗伯尼克斯/罗伯特”唤醒词后端已就绪。
[语音/UI commit `4aae9e7`](https://github.com/syswonder/robot-unitree-go2/commit/4aae9e758f61e5de3b70d2a4ce3ba2ed01786d08)：浏览器语音统一委托 Liaison/Pilot，不直接控制机器人。

[展示 `assets/03-full-stack-ui-nomotion.png`。指出绿色的是实时相机、雷达和地图；红色的是尚未完成的 odom、TF 和导航状态。]

第四，完成文本语义预览。输入“走到前面自动售货机那里”，系统能识别目标；由于没有
验证 Pose、定位和运动许可，系统明确拒绝执行，能力调用数为 0。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：受控语义链和安全门禁。

第五，完成上游兼容。Mapping、Navigation 和 Scene 的基础修复已经合并。
[Mapping PR #9](https://github.com/syswonder/service-map-rbnx/pull/9)：CycloneDDS/无头运行兼容。
[Navigation PR #6](https://github.com/syswonder/service-navigation-rbnx/pull/6)：取消任务/CycloneDDS 兼容。
[Scene PR #153](https://github.com/syswonder/robonix/pull/153)：Scene ROS 合同和模型缓存。

第六，完成生态提交。Go2 Catalog 已合并，Robonix 首页 Go2 列表 PR 已开放。
[Catalog PR #6](https://github.com/syswonder/robonix-package-catalog/pull/6)：发布 Go2 条目。
[Robonix PR #152](https://github.com/syswonder/robonix/pull/152)：增加首页本体入口。

## 4. 本轮关键修复和实机结果

Mapping 原先存在 DDS QoS 不匹配和部署参数没有真正下发的问题。修复后，实机 ICP
匹配率通常约 0.4～0.6，RTAB-Map local map 持续增长。本轮本地补丁是 `ac0c136` 和
`55373e6`，尚未另提上游 PR；不能把它们写成已包含在旧 PR #9 中。
[Mapping PR #9](https://github.com/syswonder/service-map-rbnx/pull/9)：仅作为已合并基线。

Nav2 原先有行为树动作缺失、Docker 遗留文件权限和轨迹日志冲突。现在两个 navigator
都使用部署自有行为树，运行时文件和轨迹目录按进程隔离。实机日志已经出现
`Managed nodes are active`，速度守卫输出固定为 `/robonix/nomotion/cmd_vel`。
本轮本地补丁是 `0abdd14`、`6477e68`、`087573c`，尚未另提上游 PR。
[Navigation PR #6](https://github.com/syswonder/service-navigation-rbnx/pull/6)：仅作为已合并基线。

主集成测试目前 266/266 通过，Navigation 定向测试 34/34 通过；远端旧 head 的
`offline-safety` CI 也已通过。[CI 记录](https://github.com/syswonder/robot-unitree-go2/actions/runs/29588111728/job/87910046603)：`9bdf039` 的验证结果。
最新文稿和实机截图已通过 [材料 commit `49f0aae`](https://github.com/syswonder/robot-unitree-go2/commit/49f0aae02a7ebfdbd2d11c415f81b13c58258f7c) 推送到本体 PR 分支。

## 5. 最新 UI 应该怎样解释

最新截图是 `assets/03-full-stack-ui-nomotion.png`，原始状态保存在
`assets/03-full-stack-status.json`。

页面上可以看到：

1. 相机画面是实时 1920×1080 图像；
2. MID-360 同时产生二维扫描和三维点云；
3. Mapping 已发布实时占据栅格地图；
4. 浏览器语音按钮已启用，后端是中文 FunASR；
5. `/odom`、机器人地图位姿和真实导航任务仍未就绪。

因此这张图适合证明“真实 Go2 已进入 Robonix 全栈和 UI”，不适合证明“机器人已经
自主走到自动售货机”。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：UI 与状态来源。

截图采集完成后，时间守卫因 `locked_offset_deviation_exceeded` 自动关闭全栈；这说明
失败关闭链真实生效，也意味着当前实时 URL 已离线。下一次展示前需要重新采集时间证据。

## 6. 为什么还没有直接运动

不是因为 `SportModeState.error_code=100/2010`。实机观察表明这个字段会随遥控模式
变化，不能单独当作故障码。

真正未通过的是 canonical odom、完整 TF、定位稳定性、实地图重定位和自动售货机安全
Pose。只有这些通过，且场地清空、遥控急停在手、唯一控制器和停止链验证完成，才进行
低速、短时、短距离测试。[安全文档](https://github.com/syswonder/robot-unitree-go2/blob/main/docs/SAFETY.md)：运动前置条件和失败关闭规则。

## 7. 现场展示顺序

1. 展示实体 Go2、NX、MID-360、网线和遥控器。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：硬件适配入口。
2. 打开实时地址 `http://127.0.0.1:8092/`；若现场栈未运行，则展示 `assets/03-full-stack-ui-nomotion.png`。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：实时 Dashboard。
3. 指出相机、雷达、地图和语音入口在线，再主动指出 odom/TF 红色状态，说明没有掩盖风险。[Mapping PR #9](https://github.com/syswonder/service-map-rbnx/pull/9)：地图基线。
4. 展示 `assets/04-text-semantic-preview.ndjson`，说明系统识别自动售货机但安全拒绝，调用数为 0。[语音/UI commit `4aae9e7`](https://github.com/syswonder/robot-unitree-go2/commit/4aae9e758f61e5de3b70d2a4ce3ba2ed01786d08)：受控任务入口。
5. 展示 GitHub PR：本体 #1、Catalog #6、Robonix #152、Mapping #9、Navigation #6、Scene #153，并说明本轮 Mapping/Nav2 补丁尚未另提 PR。

## 8. 下周计划

第一，补齐 `/odom` 和 `map -> odom -> base_link`，完成静止漂移、TF 单一权威和重定位验收。

第二，遥控建图，在地图中保存自动售货机前方安全 Pose。

第三，完成浏览器麦克风到中文 ASR、语义 Pose 和 Nav2 任务的端到端复验。

第四，在全部安全门通过后完成首次低速运动、取消、停止和人工接管测试。

第五，更新 [本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)，并为
[Mapping PR #9](https://github.com/syswonder/service-map-rbnx/pull/9) 和
[Navigation PR #6](https://github.com/syswonder/service-navigation-rbnx/pull/6) 之后的本地补丁分别提新 PR。

## 9. 常见问题

### Q1：现在能打开真实面板吗？

本轮采集时 10/10 组件在线，实时地址是 `http://127.0.0.1:8092/`；素材采集后时间守卫
已经安全停栈，所以当前 URL 离线。现场可先用最新截图和状态 JSON；若要实时展示，需
重新生成当次时间证据并启动无运动栈。

### Q2：现在能录什么视频？

可以录“真实相机 + 雷达 + 地图 + 语音入口”的无运动 UI 视频。语音自主导航和真实
运动视频要等 odom、TF、定位和当次运动门禁通过。

### Q3：为什么首版不实时视觉搜索未知自动售货机？

首版先使用地图中人工验证的安全 Pose，可复现也更安全；实时视觉搜索作为后续加分项。

### Q4：离最终闭环还差什么？

还差可信 odom/TF、实地图重定位、自动售货机 Pose、浏览器语音端到端，以及分阶段的
低速运动和自动停止实测。
