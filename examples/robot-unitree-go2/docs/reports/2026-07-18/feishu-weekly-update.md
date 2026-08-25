# Unitree Go2 接入 Robonix 周报

## 工作内容

- 完成 Go2 EDU 本体适配框架，覆盖 chassis、相机、MID-360、IMU、URDF/TF、部署、UI 和安全门禁。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：Go2 接入 Robonix 的主 PR，目前为 Draft。
- 完成真实传感器面板和浏览器语音入口，支持相机、雷达、地图、任务状态及中文语音预览。[语音/UI commit `4aae9e7`](https://github.com/syswonder/robot-unitree-go2/commit/4aae9e758f61e5de3b70d2a4ce3ba2ed01786d08)：增加受门禁保护的浏览器语音入口；[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：承载 UI 与实机适配。
- 修复 Mapping 的 CycloneDDS、BestEffort QoS 和部署参数传递；实机已产生 ICP 与 local map。[Mapping PR #9（已合并）](https://github.com/syswonder/service-map-rbnx/pull/9)：Mapping 基础兼容；本轮本地补丁为 `ac0c136`、`55373e6`，尚未另提 PR。
- 修复 Nav2 行为树和原生运行时隔离；实机 managed nodes 已进入 ACTIVE，速度输出仍锁到无运动话题。[Navigation PR #6（已合并）](https://github.com/syswonder/service-navigation-rbnx/pull/6)：Navigation 基础兼容；本轮本地补丁为 `0abdd14`、`6477e68`、`087573c`，尚未另提 PR。
- 完成时间戳失败关闭、命令超时和停止链；状态异常会阻止或关闭输出。[时间安全 commit `9acc362`](https://github.com/syswonder/robot-unitree-go2/commit/9acc36276c9ba9e9803f6e4251378b6cd33e3d3b)：拒绝过期机器人状态；[安全文档](https://github.com/syswonder/robot-unitree-go2/blob/main/docs/SAFETY.md)：定义运动前置条件。
- 完成 Go2 Catalog 和 Robonix 首页入口。[Catalog PR #6（已合并）](https://github.com/syswonder/robonix-package-catalog/pull/6)：发布 Go2 条目；[Robonix PR #152（开放）](https://github.com/syswonder/robonix/pull/152)：首页增加 Unitree Go2。
- 修复 Scene ROS 合同和模型缓存兼容。[Scene PR #153（已合并）](https://github.com/syswonder/robonix/pull/153)：保证 Scene 服务可在本轮 Robonix 栈中启动。

## 当前进度

- 最新无运动实机复验中，Robonix 10/10 组件均启动成功，Dashboard、Mapping、Speech、Nav2 均为 ACTIVE。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：完整部署入口。
- 最新 UI 已显示真实 1920×1080 相机、二维雷达、三维点云、实时地图和浏览器语音按钮；状态文件与截图已归档。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：UI、传感器 primitive 与证据来源。
- FunASR 中文流式识别和“罗伯尼克斯/罗伯特”唤醒词后端已就绪；浏览器语音仍处于预览模式。[语音/UI commit `4aae9e7`](https://github.com/syswonder/robot-unitree-go2/commit/4aae9e758f61e5de3b70d2a4ce3ba2ed01786d08)：浏览器录音统一进入 Liaison/Pilot，不直接控制机器人。
- 文本输入“走到前面自动售货机那里”可识别语义目标，并在 Pose、定位和运动门禁不足时安全拒绝，调用数为 0。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：语义预览和安全门禁。
- Nav2 lifecycle 已出现 `Managed nodes are active`；Mapping local map 已持续更新，但定位稳定性未验收。[Navigation PR #6](https://github.com/syswonder/service-navigation-rbnx/pull/6)：Nav2 基线；[Mapping PR #9](https://github.com/syswonder/service-map-rbnx/pull/9)：Mapping 基线。
- 远端旧 head 的 `offline-safety` CI 已通过；最新汇报材料已推送，本轮 Mapping/Navigation 本地补丁仍待分别推送和复验。[CI 记录](https://github.com/syswonder/robot-unitree-go2/actions/runs/29588111728/job/87910046603)：`9bdf039` 的验证任务成功；[材料 commit `49f0aae`](https://github.com/syswonder/robot-unitree-go2/commit/49f0aae02a7ebfdbd2d11c415f81b13c58258f7c)：最新文稿与实机截图。

## 风险 / 阻碍 / 问题

- canonical `/odom` 仍缺失，`map -> base_link` TF 尚未成立，页面尚无真实 Nav2 任务状态；因此不能进行自主导航验收。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：当前集成问题入口。
- Mapping 已有 ICP 匹配和地图输出，但漂移、外参及 TF 单一权威尚未通过静止/移动对照验证。[Mapping PR #9](https://github.com/syswonder/service-map-rbnx/pull/9)：后续定位链路基线。
- 浏览器语音入口与 FunASR 已上线，但麦克风录音到中文文本、语义 Pose、Nav2 的完整录屏尚未完成。[语音/UI commit `4aae9e7`](https://github.com/syswonder/robot-unitree-go2/commit/4aae9e758f61e5de3b70d2a4ce3ba2ed01786d08)：语音入口实现。
- 最新素材采集完成后，固定时间偏移超出锁定范围，守卫按设计关闭全栈；截图有效，但当前实时 URL 已离线。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：时间证据与失败关闭实现入口。
- 自动售货机前方安全 Pose 尚未在实地图保存；未发送真实运动命令，也未完成到达/停止验证。[Navigation PR #6](https://github.com/syswonder/service-navigation-rbnx/pull/6)：导航能力基线；[安全文档](https://github.com/syswonder/robot-unitree-go2/blob/main/docs/SAFETY.md)：运动验收规则。
- Mapping/Navigation 的本轮补丁仍是本地提交，不能把已合并的旧 PR 写成承载这些新修复。[Mapping PR #9](https://github.com/syswonder/service-map-rbnx/pull/9)：已合并基线；[Navigation PR #6](https://github.com/syswonder/service-navigation-rbnx/pull/6)：已合并基线。

## 下周计划

- 补齐 canonical `/odom` 和 `map -> odom -> base_link`，完成静止漂移与 TF 验收。[Mapping PR #9](https://github.com/syswonder/service-map-rbnx/pull/9)：Mapping/定位基线。
- 遥控完成实地图建图，在地图中保存“自动售货机”前方安全 Pose，并验证重定位。[Mapping PR #9](https://github.com/syswonder/service-map-rbnx/pull/9)：地图能力基线；[Scene PR #153](https://github.com/syswonder/robonix/pull/153)：语义场景基线。
- 完成浏览器中文语音端到端复验，保存 ASR 文本、语义解析和任务状态录屏。[语音/UI commit `4aae9e7`](https://github.com/syswonder/robot-unitree-go2/commit/4aae9e758f61e5de3b70d2a4ce3ba2ed01786d08)：语音入口。
- 在场地清空、遥控急停在手、唯一控制器和停止链通过后，分阶段做低速、短时、短距离运动测试。[安全文档](https://github.com/syswonder/robot-unitree-go2/blob/main/docs/SAFETY.md)：实机运动门禁。
- 更新本体 Draft PR，并为 Mapping/Navigation 本地补丁分别提交精确上游 PR。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：集成入口；[Mapping PR #9](https://github.com/syswonder/service-map-rbnx/pull/9) 与 [Navigation PR #6](https://github.com/syswonder/service-navigation-rbnx/pull/6)：上游基线。

## 建议插图 / 视频

- 图：`assets/03-full-stack-ui-nomotion.png`，标题写“真实 Go2 + Robonix 全栈无运动复验：相机、雷达、地图和语音入口在线；odom/TF 待完成”。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：实现与证据来源。
- 图：`assets/05-camera-full-stack.jpg`，真实 1920×1080 相机帧。[本体 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：相机 primitive 与 Dashboard。
- 文本：`assets/04-text-semantic-preview.ndjson`，展示“识别自动售货机但安全拒绝，调用数为 0”。[语音/UI commit `4aae9e7`](https://github.com/syswonder/robot-unitree-go2/commit/4aae9e758f61e5de3b70d2a4ce3ba2ed01786d08)：受控语义入口。
- 录屏：当前可录无运动实时 UI；自主运动视频必须等 odom/TF/定位和当次安全许可通过。[安全文档](https://github.com/syswonder/robot-unitree-go2/blob/main/docs/SAFETY.md)：不得用 App 遥控冒充自主导航。
