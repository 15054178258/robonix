# 组会周报（提交前草稿）

> PR 创建后，将下列仓库链接替换为对应 PR/commit 链接再粘贴到飞书。

## 工作内容

- 建立 Unitree Go2 的 Robonix 本体仓库，补齐 chassis、传感器、URDF/TF、部署和安全门禁。[GitHub](https://github.com/syswonder/robot-unitree-go2)
- 完成本机与 Go2 EDU/NX 的有线联调，接入前置相机、MID-360 点云和原始里程计。[GitHub](https://github.com/syswonder/robot-unitree-go2)
- 完成只读实机 UI，可实时显示相机、雷达和机器人原始位姿，并明确显示未通过项。[GitHub](https://github.com/syswonder/robot-unitree-go2)
- 梳理“中文语音→自动售货机语义 Pose→Robonix Navigation→Nav2→Go2”的完整数据链。[GitHub](https://github.com/syswonder/robot-unitree-go2)

## 当前进度

- 项目可离线构建，215 项离线测试通过；真实运动默认关闭。[GitHub](https://github.com/syswonder/robot-unitree-go2)
- 实机相机 1920×1080、约 10 Hz；MID-360 点云与原始里程计可稳定显示。[GitHub](https://github.com/syswonder/robot-unitree-go2)
- 中文语音预演链离线测试通过；实机浏览器麦克风入口待完整栈验证。[GitHub](https://github.com/syswonder/robot-unitree-go2)
- NX 已识别为 Jetson Orin NX，并完成 ARM64 基础镜像导入；完整常驻镜像仍在补齐。[GitHub](https://github.com/syswonder/robot-unitree-go2)
- 地图、定位、Nav2 执行和物理运动仍在安全门内，尚未宣称闭环完成。[GitHub](https://github.com/syswonder/robot-unitree-go2)

## 风险 / 阻碍 / 问题

- Go2 原始时间落后笔记本约 748 秒；已实现无运动固定偏移校正，但每次重启都要重新取证。[GitHub](https://github.com/syswonder/robot-unitree-go2)
- `SportModeState.error_code` 会随遥控模式变化；适配器按固件状态标记处理，变化即停止并重新确认。[GitHub](https://github.com/syswonder/robot-unitree-go2)
- 尚缺 3 次冷启动、长时间稳定性、TF/外参、地图定位和停止/接管实机证据。[GitHub](https://github.com/syswonder/robot-unitree-go2)

## 下周计划

- 完成时间修正无运动 profile，先跑通可信 odom/TF、mapping 和 localization。[GitHub](https://github.com/syswonder/robot-unitree-go2)
- 人工遥控建图，在地图中保存“自动售货机”前方安全 Pose。[GitHub](https://github.com/syswonder/robot-unitree-go2)
- 打通中文 ASR 和 Robonix UI 语义任务预览，再进行低速、短距离、可急停的分阶段实机验收。[GitHub](https://github.com/syswonder/robot-unitree-go2)
- 提交本体仓库、Catalog 和 Robonix 首页列表 PR。[Robonix](https://github.com/syswonder/robonix) / [Catalog](https://github.com/syswonder/robonix-package-catalog)

## 建议插图 / 视频

- 图：`go2-readonly-ui-live-20260718-1318.png`（真实相机、点云、原始位姿）。
- 视频：UI 实时相机/点云刷新 + 机器人保持静止的同框录像。
- 后续视频：中文语音识别和语义目标预览；物理运动必须等门禁通过后单独录制。
