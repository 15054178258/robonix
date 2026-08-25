# 2026-07-18 Go2 实机汇报材料

本目录是本轮组会的最新稿。最后一次证据采集来自真实 Go2 EDU 的
`workstation-nomotion-stamp.5BBjlF` 会话；该会话禁止运动，不发布真实
`/cmd_vel`，不能作为自主导航或物理运动成功证据。

## 文件位置

| 文件 | 用途 |
| --- | --- |
| `feishu-weekly-update.md` | 可直接粘贴到飞书的精简周报；每项附 GitHub PR/commit 和解释 |
| `group-meeting-talk-track.md` | 约 7 分钟组会讲稿、演示顺序和常见问答 |
| `assets/README.md` | 真实截图、状态文件、校验值和证据边界 |

## 最新实机结论

- Robonix 的 10 个组件均启动成功：Atlas、Executor、Soma、Pilot、Liaison、Scene、Mapping、Speech、Nav2、Dashboard。
- Nav2 lifecycle 日志已出现 `Managed nodes are active`；速度守卫输出仍固定到 `/robonix/nomotion/cmd_vel`。
- UI 中相机、二维雷达、三维点云和 `/map` 均为实时数据；FunASR 中文流式 ASR、中文唤醒词和浏览器语音入口已就绪。
- Mapping 已产生 ICP 匹配和 local map，但漂移与 TF 稳定性尚未验收。
- canonical `/odom` 仍缺失，`map -> base_link` TF 尚未成立，Nav2 尚未收到真实目标。因此未进行自主运动，也未完成“自动售货机”闭环。
- 素材采集完成后，时间守卫因 `locked_offset_deviation_exceeded` 按设计关闭全栈；当前 8092 已离线，需重新生成当次时间证据后才能再次打开实时页面。

最新截图是 `assets/03-full-stack-ui-nomotion.png`，配套原始状态是
`assets/03-full-stack-status.json`。截图采集时 UI、ROS bridge、相机、雷达、
地图和浏览器语音入口均在线；页面上的红色缺失项必须保留，不能裁掉或写成已完成。
截图是最新成功会话的真实快照，不代表服务此刻仍在运行。

## GitHub 状态

| 项目 | 状态 | 链接与解释 |
| --- | --- | --- |
| Go2 本体适配 | Draft 开放；最新汇报材料已推送 | [robot-unitree-go2 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：本体能力、部署、UI 和安全门禁总入口；[材料 commit `49f0aae`](https://github.com/syswonder/robot-unitree-go2/commit/49f0aae02a7ebfdbd2d11c415f81b13c58258f7c)：本页、飞书稿、讲稿和真实截图 |
| Robonix 首页列表 | 开放 | [robonix PR #152](https://github.com/syswonder/robonix/pull/152)：在 Robonix 首页本体列表增加 Unitree Go2 |
| Catalog | 已合并 | [catalog PR #6](https://github.com/syswonder/robonix-package-catalog/pull/6)：发布 Go2 Catalog 条目 |
| Mapping 基线 | 已合并 | [service-map-rbnx PR #9](https://github.com/syswonder/service-map-rbnx/pull/9)：CycloneDDS 与无头运行兼容；本轮 QoS/ICP 本地补丁 `ac0c136`、`55373e6` 尚未另提 PR |
| Navigation 基线 | 已合并 | [service-navigation-rbnx PR #6](https://github.com/syswonder/service-navigation-rbnx/pull/6)：取消任务与 CycloneDDS 兼容；本轮行为树/运行时隔离本地补丁 `0abdd14`、`6477e68`、`087573c` 尚未另提 PR |
| Scene ROS 合同 | 已合并 | [robonix PR #153](https://github.com/syswonder/robonix/pull/153)：修复 Scene ROS 合同并缓存模型构建 |

## 证据边界

可以如实汇报“真实 Go2 传感器已接入 Robonix，完整无运动服务栈和 UI 已上线，
Nav2 managed nodes 已 ACTIVE，Mapping 已出地图”。不能汇报“已定位、已到达自动售货机、
已动态避障或已完成语音自主运动”。下一门槛是可信 `/odom`、完整 TF、实地图定位、
自动售货机安全 Pose、浏览器语音端到端和分阶段停止测试。
