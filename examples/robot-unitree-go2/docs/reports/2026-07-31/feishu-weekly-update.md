# Robonix-Go2 本周简要进展

- 完成 generation 1 地图的导航侧杂点抑制，保留原始地图比例、墙体和部署侧初始位姿。
- 跑通 Client 中文语音 → 语义地标 → Nav2 → 实体 Go2 的远距离去程与语音自主返航，完整闭环成功；少量终点偏差不影响结果，成功视频已留存。
- 全程保留停止、取消、看门狗、状态新鲜度、单控制器所有权和遥控器人工接管能力。
- Client、Scene 地图、Mapping 和 Dashboard 全栈网页均已恢复并完成实体流程验证。
- MID-360 二维激光障碍层已接入 Nav2；本次未做专门的摆放障碍物验收，D435i 仍仅用于预览。

## 相关 PR

- [Go2 完整全栈 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：当前唯一标准版本，Draft；运行代码基线 `8410fd7`，本次仅追加周报文档。
- [Mapping PR #15](https://github.com/syswonder/service-map-rbnx/pull/15)：地图生命周期、点云兼容和稠密二维扫描处理，Draft，HEAD `48adfc3`。
- [Navigation PR #9](https://github.com/syswonder/service-navigation-rbnx/pull/9)：持久导航运行时与原地旋转进度处理，Draft，HEAD `9d6c5e4`。
- [Client PR #7](https://github.com/syswonder/robonix-client/pull/7)：中文语音链路所用音频服务生命周期修复，Draft，HEAD `4f29a15`。
- [Robonix API PR #188](https://github.com/syswonder/robonix/pull/188)：本机回环 MCP Host 兼容，Draft，HEAD `954b764`。
- [Scene PR #173](https://github.com/syswonder/robonix/pull/173)：Scene 实时网页响应性修复，已合并。
- [Go2 Catalog PR #6](https://github.com/syswonder/robonix-package-catalog/pull/6)：Go2 仓库目录条目，已合并。

当前收尾项：待 Mapping 和 Navigation 依赖合入后，将主 PR 的子模块引用切回官方仓库并恢复 CI 全绿。
