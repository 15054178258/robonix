# Robonix-Go2 代码与接入说明

- 更新时间：2026-07-22 14:38 CST（UTC+8）
- 适用对象：需要在 Robonix 中接入 Unitree Go2、Mapping/Nav2 或后续 VLA/VLN Skill 的开发者
- 维护账号：`Origamii520`

## 可以直接转发的说明

下面是目前已经公开的 Unitree Go2 + Robonix 代码基线。建议先看 Go2
总仓库和 Draft PR #1，再按需看 Mapping、Navigation、Scene 与官方 Client。

当前可复用的是 Go2 本体抽象、传感器、Mapping/Nav2 兼容、Scene/UI 和
运动安全门的已发布基线。完整地图保存/重载、定位稳定性、目标安全 Pose、
停止链和分阶段真机运动仍在验收；本项目也没有宣称 InternVLA、VLN/VLA
推理或自主导航已经完成。

## 仓库、PR 与用途

| 模块 | 链接 | 截至 2026-07-22 的状态 | 用途与接入注释 |
| --- | --- | --- | --- |
| Go2 总适配仓库 | [仓库](https://github.com/syswonder/robot-unitree-go2) · [Draft PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1) · [公开分支](https://github.com/syswonder/robot-unitree-go2/tree/agent/fix-humble-interface-overlays) · [公开 HEAD `3cd5d719`](https://github.com/syswonder/robot-unitree-go2/commit/3cd5d71989271b2ee06a7ef385914bac3bededf9) | 开放 Draft，尚未合并 | 本体能力、底盘、相机、MID-360/IMU、描述树、Dashboard、语义目标路由、部署配置和默认关闭的运动安全门总入口。优先从这里了解整体拓扑。 |
| Navigation | [仓库](https://github.com/syswonder/service-navigation-rbnx) · [PR #6](https://github.com/syswonder/service-navigation-rbnx/pull/6) · [合并提交 `46fb8c05`](https://github.com/syswonder/service-navigation-rbnx/commit/46fb8c05b158d9865dc019bf2df7de2e4c5de92c) | 已合并 | Robonix Navigation 到 Nav2 的服务层，包含 CycloneDDS、Provider 地址传递、异步目标取消和失败关闭。VLA/VLN 应调用该服务能力，不要绕过安全层直接发布 `/cmd_vel`。 |
| Mapping | [仓库](https://github.com/syswonder/service-map-rbnx) · [PR #9](https://github.com/syswonder/service-map-rbnx/pull/9) · [合并提交 `76d201cb`](https://github.com/syswonder/service-map-rbnx/commit/76d201cb1d33248a712dc74458956c7e3a860ffc) | 已合并 | Mapping/地图服务基线，包含 CycloneDDS、无界面运行、Provider 地址传递和 ROS 接口隔离。用于建图、保存/加载地图和定位流程。 |
| Robonix Provider 回环绑定 | [PR #151](https://github.com/syswonder/robonix/pull/151) · [合并提交 `6fd5225e`](https://github.com/syswonder/robonix/commit/6fd5225e9cc6f792843791de9f394db533dee5de) | 已合并 | 支持 Provider 显式绑定 `127.0.0.1`，避免本机部署把服务无意暴露到外网。 |
| Scene ROS/构建基线 | [PR #153](https://github.com/syswonder/robonix/pull/153) · [合并提交 `ed5486c2`](https://github.com/syswonder/robonix/commit/ed5486c20a3130c121ad7b93db534a07d12bb89e) | 已合并 | Scene 对雷达、里程计、QoS、CycloneDDS、持久化目录和模型缓存的兼容基线。 |
| Scene 实时 UI 补丁 | [当前 Draft PR #173](https://github.com/syswonder/robonix/pull/173) · [主仓库分支](https://github.com/syswonder/robonix/tree/feat/scene-live-views) · [HEAD `7d71185e`](https://github.com/syswonder/robonix/commit/7d71185e5924f393a57a6ec090b61a5849c448c0) · [历史 PR #162](https://github.com/syswonder/robonix/pull/162) | #173 开放 Draft、尚未合并 | 相机、2D、3D 和地图实时页面的响应性、首屏状态和过期回调保护。仓库整理后，维护者将旧 #162 原样迁移为 #173，并注明原贡献者为 `Origamii520`；历史验证仍需在最新 `dev` 上复测后才能合并。 |
| Robonix 官方 Client | [仓库](https://github.com/syswonder/robonix-client) | 官方 `main` 基线 | 官方聊天/语音/任务界面。当前 Go2 本机上的额外生命周期修复尚未形成公开 PR，链接不包含这些本地修改。 |
| Go2 Catalog 条目 | [PR #6](https://github.com/syswonder/robonix-package-catalog/pull/6) · [合并提交 `450b8c7d`](https://github.com/syswonder/robonix-package-catalog/commit/450b8c7d62ccd233cb9f18633c658d5ef39202b2) | 已合并 | 发布 Go2 部署仓库入口；不是运行时实现。 |
| Robonix 首页 Go2 条目 | [PR #152](https://github.com/syswonder/robonix/pull/152) · [公开分支](https://github.com/Origamii520/robonix/tree/agent/add-unitree-go2) | 已关闭、未合并 | 只修改主仓库 README 的机器人列表；不是运行核心，也不能写成已合并。 |

## Go2 总仓库的关键代码入口

- [底盘适配与停止/看门狗](https://github.com/syswonder/robot-unitree-go2/tree/agent/fix-humble-interface-overlays/packages/go2_chassis)
- [相机、MID-360、IMU 等传感器 Provider](https://github.com/syswonder/robot-unitree-go2/tree/agent/fix-humble-interface-overlays/packages/go2_sensors)
- [URDF、部件树与 TF 描述](https://github.com/syswonder/robot-unitree-go2/tree/agent/fix-humble-interface-overlays/packages/go2_description)
- [相机/雷达/地图/任务 Dashboard](https://github.com/syswonder/robot-unitree-go2/tree/agent/fix-humble-interface-overlays/packages/go2_dashboard)
- [中文语义入口](https://github.com/syswonder/robot-unitree-go2/tree/agent/fix-humble-interface-overlays/packages/semantic_intent_router)
- [语义目标到 Navigation 服务的能力](https://github.com/syswonder/robot-unitree-go2/tree/agent/fix-humble-interface-overlays/packages/semantic_navigation)
- [整机 Robonix 部署清单](https://github.com/syswonder/robot-unitree-go2/blob/agent/fix-humble-interface-overlays/robonix_manifest.yaml)
- [运动安全边界](https://github.com/syswonder/robot-unitree-go2/blob/agent/fix-humble-interface-overlays/docs/SAFETY.md)

评估公开 Draft 分支可使用：

```bash
git clone --recursive --branch agent/fix-humble-interface-overlays \
  https://github.com/syswonder/robot-unitree-go2.git
```

该分支仍是评审中的开发基线，不应直接作为无人值守或真机运动版本。

## 给 VLA/VLN 接入者的建议

1. 本次交接不包含群聊中提到的 InternVLA 部署文档，也没有在当前 Go2
   或 Orin NX 上完成 InternVLA 推理验收。
2. 建议把 VLA/VLN 做成 Robonix Skill 或规划层：输出受约束的语义目标，
   再调用标准 Navigation 能力；不要让模型直接持有底盘控制权。
3. 定点导航路线可以复用 Mapping、Navigation 和 Go2 chassis/sensors。
   目标点必须来自已保存地图中的人工核验安全 Pose，不能把物体中心或模型
   临时猜测坐标直接下发给机器人。
4. 如果 Orin NX 的算力或显存不够，可把 VLA 推理放在远端计算节点，但仍需
   保留本机取消、超时、状态陈旧、失联和停车保护。远端推理可用不等于真机
   运动链已经安全可用。

## 公开代码与本机最新工作的边界

GitHub 链接目前只代表已发布基线，并不包含本机全部最新成果：

- Go2 本机 `HEAD=f74afaa`，比公开 PR 分支 `3cd5d719` 多 1 个尚未推送的
  相机提交，工作树还包含大量未整理提交的时间修正、无运动全栈、ICP、UI、
  首次运动门禁和测试修改。
- Mapping 的后续 Go2 QoS/ICP 提交 `ac0c136`、`55373e6` 仍只有本地分支，
  没有公开 PR。
- Navigation 的 through-poses 行为树、运行时隔离和 Humble composition
  后续提交仍只有本地分支，没有公开 PR。
- Scene 实时 UI 已由维护者从历史 #162 迁移到当前 Draft #173，但仍需基于
  最新 `dev` 重新同步、复测并完成评审，不能写成已合并。
- Robonix 的 Executor、语音生命周期、Hands-free、TTS 预热和启动取消清理，
  以及官方 Client 的额外修复，也尚未形成可分享的公开 PR。

因此对外应说“已分享公开基线”，不能说“当前全部代码已经上传”。这些本地
工作需要按仓库拆分、复核、测试并遵守新提交身份规范后再分别更新链接。

## 最新 Robonix 提交身份规范

官方说明：

- [贡献 Robonix 代码](https://robonix.syswonder.org/contributing/robonix)
- [参与维护 Robonix 文档](https://robonix.syswonder.org/contributing/documentation)

核心要求如下：

1. Git `author` 和 `committer` 必须是真实承担责任的人类；提交前检查：

   ```bash
   git config user.name
   git config user.email
   ```

2. AI 不能出现在 author/committer，也不能通过 `Co-authored-by`、
   `Co-developed-by`、`Signed-off-by`、`Reviewed-by`、`Tested-by`、
   `Acked-by`、`Suggested-by` 等尾注承担作者、DCO、审查或测试身份。
3. AI 对改动有实质辅助时，只使用以下格式披露工具和准确模型版本，不写
   邮箱：

   ```text
   Assisted-by: AGENT_NAME:MODEL_VERSION [TOOL ...]
   ```

4. `Signed-off-by` 只能由本人在确实作出 DCO 声明时通过 `git commit -s`
   添加；不能复制他人签名，也不能由 AI 代签。
5. 向 `syswonder/robonix` 提交前，从最新 `dev` 建短期分支，并运行：

   ```bash
   python3 scripts/check_commit_authorship.py --base origin/dev --head HEAD
   ```

6. PR 需要写清问题、实现取舍、真实运行过的验证、能力/配置影响、相关 issue，
   UI 变化还需附截图。文档修改应在 `robonix-book` 中按文档贡献流程提交，
   并披露 AI 辅助及人工核对、验证情况。

最终责任始终由人类提交者承担；AI 可以作为工具辅助，但不能作为提交主体。
