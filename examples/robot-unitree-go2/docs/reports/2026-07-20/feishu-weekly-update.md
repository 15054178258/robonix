# Unitree Go2 接入 Robonix 周报

更新时间：2026-07-20 21:10 CST（Asia/Shanghai）

## 工作内容

- 本周我从零完成了 Go2 接入 Robonix 的无运动底座：本体状态、RGB 相机、MID-360、IMU、DDS 身份、时间修正、统一 odom/TF、Mapping/Nav2、官方 Client/Scene/Mapping/Dashboard、中文语义入口和默认关闭的运动安全门。GitHub：[Go2 Draft PR #1（开放）](https://github.com/syswonder/robot-unitree-go2/pull/1)，远端 HEAD `3cd5d719`、CI success；本地 HEAD `f74afaa`，尚未 push。
- 我完成了上游 ROS/QoS、Humble composition、导航行为树和 Scene 竞态适配。已合并：[provider #151](https://github.com/syswonder/robonix/pull/151)、[Scene 基线 #153](https://github.com/syswonder/robonix/pull/153)、[Catalog #6](https://github.com/syswonder/robonix-package-catalog/pull/6)、[Mapping #9](https://github.com/syswonder/service-map-rbnx/pull/9)、[Navigation #6](https://github.com/syswonder/service-navigation-rbnx/pull/6)；Scene 实机补丁在 [Draft PR #162（开放）](https://github.com/syswonder/robonix/pull/162)，HEAD `696938bf`，Docs Check/CI success。
- 我完成并实测了相机 latest-frame/error-watermark 本地 commit `f74afaa0e37939a31ae8bc5b75b362e4a087c940`、Executor 只读 active-plan commit `aa78ba45b0a98e636c5d5c24948f81932a1bc686`、Liaison 默认语音 WAV 显式 opt-in commit `e5f5c53f26879324fcb9f578e53f9d1789c0ebaa`。这三项均未 push、暂无 PR；Hands-free 关闭竞态为 2 文件未提交补丁，等待本地 commit 批准。
- 我完成了 PCAP、时间/静止状态、两轮 10 分钟全栈、相机 60 秒绑定复验、UI 截图和安全失败关闭证据归档，并准备了组会讲稿、飞书周报与截图索引。

## 当前进度

- 19:13 真实麦克风录入 195,200 B、6.10 s，ASR 精确得到“请带我去自动售货机”；Pilot preview calls 0，安全门拒绝执行。TTS 播放 681,216 B PCM（约 21.288 s），我现场确认听到；active plans 0、Executor/semantic-navigation 日志 0 增量、`/tmp` WAV 0。截图 `51` 已归档。
- 19:25～19:26 完成 30 秒 Hands-free listener-only 启停：enabled/listening、无 wake/error；关闭后取流停止，5 秒无新 mic request、active plans 0、无 WAV。截图 `55`/`56` 已归档；这只证明监听启停，不是唤醒词或任务闭环。
- 已离线修复“wake 后、ASR final 前关闭仍可能继续”的取消竞态：关闭会取消前台采音/ASR future 并清理后台 turn，fmt、clippy、17/17 tests 和 diff-check PASS。新增独立只读验收器，只访问 Client 的 settings、Hands-free status、active plans 和事件 WebSocket；轮询过渡窗按不确定区间 fail-closed，且陈旧 `enabled=true` 响应不能清除已观察到的事件或重置 READY 时的转录基线。关闭后连续至少 25 秒无 late ASR/Pilot/TTS/session、无新任务、转录不变才通过，26/26 离线 tests 及完整 loopback Client/WebSocket smoke 均 PASS，尚未 live。
- readiness exact-set 已收紧为 camera/IMU 两条 tuple 各恰好一次且布尔严格，25/25 tests PASS。19:29:53 IMU future timestamp 触发 fail-closed，10 组件安全停机；随后完成 no-motion-only 20 ms affine guard、5 ms soft floor、5 ppm drift 门及定量 fault 证据，32/32 affine、36/36 stamp、5/5 approval tests PASS，并对 42,511 条真实留存样本完成离线 replay。当前栈保持停止，尚无补丁后 live endurance。

## 风险/阻碍/问题

- 最后有效 live strict 仍是 `stack-readiness-20260720T111533Z.json`：35 PASS、4 FAIL、1 UNKNOWN、`ready=false`；readiness 和时间戳补丁仅完成代码/离线验证。Hands-free 2 文件补丁未提交、旧 binary 不含修复，不能直接重启冒充回归结果。
- `semantic_navigation=INACTIVE`、真实 landmark/map lifecycle、保存/重载地图、重定位、自动售货机安全 Pose、取消/停车/断网/人工接管及任何物理运动仍未验收；当前地图仍为 unsaved/mapping。
- Scene 3D 仍显示 `objects 0 / points 0`；底层点云可用不等于 3D 对象/点云渲染完成。相机和 cloud 仍保留长期抖动风险，单次干净窗口不代表长期稳定。
- COMFAST CF-812AC 无线网卡和小米 AX3000E 千兆路由器尚未送达；校园网命令行认证、机器人—电脑独立内网及电脑代理上网均未配置、未验收。

## 下周计划

- 获得 Hands-free 本地 commit 批准后构建新 binary；再取得新的 `go2-readonly` 批准，重新采 time/state/approval，启动 MOTION DISABLED 栈。启动前提醒录屏，并同时打开官方 Client、Audio、Scene 2D/3D/camera、Mapping 和 Dashboard。
- 运行补丁后的 live strict 和有界耐久观察；现场做一次 wake 后、ASR final 前关闭，使用只读验收器连续观察关闭后至少 25 秒，确认无 late ASR/Pilot/TTS/session、无任务和无 WAV，并保存 JSON、截图和录屏。
- 在无运动条件下继续收口 semantic provider lifecycle、地图保存/重载、localization、Scene 3D 和目标 Pose；真实创建 push/PR 后再补链接，不提前写成已有 PR。
- 采购件到货后，在每次网络配置单独批准下分阶段验证校园网/独立内网/电脑代理。只有另行确认清场、遥控/急停、人工接管并批准具体运动测试后，才进行厘米级低速行走、停止和目标导航验收。
