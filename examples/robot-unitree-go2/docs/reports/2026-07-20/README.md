# 2026-07-20 Go2 组会汇报材料（语音闭环、取消加固与时间戳回放已同步）

更新时间：2026-07-20 21:10 CST（Asia/Shanghai）

当前结论：真实 Unitree Go2 已在**不开放物理运动**的条件下接入 Robonix，本周完成了
传感器、本体状态、DDS 身份、时间修正、统一 odom/TF、临时实时建图、Nav2 待命、官方
Client/Scene/Mapping/Dashboard、语义入口和安全门的系统集成。充电后已多次重新完成 PCAP、
时间资格和静止状态取证；两轮 10 分钟全栈监测和多轮相机复验均已保存。针对发现的
Dashboard、Scene 和相机积压问题已形成补丁；全栈多次显示
`ROBONIX GO2 OPERATOR UI READY — MOTION DISABLED`，补丁后 60 秒 Dashboard 指标和最终
截图、600 秒 raw observer 和 600 秒 HTTP monitor 均已保存。相机 latest-frame/error-watermark
核心补丁已形成独立本地 commit `f74afaa`。较早 watermark build 的
一轮 60 秒窗口曾出现 2.8185 s source-lag 峰值和 63/121 个 unhealthy 样本，因此没有拿它
冒充最终 commit 验收；随后重新构建最终内容，取得新的 time/state/approval，并完成明确
绑定 `f74afaa` 与 binary SHA-256 的 60 秒复验：121/121 请求成功，HTTP error、non-fresh、
bridge disconnect、quality not-ready 和 unhealthy 均为 0，source lag 最大 1.4634 s，且
没有物理运动命令。这个结果证明最终 commit 在该 60 秒无运动窗口内通过，不等于对未来
任意时长作保证。

今天后续又完成三项收口。第一，Robonix Executor 新增只读
`robonix/system/executor/list_active_plans` 合同，本地 commit 为 `aa78ba45`；官方 Client
已从历史截图中的 `EXECUTOR UNAVAILABLE` 变为 `EXECUTOR VERIFIED`，当前无 active call、
0 个运行 plan。第二，官方 Client Audio Device Server 已 online，输入/输出路由和真实麦克风
采集通过，系统已安装 `ffmpeg 4.4.2-0ubuntu0.22.04.1`。第三，Liaison 原先在未配置保存目录
时回退写 `/tmp` 的偏差已改为显式 opt-in，并形成干净本地 commit
`e5f5c53f26879324fcb9f578e53f9d1789c0ebaa`；fmt、clippy、15/15 tests 和 build 均 PASS。
新 time/state/approval 下的 10 组件 MOTION DISABLED 栈已实际使用该 binary。19:13 受控
push-to-talk 实测精确识别 `请带我去自动售货机`，因运动、地图、定位、Nav2 和安全 Pose 门
未通过而拒绝执行，Pilot preview calls 为 0；TTS 播放 681,216 B PCM（约 21.288 s），用户
现场确认听到，且默认未在 `/tmp` 生成语音 WAV。19:25～19:26 又完成 30 秒 Hands-free
listener-only 启停：页面进入 enabled/listening，期间无 wake/error；显式关闭后 audio bridge
累计 303 帧停止，后续 5 秒无新 mic request，active plans 0、无 WAV。它只证明监听启停，
不是唤醒词或任务闭环。随后已离线修复 wake 后、ASR final 前关闭的取消竞态，fmt、clippy、
17/17 tests 与 diff-check PASS；新增只读验收器，只有关闭后连续至少 25 秒无 late
ASR/Pilot/TTS/session、无新任务且 transcript 不变才通过；陈旧 `enabled=true` 响应既不能清除
过渡窗事件，也不能重置 READY 时冻结的 transcript 基线，26/26 tests PASS。补丁仍未提交，
旧 binary 不含修复，尚未 live 复验。截图 `34`～`56` 已保存。

19:29:53 时间戳纪律层因 `corrected_timestamp_in_future:mid360_imu` 触发 fault，10 组件按
fail-closed 顺序关闭，`motion_ready=false`。因此当前不再把该栈写成仍在线；最后有效 live
strict 仍是 19:15 的 `111533Z`。19:31 在停栈后误跑、退回 root 路径的 strict 只有
1 PASS、6 FAIL、33 UNKNOWN，是无效运行态报告，不用于回归结论。离线分析定位到冻结 affine
模型与持续时钟频率的 ppm 级偏差会逐步吃完旧余量；no-motion 路径现已增加 20 ms anchor guard、
5 ms corrected-age soft floor 和 5 ppm locked drift 门，硬 future 门仍为 2 ms 且优先级不变。
32/32 affine、36/36 stamp、5/5 approval tests PASS，并对 42,511 条留存真机样本完成离线
qualification/lock replay；这仍不是补丁后 live endurance 结果。

这不是最终自主导航验收。尚无保存/重载后的真实场地图、重定位、自动售货机安全 Pose、
真实 Nav2 目标、voiceprint/access gate 或任何物理运动结果。最后有效 live strict
`logs/readiness/stack-readiness-20260720T111533Z.json`（SHA-256
`495dde2d439360860fdf17e0edeaddb2867f90a5b5005da5d4bb3274852271c2`）仍为
35 PASS、4 FAIL、1 UNKNOWN、`ready=false`。它在补丁前把 sensor namespace diagnostics 与
`semantic_navigation=INACTIVE` 一起列入 Atlas FAIL；随后 readiness 已增加精确四元组约束，
25 tests 和 diff-check 通过，历史 Atlas caps 离线验证两条 diagnostics 均 accepted，但整体
仍因 `semantic_navigation=INACTIVE` FAIL。尚无
补丁后重启的 live strict，所以不能把 namespace 写成仍确定失败，也不能提前写成现场已 PASS；
semantic-navigation、map/landmark、localization mode 与 map lifecycle 仍是明确未完成项。

## 文件位置

| 文件 | 用途 |
| --- | --- |
| `feishu-weekly-update.md` | 可直接粘贴到飞书共享文档；严格使用“工作内容、当前进度、风险/阻碍/问题、下周计划”四段 |
| `group-meeting-talk-track.md` | 详细、口语化组会讲稿；正文约 15～20 分钟，技术细节和问答可展开 |
| `assets/README.md` | 2026-07-20 截图、状态 JSON、日志、SHA-256 和录屏状态索引 |
| `../../../../../docs/handoffs/CONTINUE_2026-07-20_CHARGING.md` | 充电暂停点、原工作树指纹和恢复安全顺序 |

## 昨天到今天的变化

| 昨天群里的口径 | 今天新增且已有证据 | 仍未完成 |
| --- | --- | --- |
| Go2 本体、相机、MID-360、IMU 和 Robonix UI 已接入；Mapping/Nav2 能启动 | canonical `/odom`、`odom -> base_link`、RTAB-Map `map -> odom` 以及组合后的 `map -> odom -> base_link` 已在真实无运动栈跑通；Nav2 managed nodes 为 ACTIVE | 运动中的 TF/里程计稳定性、保存地图后的重定位 |
| 已有点云和实时地图，仍在补时间、odom 和 TF | 完成 affine 时间资格；机器人重启后保留一份证据不足 PCAP，并以订阅器在线的正式 20,000 包 PCAP 将 4 个当前 writer 关联到 `.161`；每次重采 time/state，旧批准没有被复用；19:29 新 IMU future fault 真实触发 fail-closed；随后完成 no-motion 余量/漂移门加固和 42,511 条留存样本 replay | 补丁后 live endurance、真实场地图保存/重载、地图 generation、自动售货机安全 Pose |
| 官方 UI、语音代码路径和“自动售货机”入口已接上 | 两轮 UI 监测完成；Executor 只读 active-plan 合同使 Client 显示 VERIFIED；Audio Server、设备路由、真实麦克风、精确中文 ASR、安全预览拒绝、扬声器 TTS、默认无 WAV 落盘和 30 秒 Hands-free listener 启停均通过；wake 后关闭竞态已完成离线代码修复和 17/17 测试，25 秒关闭后只读验收器 26/26 测试通过 | 唤醒词/任务闭环、取消补丁的新栈实测、voiceprint/access gate、语义 provider lifecycle、语音到真实目标和运动闭环 |
| 相机有真实画面，但 strict header age 一度约 2.09～2.46 s | 增加单槽 latest-frame mailbox 和 error watermark；不改源 stamp、不放宽 2 s 门限。最终重编后将 git commit `f74afaa` 与 binary SHA 写入 60 秒证据，121/121 成功、最大 lag 1.4634 s、零 unhealthy | 较早 build 曾出现 2.8185 s 峰值、vendor 4 次 API error 和 3 次 source reject；最终 60 秒通过不等于长期质量风险已消失 |

所以不能把今天概括为“只差让机器狗走”。当前跨过的是一个可复核的无运动全栈里程碑；
地图复用、定位、目标安全性、停止链和分阶段运动仍是彼此独立的验收门。

## 2026-07-20 最新证据

### 身份、时间和静止状态

- 机器人重启后第一份新 PCAP：
  `logs/go2-readonly/workstation-nomotion-identity-20260720T090311Z/go2-rtps.pcap`，
  20,000 包、26,863,887 B，SHA-256
  `461706df8678f30fc46692dbc1b332e2bfc1b1dc28e6b183d95e2e8b5dc86a1a`。它只看到
  participant prefix，没有捕获当前目标 writer 的 DATA，因此结论是
  `participant_prefix_seen_but_writer_data_not_proven`。这是有效的负面/证据不足样本，不是
  损坏文件，也不作为当前身份结论。
- 当前有效 PCAP：
  `logs/go2-readonly/workstation-nomotion-identity-20260720T092542Z-with-subs/go2-rtps.pcap`，
  在 sport/cloud/IMU/odom 订阅器在线时抓取 20,000 包、24,184,740 B、内核零丢包，SHA-256
  `004eade798c6cca068d30a5e1e3bb39eb4dca4671807affb4c28a70c01872841`。RTPS DATA writer
  关联分别为 sport primary 1,866、cloud 971、IMU 1,558、odom 941，四项结论均为
  `single_source_proven_by_rtps_data_writer`，唯一来源 `192.168.123.161`。本轮没有重新证明
  sport fallback，不能沿用旧 GID 冒充当前第五个 writer。
- post-commit 会话又采集 60 秒时间证据
  `logs/go2-readonly/workstation-nomotion-time-post-commit-20260720T101227Z/`，共 43,680 个样本；
  30 秒静止状态
  `logs/go2-readonly/20260720T101227Z-post-commit-stationary-20260720T101227Z/sport-state-summary.json`
  中 primary 8,890、fallback 598 个样本均为 mode 0、gait 0、error 100。对应只读批准为
  `rbnx-build/run/go2-nomotion-post-commit-20260720T101336Z-approval.json`；运动始终为 false。
- Liaison opt-in commit 验收没有复用上一个会话：新的时间证据
  `logs/go2-readonly/workstation-nomotion-time-liaison-opt-in-20260720T1052Z/summary.json`
  持续 60.22 秒、43,710 个样本，SHA-256
  `d19e36f77eac9fd6825a1b3636cd55ea06c1ffe0b2057b028c144242c279ba57`；静止状态
  `logs/go2-readonly/20260720T104933Z-liaison-opt-in-stationary/sport-state-summary.json`
  中 primary 8,885、fallback 600 个样本均为 mode 0、gait 0、error 100，SHA-256
  `23ac60c936f61fe3c8da83db7eb4577f9c7b4c4eee4f9f85c5cdca07b919d883`。对应无运动批准为
  `rbnx-build/run/go2-nomotion-liaison-opt-in-20260720T1053Z-approval.json`，SHA-256
  `9a7b90d9901a6d925e0943522b0e991fb304c6db67d38caf0b04e7d928663efe`；
  `motion_enabled=false`，有效窗为 18:52:23～19:52:23 CST。
- 较早、机器人重启前的正式 PCAP：
  `logs/go2-readonly/workstation-nomotion-identity-20260720T032526Z/go2-rtps.pcap`，
  20,000 包、24,157,880 B、内核零丢包，SHA-256
  `01528787cd1d453518827bc4e5cf38c22edff85abf3f205e2ca8cb7caa70a8c5`。
- 解码结果把 sport primary 1,877、sport fallback 126、cloud 966、IMU 1,577、odom 948
  个 DATA writer 样本唯一关联到机器人侧 `192.168.123.161`。这是当时会话的有效历史证据；
  机器人重启后 writer GID 已变化，不能替代上面的当前 PCAP。第一次不完整抓包单独保留为
  `go2-rtps-partial.pcap`，不作为结论来源。
- 最终重启前重新采集 60 秒时间证据：
  `logs/go2-readonly/workstation-nomotion-time-final-20260720T042738Z/`；43,862 个样本，
  只容忍已定义的重复/异常样本，没有复用旧批准。新的无运动批准为
  `workstation-nomotion-approval-final-20260720T0429Z.json`，只授权本次无运动会话。
- 30 秒静止状态：
  `logs/go2-readonly/20260720T042854Z-final-ui-restart/sport-state-summary.json`；primary 8,917、
  fallback 600 个样本，均为 mode 0、gait 0、error 100，零时间回退，最大线速度仅约
  `1.33e-6 m/s`。这只能证明采样期间静止，不能授权运动。

### 第一轮 10 分钟无运动全栈

- HTTP/UI 监测目录：`logs/go2-readonly/full-stack-ui-http-20260720T034700Z/`。600 秒、
  120 轮采样；Client、Scene `/user`、`/2d`、`/3d`、`/cam`、Mapping、Dashboard health
  和 status 共 8 个端点均零失败，bridge 零断连。
- raw observer 目录：`logs/go2-readonly/full-stack-ui-observer-20260720T034700Z/`。
  odom 86,977 条、最大 gap 45.82 ms；sport 160,401 条、最大 gap 29.54 ms；二者均无
  `>100/150/200 ms` 的 gap。cloud 9,229 条中有 2 次 295.36/355.56 ms gap，因此点云
  调度抖动仍保留为风险，不能写成已完全解决。
- Dashboard 的 map/camera/odom 在 120 轮中均未出现 non-fresh；pose_map 有 14 轮
  non-fresh。进一步定位到 1920×1080 图像转换在单线程 ROS executor 中逐像素运行，
  Dashboard 进程曾接近占满一个 CPU 核。
- 第一组截图和状态 JSON 已保存在 `docs/reports/2026-07-20/assets/`。其中 Scene `/user`
  和 `/2d` 的 headless 截图取帧过早，不能单独作为数据就绪证明；第二轮已等待页面
  `data-ready` 后重拍并归档为 `15`、`16`。相机只有 RGB 证据，没有 depth topic。

### 代码和离线验证

- Dashboard 图像解析改为按行批量转换并保留 padding/BGR/RGBA/mono 语义。本地 1080p
  BGR 基准约从 742 ms 降到 31 ms；项目 venv 70/70 测试通过。这个数字是本机局部基准，
  第二轮长时间真机结论以随后 600 秒 HTTP monitor 为准。
- 相机 bridge 增加单槽 latest-frame mailbox 和 error watermark：接收线程只保留最新帧，
  JPEG 解码/ROS 发布若落后就覆盖旧待处理帧；旧 reader/connection error 只有在相同或更新
  水位得到成功证据后才清除。代码仍使用原始 capture stamp；strict 2 s 门限未放宽，也
  没有伪造 stamp。全套 sensors 离线测试 PASS；240 帧压力测试覆盖 224 帧、最大 pending
  depth 1、最终 sequence 240；最终内容的 Humble overlay 编译 PASS。8 个文件已形成独立本地 commit
  `f74afaa0e37939a31ae8bc5b75b362e4a087c940`：
  `packages/go2_sensors/{README.md,src/camera_bridge_node.cpp,include/go2_sensors/camera_error_watermark.hpp,include/go2_sensors/latest_frame_mailbox.hpp,tests/test_camera_error_watermark.cpp,tests/test_latest_frame_mailbox.cpp,tests/run_offline_tests.sh,tests/static_safety_test.py}`。
  该 commit 尚未 push、暂无 PR，不在 Draft PR #1 的远端 CI 中。最终 installed binary
  已于 14:00:59 重编，SHA-256
  `c4be3e8a6de71677831ce868d2fca35f12c4ccdc1a3604a25f2535075e42561c`、Build ID
  `ddcf74727541197782bbdcd9d3dc296e17914277`；后续 60 秒证据同时记录该完整 hash 与
  git commit `f74afaa`，补齐了此前缺失的 commit-bound 绑定。
- readiness 检查修复了当前会话解析、IMU topic、transient-local `/map`、uint8 diagnostic
  level 等误判；定向测试 30/30 通过。真实 camera、map generation、Client 版本偏差等
  问题仍会继续作为真实门禁，不被隐藏。
- Scene 本地补丁把相机编码移出 asyncio loop，增加单飞/节流缓存，并为 `/user`、`/2d`
  增加加载、错误、页面恢复和过期回调 token+stamp 防护。focused、Scene suite、build
  contract 与 Node 语法/竞态检查通过：容器 focused 5 pass/5 skip，完整 Scene 113 pass/
  5 个 Node 依赖 skip，主机 build contract 5 pass；主机 Node 的三页语法和竞态 guard 通过。
  原始本地来源 commit 为 `6e501701`；补丁已挑选/重放到
  [Scene Draft PR #162](https://github.com/syswonder/robonix/pull/162)，标题
  `fix(scene): keep workstation live views responsive`，base `dev@b48f921c`，远端 HEAD
  `696938bf`，5 commits、11 files、+822/-62。Docs Check run `29720084954` 和 CI run
  `29720084977` 均为 completed/success；PR 仍为 Draft、未合并。无关 Soma `71d02bfd`
  未纳入该 PR。
- Executor 新增只读合同 `robonix/system/executor/list_active_plans`，返回权威 active-plan
  快照的 ops、state、provider/contract 标识，不增加 `control_plan`、取消、写入或运动路径。
  7 个文件、+146/-9 已形成本地 commit
  `aa78ba45b0a98e636c5d5c24948f81932a1bc686`；`cargo fmt --all -- --check`、
  `cargo clippy --workspace --all-targets -- -D warnings` 和 Executor 34/34 测试均 PASS。
  commit 尚未 push、暂无 PR。
- 官方 Client 音频依赖和设备链已恢复：Client venv 使用 `sounddevice 0.5.5`，Audio Device
  Server online，`audio_client_bridge` 选择当前 `#11 default (32 ch)` 输入/输出；系统安装的
  ffmpeg 为 `4.4.2-0ubuntu0.22.04.1`。Liaison 未配置
  `ROBONIX_LIAISON_VOICE_SAVE_DIR` 时回退到 `/tmp` 的偏差已改为只有显式非空目录才保存 WAV，
  README 同步说明；2 files、+24/-6 已形成干净本地 commit
  `e5f5c53f26879324fcb9f578e53f9d1789c0ebaa`，fmt、clippy、Liaison 15/15 测试和 build PASS，
  未 push、暂无 PR。新栈实际运行的 Liaison binary SHA-256 为
  `0bbc0a5ade430916c60dd7b60ccb73b10220ee1ee42ecee09de28262280574e7`，进程环境未设置保存目录。
- 在正确 source `/opt/ros/humble` 和 overlay 的环境中，完整 `validate_offline.sh` 通过，
  顶层发现 332 项测试。一次未 source ROS 的运行因缺 `rclpy` 正确暴露环境问题，不算
  代码失败，也没有通过安装依赖来掩盖。

### 补丁后第二轮复核

- `operator_ui_nomotion_readiness.py --phase full` 通过，Client、Scene root、Mapping、
  Dashboard 四项只读 UI HTTP 检查也全部通过。这里的“operator UI ready”只表示无运动
  展示面就绪，不表示完整导航 readiness 已通过。
- Dashboard 60 秒、2 Hz 采样 121/121 成功；camera、point_cloud、map、odom、pose_map
  全程 fresh，bridge 零断连。pose_map sequence 从 3,356 增至 3,956（+600），零回退、
  零相邻停滞，最大 source age 0.141 s。
- Dashboard CPU 平均 26.34%、P95 32.07%、最大 34.04%，RSS 约 101～125 MiB，HTTP
  P95 9.44 ms；相比修复前约 102% CPU 且首轮 120 个样本中 pose_map 14 次 non-fresh，
  即时结果明显改善；随后完成的第二轮 600 秒 HTTP monitor 也保持五路数据零 non-fresh。
- 第二轮及相机补丁后截图/结构化证据 `14`～`29` 已保存：Scene `/user` 和 `/2d` 已在加载完成后取帧，
  `/3d`、`/cam`、Mapping、Dashboard 均有真实数据，Client 文本安全拒绝和 Audio offline
  也有独立证据。完整文件名与 SHA-256 见 `assets/README.md`。
- 相机补丁前严格 readiness 连续两次为 34 PASS、5 FAIL、1 UNKNOWN；camera header age
  约 2.09～2.46 s，略超 strict 2 s 上限。latest-frame 补丁和新 time/state/approval 后，
  连续两次变为 35 PASS、4 FAIL、1 UNKNOWN，相机项 PASS。剩余项集中于尚未保存/实测的
  地图 generation 与售货机 Pose、仍处 mapping mode，以及 Atlas provider 生命周期/
  命名空间契约；不会把相机 PASS 写成完整自主导航 PASS。
- 核心 mailbox 轮 `logs/go2-readonly/camera-latest-runtime-20260720T050941Z/` 中，相机重启后 60.0069 秒、
  121/121 采样成功，HTTP error、non-fresh、bridge disconnect
  均为 0；sequence `158 -> 251`（+93）且零回退。source lag 最小 0.6875 s、中位
  1.0157 s、平均 1.0367 s、P95 1.3247 s、最大 1.3439 s，receipt age 最大 0.648 s。
  `quality_ready` 始终为 true，但 `healthy` 有 15 个瞬时 false 样本后恢复，说明 freshness
  已通过而 vendor 瞬时质量风险仍需保留。这是最终微调前一轮，最大 1.3439 s 不能代表
  后续所有窗口始终低于 2 s。截图 `24`、`25` 和完整 hash 见 assets 索引。
- watermark 版本最终保存会话的第一份新时间证据因
  `mid360_cloud source_receipt_delta_discontinuity` 被拒签，没有强行启动；retry 时间资格、
  新 30 秒静止状态和新 approval 随后通过，栈以 MOTION DISABLED 成功重启。最终硬件复跑
  `logs/go2-readonly/camera-watermark-runtime-20260720T053500Z/` 持续 60.0113 秒，121/121
  请求成功，HTTP error、bridge disconnect、non-fresh 均为 0；sequence `98 -> 186`
  （+88）且零回退。source lag 中位 1.0357 s、P95 1.4310 s、最大 2.8185 s，receipt age
  最大 1.941 s；`healthy=false` 63/121。daemon 累计有 4 次 vendor API error 和 3 次
  source reject，后续诊断恢复为 `healthy=true`、`last_error` 为空，说明错误水位恢复逻辑
  有效，但 strict 2 s 延迟尚非持续保障。该会话实际启动的 installed binary 时间为
  13:26:48；`f74afaa` 中 `camera_error_watermark.hpp` 最后一处跨 generation 恢复微调写于
  13:27:54，因此这轮较早证据不能严格绑定到最终 commit 内容；后面的 14:00:59 重编会话
  已另行完成精确绑定。
- 该较早 watermark build 会话的严格 readiness 快照为 35 PASS、4 FAIL、1 UNKNOWN，其中 camera snapshot PASS，
  检查时 age 0.916 s。剩余项仍集中于尚未保存/实测的地图 generation 与售货机 Pose、
  mapping mode，以及 Atlas provider 生命周期/命名空间契约。这个单点 PASS 不覆盖上一条
  60 秒窗口的 2.8185 s 峰值，也不等于完整自主导航 PASS。最终页面证据 `26`～`29` 显示
  Client Connected 但 Executor unavailable/hands-free off、Dashboard 真实数据和零速度、
  Scene RGB 无 depth，以及 Mapping 仍为 mapping 模式且没有已保存地图。
- 本轮 approval 的有效窗为 13:32:29～13:47:29；60 秒 runtime、strict 快照和 `26`～`29`
  截图均在有效窗内。到期后该会话的 wrapper 与传感器栈按设计终止，并记录 approval no
  longer valid 的 fault。后续 final-commit 复验使用的是另一组新证据和新批准，不能把两轮
  会话混为同一个持续在线会话。
- 最终 commit-bound 会话重新采集
  `workstation-nomotion-time-final-commit-20260720T060200Z` 60 秒时间证据和
  `20260720T060349Z-final-commit-state` 30 秒静止状态，生成 session
  `go2-nomotion-final-commit-20260720T060430Z` 的新无运动 approval；栈再次达到 10 components
  与 `OPERATOR UI READY — MOTION DISABLED`。随后
  `camera-final-commit-runtime-20260720T060900Z` 运行 60.0038 秒、121/121 成功，binary SHA
  前后均为 `c4be3e8a...42561c`，证据内 git commit 为 `f74afaa`；sequence `228 -> 320`
  （+92）、零回退，source lag 中位 1.0331 s、P95 1.3422 s、最大 1.4634 s，receipt age
  最大 0.656 s，所有质量/新鲜度/断连计数为 0，且明确
  `physical_motion_commands_performed=false`。最终页面截图 `30`～`33` 显示 Client 仍为
  hands-free off/Executor unavailable，Scene RGB 约 1.2 s 且无 depth，Mapping 仍无已保存
  地图，Dashboard 主要观测数据 fresh、导航任务缺失；这些边界没有因相机通过而改变。
- 同会话 strict 快照为 35 PASS、4 FAIL、1 UNKNOWN、`ready=false`；camera、cloud、IMU、
  map、odom、scan、三段动态 TF、Nav2 actions/lifecycle 和 `motion_gate_locked` 均 PASS，
  未通过项仍是 Atlas provider、map id、必须切换 localization mode、landmark binding 与 map
  lifecycle。验收完成后于 14:17 主动 Ctrl-C 安全关闭；supervisor exit 130 是 SIGINT 预期，
  相关 UI/服务按序停止，端口无监听、processes 为空、无残留 stack/UI/camera bridge 进程。
  全程没有物理运动命令。
- 第二轮 raw observer 已自然运行 600.210 秒：odom 86,417 条、最大 gap 90.76 ms，
  sport 158,766 条、最大 gap 40.00 ms，两者仍均为零次 `>100/150/200 ms`；cloud
  9,234 条、最大 gap 194.63 ms，出现 3 次 `>100 ms`、1 次 `>150 ms`、零次
  `>200 ms`。相较第一轮 cloud 最大 355.56 ms 有下降，但抖动没有消失。
- 第二轮 HTTP monitor 自然运行 600 秒，共 121 轮；Client、Scene 5 个页面与 2 个 API、
  Mapping、Dashboard health/status 共 10 个端点全部零失败，bridge 零断连，camera、
  cloud、map、odom、pose_map 均为零 non-fresh。最慢的 Scene `/user` 最大 58.486 ms，
  其余端点最大延迟均低于 30 ms。
- Client 文本实测能识别“请带我去自动售货机”，随后因
  `GO2_ALLOW_MOTION=false`、approach Pose 未验证和 map/localization/Nav2 门禁未通过而
  明确拒绝执行；调用 Robonix navigation、Nav2、Unitree API 的数量为零。对应截图为
  `assets/21-client-readonly-semantic-preview-20260720T044718Z.png`。
- 官方 Client 的 `22` 号 Audio 页面是 12:49 CST 的历史负面证据：当时 Audio Device
  Server offline/connection refused，输入输出未选，且缺 `sounddevice` 与 PortAudio；当时
  不能宣称中文语音可用。该截图和失败原因保留用于说明修复过程，后续最终结论以后面的
  `34`～`56` 号设备、ASR、安全拒绝、TTS 和监听启停证据为准。

### Executor、音频和受控语音最新复核

- `22` 是 12:49 CST 的 Audio Server offline 历史负面证据，`48` 是随后首次语音尝试中
  ffmpeg 缺失、TTS 跳过的历史负面证据；两者均保留，但不再代表最终状态。系统安装
  `ffmpeg 4.4.2-0ubuntu0.22.04.1` 后，Audio Device Server、input/output route、真实麦克风
  和扬声器播放均已复核。`34`～`39` 是修复前一轮设备/Executor 证据，`40`～`56` 是新
  Liaison 栈、完整语音和最终 UI 证据，文件名与 SHA-256 见 `assets/README.md`。
- 19:13 的受控 push-to-talk 窗口录入 195,200 B、6.10 s、61 帧音频，Client 无 error，ASR
  精确得到 9 字 `请带我去自动售货机`。Liaison/Pilot 将其识别为导航意图，但
  `GO2_ALLOW_MOTION=false`，地图仍 unsaved、localization/Nav2 readiness 和售货机安全 Pose
  未通过，所以只做安全预览拒绝；Pilot preview calls 为 0，没有生成实体 approach Pose。
- TTS 输出为 681,216 B PCM、约 21.288 s，Audio Server speaker 已连接，用户现场确认确实
  听到较长回复；本窗口 speech 无 ASR/TTS error。默认保存目录变量未设置，`/tmp` 语音 WAV
  数量为 0，证明 commit `e5f5c53f` 的 opt-in 行为在该无运动窗口生效。截图 `51` 标记为
  “语音识别 + 无运动语义预览 + TTS 播放，用户现场听觉确认”；19:13 这一轮不是
  Hands-free 或唤醒词闭环。
- Executor 日志全程仍为 6 行/752 B，semantic-navigation 日志仍为 11 行/1,824 B，二者本轮
  0 增量；active plans 为 0。因此未调用 Executor、Robonix Navigation、Nav2 或 Unitree API，
  更没有运动。voiceprint provider 不可用/未启用，access gate 未启用。
  这只证明受控 push-to-talk 的“采集—ASR—语义预览/安全拒绝—TTS”闭环，不是实体导航、
  访问控制、声纹认证或长期语音稳定性验收。
- 19:25～19:26 单独进行了 30 秒 Hands-free listener-only 测试：Client 显示 enabled/listening，
  监听期间无 wake/error；显式设回 false 后 audio bridge 共 303 帧停止，连续 5 秒无新增 mic
  request，active plans 仍为 0，未生成 WAV。截图 `55`/`56` 分别保存 listening 和 disabled
  状态。这只证明 listener 可以启停并在关闭后停止取流，不证明唤醒词、wake 后 ASR、任务或
  运动闭环。随后已在 Liaison 修复“wake 已触发但 ASR final 返回前用户关闭”的取消竞态：
  关闭会立即 drop 前台确认/采音/ASR future，并清理后台 turn；通知注册前已关闭的边界也有
  测试。fmt、clippy、全 targets 17/17 tests 和 diff-check PASS。该 2 文件补丁仍未提交、未在
  新栈做真实 wake 复验；access gate 仍 disabled，voiceprint 仍 unavailable。
- 最终页面中 RGB、2D occupancy、robot pose 和 Dashboard 数据正常；`52` 显示 PointCloud2
  1,035 点、83×118 fresh map、Nav2/semantic idle、VX/VY 0.00。WZ -0.01 属于静止传感器
  抖动，不作为运动证据。Scene 3D 的 `42`/`54` 虽有栅格与 robot，但仍为
  `objects: 0 · points: 0`，因此只能说底层点云数据存在，不能说 Scene 3D 点云/对象渲染完成。
- 最后有效 live strict 文件为 `logs/readiness/stack-readiness-20260720T111533Z.json`，SHA-256
  `495dde2d439360860fdf17e0edeaddb2867f90a5b5005da5d4bb3274852271c2`；结果仍为
  35 PASS、4 FAIL、1 UNKNOWN、`ready=false`。FAIL 为 `go2_sensors` camera/IMU provider
  namespace diagnostics 与 `semantic_navigation=INACTIVE`、`lab_go2` 无 landmark、仍为 mapping
  mode、landmark generation 未实测为正；UNKNOWN 为无可信 landmark 的精确 map lifecycle。
  这是 readiness allowlist 补丁前最后一份有效 live strict。
- 随后 `scripts/stack_readiness.py`、`tests/test_stack_readiness.py` 和 `docs/READINESS_GATE.md`
  增加精确 fail-closed allowlist：仅接受 provider `go2_sensors`、runtime namespace
  `robonix/primitive/lidar`、transport `ros2` 下的 camera/rgb 与 imu/imu 两个 contract；provider、
  namespace、contract、transport 任一变化、畸形布尔、缺失/重复允许项或额外 mismatch 都
  失败。25/25 tests 与 `git diff --check` PASS；历史 `atlas-caps.json` 离线验证两条均进入
  `accepted_namespace_diagnostics`，但整体仍因 `semantic_navigation=INACTIVE` FAIL。补丁尚未经过重启后的
  live strict，因此 namespace 项当前应写“代码/离线收口，待 live 确认”。
- 19:29:53 `workstation-nomotion-stamp.uFHTUb/fault.json` 记录
  `corrected_timestamp_in_future:mid360_imu`，SHA-256
  `5dcb579bd3391e51087f21372154aff96787e3ae77a7f11429dc7295d9456e27`；10 组件随后 fail-closed
  停止，motion/canonical odom readiness 均 false。19:31 停栈后误跑的
  `logs/readiness/stack-readiness-20260720T113143Z.json`，SHA-256
  `e198b5d4d212ab952e7557d9ad6b2d739f501bda4773946593e5e27c6485d8e2`，结果
  1 PASS、6 FAIL、33 UNKNOWN；其探针无监听、缺运行态且退回 root，所以只作为“停栈误跑”
  负面证据，不能用于补丁回归或替代 `111533Z`。

## 已解决与仍在收口的问题

1. **已解决：旧会话批准不能复用。** 重启后旧 state 证据被拒绝；重新采集 state 后，
   旧 time approval 又因 affine offset 与当前资格不一致被拒绝，新的 time/state/approval
   才能启动。门禁按设计失败关闭。
2. **已解决：Dashboard 图像热点已定位并优化。** 逐像素 Python 循环是 pose freshness
   周期性变差的主要本地瓶颈，代码和离线测试已完成；第二轮 cadence/CPU 和 600 秒
   HTTP 结果均已记录。
3. **已形成 Draft PR：Scene 页面响应与竞态。** `6e501701` 是原始本地来源提交；补丁已
   挑选/重放到 [Draft PR #162](https://github.com/syswonder/robonix/pull/162)，远端 HEAD
   `696938bf`，Docs Check 与 CI 均 success。PR 尚未合并，不能写成已进入 `dev`。
4. **仍在收口：点云偶发大 gap。** 第一轮 observer 测到 2 次大 gap，第二轮仍有 3 次
   `>100 ms`、其中 1 次 `>150 ms`；窄化处理只允许
   affine LOCKED、完全无运动、非核心 cloud 的孤立抖动帧被丢弃，并且不能刷新 last-good
   liveness。sport/IMU/odom、资格阶段和任何 motion profile 仍立即 fault，阈值未放宽。
5. **仍待现场：地图、定位、目标和停止链。** 临时 mapping 和静止 TF 不等于已保存地图；
   自动售货机 Pose、重定位、取消/停车/断网/接管和运动中传感器稳定性都没有完成。
6. **受控 push-to-talk 语音闭环已完成，长期与身份门仍待验。** Audio Device Server、设备
   路由、真实麦克风、精确中文 ASR、安全预览拒绝和用户可闻的 TTS 已通过；Hands-free
   listener-only 的 30 秒 enabled/listening→disabled 启停也通过，关闭后停止取流。它没有
   触发 wake，不是唤醒词或任务闭环；wake 后 ASR final 前关闭的取消竞态已经完成离线补丁与
   17/17 测试，但尚未新栈实测。voiceprint/access gate、多人、噪声、重复和失败恢复仍待测。
7. **已补齐 final commit 的 60 秒无运动绑定，长期质量仍需保留观察。** 较早 watermark
   build 的窗口最大 source lag 2.8185 s、`healthy=false` 63/121，并出现 4 次 vendor API
   error 和 3 次 source reject；最终 `f74afaa` 重编绑定窗口则最大 1.4634 s、121/121
   healthy 且所有新鲜度/断连计数为 0。后者解决了“最终 commit 未实机绑定”的证据缺口，
   但一次 60 秒通过仍不能证明 vendor 流在未来任意窗口都不会再抖动。
8. **Liaison 默认落盘偏差已提交并完成无运动实测。** 未设置保存目录时原实现会回退
   `/tmp`；2 文件补丁已改为显式 opt-in，形成本地 commit `e5f5c53f`，并通过 fmt、clippy、
   15/15 测试、build 和新栈实测。最终语音窗口 `/tmp` WAV 为 0。该 commit 尚未 push、
   暂无 PR；这项收口只涉及默认不落盘，不代表 voiceprint/access gate 或实体导航完成。
9. **新发现：IMU corrected timestamp 进入未来触发安全关闭。** 19:29:53 时间戳层以
   `corrected_timestamp_in_future:mid360_imu` fail-closed 关闭 10 组件；运动为 false。
   当前最可能是约 30 秒拟合后冻结的 affine 模型小幅斜率误差，叠加主机 NTP 渐进校时；
   旧 fault 缺 exact corrected age，因此仍不能排除单帧小幅 future。现已补充 future fault
   的触发流、精确 corrected age、2 ms 限值和全流 source/receipt/age 证据，且没有放宽
   2 ms 门限或自动 relock。no-motion-only 配置增加 20 ms affine anchor guard、5 ms soft
   corrected-age floor 和 5 ppm locked drift 门；hard-future 判定仍优先。仿射专项 32/32、
   时间戳层相关 36/36、approval 5/5 测试和 diff-check 均 PASS；新增 3 ppm 模型误差外推
   2,200 秒的确定性复现，并以 42,511 条真实留存样本完成 qualification/lock 离线 replay。
   回执 SHA-256 为 `49359dc9e2a64b28208555bc9bace9d13fbcffd51c7bc02ff9d716e9e1d58ae9`。
   这改善了有界耐久性和提前失败证据，但仍需新的 time/state/approval 做无运动 live endurance，
   不能写成已现场根治或绕过该 fault。
10. **readiness namespace 判断已做精确代码收口，待 live 回归。** 两条历史 Atlas diagnostics
    已被 exact-set 规则离线接受，25 tests/diff-check PASS；任何不精确、缺失、重复或畸形项
    仍失败，整体仍因 `semantic_navigation=INACTIVE` FAIL。
    尚无补丁后 live strict，因此不能把最后有效报告里的 namespace FAIL直接写成已现场解决。
11. **Hands-free 取消验收已具备独立只读工具，待新 binary 实测。** 工具只允许读取 Client
    settings、Hands-free status、active plans 和事件 WebSocket，不启动/关闭功能、不提交任务、
    不触碰机器人；关闭后不足 25 秒、出现 late ASR/Pilot/TTS/session、transcript 变化或非零
    plan 均 fail。READY 后的陈旧 enabled 响应不能清除过渡窗事件或重置 transcript 基线；
    26/26 离线测试、完整 loopback Client/WebSocket smoke、compile 和 diff-check PASS，尚未
    产生 live acceptance 回执。

## GitHub、提交和 PR 状态

| 项目 | 当前真实状态 | 边界 |
| --- | --- | --- |
| Go2 本体 | [Draft PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1) 开放；远端 HEAD `3cd5d719`，`offline-safety` CI run 17 success；本地 HEAD `f74afaa`、领先远端 1 commit | camera latest-frame/error-watermark 的 8 文件补丁已独立本地 commit `f74afaa`，未 push、无 PR、不在远端 CI；Dashboard、time、observer、readiness 和报告等其他修改仍在本地工作树 |
| Robonix provider | [PR #151](https://github.com/syswonder/robonix/pull/151) 已合并，merge commit `6fd5225e` | 已合并基线 |
| Robonix Executor | 本地 commit `aa78ba45b0a98e636c5d5c24948f81932a1bc686` | 只读 active-plan 合同；未 push、无 PR；fmt/clippy/34 tests PASS |
| Robonix Liaison | 本地 commit `e5f5c53f26879324fcb9f578e53f9d1789c0ebaa`，2 files、+24/-6；其上还有 Hands-free 2 文件未提交补丁、+123/-2 | `e5f5c53f` 的默认语音 WAV 显式 opt-in 已实测；Hands-free 取消补丁 fmt/clippy/17 tests PASS，但未 commit、旧 binary 不含修复、尚无 live 回执；均未 push、无 PR |
| Scene | [PR #153](https://github.com/syswonder/robonix/pull/153) 已合并基线，merge commit `ed5486c2`；[Draft PR #162](https://github.com/syswonder/robonix/pull/162) 开放，HEAD `696938bf`，Docs Check/CI success | `6e501701` 仅为原始本地来源；已挑选/重放为 #162 的 5 commits、11 files、+822/-62，base `dev@b48f921c`；#162 未合并且排除无关 Soma `71d02bfd` |
| Robonix 首页 | [PR #152](https://github.com/syswonder/robonix/pull/152) 开放、非 Draft；HEAD `210e9236` | 尚未合并 |
| Catalog | [PR #6](https://github.com/syswonder/robonix-package-catalog/pull/6) 已合并，merge commit `450b8c7d` | 已合并基线 |
| Mapping | [PR #9](https://github.com/syswonder/service-map-rbnx/pull/9) 已合并，merge commit `76d201cb` | 后续本地 `ac0c136`、`55373e6` 尚无新 PR |
| Navigation | [PR #6](https://github.com/syswonder/service-navigation-rbnx/pull/6) 已合并，merge commit `46fb8c05` | 后续本地 `0abdd14`、`6477e68`、`087573c`、`8b126e1`、`b4402e3` 尚无新 PR |

以上远端状态于 2026-07-20 19:30 CST 汇总（远端状态沿用本轮 18:35 实时核对）：Go2 Draft PR #1 为 open/mergeable，
远端 HEAD `3cd5d719...`、`offline-safety` success；Scene Draft PR #162 为 open/mergeable，
HEAD `696938bf...`、CI 与 Docs Check success；Mapping #9、Navigation #6、Catalog #6、
Robonix #151/#153 已合并，Robonix #152 仍 open 且非 Draft。Scene 发布状态已由远端 PR
和 Actions 复核；camera `f74afaa`、Executor `aa78ba45` 与 Liaison `e5f5c53f` 均仍未
push、无 PR。任何新 PR 只有真实创建后才会补链接。主仓仍在
`agent/fix-humble-interface-overlays`，本地 HEAD
`f74afaa0e37939a31ae8bc5b75b362e4a087c940`，远端仍为
`3cd5d71989271b2ee06a7ef385914bac3bededf9`；原有大量 staged、unstaged 和 untracked
集成修改均保留，没有 reset、stash、clean、重新初始化或克隆。

## 采购与网络边界

- 已购买但尚未送达：商品标称免驱、1300M 的 COMFAST CF-812AC 无线网卡，以及
  小米 AX3000E 千兆路由器。两件设备均未配置、未验收，不列为已完成能力。
- 到货后评估两种方案：机器人侧 NX 通过无线网卡接校园网并完成命令行认证；或 AX3000E
  组成狗和电脑的独立内网，再让机器人通过电脑受控代理上网。
- 当前专用有线链路足够继续软件开发和无运动验证，采购未到货不阻塞本轮工作；但驱动兼容、
  校园网认证、DDS 组播隔离、代理、断网回退和长期稳定性必须到货后实测。任何网络配置改动
  仍需单独批准，仓库不得保存认证或代理凭据。

## 验收边界与下一步

可以汇报：真实 Go2 已接入 Robonix；DDS writer 已取证；affine 时间、canonical odom/TF、
临时 Mapping、Nav2 ACTIVE、官方 Client/Scene/Mapping/Dashboard 已在无运动实栈工作；第一轮
和补丁后第二轮 10 分钟监测、文本语义安全拒绝及完整离线测试有可复核证据；Scene 已有
Draft PR #162 且 Docs Check/CI 均 success；相机 latest-frame/error-watermark 已形成独立
本地 commit `f74afaa`，并在不改 stamp、不放宽 strict 门限的前提下完成离线、编译和多轮
60 秒实机复核。较早 build 曾有 2.8185 s 峰值和 63/121 unhealthy；最终重编窗口已精确
绑定 `f74afaa` 与 binary SHA，121/121 healthy、最大 1.4634 s。最终 strict 的 camera PASS，
但 19:15 最后有效 live strict 仍为 35 PASS、4 FAIL、1 UNKNOWN、`ready=false`。Executor
`aa78ba45` 的只读查询已让 Client 显示 VERIFIED；Audio Server、设备路由和真实麦克风采集
已通过；Liaison `e5f5c53f` 已在新无运动栈完成精确中文 ASR、安全拒绝、用户可闻 TTS 和
默认无 `/tmp` WAV 验收。`34`～`56` 号截图和 SHA-256 均已归档。
Hands-free 另完成 30 秒 listener-only 启停并保存 `55`/`56`；取消竞态补丁及 25 秒关闭后
只读验收器均已离线通过。随后 IMU future-timestamp fault 按设计停栈；no-motion 时间戳
耐久性加固已通过 42,511 条留存样本 replay。最后有效 live strict 仍是 `111533Z`，不是
停栈后的 `113143Z`，以上离线结果都没有被冒充为 live PASS。

不能汇报：Scene Draft PR #162 已合并、camera `f74afaa` 已 push/已有 PR、今天 Go2 本地修改已进入远端 CI、相机在未来任意窗口始终低于 strict 2 s、cloud 抖动已根治、
Executor `aa78ba45` 或 Liaison `e5f5c53f` 已 push/已有 PR、voiceprint/access gate 已启用、
Hands-free 唤醒词/任务闭环和长期语音稳定性已验收、readiness allowlist 已通过 live strict、
Scene 3D 点云/objects 已完成、已保存真实地图、已完成
重定位、已到达自动售货机、已动态避障、已完成语音自主导航或发生过任何物理运动。

本轮无运动 UI 与 camera `f74afaa` commit-bound 复验已完成；接下来归档
第二段录屏（若文件已落盘），继续分析早先窗口中的 vendor API error/source reject、
>2 s source-lag 峰值和点云 gap，同时排查 Atlas 契约，
先提交/构建经批准的 Hands-free 补丁，再重新取得 time/state/approval，以新 MOTION DISABLED
栈验证时间戳 endurance 和 readiness exact allowlist；同时用只读验收器完成关闭后至少 25 秒
的 live wake-cancel 复验并保存回执。之后再收口 semantic-navigation 生命周期、
voiceprint/access gate 和 Scene 3D 数据展示。按真实
边界推送 `f74afaa`、`aa78ba45`、`e5f5c53f` 并拆分其余本地
commit/PR。其后只有
获得“人工遥控建图”的单独具体批准并完成清场、遥控/急停和人工接管确认，才可人工遥控
建图、保存/重载地图和测量自动售货机安全 Pose。任何 Robonix 控制链运动还需要新的单独
批准，首次限制为 `0.05 m/s`、`2 s`、约 `10 cm`，不能从无运动 UI 直接跳到自主导航。
