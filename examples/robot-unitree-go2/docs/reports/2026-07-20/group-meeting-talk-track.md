# Unitree Go2 接入 Robonix：组会详细讲稿

更新时间：2026-07-20 21:10 CST（Asia/Shanghai）

当前版本：Executor、音频设备链、Liaison opt-in commit、最终无运动语音、Hands-free
取消加固、时间戳离线回放、最新 PCAP/strict 和 Scene Draft PR 均已同步的汇报稿。充电后已经程序化确认有线
接口、DDS writer、当次 affine 时间资格和 30 秒静止状态，旧批准没有被复用；两轮全栈
UI、600 秒 raw observer、600 秒 HTTP monitor，以及相机补丁后的新 time/state/approval、
三轮 60 秒实机监测和 `14`～`56` 号截图都已保存。较早 build 的异常窗口被如实保留，
最终 `f74afaa` 已重编并完成 commit/binary hash 绑定复验；14:17 验收完成后主动安全关闭，
随后又以新证据启动无运动栈，完成 Executor 和音频设备链复核。Liaison 默认 `/tmp` 落盘
偏差已改为显式 opt-in，形成干净本地 commit `e5f5c53f` 并在新 time/state/approval 下实测：
精确中文 ASR、安全预览拒绝、用户可闻 TTS 和默认无 `/tmp` WAV 均通过；后续 30 秒
Hands-free listener-only 启停也完成，但没有 wake，不是唤醒词闭环。19:29:53 IMU future
timestamp fault 又真实触发 fail-closed，10 组件已停止；此后完成的是不接机器人进程的
no-motion 时间戳耐久性加固和 42,511 条留存样本回放，不是 live 复验。地图保存、localization、
voiceprint/access gate、Scene 3D 点云/objects 和任何物理运动仍列为未完成。

建议核心口径讲 12～15 分钟；完整技术稿约 25～35 分钟，问答另计。本文是详细备稿，
主讲时优先保留第 1、4、6、7、9、10、12 节；链接、GitHub 哈希和方括号提示不需要念，
第 3 节技术细节与第 13 节问答按现场时间展开。

## 1. 开场：先把本周结论和边界讲清楚

大家好，我这周主要做的是 Unitree Go2 EDU 接入 Robonix。这个工作是从零开始搭起来的，
不只是接一个机器人驱动，而是同时涉及真实硬件、ROS 和 DDS 数据、时间同步、坐标系、
建图和导航、官方 Client 和 Scene、中文文本语义与受控语音、安全门禁、自动化测试、实机取证，
以及多个上游仓库的兼容修改。

先讲目前最重要的结果：我已经在真实 Go2 上两次跑通一套**完全无运动的 Robonix 全栈**。
Robonix 10 个组件、官方 Client、Scene、Mapping、Nav2 和 Go2 Dashboard 可以同时工作；
统一的 `/odom` 以及 `map -> odom -> base_link` 坐标链已经建立。昨天之前已有一次 30 分钟
底座稳定性证据；今天充电后又完成正式 20,000 包抓包、时间和静止状态重取证，以及第一轮
10 分钟全栈监测。发现 UI 性能问题后，我完成了 Dashboard 本地修复和 Scene 本地 commit，
第二轮 UI 的 600 秒结果已经完成；随后相机 latest-frame/error-watermark 补丁也在不放宽
门限、不改源 timestamp 的前提下通过离线、编译和多轮重启后实机复核，并形成独立本地
commit `f74afaa`。一轮较早 build 的 60 秒窗口出现过 2.8185 秒峰值和 63/121 unhealthy，
所以我没有把它包装成最终结果。之后我重新构建最终 commit，重新取得 time/state/approval，
并把 git commit、运行 binary SHA 和 60 秒采样绑定到同一份证据：121/121 成功，最大
source lag 1.4634 秒，质量、新鲜度和断连异常计数全为 0。

下午我又补齐了 Executor 的只读 active-plan 合同，本地 commit 是 `aa78ba45`。官方 Client
现在已经从历史截图中的 `EXECUTOR UNAVAILABLE` 变成 `EXECUTOR VERIFIED`，并明确显示
没有 active Executor call、运行 plan 数量为 0。Audio Device Server、输入输出设备路由和
真实麦克风采集也已经通过。19:15 最后有效 live 严格检查仍是 35 PASS、4 FAIL、1 UNKNOWN、
`ready=false`：相机、TF、Nav2、Executor loopback 和运动锁通过；当时 Atlas 将两条 namespace
diagnostic 和 inactive semantic provider 一起列为 FAIL，地图、localization mode、landmark
generation 和 map lifecycle 也没有通过。之后我已加精确 allowlist 并完成 25 项测试与离线
Atlas caps 验证，但还没有补丁后 live strict，所以 namespace 当前是“代码收口、待实栈确认”，
不能继续说成确定失败，也不能提前说现场通过。

音频这边还发现了一个必须主动说明的偏差：Liaison 在保存目录未配置时原来会默认写 `/tmp`，
这和“默认不落盘”的约定不一致。我已经把它改成显式 opt-in，2 个文件通过 fmt、clippy、
15 项测试和 build，并形成干净本地 commit `e5f5c53f`。随后我用新 time/state/approval
重启 10 组件 MOTION DISABLED 栈。19:13 受控 push-to-talk 录入 6.10 秒真实音频，ASR
精确识别 `请带我去自动售货机`；安全门拒绝执行，Pilot preview calls 为 0；TTS 播放
681,216 B、约 21.288 秒，用户现场确认听到，`/tmp` 没有生成语音 WAV。这个结果证明的是
无运动 ASR、安全拒绝、TTS 和默认不落盘，不代表 voiceprint、access gate 或实体导航已完成。
19:25～19:26 我又单独做了 30 秒 Hands-free listener-only 启停：Client 显示 listening，期间
无 wake/error；显式设回 false 后 303 帧取流停止，连续 5 秒没有新 mic request，active plans
仍为 0，也没有 WAV。它只证明监听能启动、能停止，不是唤醒词、ASR 或任务闭环；wake 已触发
但 ASR final 返回前关闭的取消竞态当时尚未实测。随后我已完成离线补丁：关闭会立即 drop
前台确认/采音/ASR future 并清理后台 turn，通知注册前已关闭的竞态也有测试；fmt、clippy、
17/17 tests 和 diff-check 均通过。我又增加了独立只读验收器：不控制 Hands-free、不提交
任务，只读取本地 Client 的 settings/status/active plans 和事件流；上一次已确认 enabled 到
第一次确认 disabled 的轮询过渡窗也按不确定区间 fail-closed，关闭后还必须连续观察至少
25 秒。代理和 WebSocket 重定向被锁死在预连接的 `127.0.0.1` socket，发送给事件端点的 settings
副本会清空音频路由，证据目录 0700、文件 0600 且不覆盖已有文件；READY 后陈旧的
`enabled=true` 响应不能清除已观察到的事件，也不能重置 READY 时的转录基线。26/26 离线 tests、完整
loopback Client smoke 和提前 EOF WebSocket smoke PASS。
它仍未用新 binary 做现场 wake 复验，access gate 仍 disabled、voiceprint unavailable。

19:29:53 时间戳层又发现 `corrected_timestamp_in_future:mid360_imu`，按设计 fail-closed
关闭全部 10 组件，motion readiness 为 false。19:31 在停栈后误跑的 strict 出现 1 PASS、
6 FAIL、33 UNKNOWN，因为服务已停且路径退回 root，只是无效运行态负面样本，不能覆盖
19:15 最后一份有效 live strict。离线分析已把问题收敛到冻结 affine 模型与持续主机时钟频率
之间的 ppm 级偏差会逐步耗尽旧余量；no-motion 路径现增加 20 ms anchor guard、5 ms soft
corrected-age floor 和 5 ppm locked drift 门，2 ms hard-future 门没有放宽并保持最高优先级。
32/32 affine、36/36 stamp、5/5 approval tests PASS，42,511 条留存真机样本 qualification/lock
replay 通过。当前栈已经停止，这些都是离线结果，不再汇报为“仍在线”或“已经现场根治”。
整个过程没有运行原厂运动示例，也没有向真实运动入口发送命令，机器人始终保持静止。

这里也先把边界讲清楚：目前完成的是“真实硬件接入、无运动全栈、实时建图和导航服务
就绪”，还不能说“自主导航已经完成”。真实地图的保存和重载、重定位稳定性、自动售货机
前方的安全目标点、取消和停止、网络断开后的停止，以及最终实体运动，都还要分阶段验收。

## 2. 最终要做成什么

这个项目最终希望实现的效果是：用户在 Robonix UI 里说“走到前面的自动售货机那里”，
系统先做中文语音识别，再把“自动售货机”转换成地图中人工确认过的安全目标位姿，交给
Robonix Navigation 和 Nav2 做规划，最后才通过 Go2 chassis adapter 低速执行；到达、
取消、异常或失联时都必须停车。

我这里特意把链路拆开，是因为语音不能直接变成电机命令。系统必须依次确认数据确实来自
这台机器人、各路时间可比较、地图和机器人坐标正确、定位有效、目标点安全、导航服务就绪、
只有一个控制器拥有速度出口，并且停止链真实可用。任何一步不满足，系统都应拒绝执行。

所以这周的主要工作量，其实是在把这条链从“能启动一些组件”，推进到“每一层都有明确
输入、输出、安全边界和可复核证据”。

## 3. 本周从零完成的主要工作

### 3.1 建立 Go2 的 Robonix 本体适配框架

第一部分是从零建立 Go2 本体接入的整体框架。我补了 chassis、前置相机、MID-360、IMU、
URDF/TF、部署配置、Dashboard、语义意图路由，以及运动默认关闭的安全门。换句话说，
我不是只让某个 ROS topic 出现，而是把机器人在 Robonix 里需要的“身体、传感器、界面、
导航入口和安全规则”都放进了同一套可构建、可测试、可部署的工程里。

总集成入口是 [robot-unitree-go2 Draft PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)。
已经推送的安全相关代表修改包括：
[拒绝过期状态的 commit `9acc362`](https://github.com/syswonder/robot-unitree-go2/commit/9acc36276c9ba9e9803f6e4251378b6cd33e3d3b)
和 [补齐 MID-360 IMU frame 的 commit `6e02e5f`](https://github.com/syswonder/robot-unitree-go2/commit/6e02e5ffd2f61857fb6e38f52f9b115f5a2c396d)。

### 3.2 打通真实硬件和专用有线数据链

第二部分是真实硬件联调。现场已经确认并使用的是 Go2 EDU、背部 Unitree 100 TOPS
Jetson Orin NX、Livox MID-360、前置 RGB 相机、专用网线和原厂遥控器。笔记本通过
独立有线网口读取数据，不把普通互联网链路和机器人 DDS 链路混在一起。

目前前置相机已经观测到真实 1920×1080 RGB 图像；latest-frame 补丁后 60 秒窗口的有效
发布率约为 1.44～1.64 Hz。MID-360 的三维点云、二维
扫描、IMU、lidar odom 和 Go2 本体状态也都已经进入 Robonix 数据链。这里的“接入”
是指真实数据能被读取、转换和展示，不是用模拟数据替代。

### 3.3 确认数据源身份，而不是只看到 topic 就算成功

第三部分是 DDS 数据源身份取证。只看到 topic 名还不够，因为同一网络里可能存在缓存、
桥接或其他 participant。机器人重启后 writer GID 发生了变化，我没有沿用早先结论。

第一份新抓包 `090311Z` 有 20,000 包、26,863,887 字节，但因为抓包时目标订阅器没有持续
在线，只看到 participant prefix，没有目标 writer DATA。这个文件不是损坏，而是一个明确的
“证据不足”结果，我把它保留下来，没有拿来证明身份。随后让 sport、cloud、IMU 和 odom
订阅器在线，再抓取 `092542Z-with-subs`：20,000 包、24,184,740 字节、内核零丢包。
这次分别捕获 sport 1,866、点云 971、IMU 1,558、lidar odom 941 个目标 DATA 样本，四项
都唯一关联到 `192.168.123.161`。当前会话没有重新证明 sport fallback，所以没有把旧的
第五个 writer GID 拼到新证据里。

通俗地说，这一步是在确认“页面里看到的数据，确实来自当前 Go2 本体的 `.161` 数据源
和时钟域”，而不是把 NX 的 `.18` 地址或 topic 名当作来源证明。命令端点目前只有
participant 前缀证据，没有运动 payload 证据，所以我不会把它说成已经验证过真实命令来源。

### 3.4 解决机器人和笔记本时间不一致的问题

第四部分是时间基。机器人源时间和笔记本时间存在明显偏差，而且两个时钟的走速还有很小
差异，所以不能简单地永久减去一个固定数字。我先做只读采样，再冻结一个 affine，也就是
“固定偏差加缓慢漂移”的修正模型；运行时不随意追着数据调时间，避免把异常抹掉。

充电恢复时，这个门禁也被真实触发过：旧 state 证据先因过期被拒绝；补了新的 state 后，
旧 approval 又因为冻结 affine offset 与当前资格证据不一致被拒绝，偏差超过允许范围。
我随后重新采集了约 60 秒、43,862 个样本的时间证据，并生成新的当次无运动 approval。
也就是说，系统没有因为“昨天通过过”就沿用旧许可。新的批准同样只对当前无运动会话有效，
机器人重启、证据变化或过期后仍必须重新取证。

### 3.5 补齐统一里程计和完整坐标链

第五部分是 canonical `/odom` 和 TF。这里的 canonical 可以理解为“系统统一认可的
里程计”；TF 可以理解为“地图、里程计起点、机器人机身和传感器之间的坐标关系”。

昨天群里汇报时，这一块还是主要缺口。现在 lidar odom 已经经过时间纪律层转换成统一
`/odom`，`odom -> base_link` 已成立，RTAB-Map 能提供 `map -> odom`，组合后
`map -> odom -> base_link` 已经在真实 Go2 的无运动全栈里跑通。这样 Mapping、Nav2
和 UI 终于能够围绕同一套机器人位姿工作，而不是各看各的坐标。

这一结果目前证明的是静止状态和无运动栈成立；机器人运动后的长期漂移、重新定位和地图
复用，还不能由这一次静止证据代替。

### 3.6 让 Mapping 和 Nav2 真正进入同一套部署

第六部分是 Mapping 和 Navigation。我处理了 CycloneDDS、BestEffort QoS、无头运行、
参数实际下发、ICP 配置、Humble composition、行为树、运行目录隔离和取消失败关闭等
问题。成功会话中实时地图能够持续产生，Nav2 managed nodes 已经进入 ACTIVE。

这里的 ACTIVE 只表示导航服务已经启动并处于待命状态，不代表我已经给实体机器人发送过
目标，更不代表机器人已经走过。Nav2 的速度输出只被路由到无运动隔离 topic
`/robonix/nomotion/cmd_vel`，没有接入真实 chassis；运动适配器保持 PASSIVE。

Mapping 基线对应 [Mapping PR #9（已合并）](https://github.com/syswonder/service-map-rbnx/pull/9)，
Navigation 基线对应 [Navigation PR #6（已合并）](https://github.com/syswonder/service-navigation-rbnx/pull/6)。
本周后续适配还有本地提交，稍后 GitHub 状态部分会单独说明，不能混到旧 PR 里。

### 3.7 打通官方 Client、Scene、Mapping 和 Dashboard 展示面

第七部分是 UI 和官方客户端。我完成了 provider loopback、ROS/QoS 合同、CycloneDDS
接口选择和 Scene 模型缓存兼容，使官方 Robonix Client、Scene 的 2D/3D/相机页面、
Mapping 页面和 Go2 Dashboard 能在同一个无运动会话中实时打开。

今天第一轮 10 分钟全栈把一个比较隐蔽的问题暴露出来：所有页面 HTTP 都是 200，
但 Dashboard 的 1920×1080 图像还在 ROS 单线程 executor 中用 Python 逐像素转换，进程
一度约占 102% CPU，120 个状态样本中 pose_map 有 14 次变成 non-fresh。也就是说，
“网页能打开”不等于“里面每路数据都按预期刷新”。我把图像转换改成按行批量处理，
本地基准大约从 742 ms 降到 31 ms，Dashboard venv 70 项测试全部通过。

Scene 侧也发现相机编码会占用 asyncio 主循环，而且 `/user`、`/2d` 的异步 onload 可能
发生旧回调覆盖新数据。我把编码移出事件循环，加了单飞、节流缓存、加载/错误状态和
token+stamp 竞态保护，原始本地来源 commit 为 `6e501701`。补丁现已挑选/重放到
[Scene Draft PR #162](https://github.com/syswonder/robonix/pull/162)，远端 HEAD `696938bf`，
5 commits、11 files、+822/-62；Docs Check 和 CI 均 success。PR 仍为 Draft、未合并，
无关 Soma `71d02bfd` 没有纳入；已合并的 PR #153 只代表以前的 Scene 基线。

补丁后的 60 秒 Dashboard 复核是 121/121 次请求成功，camera、cloud、map、odom 和
pose_map 全部 fresh；pose sequence 从 3,356 增加到 3,956，没有回退或相邻停滞，
CPU 平均 26.34%、P95 32.07%，HTTP P95 9.44 ms。Scene `/user` 和 `/2d` 也已经在
加载完成后重新截图。随后第二轮 600 秒 HTTP monitor 也完成了 121 轮采样，五路数据
保持零 non-fresh，说明无运动 UI 的长时间复验通过；这仍不等于严格导航 readiness 通过。

严格门随后又抓到一个更细的问题：相机 sequence 一直增长，Dashboard 也认为 fresh，但
保存状态的 header age 约 2.09～2.46 秒，略超 strict 2 秒。这里没有改成更宽的阈值，也
没有把时间戳换成“现在”；我在 camera bridge 的接收与 JPEG 解码/ROS 发布之间增加了
单槽 latest-frame mailbox。处理跟不上时只覆盖旧的待处理帧，发布时仍保留最新帧原始
capture stamp，从源头避免排队越积越旧。

后续微调又增加了 error watermark：旧 reader 或连接错误不能被更旧状态误清除，只有相同
或更新水位的成功证据才能让诊断恢复。相机修改现在覆盖 `packages/go2_sensors` 的 README、
camera bridge、两个 header、两个 C++ 测试、离线测试入口和静态安全测试，共 8 个文件。
全套 sensors 离线测试 PASS，240 帧压力测试中 224 帧被较新帧覆盖、最大 pending depth
始终为 1、最后消费 sequence 240。8 个文件已经形成独立本地 commit
`f74afaa0e37939a31ae8bc5b75b362e4a087c940`，但尚未 push、没有 PR，也不在 Draft PR #1
远端 `3cd5d719` 的 CI 中。Humble overlay 最终内容已重编，installed binary 时间为
14:00:59，SHA-256 为 `c4be3e8a…42561c`、Build ID 为 `ddcf7472…14277`；后面的 runtime
summary 同时记录了这个完整 hash 与 git commit `f74afaa`。

核心 mailbox 补丁后重新取 time/state/approval 并启动无运动栈，我先监测 60.0069 秒，
共 121 次请求全部成功，
error、non-fresh、disconnect 都是 0；相机 sequence 从 158 增到 251。source lag 中位
1.0157 秒、P95 1.3247 秒、最大 1.3439 秒，receipt age 最大 0.648 秒。严格 readiness
连续两次从补丁前 34 PASS、5 FAIL、1 UNKNOWN 改善到 35 PASS、4 FAIL、1 UNKNOWN，
相机项 PASS。不过 `quality_ready` 虽始终为 true，`healthy` 仍有 15 个瞬时 false 样本后
恢复。这是 error-watermark 最终微调之前的一轮，不能拿最大 1.3439 秒推导后续每个窗口
都会持续低于 2 秒。

watermark 已构建版本的最终保存会话中，我没有沿用旧批准。第一份新 time 证据因为
`mid360_cloud source_receipt_delta_discontinuity` 被门禁拒签；retry 时间资格、新静止 state
和新 approval 通过后，才以 MOTION DISABLED 重启。本次 60.0113 秒硬件复跑仍是 121/121
成功、HTTP error 为 0、bridge disconnect 为 0、non-fresh 为 0，sequence 从 98 增到 186，
没有回退。source lag 中位 1.0357 秒、P95 1.4310 秒，但最大达到 2.8185 秒，receipt age
最大 1.941 秒；121 个样本中 63 个 `healthy=false`。daemon 累计记录 4 次 vendor API error
和 3 次 source reject，后续诊断又恢复为 `healthy=true`、`last_error` 为空。

这里还有一个必须主动说明的历史构建绑定限制。上面出现 2.8185 秒峰值的那轮实际运行的是
13:26:48 生成的 installed binary，
SHA-256 为 `247df931…b66a1`；`f74afaa` 的 `camera_error_watermark.hpp` 最后一次写入是
13:27:54。最后这处修改允许更新 generation 的成功帧清除旧 generation 的 stream-record
error，尚未进入该实机 binary。binary 本身已有 mailbox/watermark 符号，因此这不是“完全
没跑补丁”，但只能表述为“较早 watermark build 做过实机复跑”，不能表述为“`f74afaa`
最终内容已完成 commit-bound 实机复跑”。这条限制只针对那轮旧证据，不能外推到下面的
最终重编会话。

为补齐这个缺口，我重新采集了 60 秒 time 和 30 秒静止 state，生成新的只读 approval，
最终栈再次达到 10 components 和 `OPERATOR UI READY — MOTION DISABLED`。随后
`camera-final-commit-runtime-20260720T060900Z` 连续运行 60.0038 秒，证据内 git commit
明确为 `f74afaa`，binary SHA 在运行前后均为 `c4be3e8a…42561c`。121 次请求全部成功，
HTTP error、non-fresh、disconnect、quality not-ready 和 unhealthy 都是 0；sequence 从
228 增到 320、零回退，source lag 中位 1.0331 秒、P95 1.3422 秒、最大 1.4634 秒，
receipt age 最大 0.656 秒，并明确记录 `physical_motion_commands_performed=false`。

这说明两个结论要同时说：第一，最终 `f74afaa` 在这一个绑定清楚的 60 秒无运动窗口内通过；
第二，较早窗口出现过 vendor error 和 2.8185 秒峰值，所以仍不能承诺未来任意时长都无抖动。
最终同会话 strict 快照是 35 PASS、4 FAIL、1 UNKNOWN、`ready=false`，camera 检查时 age
0.922 秒并 PASS；未通过的是 Atlas provider、map id、必须切换 localization mode、landmark
binding 和 map lifecycle。`30`～`33` 号最终截图也保留负面状态：Client Connected 但
Hands-free off、`EXECUTOR UNAVAILABLE`；Scene RGB 约 1.2 秒且无 depth；Mapping 仍是
mapping 模式并显示 no saved maps；Dashboard 主要观测数据 fresh，但 navigation task
缺失。14:17 验收后主动 Ctrl-C，所有 UI/服务按序停止，相关端口无监听、processes 为空，
没有残留 stack/UI/camera bridge 进程。

对应的上游基线是
[Robonix provider PR #151（已合并）](https://github.com/syswonder/robonix/pull/151)
和 [Scene PR #153（已合并）](https://github.com/syswonder/robonix/pull/153)。
新 live-view 修复由 [Scene Draft PR #162](https://github.com/syswonder/robonix/pull/162)
承载，Docs Check 与 CI 均 success，但尚未合并。
这解决的是“官方界面能真实消费 Go2 数据”的问题，不只是我另外做一个独立调试页。

在 18:19～18:24 的前一轮无运动会话里，我又把官方 Client、Audio debug、Dashboard、Mapping
和 Scene 放在同一组当前截图中复核。Client Chat 显示 `EXECUTOR VERIFIED`、没有 active call、
0 plans；Client Audio 显示 Server online、设备 route 已应用、麦克风测试成功；Audio debug
显示 `#11 default` 输入输出和实时 VU。Dashboard 同时显示真实 1920×1080 相机、LaserScan、
79×74 临时地图、map pose 和近零速度；Mapping 仍明确是 mapping mode、no saved maps；
Scene `/user` 也写明 live session/unsaved。截图编号为 `34`～`39`。

之后为了验收 Liaison `e5f5c53f`，我重新取得 time/state/approval 并启动新栈，保存了
`40`～`56` 号截图。官方 Client 仍是 Executor verified、0 active plans；
Scene 相机和 2D occupancy 正常，Mapping 仍是 mapping/no saved maps。最终 Dashboard
显示相机健康、PointCloud2 1,035 点、83×118 fresh map、Nav2/semantic idle、VX/VY 0.00；
WZ -0.01 是静止传感器抖动，不是运动证据。Scene 3D 虽能显示栅格和 robot，但仍写着
`objects: 0 · points: 0`。因此底层点云存在和 Scene 3D 尚未渲染点云/对象要同时汇报。

这里需要区分“官方 Client”和“配套 UI”：Client 自身主要负责 Chat、Audio 和 Settings；
相机、二维/三维地图、雷达与机器人姿态由 Scene、Mapping 和 Go2 Dashboard 展示。它们组合成
完整操作展示面，但不能把这些页面都说成 Client 自身内置功能。

### 3.8 打通受控中文 ASR、安全拒绝、TTS 和默认不落盘

第八部分是语音和语义任务。这里我把“单次受控 push-to-talk 已通过”和“完整语音自主导航
已完成”严格分开。任何文本或语音请求仍必须经过 Liaison/Pilot、地图、定位和运动门禁，
不会直接变成电机命令。

这条链也是分阶段排查出来的。早先截图 `22` 里 Audio Device Server 确实 offline，当时缺
`sounddevice` 和 PortAudio；依赖补齐后，Server、输入输出 route 和真实麦克风采集恢复。
第一次语音尝试又在截图 `48` 里暴露 ffmpeg 缺失，所以只有很弱的 transcript `嗯`，TTS
被明确跳过。我保留这两个历史失败，没有把后来的成功反写成“一开始就正常”。随后系统安装
`ffmpeg 4.4.2-0ubuntu0.22.04.1`，再进入最终测试。

审计时我还发现 Liaison 未配置保存目录会回退写 `/tmp`，这和默认不落盘的约定不一致。
我把保存逻辑改成只有显式配置非空 `ROBONIX_LIAISON_VOICE_SAVE_DIR` 才落 WAV，补了 README
和测试。2 files、+24/-6 已提交为本地 commit
`e5f5c53f26879324fcb9f578e53f9d1789c0ebaa`；fmt、clippy、15/15 tests 和 build 全部 PASS。
该 commit 尚未 push、暂无 PR，所以 GitHub 状态仍写“本地 commit”。

为了不把旧进程冒充新补丁，我又采了 60.22 秒、43,710 个时间样本和 30 秒静止状态；
primary 8,885、fallback 600 个样本全部 mode 0、gait 0、error 100，再生成仅无运动 approval。
新栈的 10 个组件全部启动，Liaison 运行 binary SHA 是 `0bbc0a5a…80574e7`，进程环境确认
没有保存目录变量。

最终 19:13 的 push-to-talk 窗口录入 195,200 字节、6.10 秒、61 帧，Client 没有 error，
ASR 精确得到 9 个字“请带我去自动售货机”。系统识别到导航意图，但
`GO2_ALLOW_MOTION=false`，地图仍 unsaved，localization/Nav2 readiness 和售货机安全 Pose
都没有通过，所以只返回安全拒绝；Pilot preview calls 是 0，没有生成实体 approach Pose。
TTS 随后向已连接的 speaker 播放 681,216 字节 PCM，约 21.288 秒，用户现场确认确实听到；
本窗口 speech 没有 ASR/TTS error，`/tmp` voice WAV 数量为 0。截图 `51` 的准确标签是
“语音识别 + 无运动语义预览 + TTS 播放，用户现场听觉确认”。它不是 Hands-free 或唤醒词
闭环；19:13 这轮 push-to-talk 未启用 Hands-free。

我还对调用侧做了反向审计：Executor 日志全程保持 6 行/752 字节，semantic-navigation 日志
保持 11 行/1,824 字节，两者在这轮都是 0 增量；Client active plans 为 0。因此没有调用
Executor、Robonix Navigation、Nav2 或 Unitree API，更没有运动。截图 `50` 是用户桌面现场
听音确认上下文，显示较早一轮 `请带我去` 和 221,184 字节播放；最终完整九字 transcript
和 681,216 字节播放以截图 `51` 为准，不能把两轮证据混为一张。

19:25～19:26 我把 Hands-free 单独限制为 listener-only 做了 30 秒启停测试。截图 `55`
显示 enabled/listening、event stream connected，期间没有 wake/error；随后显式设为 false，
截图 `56` 显示 Hands-free off。audio bridge 共收到 303 帧后停止，连续 5 秒没有新 mic
request，active plans 仍为 0，`/tmp` 也没有 WAV。这证明监听器可以按开关启动和停止，不是
唤醒词识别、wake 后 ASR、语义任务或运动闭环。

这里还发现过一个取消竞态：如果 wake 已经触发，但 ASR final 还没返回，用户此时关闭
Hands-free，旧实现只会 abort 已进入后台列表的 turn，前台 future 可能继续。现在代码已改为
先注册 race-safe 通知，再检查 enabled；关闭会立即 drop 前台 future，现有 VoiceSessionStream
Drop 会 abort producer，而且取消不伪报 KIND_ERROR。两条 Tokio 测试覆盖运行中关闭和关闭先
发生两种顺序，Liaison 全 targets 17/17 与 clippy 均通过。不过目前不能拿离线测试覆盖真实
wake 现场复验；access gate 仍 disabled，voiceprint provider 仍 unavailable。

为避免下一轮只靠 UI 肉眼判断，我又补了一个完全独立的有界 observer。它不会帮我开关
Hands-free，也不会提交或取消任务，更不会碰运动接口；它只观察本地 Client 的状态、active
plans 和事件流。通过条件不是“关闭后没看到一条消息”这么宽松，而是：上一个 enabled 样本到
第一个 disabled 样本之间如果出现 ASR final、Pilot、TTS 或 session done，就按边界不确定直接
失败；确认关闭后还要至少连续 25 秒 transcript 不变、active plans 始终为 0、没有任何上述
late event。WebSocket 使用预连接 loopback socket、禁用代理和重定向，观察副本清空音频路由；
输出目录和文件也做了 symlink、覆盖和权限保护，陈旧的 enabled 响应既不能抹掉过渡窗事件，
也不能重置转录基线。当前 26/26 离线测试及完整 loopback Client/
WebSocket smoke 均通过，但还没有 live
acceptance JSON，所以组会上只能说“验收方法准备好”，不能说 wake-cancel 已现场通过。

所以现在可以准确说：受控 push-to-talk 的“真实采集—中文 ASR—语义预览/安全拒绝—TTS
扬声器播放—默认不落盘”已经在无运动栈通过，Hands-free listener-only 启停也通过。仍不能
说的是 Hands-free 唤醒词/任务闭环已通过、voiceprint 或 access gate 已启用、
semantic-navigation provider 已 active、语音已转换成真实目标，
或者机器人已经走到自动售货机。对应已推送基线代表修改仍是
[语音/UI commit `4aae9e7`](https://github.com/syswonder/robot-unitree-go2/commit/4aae9e758f61e5de3b70d2a4ce3ba2ed01786d08)。

### 3.9 建立安全门、自动化测试和实机证据链

第九部分是安全和验证。本周没有运行 `sport_mode_ctrl`、`go2_sport_client`、
`go2_stand_example` 或低层控制示例，也没有向 `/api/sport/request`、`/lowcmd` 或真实
`/cmd_vel` 发布。所有实机验证都先在只读或无运动 profile 下进行。

此前成功会话中我连续观察了 30 分钟：odom 收到 261,210 条、最大接收间隔约 46.89 ms；
sport 收到 488,193 条、最大间隔约 31.24 ms，二者都没有超过 100、150 或 200 ms
的 gap。今天第一轮又完成了 10 分钟 cloud-inclusive 观察：odom 86,977 条、最大
45.82 ms；sport 160,401 条、最大 29.54 ms，仍然没有超过三个门限；cloud 9,229 条
里有两次 295.36 和 355.56 ms 的 gap，所以 cloud 抖动不能写成已经完全解决。

同期第一轮 10 分钟 UI monitor 每 5 秒检查一次，共 120 轮；Client、Scene 四个页面、
Mapping、Dashboard health/status 共 8 个端点全部零失败，bridge 零断连。补丁后的第二轮
operator UI readiness 和四项 UI 检查已经 PASS，最终截图也已保存；第二轮 600 秒 raw
observer 和 600 秒 HTTP monitor 均已自然结束。

本轮后续又出现一条新的安全证据：19:29:53 时间戳层检测到
`corrected_timestamp_in_future:mid360_imu`，立即把 canonical odom 和 motion readiness 置为
false，并按顺序关闭 10 组件。fault JSON 的 SHA-256 是 `5dcb579b…56e27`。这说明失败关闭
生效。后续离线根因分析和门限加固已经完成，但还没有新栈 endurance，不能把安全关闭或
离线 replay 写成现场问题已解决。
停栈后 19:31 误跑的 strict 因服务全部离线而得到 1 PASS、6 FAIL、33 UNKNOWN，只保留为
错误运行上下文，不作为回归结果。

这组数据的意义不是“机器人已经能安全自主运动”，而是证明当前无运动底座不是偶然亮一下
页面，而是能够在真实数据压力下持续运行，并且会把 cloud gap、camera 时间年龄之类的
真实残余问题保留下来，不用一个“页面能开”掩盖它们。

## 4. 和昨天群里汇报相比，今天新增了什么

昨天我在群里汇报的是：Go2 本体、相机、MID-360、IMU 和 Robonix UI 已接入，Mapping
和 Nav2 能启动，已经有点云和实时地图；当时主要还在补 `/odom` 和完整 TF，也没有完成
定位链的验收。

今天新增的重点如下：

1. canonical `/odom` 和静止的 `map -> odom -> base_link` 已经在真实无运动栈中跑通；
2. 时间修正从固定偏移推进到有实测资格证据的 affine 模型；
3. 机器人重启后先保留一份“只有 participant、没有 writer DATA”的负面 PCAP，再用订阅器
   在线的 20,000 包 PCAP 将当前 sport、cloud、IMU、odom 四个 writer 精确关联到 `.161`，
   并重新完成 time/state/approval；
4. 官方 Client、Scene、Mapping、Nav2 和 Dashboard 已在同一无运动会话内组成展示面；
   odom/sport 完成了 30 分钟接收观察，四个 UI 完成 45 轮 HTTP 检查，Nav2 为 ACTIVE；
5. 第一轮 10 分钟 UI 端点零失败，同时用 raw observer 把 odom/sport 的稳定性和 cloud
   的两次大 gap 分开量化，不把“HTTP 正常”和“底层没有抖动”混为一谈；
6. Dashboard 图像热点已优化，60 秒 121/121 次采样五路全 fresh，CPU 从修复前约 102%
   降到平均 26.34%；Scene 响应与竞态修复已进入 Draft PR #162，两项 CI 均 success；
7. 补丁后第二轮全栈已 READY，Scene `/user`、`/2d`、`/3d`、相机、Mapping、Dashboard
   的最终截图已保存；第二轮 raw observer 和 HTTP monitor 均已完成，但严格导航门仍有
   未通过项，所以只写“无运动 UI 复验完成”，不写“自主导航最终验收完成”；
8. 相机 latest-frame mailbox 在不改 stamp、不放宽 2 秒门限的前提下完成离线、压力、
   Humble 编译和第一轮重启后 60 秒实测；该核心 mailbox 轮最大 source lag 1.3439 秒；
9. error-watermark 最终内容已形成 8 文件独立本地 commit `f74afaa`；较早 watermark build
   的一轮窗口最大 2.8185 秒、63/121 unhealthy，这段异常证据被保留为历史风险；
10. 最终内容重新编译后，使用新 time/state/approval 做了 commit-bound 无运动复验：
   60 秒 121/121 成功，最大 lag 1.4634 秒、零 unhealthy，git commit 与 binary SHA 已写入
   同一证据；
11. Executor 只读 active-plan 合同形成 `aa78ba45`，Client 从 unavailable 更新为 verified、
   当前 0 plans；补丁后的 strict 仍为 35 PASS、4 FAIL、1 UNKNOWN，所以 Executor 可查询
   不等于完整导航 ready；
12. Audio Server、设备路由和真实麦克风采集已通；Liaison 默认 `/tmp` 落盘偏差形成
   `e5f5c53f`，新栈验证默认无 WAV。19:13 精确识别“请带我去自动售货机”，安全预览
   calls 为 0，TTS 播放 681,216 字节且用户确认听到；Executor/semantic-navigation 日志
   0 增量、active plans 0，全程没有运动；
13. `40`～`56` 号最终截图和 19:15 strict 已归档。Dashboard 看到 PointCloud2 1,035 点，
   但 Scene 3D 仍为 objects 0/points 0；最后有效 live strict 仍是 35 PASS、4 FAIL、1 UNKNOWN，
   所以语音成功和底层点云存在不能覆盖 provider/map/localization/3D UI 的缺口；
14. 30 秒 Hands-free listener-only 启停完成，无 wake/error，关闭后 5 秒无新 mic request；
   这不是唤醒词/任务验收；wake 后 ASR final 前关闭竞态已离线修复并通过 17/17 测试，待 live；
15. readiness exact allowlist 已通过 25 tests、diff-check 和历史 Atlas caps 离线验证，但没有
   post-patch live strict；19:29 IMU future timestamp fault 随后 fail-closed 停栈。19:31
   停栈后报告无效，最后有效 live strict 仍是 `111533Z`。

所以今天的进度不是简单地说“现在只差让机器人走”。更准确的说法是：底层统一时间、
统一里程计、坐标链、建图导航服务和官方 UI 已经跨过一个关键无运动里程碑；接下来要把
地图复用、重定位、安全目标、停止链和实体运动逐项验收。

## 5. 本周已经解决的问题

第一类是 ROS/DDS 和部署兼容问题。Mapping 原来存在 QoS 不匹配、参数没有真正下发等
情况；Scene 和 provider 也需要适配 loopback、ROS 合同、CycloneDDS 和模型缓存。
这些基线修改已经分别进入已合并 PR，其中较早的 Go2 专项适配也在无运动实栈中运行过。
今天新增的 Dashboard、time、observer 和 readiness 修改已通过当前离线回归，但仍在
Go2 本地工作树；camera 的 8 文件补丁已独立 commit 为 `f74afaa`，尚未 push、无 PR；
Scene 新修复已从原始来源 `6e501701` 挑选/重放到 Draft PR #162，两项 CI 均 success，
但尚未合并。补丁后即时真机结果已经明显改善，第二轮 600 秒 HTTP 与 raw observer、
相机三轮 60 秒复跑都已完成，其中最后一轮是最终 commit-bound 复验。

第二类是时间问题。原先只能看到机器人和笔记本时间相差很大，现在已经有“只读采样、
资格判断、冻结 affine 模型、运行时异常失败关闭”的完整路径。它仍要求每次会话重新取证，
但不再依赖人工猜一个固定偏移。

第三类是 odom 和 TF 问题。昨天缺失的统一 `/odom` 和 `map -> odom -> base_link` 已经
建立，使 Mapping、Nav2 和 UI 能围绕同一位姿源工作。

第四类是 Nav2 部署和控制权问题。行为树、运行目录、Humble composition、取消失败
关闭以及速度出口隔离已经补齐；无运动会话中 Nav2 可以 ACTIVE，但真实 chassis adapter
仍然 PASSIVE，不会因为服务启动就获得运动权限。取消和失败关闭在软件及无运动条件下已有
证据，实体运动条件下仍要单独验收。

第五类是异常关闭链。我遇到过一轮点云时间异常导致全栈自动退出。虽然根因还在继续收口，
但已经实证：超过门限时系统会停止全栈，而不是带着不可信状态继续运行，也没有产生运动。

第六类是相机积压。之前 sequence 仍增长，但旧帧在解码/发布队列里排队，导致 strict
header age 超过 2 秒。latest-frame mailbox 保持原 stamp 和原门限，只让最新待处理帧覆盖
旧待处理帧，error watermark 也能防止旧状态误清除新错误。队列水位和错误恢复机制已经
通过离线、编译及实机复核。较早 build 的窗口曾有 2.8185 秒峰值和 63/121 unhealthy；
最终 `f74afaa` 绑定窗口最大 1.4634 秒且 121/121 healthy。所以能说“最终 commit 在本次
60 秒窗口通过”，但仍不能把一次通过扩展成 vendor 流长期无异常。

第七类是 Client 的 Executor 状态查询。之前 Client 只能把缺少 active-plan 合同显示成
`EXECUTOR UNAVAILABLE`。我新增只读 `list_active_plans` 合同，返回权威快照而不开放任何
控制路径；`aa78ba45` 通过 fmt、全 workspace clippy 和 34 项 Executor 测试。现场页面已经
显示 VERIFIED 和 0 plans，这解决的是“状态可审计”，不是“已经执行过任务”。

第八类是音频和默认隐私行为。系统依赖、Audio Server、设备选择、麦克风和 ffmpeg 问题已经
解决；Liaison 默认 `/tmp` 落盘和“不落盘”约定的偏差也已改成显式 opt-in，形成本地 commit
`e5f5c53f` 并通过新栈实测。最终窗口精确中文 ASR、安全拒绝、TTS 听音和 `/tmp` WAV 为 0
都有证据。这里关闭的是受控无运动语音和默认不落盘问题，voiceprint/access gate、Hands-free、
长期稳定性和实体导航仍在后续项。Hands-free 的 listener-only 启停随后也通过，但唤醒词和
wake 后任务尚未实测；关闭竞态只完成离线修复、尚未 live 复验，所以不能把“监听能停”写成
“Hands-free 完整完成”。

第九类是 readiness 对 Atlas namespace diagnostic 的误判。我没有把所有 mismatch 一概忽略，
而是只允许 `go2_sensors + robonix/primitive/lidar + camera/rgb或imu/imu + ros2` 两个精确四元组；
provider、namespace、contract、transport 任一变化、畸形布尔、缺失/重复允许项或额外
mismatch 仍失败。25 项测试和 diff-check 通过，历史 Atlas caps 离线确认两条均 accepted，
且整体仍因 `semantic_navigation=INACTIVE` 失败。因为新补丁还没有重启后 live
strict，这一项只能写“代码与离线收口”，不能写成现场 gate 已通过。

## 6. 尚未完成、正在解决的问题

这部分我会主动说清楚，避免把候选修复或服务启动说成最终完成。

### 6.1 Dashboard 无运动 UI 长时间复验已通过，严格导航门仍未通过

Dashboard 原先从 TF 消费 pose 时发生积压，曾把落后约 4.86 秒的位姿标成 fresh，
而 RTAB-Map 自身只落后约 0.09 秒。现在工作树已改成直接订阅 Mapping 输出的
`/robonix/map/pose`，并同时检查数据源时间年龄和本机接收年龄，也就是 source age 与
receipt age；地图仍保留“后启动的页面也能拿到最近快照”的 transient-local 机制。
同时我把 1080p 图像从逐像素 Python 转换改成按行批量转换，避免长期占用单线程 executor。

补丁后 60 秒 2 Hz 真机采样中，121 次请求全部成功，五路状态全 fresh，pose sequence
增加 600 且没有回退或停滞，最大年龄 0.141 秒，CPU 平均 26.34%。所以我可以说
“即时复核已经改善”。随后第二轮 600 秒 HTTP monitor 的 121 轮采样也全部成功，五路
数据零 non-fresh，说明 UI 长时间复验通过；这仍不替代严格完整导航 readiness。

### 6.2 大点云回调偶发抖动的根因仍在收口

另一个会话中，点云数据源时间和本机接收/调度时间的差值突变，也就是 source/receipt
delta error，达到 223.94 ms，超过 150 ms 门限，全栈按设计失败关闭。现有时间线
不能证明它由抓包造成，也不能简单归因于某一个工具。

我已经实现一个非常窄的候选处理：只在 affine 已 LOCKED、完全无运动、而且对象是非核心
`mid360_cloud` 时，允许把孤立抖动帧丢掉；丢弃后不发布、不进入统计或缓存，也不重置
“最近一次正常数据”的存活计时。sport、IMU、odom、资格阶段以及任何运动 profile 仍然
立即 fault，150 ms 和 500 ms 安全阈值都没有放宽。只读点云 observer 也已经补上。

恢复后我又完成了两项关键复核：第一，点云帧能被丢弃之前，冻结模型、cache 对齐、
所有数据流的存活性和定期统计硬门都会先检查，任何一项失败都会先失败关闭；
第二，被丢弃帧后又到达的相同 source timestamp 不会污染 last-good duplicate 统计。
相关 timestamp 定向测试现为 77/77。第一轮 cloud-inclusive 真机只读观察已经完成：
10 分钟内 odom 和 sport 没有超过 100/150/200 ms 的 gap，但 cloud 仍有两次
295.36/355.56 ms gap。这说明核心小消息流稳定，而大点云的调度抖动确实仍存在。
第二轮 observer 也已自然运行 600.210 秒：odom 86,417、sport 158,766 条仍无任何
`>100/150/200 ms` gap；cloud 9,234 条、最大 194.63 ms，有 3 次 `>100 ms`、
1 次 `>150 ms`、零次 `>200 ms`。峰值比第一轮低，但抖动仍存在，所以根因与修复效果
仍不能写成最终解决。

### 6.3 当前离线回归及无运动实证已通过，整体实机闭环未完成

恢复后，timestamp 定向测试已由 75/75 增加到 77/77；完整
`validate_offline.sh` 也已在正确 ROS/overlay 环境通过，当前顶层发现 332 项测试；
Dashboard 项目 venv 70/70，readiness 定向 30/30，C++ 与 sensor guards 也通过。
系统 Python 下存在有明确依赖原因的跳过项，但项目 venv 内 Dashboard 测试全部运行。
相机 latest-frame/error-watermark 修改又完成了全套 sensors 离线测试和 240 帧单槽压力
测试；Humble overlay 最终内容已重编，并取得 commit `f74afaa`、binary SHA 前后值和
60 秒采样相互绑定的无运动实机证据。
Go2 Draft PR #1 远端已推送基线的 `offline-safety` CI run 17 也是 success。

但这里要分清两个层次：第一，camera 的 8 个文件已独立本地 commit `f74afaa`，但未 push、
无 PR；Dashboard、time、observer、readiness 和其他新增测试仍在本地工作树，因此远端 CI
都不包含这些本地改动；第二，Dashboard
和相机补丁已有无运动实机证据，但 cloud 抖动根因、保存地图后的定位、停止链和运动闭环
仍未最终验收。所以我能说“当前离线回归和无运动实证已过”，但不会说“整个实机闭环已完成”。

### 6.4 严格完整导航 readiness 仍有真实未通过项

operator UI readiness 已经 PASS。相机补丁前，更严格的完整导航 readiness 连续两次是
34 PASS、5 FAIL、1 UNKNOWN；latest-frame 补丁、新 time/state/approval 和 60 秒实测后，
连续两次变为 35 PASS、4 FAIL、1 UNKNOWN，相机项 PASS。这两种检查不能混用：前者回答
“无运动 UI 能不能展示”，后者回答“地图、定位、语义目标和整条导航契约是否满足”。

Executor `aa78ba45` 和 Liaison `e5f5c53f` 实际运行后，19:15 CST 最后一份有效 live strict 是
`logs/readiness/stack-readiness-20260720T111533Z.json`，SHA-256
`495dde2d439360860fdf17e0edeaddb2867f90a5b5005da5d4bb3274852271c2`。结果仍是
35 PASS、4 FAIL、1 UNKNOWN、`ready=false`。第一个 FAIL 当时把 `go2_sensors` 的 camera/IMU
namespace diagnostics 和 `semantic_navigation` INACTIVE 一起列出；其余 FAIL 是配置
`lab_go2` 没有 landmark 绑定、仍处 mapping mode、landmark generation 不是实测正值；
UNKNOWN 是没有可信 landmark 可比较的精确 map lifecycle。Executor loopback、相机、cloud、
IMU、odom、scan、动态 TF、Nav2 action/lifecycle、ASR backend 状态和 motion gate 均 PASS。

之后的 readiness 补丁把 namespace 判断改成精确 fail-closed allowlist。只允许 provider
`go2_sensors`、runtime namespace `robonix/primitive/lidar`、transport `ros2` 下的
`robonix/primitive/camera/rgb` 和 `robonix/primitive/imu/imu`；任何字段变化或额外 mismatch
继续 FAIL。25/25 tests 和 diff-check PASS；把历史 Atlas caps 喂给新逻辑后，两条都记录为
`accepted_namespace_diagnostics`。但这是离线复核，不是重启后的 live strict；旧报告里的
namespace 不能继续写成确定失败，也不能提前说 live PASS。`semantic_navigation=INACTIVE`
仍是历史 payload 下导致 Atlas 整体 FAIL 的明确问题。

19:31 的 `stack-readiness-20260720T113143Z.json` 不是新补丁的 live 回归。它是在 10 组件
因 timestamp fault 停止后误跑，而且运行路径退回 root，结果 1 PASS、6 FAIL、33 UNKNOWN，
大量探针只是因为端口和 ROS 数据已经不存在。它的 SHA-256 是 `e198b5d4…d8e2`，只作为
无效运行态报告保留；最后有效 live strict 仍是 `111533Z`。

现在 ASR 不再只有 backend 自报 ready：19:13 已有真实麦克风精确 transcript 和 TTS 听音
证据。但这仍不会自动让 `semantic_navigation` provider active，也不会消除地图和 Atlas 门。
同样，Executor VERIFIED 只表示只读 active-plan 查询成立。最终
camera commit-bound 60 秒窗口最大 source lag 1.4634 秒、121/121 healthy；较早 build 另有
2.8185 秒峰值、63/121 unhealthy、4 次
vendor API error 和 3 次 source reject，仍作为长期 freshness/vendor 质量风险继续观察，
不反向放宽 strict 门。

### 6.5 地图、定位和实体导航仍是后续验收项

现在的地图是会话内实时产生的临时地图，也就是 ephemeral map；还没有在另行获得运动
批准后，用人工遥控完成真实场地建图、保存和重载，也没有验证机器人重启或重新进入场地
后的重定位稳定性。自动售货机前方的安全 Pose 还没有在实地图中人工确认。

因此文本或语音还没有转换成实体 Nav2 目标；取消、停车、网络断开、导航失败和人工接管
也没有完成真实运动条件下的闭环验收。任何物理运动都还没有做。

### 6.6 外参和相机标定还需现场测量

当前 URDF/TF 足够支持静止无运动集成，但移动 footprint、`base_link` 到 MID-360、
前置相机和 IMU 的精确外参仍需现场复核；前置相机 CameraInfo 也未完成正式标定，而且
当前证据只有 RGB，不是深度相机能力。这些项目不会被写成已经标定完成。

### 6.7 IMU future timestamp fault 需要重新取证复验

19:29:53 当前时间戳纪律层检测到 `corrected_timestamp_in_future:mid360_imu`。fault JSON
同时写明 `canonical_odom_ready=false`、`motion_ready=false`，10 组件随后按 fail-closed
顺序退出，SHA-256 为
`5dcb579bd3391e51087f21372154aff96787e3ae77a7f11429dc7295d9456e27`。机器人没有运动。

进一步离线分析表明，普通回调延迟只会让消息变旧，不会制造 future。当前最可能的原因是
大约 30 秒短窗拟合后冻结的 affine 模型存在小幅斜率误差，又叠加主机 NTP 渐进校时；
模型运行约 2,205 秒后，只需约 3 ppm 的误差就会吃完初始余量。因为旧 fault 没有写 exact
corrected age，所以仍不能完全排除单帧 IMU 源时间小幅超前。我没有直接放宽 2 ms 门限，
也没有原地自动 relock；当前补丁除增加触发流、精确 corrected age、future limit 和全流
source/receipt/age 证据外，还只在 no-motion 配置增加 20 ms affine anchor guard、5 ms
corrected-age soft floor 和 5 ppm locked drift 门。hard-future 判定仍先执行。仿射专项 32/32、
时间戳层相关 36/36、approval 5/5 测试和 diff-check 均通过，并新增 3 ppm 模型误差外推
2,200 秒的确定性复现。随后把新配置离线喂给同一真实会话留存的 42,511 条样本，qualification
和 lock 通过，Sport/IMU/odom corrected-age 基线约为 20.04/19.50/19.65 ms，PointCloud2
约 93.73 ms；mode-0600 回执 SHA-256 是
`49359dc9e2a64b28208555bc9bace9d13fbcffd51c7bc02ff9d716e9e1d58ae9`。这证明初始余量和
离线回放成立，不证明补丁后 live endurance，更没有解决长期共同钟架构；长期仍优先让
NX/Go2 使用共同钟。

这条 fault 不能简单标成“门禁工作所以问题解决”。下一步要审计 IMU 源时间、冻结 affine
模型和 future-skew 边界，重新采 time/state 并生成新 approval 后，才可再次启动无运动栈；
随后才能运行 readiness allowlist 补丁的第一份有效 live strict。

## 7. 现有硬件、采购未到货和充电后恢复状态

硬件方面，现有 Go2 EDU、Orin NX、MID-360、前置相机、专用网线和原厂遥控器已经在场，
足以支撑本周完成的软件集成、真实传感器读取和无运动验证。

采购方面，我已经买了两件网络配套设备：一件是标称免驱、1300M 的 COMFAST CF-812AC
无线网卡，另一件是小米 AX3000E 千兆路由器。截至本次汇报，两件设备都还没有送达，
尚未配置、尚未验收，所以本周没有把它们算作已接入或已验收能力。

这两件设备是为后续网络方案准备的。目前有两种待实测的用途：第一种是让机器人侧的 NX
通过无线网卡接入校园网，再完成命令行网络认证；第二种是由 AX3000E 组成机器人和电脑的
独立内网，再让机器人按受控方式通过电脑代理访问外网。两种方案都还只是设计选项，到货前
不能确认无线网卡在 NX/Ubuntu 上的实际兼容性，也没有验证校园网认证、路由、DDS 组播
隔离或代理稳定性。

这些采购件未到货没有阻塞本周基于现有有线链路的软件开发和无运动验证，但网络接入方案的
正式验收要顺延。到货后我会按“核对实物型号、驱动和设备识别、局域网连通、校园网认证或
代理、DDS 不串网、断网回退和长时间稳定性”的顺序验证。任何网络配置修改都会另行说明并
获得批准，认证密码和校园网凭据也不会写进仓库。

用户先现场确认机器人充满电、网线与电脑连接、网口黄灯亮；在此基础上，我又程序化完成了
carrier/IP、正式 PCAP、DDS writer、当次时间资格和 30 秒静止状态取证。旧状态和旧 approval
都曾被门禁明确拒绝，重新取证后才启动第二轮无运动栈。恢复过程保留原工作树，没有复用
过期批准，也没有因为电量恢复就自动开放任何运动权限。

较早 watermark build 的保存会话也重复了同一原则：第一份新 time 证据因
`mid360_cloud source_receipt_delta_discontinuity` 被拒签，retry 时间资格、新 state 和
approval 通过后才重新启动 MOTION DISABLED 栈。这不是额外失败，而是资格门真实生效。
该 approval 到期后栈也按设计退出。最终 `f74afaa` 复验又重新取得一组 time/state/approval，
完成 60 秒 monitor、strict 和 `30`～`33` 号截图后，于 14:17 主动安全关闭；该轮关闭后没有残留
stack/UI/camera bridge 进程，也没有因为重复验证而开放运动权限。

后续 Executor/Audio 复核也没有复用下午的旧批准。我重新采了 60 秒、43,680 个时间样本和
30 秒静止状态；primary 8,890、fallback 598 个状态样本均为 mode 0、gait 0、error 100，
再生成 post-commit 无运动 approval。该会话取得 `34`～`39` 号截图和 18:25 strict，全程
运动锁定。Liaison 2 文件修改随后先整理为干净本地 commit `e5f5c53f`；为避免用旧进程冒充
新补丁，我再次采集 60.22 秒/43,710 个 time 样本、primary 8,885 和 fallback 600 个静止
state 样本，并生成新的无运动 approval。新 10 组件栈实际使用 commit 对应 binary，完成
`40`～`56` 号 UI/语音/监听截图和 19:15 strict，全程仍是 MOTION DISABLED。19:29:53
IMU future timestamp fault 触发后，该栈已按设计关闭；当前不能说仍在线。
停栈后没有重新接触机器人进程；完成的是上述时间戳离线测试/回放、readiness 精确规则以及
Hands-free 取消补丁和只读验收器。下一次必须重新取得当次 time/state/approval，再启动新的
MOTION DISABLED 栈，旧批准不能复用。

## 8. 当前能力边界，用最通俗的话怎么回答

- **传感器接入：**真实相机、点云、IMU、lidar odom 和本体状态已经接入并展示；相机
  final-commit strict 快照 PASS，60 秒绑定窗口最大 1.4634 秒、121/121 healthy。较早窗口
  仍留有 2.8185 秒峰值和 vendor 异常，长期 freshness、标定和部分精确外参还待补。
- **建图：**实时地图已经能生成并稳定刷新；真实场地图还没有保存、重载和复用验收。
- **定位：**统一 `/odom` 和静止 TF 链已经跑通；保存地图后的重定位与运动中稳定性未验收。
- **导航：**Nav2 服务已经 ACTIVE，规划软件处于待命状态；没有给实体 Go2 发送目标或
  速度，不能称为完成真实导航。
- **语音：**受控 push-to-talk 已精确识别“请带我去自动售货机”，完成安全拒绝、用户可闻
  TTS 和默认无 WAV 落盘；Pilot/Executor/semantic-navigation 调用增量与 active plans 均为 0。
  Hands-free listener-only 启停已通过，但没有 wake；late ASR final 竞态已离线修复、尚未
  live；关闭过渡窗和关闭后 25 秒的独立只读 observer 已通过 26/26 离线测试、尚无 live 回执。
  唤醒词/任务、voiceprint/access gate、semantic provider active、语音到安全 Pose、Nav2 任务和
  到达停止仍未完成。
- **3D 展示：**Dashboard 已观测 PointCloud2 1,035 点，但 Scene 3D 仍显示 objects 0/points 0；
  只能说底层点云接入，不能说 Scene 3D 点云与对象层已完成。
- **行走：**本周没有做任何物理运动。原厂能走不等于 Robonix 的地图、定位、控制权和
  停止链已经验收，所以不能跳过这些门直接演示。

## 9. GitHub 修改、提交和 PR 状态

[这一节现场不必逐个念哈希，建议打开 GitHub 页面，用下面第一段概括；老师追问时再展开。]

本周的修改跨了多个仓库。目前可以准确概括为：Go2 本体有 1 个 Draft 总 PR；Robonix
provider、Scene、Catalog、Mapping 和 Navigation 共 5 个基线 PR 已合并；Robonix 首页
Go2 条目还有 1 个 PR 开放；今天又发布了 Scene Draft PR #162，camera 则形成独立本地
commit `f74afaa`，Executor 形成独立本地 commit `aa78ba45`。Scene #162 两项 CI 均通过但
未合并；Liaison opt-in 修复形成独立本地 commit `e5f5c53f`；camera、Executor 和 Liaison
均未 push、无 PR。除此之外还有
一批实机联调后的未提交工作，不能说已经进入旧 PR。

- [robot-unitree-go2 Draft PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)：
  开放的 Draft 本体总集成和评审入口，远端当前 HEAD 为完整 `3cd5d719`；
  `offline-safety` CI run 17 为 success。本地 HEAD 为 `f74afaa`、领先远端 1 commit；
  camera latest-frame/error-watermark 的 8 文件补丁已独立本地提交，但未 push、无 PR，
  不属于该次远端 CI。Dashboard、时间层、observer、readiness 等其他修改仍在工作树。
- [Robonix provider PR #151（已合并）](https://github.com/syswonder/robonix/pull/151)：
  provider loopback 基线，merge commit `6fd5225e`。
- Robonix Executor：本地 commit
  `aa78ba45b0a98e636c5d5c24948f81932a1bc686`，7 files、+146/-9；只读 active-plan 合同，
  fmt、clippy、34/34 tests PASS；未 push、无 PR。
- Robonix Liaison：本地 commit
  `e5f5c53f26879324fcb9f578e53f9d1789c0ebaa`，README 与 `src/voice.rs` 共 2 files、+24/-6；
  默认 WAV 保存由 `/tmp` 回退改为显式 opt-in，fmt、clippy、15/15 tests、build 和新栈
  默认无 WAV 实测 PASS；其上另有 Hands-free `README.md`/`src/handsfree.rs` 2 文件
  未提交补丁、+123/-2，fmt、clippy、17/17 tests PASS，但旧 binary 不含该修复、尚无 live
  wake-cancel 回执；均未 push、无 PR。
- [Scene PR #153（已合并）](https://github.com/syswonder/robonix/pull/153)：
  Scene ROS 合同、QoS 和模型缓存基线，merge commit `ed5486c2`。
- [Scene Draft PR #162（开放）](https://github.com/syswonder/robonix/pull/162)：
  `fix(scene): keep workstation live views responsive`，base `dev@b48f921c`，head
  `Origamii520:agent/go2-scene-live-views@696938bf`，5 commits、11 files、+822/-62；Docs
  Check run `29720084954` 和 CI run `29720084977` 均 completed/success。`6e501701` 只是
  原始本地来源提交，已挑选/重放到新分支；无关 Soma `71d02bfd` 被排除。PR 尚未合并。
- [Catalog PR #6（已合并）](https://github.com/syswonder/robonix-package-catalog/pull/6)：
  Go2 Catalog 条目，merge commit `450b8c7d`。
- [Mapping PR #9（已合并）](https://github.com/syswonder/service-map-rbnx/pull/9)：
  已合并基线，merge commit `76d201cb`；本地后续 `ac0c136`、`55373e6`
  尚无新 PR。
- [Navigation PR #6（已合并）](https://github.com/syswonder/service-navigation-rbnx/pull/6)：
  已合并基线，merge commit `46fb8c05`；本地后续 `0abdd14`、`6477e68`、`087573c`、
  `8b126e1`、`b4402e3`
  尚无新 PR。
- [Robonix 首页 PR #152（开放、非 Draft）](https://github.com/syswonder/robonix/pull/152)：
  首页增加 Unitree Go2，当前 HEAD `210e9236`，目前不能说已合并。
- Robonix 本地 build 仍保留既有集成提交；Scene 对外评审以 Draft PR #162 的 5 个远端
  commits 和 HEAD `696938bf` 为准，不能把原始本地 `6e501701` 当成远端 HEAD。

[以上远端状态于 2026-07-20 19:30 CST 汇总，远端状态沿用本轮 18:35 实时复核：Go2 Draft PR #1 open/mergeable，远端
HEAD `3cd5d719...` 且 `offline-safety` success；Scene Draft PR #162 open/mergeable，HEAD
`696938bf...` 且 CI/Docs Check success；Mapping #9、Navigation #6、Catalog #6、Robonix
#151/#153 已合并；Robonix #152 仍 open、非 Draft。]

这样说明的重点是：本周确实完成了较多跨仓库工作，也已经有多项上游基线合并；同时我会
严格区分“已合并”“开放”“Draft”“仅本地 commit”和“工作树未提交”，不把本地结果
包装成已经进入 GitHub。

## 10. 下一阶段计划和验收顺序

第一步已经完成：充电后从原暂停点恢复，程序化确认 carrier/IP、DDS writer、时间模型和
静止状态，重新生成 identity、time、state 和仅无运动 approval；旧证据被正确拒绝。

第二步也已推进：time/observer 安全硬门、Dashboard 图像性能、Scene 响应/竞态、camera
latest-frame/error-watermark 和 readiness 误判已分别修复并通过离线测试；完整
`validate_offline.sh` 顶层 332 项通过。Scene 已有 Draft PR #162 且两项 CI success；
camera 有独立本地 commit `f74afaa`，但未 push、无 PR。最终 binary 已构建，commit/hash
绑定的 60 秒无运动复验已通过；Executor 只读合同已形成 `aa78ba45`，未 push、无 PR；
Liaison 默认不落盘修复已形成 `e5f5c53f`，并完成新栈/真实语音/无 WAV 复验，未 push、
无 PR。readiness exact namespace allowlist 已通过 25 tests/diff-check 和历史 caps 离线验证，
但没有 post-patch live strict。Go2 其余修改仍在工作树。

第三步已经完成无运动 UI 复验：补丁后的带 cloud observer 全栈 READY，60 秒 Dashboard
指标、600 秒 raw observer、600 秒 HTTP monitor、相机三轮 60 秒监测和
`14`～`56` 号截图/结构化证据都已保存。第二段视频仍需确认落盘。较早 watermark build
窗口最大 2.8185 秒；最终 `f74afaa` 绑定窗口最大 1.4634 秒、零 unhealthy，commit/binary
证据缺口已补齐。Atlas 契约以及地图/landmark/mapping mode 未通过项继续保留，不能用
UI PASS 或 camera PASS 覆盖。最新会话在完成语音与 listener 测试后，又因 IMU future
timestamp fault 自动 fail-closed；当前栈已停止。

第四步，人工遥控建图本身也是实体运动，必须另行获得“人工遥控建图”的具体批准，并先
确认场地清空、原厂遥控器/紧急停止方式和人工接管。得到这项批准后，才人工遥控完成真实
场地建图、保存和重载地图、验证重定位，并在地图中确认“自动售货机”前方的安全 Pose；
没有这项批准时只准备脚本和验收表，不驱动机器人。

第五步，受控 push-to-talk 的中文 ASR、安全拒绝、用户可闻 TTS 和默认无 WAV 落盘已经
通过，Hands-free listener-only 启停也通过。关闭后的 late ASR final 竞态已完成离线修复；
下一步在保持运动关闭时用新 binary 做 wake/关闭实测，再补 voiceprint/access gate、唤醒词、
噪声、重复和失败恢复测试，并修复
`semantic_navigation` lifecycle；其后才打通已人工确认的语义 Pose、
Nav2 任务预览，以及取消、停止、失联和导航失败链路。

第六步，camera `f74afaa` 的重编和 commit-bound 无运动复验已经完成，Executor
`aa78ba45` 和 Liaison `e5f5c53f` 也已通过各自离线/只读现场验证；下一步整理证据后按真实
边界 push/建 PR。同时跟进 Scene Draft PR
#162 的评审与合并，并为 Mapping、Navigation 后续补丁分别创建准确的新 PR，不把它们
归到旧的已合并 PR。
最终无运动与后续分阶段验收完成后，还要第二次同步本讲稿、飞书周报、README、
截图/录屏索引以及最终 commit/PR 状态。

在这些步骤之前，当前最先做的是定位 `corrected_timestamp_in_future:mid360_imu`，重新采
time/state 并生成新 approval，再启动 MOTION DISABLED 栈。启动后第一份 live strict 要验证
exact allowlist 的真实效果；停栈后的 `113143Z` 不能当作这次验证。

最后，只有地图、重定位、TF、传感器、唯一控制器、命令超时、停止链、遥控接管和现场条件
全部通过，并再次得到 Robonix 控制链测试的单独、具体批准后，才做第一次由 Robonix 输出
控制的 `0.05 m/s`、`2 s`、约 `10 cm` 低速直线测试。通过停车和接管证据后，再考虑
下一阶段，不能直接跳到语音自主导航。

## 11. 现场展示和录屏顺序

当前页面证据已经保存，但 19:29 timestamp fault 后 10 组件已经停止，现在只能展示已保存的
`40`～`56` 号截图，不能声称仍是实时页面。下一次完成新 time/state/approval 且 MOTION
DISABLED 页面 ready 时，应立即提醒“现在开始录屏”。建议按下面顺序录制：

1. 先拍实体 Go2、NX、MID-360、网线和手持遥控器，说明机器人保持静止；
2. 打开官方 Robonix Client Chat，展示 `EXECUTOR VERIFIED`、0 plans 和 Hands-free off；
3. 打开 Client Audio 与 Audio debug，展示 Server online、route、麦克风结果和实时 VU；
4. 展示 Scene `/user`、`/2d`、`/3d` 和 `/cam`，说明 RGB 有数据但无 depth，3D 仍为
   objects 0/points 0；
5. 展示 Mapping 的实时地图和 robot pose，同时指出 mapping mode/no saved maps；
6. 展示 Dashboard 的相机、雷达、地图、odom、TF、静止速度和 navigation idle；
7. 展示最终 `请带我去自动售货机` transcript、TTS `PLAYED 681216 BYTES`，并口头确认听到；
8. 展示安全拒绝、Pilot calls 0、active plans 0，说明没有导航/运动调用；
9. 展示 `55`/`56` 的 Hands-free listening→off，并说明无 wake、只验监听启停；
10. 展示 fault JSON 与最后有效 `111533Z`，说明 `113143Z` 是停栈误跑；
11. 最后展示 GitHub PR 页面，说明已合并、开放、Draft、本地 commit 和未提交补丁边界。

[第一段录屏已由用户完成，但视频尚未复制进仓库，所以不登记文件名或 SHA。若下一次
MOTION DISABLED 页面 ready，应立即提醒“现在开始第二段录屏”。录完后保存视频实际时间和 SHA；当前
`14`～`56` 号补丁后截图/结构化证据已保存并完成 SHA-256。`22`/`30`/`48` 分别是历史
Audio offline、Executor unavailable 和 ffmpeg missing 负面证据；当前 `51` 是完整 ASR/TTS
结果。Scene 无 depth/无 3D points/objects、Mapping no saved maps、Dashboard navigation
idle 仍要原样录下。开始下一轮完整实时 UI 或语音验收时，应
立即提醒“现在开始录屏”。]

## 12. 结尾总结

最后总结一下：这周我从零完成的不是一个孤立功能，而是一套跨硬件、数据、时间、坐标、
建图导航、官方 UI、文本语义、受控语音、安全和上游协作的较完整无运动接入底座。现在已经有真实 Go2
的两轮无运动全栈、统一 odom/TF、临时实时地图、Nav2 待命、官方 UI、正式 PCAP、
30 分钟既有稳定性证据和今天两轮 10 分钟 cloud-inclusive/UI 证据，也有多项上游 PR 合并。
Dashboard/Scene 性能问题已经从“页面偶尔看着不对”推进到有根因、有补丁、有测试、有
60 秒真机改善数字；第二轮 raw observer 和 HTTP monitor 均已完成。相机队列已通过
单槽 latest-frame 和 error watermark 设计收口，保持原 stamp 和 2 秒门限，并形成独立
本地 commit `f74afaa`。较早 build 的 60 秒窗口最大 2.8185 秒、63/121 unhealthy；我随后
重编最终内容并完成精确 commit-bound 复验，最终窗口最大 1.4634 秒、121/121 healthy，
binary SHA 前后不变。这个结果能证明最终 commit 在本次 60 秒无运动窗口通过，但不会把
一次干净窗口说成未来任意时长都满足 2 秒门。

语音链也已经从“设备能采集”推进到真实结果：Liaison `e5f5c53f` 默认不落盘新行为在新栈
生效，6.10 秒输入精确识别“请带我去自动售货机”，安全门拒绝执行，TTS 约 21.288 秒并由
用户确认听到；调用日志和 active plans 保持零，没有运动。这是受控无运动语音闭环，不是
语音自主导航。Hands-free listener-only 的 30 秒启停也通过，但无 wake，不能冒充唤醒词或
任务闭环。取消竞态补丁和独立 observer 已离线完成；observer 把关闭轮询过渡窗也按
fail-closed 处理，并要求关闭后至少 25 秒无 late event/任务，26/26 tests PASS，但尚无 live 回执。

readiness namespace 误判已用两个精确 tuple 做代码/离线收口，25 tests 通过；它还没有
post-patch live strict。19:29 的 IMU future timestamp fault 随后按设计关闭全栈，最后有效
live strict 仍是 `111533Z`。离线已完成 20 ms/5 ms/5 ppm no-motion 加固、32+36+5 项测试和
42,511 条留存样本 replay；它说明根因和缓解方向已收敛，但还不是 post-patch live endurance。

没有完成的部分我也划得很清楚：camera `f74afaa` 的 push/PR、Go2 其余本地补丁的 commit/
push、Scene Draft PR #162 的评审/合并、第二段视频归档、vendor 相机 error/source reject、
>2 秒峰值、Atlas 契约、Executor `aa78ba45` 和 Liaison `e5f5c53f` 的 push/PR、Scene 3D
points/objects、semantic provider lifecycle、readiness allowlist live 复验、IMU future
timestamp 根因、voiceprint/access gate、Hands-free 取消补丁 live/唤醒和长期语音、
采购件到货后的对应验收、真实地图保存和重定位、自动售货机安全 Pose、语音实体任务、
停止/失联以及任何物理运动。
下一步会继续按安全门逐层推进，每通过一层就保存对应证据，不会用“组件能启动”替代
“真实闭环已验收”。

## 13. 老师可能追问的问题

### Q1：这周的工作量主要体现在哪里？

不是某一个驱动文件，而是同时跨了九块：本体框架、真实硬件、DDS 数据身份、时间修正、
odom/TF、Mapping/Nav2、官方 UI、文本语义与受控语音、安全测试和证据归档；同时还需要在 Robonix、
Catalog、Mapping、Navigation、Scene 等多个上游仓库提交兼容修改。现在已有 5 个基线
PR 合并，但后续本地补丁仍按真实状态标记，没有把工作量靠夸大完成度来体现。

### Q2：现在能走了吗？

导航软件链已经进入第一次分阶段测试前的准备阶段，但地图重定位、安全 Pose、停止/失联、
唯一控制器和当次运动许可都还没有全部通过，尚未达到运动门条件，所以现在不运动。

### Q3：Nav2 已经 ACTIVE，为什么还不能说导航完成？

ACTIVE 只说明导航服务开机待命。要称为实体导航完成，还必须有可复用地图、稳定定位、
合法目标、真实规划与执行、动态过程中的 TF、取消和异常停车证据。

### Q4：建图、定位、导航分别到什么程度？

实时建图已经能运行；统一 odom 和静止 TF 已跑通；Nav2 服务已 ACTIVE。尚未完成的是地图
保存/重载、重定位、自动售货机安全 Pose，以及任何真实路径执行。

### Q5：为什么会话会自动退出？

点云 callback 出现 223.94 ms 的 source/receipt delta error，超过 150 ms 门限，安全
守卫主动关闭了全栈。这证明失败关闭有效，但具体调度根因仍在验证，不能简单归因于抓包。
今天第一轮独立 observer 也记录到两次 295.36/355.56 ms cloud gap，而 odom/sport 无同级
gap，说明问题更像大点云回调/调度路径的孤立抖动；第二轮 observer 虽已结束但仍测到
cloud gap，暂不做最终归因。

最新一次是 19:29:53 的 `corrected_timestamp_in_future:mid360_imu`，同样触发 fail-closed，
10 组件停止且 motion readiness false。这不是运动结果；需要重新检查 IMU 源时间和 affine
边界，再重取证启动。19:31 停栈后误跑的 strict 不代表新的 live 回归。

### Q6：采购件没有到，会不会把项目完全卡住？

不会。现有专用网线已经足够完成本周的软件开发和无运动验证。未到货的是 COMFAST
CF-812AC 无线网卡和小米 AX3000E 路由器，它们主要用于后续校园网命令行认证，或者建立
机器人—电脑独立内网并由电脑代理上网。依赖它们的网络方案验收会顺延，到货前不会把对应
能力写成已完成。

### Q7：现在可以展示什么？

最终无运动栈在各自 approval 有效窗内完成过展示；19:29 fault 后当前栈已停止。现在可以
展示官方 Client、Audio debug、Scene、Mapping、Dashboard 和语义任务的 `14`～`56` 号截图
及结构化证据。`51` 展示完整
九字 transcript 和 681,216 B TTS，`52` 展示语音后的静止 Dashboard，`53`/`54` 展示最新
2D/3D Scene，`55`/`56` 展示 Hands-free listening→off。若需继续实时演示，必须重新取
time/state/approval 并启动 MOTION DISABLED 栈。
第二轮 raw observer、HTTP monitor 和
相机三轮 60 秒 monitor 都已结束。最新截图明确显示 Client Connected 但 Executor
verified/0 plans、Audio Server online 且麦克风采集通过、Dashboard 为真实
画面/雷达/实时图且主要数据 fresh、
Scene RGB 约 1.2 秒但无 depth、Mapping 仍为 mapping 模式且没有 saved map。展示时还要
说明最后有效 live strict 仍有 4 FAIL、1 UNKNOWN；受控 ASR/TTS 通过不代表 voiceprint/access gate、
semantic provider 或实体导航通过，camera 快照 PASS 也不代表未来任意 60 秒都持续低于 2 秒。

### Q8：什么时候才算完成第一阶段？

完成最后补丁回归、无运动复验、地图保存和重定位、安全 Pose、取消/停止/失联，并在单独
批准下完成厘米级低速运动和停车证据后，才算进入语音实体导航闭环的下一阶段。

### Q9：Hands-free 现在算完成了吗？

还不算。完成的是 30 秒 listener-only 启动和显式关闭：无 wake/error，关闭后 5 秒没有新
麦克风请求。ASR final 返回前关闭的代码竞态随后已离线修复并通过 17/17 tests/clippy；独立
observer 又把关闭轮询过渡窗、关闭后至少 25 秒、late event、transcript 和 active plan 都纳入
fail-closed 条件，且陈旧 enabled 响应不能清除已观察到的过渡窗事件或重置 transcript 基线，
26/26 离线测试通过。但还没有用新 binary 验证真实唤醒词、wake 后 ASR/任务
与现场关闭，也没有 live acceptance JSON；access gate 和 voiceprint 仍未启用。
