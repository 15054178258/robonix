# Go2 遥测与 Liaison 语音面板

这是 Robonix Unitree Go2 部署内的遥测 Service。Provider 固定为
`Service(id="go2_dashboard", namespace="robonix/service/telemetry/dashboard")`，
由生命周期回调拥有一个 Web 面板子进程。面板把 ROS 2 数据转换为有界的浏览器预览，单页显示：

- `/camera/color/image_raw` Go2 内置相机图像；
- D435i 彩色、对齐深度伪彩预览和 `CameraInfo`，三路缓存彼此独立；
- `/scanner/scan` 或 `/scanner/cloud` 雷达俯视图；
- `/map` 占据栅格；
- `/robonix/map/pose` 中经过 Mapping TF 适配的机器人位置和朝向；
- `/odom` 位置与速度；
- `/navigate_to_pose/_action/status` Nav2 动作状态；
- 由语义导航组件注入的任务显示状态。

本包向 Atlas 注册 lifecycle driver 和只读 status 两条 capability，不提供机器人运动命令接口。默认情况下 Web 子进程只创建 ROS 订阅；显式配置 `initial_pose_maps_dir` 后，仅额外创建 `/initialpose` 定位种子发布器。它不创建 Nav2 goal 或 chassis publisher。语义任务接口只更新该进程内的 UI 状态，不调用导航服务。

另有一个**默认关闭**的浏览器 push-to-talk 入口。显式启用后，它只接受
回环同源页面上传的有界 16 kHz 单声道 PCM WAV，通过本机
`audio_client_bridge` 交给官方 `robonix/system/liaison/voice` contract。
Dashboard 不自行做 ASR，不调用 navigation/ROS/Unitree，也不创建 publisher。
Liaison/Pilot 之后是否接受任务，仍由 Robonix 访问策略、语义解析以及 Go2
独立运动安全门决定；按钮本身不是运动授权。

当根启动器判定 `GO2_ALLOW_MOTION=false` 时，它会强制语义路由器和面板使用
`SEMANTIC_INTENT_EXECUTION_MODE=preview`。此时浏览器仍走真实的
Speech → Liaison → Pilot 链路并显示 ASR 文本；语义路由器只返回空 RTDL tree，
面板从 Pilot `TaskState` 显示“自动售货机 · 已解析（未执行）”及缺失物理验证
Pose、map/localization/Nav2 未就绪等阻塞原因。该路径不会把 navigation、Nav2
或 Unitree capability leaf 交给 Executor。若 preview 会话观察到任何
call-bearing Pilot plan，gateway 会把会话标为失败并显示安全策略违规。

## Robonix 生命周期

`rbnx boot` 中的部署名称必须是 `go2_dashboard`，与 provider id 完全一致：

```yaml
service:
  - name: go2_dashboard
    path: packages/go2_dashboard
    config:
      host: 127.0.0.1
      port: 8092
      camera_topic: /camera/color/image_raw
      d435i_color_topic: /go2/d435i/color/image_raw
      d435i_depth_topic: /go2/d435i/aligned_depth_to_color/image_raw
      d435i_camera_info_topic: /go2/d435i/color/camera_info
      scan_topic: /scanner/scan
      cloud_topic: /scanner/cloud
      map_topic: /map
      pose_topic: /robonix/map/pose
      odom_topic: /odom
      nav_status_topic: /navigate_to_pose/_action/status
      initial_pose_topic: /initialpose
      map_lifecycle_topic: /robonix/map/lifecycle
      initial_pose_maps_dir: /absolute/deployment/rbnx-build/data/maps
      initial_pose_auto_restore: false
      map_frame: map
      base_frame: base_link
```

Provider 启动后先向 Atlas 注册。`Driver(CMD_INIT)` 会严格验证 config，并启动唯一的直接子进程；`CMD_ACTIVATE` 验证子进程仍在线。`CMD_DEACTIVATE`、`CMD_SHUTDOWN` 或 provider 收到终止信号时，只通过保存的 `Popen` 对象终止自身创建的子进程，不按进程名杀死其它程序。

注册能力：

- `robonix/service/telemetry/dashboard/driver`：包内 lifecycle contract；
- `robonix/service/telemetry/dashboard/status`：包内 `GetDashboardStatus.srv`，同时提供 MCP 与 unary gRPC 只读查询，返回 `ok`、`url`、`detail` 和有界 `status_json`。

## 运行要求

- Ubuntu 22.04、ROS 2 Humble；
- `rclpy`、`sensor_msgs`、`geometry_msgs`、`nav_msgs` 和 `action_msgs` 来自已安装的 ROS 环境，不从 PyPI 安装；
- Python 3.10 及 `python3-venv`；
- 可用的 `rbnx` CLI 以及 Robonix `robonix_api`；
- FastAPI、Uvicorn、Pillow 以及 `robonix_api` 所需的 gRPC/MCP 运行依赖由 `requirements.txt` 安装到包内 `rbnx-build/venv`；`robonix_api` 本体从 `ROBONIX_SOURCE_PATH/pylib/robonix-api` 加入 Python 路径。

构建命令会先执行不依赖 ROS 的解析与安全测试，再执行 `rbnx codegen -p <包目录> --mcp`，最后创建虚拟环境并验证 provider 与生成类型可导入：

```bash
bash scripts/build.sh
```

若依赖已由离线镜像预装，可设置 `GO2_DASHBOARD_SKIP_PIP=1`，构建仍会验证这些模块可导入。

正常部署由 `rbnx boot` 调用 build/start/lifecycle。单独调试 provider 时，Atlas 必须已经运行；启动脚本只启动 provider，Web 子进程要等 lifecycle INIT：

```bash
bash scripts/start.sh
```

默认 config 下，INIT 成功后访问 `http://127.0.0.1:8092/`。停止 provider：

```bash
bash scripts/stop.sh
```

停止脚本只向经过命令行校验的 provider PID 发送终止信号；子进程由 provider 的 `on_shutdown` 回收。面板自身不实现登录，因此 Provider 配置和直接命令行入口都只接受 `host: 127.0.0.1`。远程访问必须使用经过认证的 SSH 隧道，不得把面板直接暴露到局域网。

## 配置

以下字段来自 `robonix_manifest.yaml` 中该 service 的 `config:`。未知字段会使 INIT 失败，避免拼写错误被静默忽略：

| config 字段 | 默认值 | 作用 |
| --- | --- | --- |
| `camera_topic` | `/camera/color/image_raw` | 彩色图像 |
| `d435i_color_topic` | `/go2/d435i/color/image_raw` | D435i 彩色预览 |
| `d435i_depth_topic` | `/go2/d435i/aligned_depth_to_color/image_raw` | D435i 对齐深度伪彩预览 |
| `d435i_camera_info_topic` | `/go2/d435i/color/camera_info` | D435i 标定摘要 |
| `scan_topic` | `/scanner/scan` | 二维扫描 |
| `cloud_topic` | `/scanner/cloud` | 三维点云 |
| `map_topic` | `/map` | 占据栅格 |
| `pose_topic` | `/robonix/map/pose` | Mapping 输出的 `map -> base_link` 位姿 |
| `odom_topic` | `/odom` | 里程计 |
| `nav_status_topic` | `/navigate_to_pose/_action/status` | Nav2 状态 |
| `initial_pose_topic` | `/initialpose` | 操作员定位种子输入与显式恢复输出 |
| `map_lifecycle_topic` | `/robonix/map/lifecycle` | 具名地图的 `map_id/mode/generation` |
| `initial_pose_maps_dir` | 空 | 显式启用地图代际绑定的 sidecar 保存目录；必须是绝对路径 |
| `initial_pose_auto_restore` | `false` | 严格 `0`/`1`；实体 profile 保持关闭，避免机器人未回到标记点时误恢复 |
| `map_frame` | `map` | 全局地图坐标系 |
| `base_frame` | `base_link` | 机器人基座坐标系 |
| `host` | `127.0.0.1` | 固定的回环 HTTP 监听地址；其他值会被拒绝 |
| `port` | `8092` | HTTP 端口 |
| `public_url` | 空 | status capability 对外报告的 HTTP(S) URL |
| `log_level` | `info` | Web 日志等级 |
| `startup_timeout_s` | `8.0` | 等待本地 health endpoint 的秒数 |
| `stop_timeout_s` | `5.0` | 优雅停止子进程的秒数 |
| `browser_voice_enabled` | `false` | 必须是 `0`/`1`；父 provider 显式传给 Web child |
| `liaison_endpoint` | `127.0.0.1:50081` | 固定字面量回环 Liaison endpoint |
| `audio_bridge_url` | `ws://127.0.0.1:60002/client` | 固定字面量回环 reverse-audio URL |
| `browser_mic_provider` | `audio_client_bridge` | Liaison voice request 显式 pin 的 mic provider |

为兼容现有 Go2 部署清单，`image_topic` 可作为 `camera_topic` 的别名；若两者同时设置但值不同，INIT 会失败。

启动脚本另外读取两个本机环境变量：

| 环境变量 | 默认值 | 作用 |
| --- | --- | --- |
| `ROS_SETUP_FILE` | `/opt/ros/humble/setup.bash` | ROS 环境脚本 |
| `GO2_ROS_OVERLAY_SETUP` | 未设置 | 可选工作区 overlay 环境脚本 |

相机和雷达预览在观察线程内限频到 5 Hz，点云预览最多 1200 点。地图使用缓存 PNG；状态响应不携带二进制图像。页面会把每个输入标记为 `fresh`、`stale`、`missing` 或 `error`，并显示最近帧年龄、`frame_id` 和解析错误。地图位姿的有效年龄同时包含收件年龄和 ROS header 源时间偏差，并采用只增不减的保守上界；旧帧或异常未来帧不会随本机时钟追上而从 stale 变回 fresh。页面仅在 ROS 观察线程在线且位姿为 fresh 时显示地图位姿和机器人箭头。ROS 断开或地图 topic 报错后仍可查看最后一张有效地图，但 UI 会显式标为“缓存快照”，不会把它呈现为当前 Mapping 正常。

## HTTP 接口

| 方法与路径 | 内容 |
| --- | --- |
| `GET /` | 单页态势面板 |
| `GET /healthz` | HTTP 与 ROS 观察线程健康摘要 |
| `GET /api/status` | 有界 JSON 状态快照 |
| `GET /api/camera.jpg` | 最近相机预览 |
| `GET /api/cameras/{go2,d435i-color,d435i-depth}.jpg?sequence=<n>` | 三路独立、带快照序号竞争保护的 JPEG 预览 |
| `GET /api/map.png?sequence=<n>` | 与状态快照序号一致的地图预览；序号已变化时返回 `409`，省略参数则返回当前快照 |
| `GET /api/initial-pose` | 当前地图代际与已保存定位种子摘要 |
| `POST /api/initial-pose/restore` | 仅在请求与 live `map_id+generation` 完全一致且为 localization 时排队恢复 |
| `POST /api/initial-pose/reset` | 仅在精确确认当前地图代际后把 active sidecar 改名归档 |
| `GET /api/semantic-task` | 当前语义任务显示状态 |
| `POST /api/semantic-task` | 覆盖语义任务显示状态 |
| `GET /api/voice` | 浏览器语音开关、限额、内存状态和同源 nonce；关闭时不返回 nonce |
| `POST /api/voice` | 可选的同源 PCM WAV → Liaison handoff；默认返回 404 |

语义导航组件可以在自身状态变化后写入显示信息。例如，下面只会让 UI 显示“自动售货机”已解析，不会创建 ROS 实体或触发任务：

```bash
curl --fail-with-body \
  --request POST http://127.0.0.1:8092/api/semantic-task \
  --header 'Content-Type: application/json' \
  --data '{"task_id":"demo-1","target_name":"自动售货机","status":"resolved","message":"已匹配保存的地图 Pose","pose":{"frame_id":"map","x":1.2,"y":-0.4,"yaw":0.0}}'
```

允许的显示状态为 `idle`、`received`、`resolving`、`resolved`、`navigating`、`succeeded`、`canceled` 和 `failed`。接口没有“执行”字段，也不会把 Pose 转发到 ROS 或 Robonix navigation service。

## 可选浏览器语音入口

仓库 `.env.example` 明确把开关设为 `0`，root manifest 将它映射到 dashboard
service config，父 provider 复验后再以 `GO2_DASHBOARD_BROWSER_VOICE_ENABLED`
传给 Web child。唯一启用开关是：

```bash
export GO2_DASHBOARD_BROWSER_VOICE_ENABLED=1
```

未设置或设为 `0` 时，UI 按钮不可用，`POST /api/voice` 返回 404；其他拼写
会使 Web 子进程启动失败，避免意外启用。默认连接保持为
`127.0.0.1:50081`（Liaison）和
`ws://127.0.0.1:60002/client`（reverse audio bridge），目标必须是字面量
IPv4 回环地址，不能改为 NX、Go2 或 LAN 地址。可选的高级覆盖同样会被严格
验证：

| 环境变量 | 默认 | 硬限制 |
| --- | --- | --- |
| `GO2_DASHBOARD_LIAISON_ENDPOINT` | `127.0.0.1:50081` | 字面量回环 `host:port` |
| `GO2_DASHBOARD_AUDIO_BRIDGE_URL` | `ws://127.0.0.1:60002/client` | `ws`、字面量回环、固定 `/client`、无凭据/查询 |
| `GO2_DASHBOARD_BROWSER_MIC_PROVIDER` | `audio_client_bridge` | 有界 provider id；显式 pin，禁止 Atlas 静默回退 |
| `GO2_DASHBOARD_VOICE_MIN_SECONDS` | `0.25` | `0.1..2.0` 且小于最大值 |
| `GO2_DASHBOARD_VOICE_MAX_SECONDS` | `8.0` | `1.0..10.0` |
| `GO2_DASHBOARD_VOICE_MAX_UPLOAD_BYTES` | `300000` | `64000..400000`，同时受 PCM 时长限制 |
| `GO2_DASHBOARD_VOICE_BRIDGE_TIMEOUT_S` | `3.0` | `0.5..5.0` |
| `GO2_DASHBOARD_VOICE_MIC_TIMEOUT_S` | `6.0` | `1.0..10.0` |
| `GO2_DASHBOARD_VOICE_SESSION_TIMEOUT_S` | `45.0` | `10.0..60.0` |
| `SEMANTIC_INTENT_EXECUTION_MODE` | `preview` | 仅接受 `preview`/`live`；由根 `start.sh` 根据完整运动门强制设置 |

请求必须满足全部条件：TCP 客户端、`Host` 和 `Origin` 都是当前 dashboard
回环 origin；没有 `Forwarded`/`X-Forwarded-For`；`Sec-Fetch-Site`（若有）
为 `same-origin`；先读取的随机 nonce 必须用自定义 header 回传；必须有准确
`Content-Length`；MIME 为 `audio/wav` 或 `audio/x-wav`；WAV 必须是
16 kHz、单声道、16-bit little-endian PCM。一次只允许一个会话，所有网络
操作有截止时间。音频只存在内存，worker 结束时覆盖并清空；Dashboard 不写
录音文件。Liaison 自身若配置 `ROBONIX_LIAISON_VOICE_SAVE_DIR`，那是 Liaison
的独立策略，验收环境应保持未设置。

浏览器 gateway 会在该次会话中临时占用 reverse `audio_client_bridge`。若已有
另一个 `robonix-client` 音频会话，bridge 的单客户端语义会替换旧连接；因此
正式演示时只保留一个音频前端。为避免 Dashboard 接收 TTS 音频，voice request
固定 `tts_enabled=false`；页面只显示 Liaison 事件、识别文本、语义解析和 Nav2
状态。

## Atlas status 返回

只读 status capability 的 `ok` 仅在 Web 子进程、health endpoint 和 ROS 观察线程都在线时为 `true`。`status_json` 只包含只读标志、子进程状态、最近退出码、health 摘要和已验证的公开 config，不复制父进程环境或凭据。创建子进程时也会移除代理变量及名称包含 password、secret、token、credential、private key 或 API/access key 的环境项；ROS、DDS 和 Python 运行环境保留。查询不会启动、重启或停止任何进程。

## 离线验证

测试只解析内存中的伪造消息字节与源码，不启动 ROS、HTTP 监听或任何硬件连接：

```bash
python3 -m unittest discover -s tests -v
```

覆盖范围包括图像行步长和编码、LaserScan 边界、PointCloud2 字段/大小端、OccupancyGrid 朝向、四元数、数据新鲜度、语义状态校验、manifest/provider 身份、config 到子进程环境映射、只终止自身子进程，以及运行时代码的只读静态约束。
