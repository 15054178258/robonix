# 真实截图与录屏素材

本目录只保存真实 Go2/Robonix 运行证据。`03-full-stack-*`、`05-*` 和 `06-*`
来自最新无运动会话 `workstation-nomotion-stamp.5BBjlF`；采集时 Robonix 10/10
组件在线。它们证明真实感知、地图、Nav2 进程、中文语音后端和 UI 已接入，
不证明定位、导航到达或物理运动成功。

素材采集完成后，该会话因 `locked_offset_deviation_exceeded` 触发失败关闭；当前实时
Dashboard 已离线。下列文件是在关闭前 10/10 组件在线时保存的真实快照。

## 已存在素材

| 文件 | 真实内容 | 证据边界 |
| --- | --- | --- |
| `01-real-sensors-ui-nomap.png` | 第一批真实面板：相机、MID-360 点云和语音入口 | 当时地图/odom 未就绪；用于展示早期进度 |
| `02-camera-live.jpg` | 第一批真实 1920×1080 相机帧 | 只证明相机数据链 |
| `02-camera-live.headers` | 第一批相机接口 HTTP 头 | HTTP 200、`image/jpeg`、`no-store` |
| `03-full-stack-ui-nomotion.png` | 最新 1920×2160 实时面板：相机、二维/三维雷达、地图、语音入口和任务状态 | 页面同时显示 `/odom` 缺失、`map -> base_link` 错误和导航状态缺失；无运动 |
| `03-full-stack-status.json` | 最新面板 `/api/status` 原始响应 | 相机/雷达/地图 fresh；`telemetry_read_only=true`；odom/pose/nav 未就绪 |
| `04-text-semantic-preview.ndjson` | 真实文本事件流，识别“自动售货机” | 因 Pose/定位/运动门禁不足而拒绝，调用数为 0 |
| `05-camera-full-stack.jpg` | 最新全栈会话中的真实 1920×1080 相机帧 | 只证明最新相机链路 |
| `05-camera-full-stack.headers` | 最新相机接口 HTTP 头 | 实时接口响应，不是浏览器缓存 |
| `06-map-full-stack.png` | 最新 Mapping 发布的实时占据栅格图 | 尚无可信机器人地图位姿，不能证明定位 |
| `06-map-full-stack.headers` | 最新地图接口 HTTP 头 | 证明图片来自实时 Dashboard 接口 |

## SHA-256

```text
7f33abfc8e77717d24fbd471d1427a81cea252adf9f23f51b162ffec33af79a7  01-real-sensors-ui-nomap.png
14e71710f9ef834d1f76aed7bf48e4be1bb55a8b0c25410db92a1eed777b2906  02-camera-live.headers
21fc5132f49de87ea6bb2d38fb15a9b6dc896af6e1710de2fe8e60f649afeae0  02-camera-live.jpg
566c6157ca77e206920b8a227f55c3de990ac57e5ae303a507d64a04d164615b  03-full-stack-status.json
f3b035c87c16a50906f3c03eb318c6e826da76256ef13f1b987d8cca2bc6aae2  03-full-stack-ui-nomotion.png
cc2fa63e002ea6b097e7e4f9fc273c75aa59d58925449fce21351e655f1a8b86  04-text-semantic-preview.ndjson
53b8b8cd1679668981975bc649c68848e83f0fa9d7a7d06291950251b60461d1  05-camera-full-stack.headers
7ebf9e17572b98f3786528d466db7fc4574cd837b81fd5fbd8e5f560eebcadf1  05-camera-full-stack.jpg
07ca85572ddc922cd95f0d81a480c88c1e0e5926e48eb10162c0a30889cc711f  06-map-full-stack.headers
5bfeaaff90e3346e843ddb96a137022e5c30a0c34d8871276be8a9b1e6402b48  06-map-full-stack.png
```

## 当前可录与待补素材

| 建议文件名 | 内容 | 当前状态 |
| --- | --- | --- |
| `video-01-nomotion-ui-voice.mp4` | 实时相机、雷达、地图、状态和浏览器语音预览；机器人静止 | 可以录制；标题必须注明“无运动复验” |
| `05-voice-asr-preview.png` | 麦克风录音、中文识别文本和语义结果 | 待浏览器语音端到端复验 |
| `07-map-landmark.png` | 地图、机器人位置、自动售货机和安全 approach Pose | 待 odom/TF/重定位通过 |
| `08-first-motion-evidence.png` | 首次低速测试的速度、距离、停止原因和许可 | 待全部运动门禁通过 |
| `09-final-robonix-ui.png` | 相机、雷达、地图、机器人位置和任务状态同屏 | 待最终闭环通过 |
| `video-02-navigation-acceptance.mp4` | 语音、语义 Pose、Nav2 路径、避障、到达停止和接管 | 待完整验收通过 |

## 录制规则

- 当前无运动 UI 可以录制，页面失败项必须保留并解释。
- 截图或录屏不得包含终端密码、Token、API Key、设备 SN 或个人信息。
- 不使用模拟器、旧截图或 Unitree App 遥控片段冒充 Robonix 自主导航。
- 物理运动视频必须包含当次许可、遥控急停、速度/距离限制和停止证据。
- 只有文件真实存在且校验完成后，才在飞书周报和讲稿中引用。
