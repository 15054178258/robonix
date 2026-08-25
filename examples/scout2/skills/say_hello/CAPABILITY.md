# Say Hello Skill (`robonix/skill/say_hello`)

通过机器人扬声器播放语音，支持自定义文字播报或姓名问候。

## 使用方法

调用 `robonix/skill/say_hello/say`，参数如下：

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `text` | string | 否 | 要播报的自定义文字（优先级最高） |
| `name` | string | 否 | 被问候者姓名，留空则说"你好！"（当 `text` 未提供时生效） |

### 调用示例

```json
{"text": "我已到达电视前方，正在对电视拍照，电视处于开启状态。"}
{"name": "小明"}
{"name": ""}
{}
```

### 输出示例

- `{"text": "我已到达电视前方..."}` → 播报原文，返回 `{greeting: "我已到达电视前方..."}`
- `{"name": "小明"}` → 播报 `"你好，小明！"`，返回 `{greeting: "你好，小明！"}`
- `{"name": ""}` 或 `{}` → 播报 `"你好！"`，返回 `{greeting: "你好！"}`

## 技术细节

- 通过 Atlas 动态发现 `robonix/service/speech/speak` MCP 端点（轮询超时 30s）
- 使用 `fastmcp.Client` 异步调用 speech 服务的 `speak` 工具
- 同步包装：`ThreadPoolExecutor(asyncio.run())` 将异步 MCP 调用转为同步，避免事件循环冲突
- 无任务状态管理：调用即播，无 status/cancel 接口
- 失败处理：TTS 失败仅打 warning，不重试

## 依赖项

| key | contract | transport |
|-----|----------|-----------|
| speech_speak | robonix/service/speech/speak | MCP |

Skill 在激活时通过 Atlas 解析 speech 端点，解析失败则拒绝启动。
