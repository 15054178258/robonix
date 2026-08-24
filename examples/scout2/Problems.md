# Problems & Solutions

## say_hello 技能语音输出问题

### 问题一：事件循环冲突

**现象：** 调用 say_hello 技能时报错「事件循环冲突」，语音问候无法执行。

**根因：** `speak_sync()` 使用 `asyncio.run(_speak(text))` 尝试创建新事件循环，但 fastmcp 的 `@skill.mcp` handler 已经在运行一个循环，`asyncio.run()` 拒绝创建嵌套循环。

**修复：** `skills/say_hello/say_hello/main.py` 中将 `speak_sync()` 改为用 `ThreadPoolExecutor` 在子线程执行 `asyncio.run()`：

```python
from concurrent.futures import ThreadPoolExecutor

def speak_sync(text: str) -> dict:
    with ThreadPoolExecutor(max_workers=1) as pool:
        future = pool.submit(asyncio.run, _speak(text))
        return future.result()
```

---

### 问题二：PCM 音量 0%（完全静音）

**现象：** 代码修复并部署后，skill 返回成功但机器人无声。

**根因：** ALSA mixer 的 `PCM Playback Volume`（numid=56）值为 0，所有 PCM 音频被完全静音。`Master Playback Switch` 和 `Line Out Playback Switch` 均为 on，但 PCM 本身被关死。

**排查方法：**
```bash
amixer -c 1 cget numid=56
# 输出: values=0,0
```

**修复：**
```bash
amixer -c 1 cset numid=56 200,200
```

---

### 问题三：音频输出设备选择

**现象：** PCM 音量调高后仍然无声。

**根因：** 音频输出设备选错。`aplay -l` 列出两个声卡：
- **card 0: HDA NVidia** — 4 路 HDMI/DP 输出（hw:0,0 ~ hw:0,3）
- **card 1: HDA Intel PCH** — ALCS1200A 模拟音频（hw:1,0）+ 4 路 HDMI（hw:1,3 ~ hw:1,9）

manifest 最初配置 `speaker_device: "hw:1,0"`（主板 3.5mm 插孔），但用户实际音箱/显示器接在 **HDMI（hw:0,3）** 上，所以声音从显示器扬声器输出。

**排查方法：**
```bash
aplay -l          # 列出所有播放设备
arecord -l        # 列出所有录音设备
pacmd list-sinks  # 查看 PulseAudio 音频节点及音量
```

**修复：** `robonix_manifest.yaml` 中配置 `speaker_device: "plughw:0,3"`，HDMI 通过 `plughw:` 插件自动处理格式转换。

---

### 问题四：采样率不匹配导致音质差

**现象：** 能听到声音，但内容含糊不清、听不出说的什么。

**根因：** speech 服务的 Edge TTS 输出 **16kHz 单声道** PCM，但 `SpeakerDriver` 默认以 **24kHz 立体声** 播放，`aplay` 的重采样质量差，导致语音失真。

**排查方法：** 用 16kHz 单声道直接播放测试，有声则说明是重采样问题：
```bash
dd if=/dev/urandom bs=1 count=96000 2>/dev/null | \
  aplay -D plughw:0,3 -f S16_LE -r 16000 -c 1 -t raw
```

**修复：** `robonix_manifest.yaml` 中配置 TTS 输出参数，匹配 speech 服务：
```yaml
primitive:
  - name: audio_driver
    config:
      speaker_device: "plughw:0,3"
      speaker_sample_rate: 16000
      speaker_channels: 1
```

---

### 问题五：重新构建/重启后 PCM 音量重置

**现象：** 重新构建和重启 rbnx 后，之前设置的 PCM 音量（200）被重置为 0，导致无声。

**根因：** ALSA mixer 设置（`amixer -c 1 cset numid=56 200,200`）是运行时设置，不会被持久化到磁盘。每次 rbnx boot 重新初始化 audio_driver 时，ALSA 恢复默认值（音量 0）。

**排查方法：**
```bash
amixer -c 1 cget numid=56
# 如果输出 values=0,0，说明音量被重置
```

**修复：**
```bash
amixer -c 1 cset numid=56 200,200
```

**持久化方案：** 将音量设置写入 ALSA 配置文件，或添加到 rbnx boot 脚本中自动执行：
```bash
# 添加到 ~/.asoundrc 或 /etc/asound.conf
# 或添加到 rbnx boot 启动脚本中
amixer -c 1 cset numid=56 200,200
```

---

### 问题六：_speak_endpoint 未初始化导致 Client 报错

**现象：** 直接调用 `speak_sync()` 时，`speak_endpoint` 为空字符串，fastmcp `Client('')` 抛出 `ValueError: Unsupported script type:` 错误。

**根因：** 当 `speak_sync()` 被直接调用（而非通过 `@skill.mcp` handler）时，`on_activate` 中的 `_speak_endpoint = resolve_speak_endpoint()` 不会被执行，导致 `_speak_endpoint` 保持空字符串。`Client('')` 无法推断传输类型。

**排查方法：**
```python
# 检查 speak_endpoint 是否为空
import say_hello.main as main
print(f"_speak_endpoint: '{main._speak_endpoint}'")
# 如果为空，直接调用 speak_sync() 会报错
```

**修复：** `speak_sync()` 中增加自动解析逻辑：

```python
def speak_sync(text: str) -> dict:
    """Run async _speak in a separate thread to avoid event-loop conflict."""
    # Auto-resolve endpoint if not yet initialized
    global _speak_endpoint
    if not _speak_endpoint:
        try:
            _speak_endpoint = resolve_speak_endpoint()
        except RuntimeError as e:
            log.error("failed to resolve speech endpoint: %s", e)
            return {"ok": False, "detail": str(e)}
    with ThreadPoolExecutor(max_workers=1) as pool:
        future = pool.submit(asyncio.run, _speak(text))
        return future.result()
```

---

### 音频链路完整排查步骤总结

```
1. aplay -l          # 确认物理音频设备
2. alsamixer -c X     # 检查各通道是否 Muted（MM 状态）
3. amixer -c X cset   # 调整 PCM/Speaker 音量
4. aplay 测试音       # 验证音频输出
5. 检查 manifest 配置  # 确保 device + sample_rate + channels 匹配
```

### 音频链路

```
用户指令 → say_hello skill (MCP)
         → speech 服务 (Edge TTS 合成 16kHz 单声道 PCM)
         → audio_driver (gRPC 流式传输)
         → aplay → plughw:0,3 (自动重采样) → HDMI 显示器扬声器
```

### 关键配置文件

| 文件 | 作用 |
|------|------|
| `skills/say_hello/say_hello/main.py` | 技能代码，事件循环修复 |
| `robonix_manifest.yaml` | 音频设备、采样率、声道配置 |
