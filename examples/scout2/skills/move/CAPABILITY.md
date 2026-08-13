# Move Skill (`robonix/skill/move`)

用户可调用的基础移动技能。支持前进、后退、左转、右转、停止等命令。使用 odom 闭环控制，精确定位。

## 使用方法

调用 `robonix/skill/move/move`，参数如下：

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `direction` | string | 是 | 移动方向 |
| `value` | float | 是 | 距离（米）或角度（度） |
| `speed` | float | 否 | 速度，默认 0.2 m/s 或 0.5 rad/s |

### direction 可选值

- `forward` - 向前移动（value = 距离，单位：米）
- `backward` - 向后移动（value = 距离，单位：米）
- `rotate_ccw` - 逆时针旋转/向左转（value = 角度，单位：度）
- `rotate_cw` - 顺时针旋转/向右转（value = 角度，单位：度）
- `stop` - 立即停止（value 忽略）

## 调用示例

```json
{"direction": "forward", "value": 1.0}
{"direction": "rotate_ccw", "value": 90}
{"direction": "backward", "value": 0.5, "speed": 0.1}
{"direction": "rotate_cw", "value": 45, "speed": 0.3}
{"direction": "stop"}
```

## 状态查询

调用 `robonix/skill/move/move/status` 查询任务状态：
- `run_id`: 任务 ID（从 move 返回值获取）

## 取消命令

调用 `robonix/skill/move/move/cancel` 取消正在执行的移动：
- `run_id`: 任务 ID（可选，为空则取消最近的任务）

## 技术细节

- 使用 odom 闭环控制，精确到达目标位置/角度
- 距离容差：2cm
- 角度容差：约 1.7 度
- 无进展超时：5 秒
- 支持任务取消
