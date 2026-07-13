# EX5 超声波安全监督版

EX5 在 EX4 的平衡、蓝牙遥控和 OLED 功能上增加前向超声波防撞。蓝牙命令只表示驾驶意图，最终前进速度必须经过超声波安全监督器，不能覆盖防撞状态。

## 核心行为

- 超声波：`TRIG=PC15`，`ECHO=PA1/TIM2_CH2`，测量周期 65 ms。
- 安全状态：`OFF / CLR / SLW / BLK / DGD`。
- 动态停车距离：`clamp(120 + v_ref × 0.18, 120, 280) mm`。
- 提前减速距离：停车距离再加 180 mm，区间内线性限制前进速度。
- 障碍物进入停车距离后锁住前进；持续发送蓝牙前进命令也不会越过锁定。
- 后退命令不受前向传感器限制。距离连续 3 个有效样本超过释放阈值后解锁。
- 连续 3 次无有效回波进入降级状态，前进上限为 100 mm/s；连续 2 次有效安全样本后恢复。

典型阈值：

| 请求速度 | 停车距离 | 开始减速距离 |
|---:|---:|---:|
| 100 mm/s | 138 mm | 318 mm |
| 300 mm/s | 174 mm | 354 mm |
| 600 mm/s | 228 mm | 408 mm |

## 串口接口

USART1 使用 115200 8N1：

```text
STATUS
UGUARD ON
UGUARD OFF
STOP
ARM
```

兼容旧命令：`UAUTO/UAVOID` 等同于 `UGUARD ON`，`UOFF/UNORMAL` 等同于 `UGUARD OFF`。

运行时不能关闭安全监督器。必须先执行 `STOP` 或使用硬件禁能，再执行 `UGUARD OFF`；`ARM` 会自动恢复监督器。

`STATUS` 关键字段：

- `DIST/UOK/MISS`：最近距离、当前样本有效性、连续丢失次数。
- `OBS`：1 表示前进已锁定。
- `UGUARD/USTATE`：监督器开关和状态编号。
- `USTOP/USLOW`：当前动态停车、减速阈值。
- `VREQ/VSAFE`：蓝牙请求速度和安全裁剪后的速度。

状态编号：`0=OFF, 1=CLEAR, 2=SLOW, 3=BLOCKED, 4=DEGRADED`。

## 构建与固件

Keil 工程：`USER/MiniBalance.uvprojx`。

固件：

```text
Firmware/EX5_Ultrasonic_Avoid.hex
Firmware/EX5_Ultrasonic_Avoid.bin
```

烧录会改写实车，必须得到用户确认后再执行。详细测试步骤见 `Docs/test_flow.md`，设计与参数依据见 `Docs/ultrasonic_guard_design.md`。
