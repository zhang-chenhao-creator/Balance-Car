# EX4 蓝牙遥控与 OLED 状态显示

EX4 在平衡控制主链路上加入 USART3 蓝牙遥控和 OLED 状态显示。

## 主要入口

- Keil 工程：`USER/MiniBalance.uvprojx`
- 程序入口：`USER/MiniBalance.c`
- 平衡与运动控制：`MiniBalance/CONTROL/control.c`
- 蓝牙命令：`MiniBalance_HARDWARE/USART3/usart3.c`
- OLED 状态页：`MiniBalance/OLED_STATUS/oled_status.c`
- 最终固件：`firmware/EX4_Bluetooth_OLED.hex`

## 当前功能

- USART3 接收前进、后退、左转、右转和停止命令；
- 平衡、速度与转向控制共同生成左右电机输出；
- OLED 显示蓝牙命令、ARM/STOP 状态、车身角度、电池电压、目标速度和 PID 参数；
- USART1 的 `STOP` 会锁停，`ARM` 才能重新允许输出；
- 停机或倒地时清除速度积分，降低再次启动时突冲风险。

`OBJ/` 是本地 Keil 构建目录，不进入 GitHub。最后一次整理的 hex 已复制到 `firmware/`。
