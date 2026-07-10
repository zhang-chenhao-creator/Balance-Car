# EX3 实验三平衡串口精简版

本目录来自 `实验三平衡串口精简版.zip`，现作为仓库中的实验三唯一主线。旧 GitHub EX3 使用另一套 HAL 工程并混入实验二、构建产物和历史材料，已被整体替换。

## 这个版本保留什么
- 平衡控制主链路：MPU6050 姿态角、编码器速度、电机 PWM、角度环 PD、速度环 PI。
- 串口 1 调参：`BAL kp kd`、`VEL kp ki`、`PID bk bd vk vi`、`ARM`、`STOP`、`STATUS`。
- 必要安全模块：KEY2 使能、电池电压保护、倾角超过 40 度停机、拎起/放下检测。

## 这个版本删除什么
蓝牙、OLED、雷达、超声波、PS2 手柄、巡线、APP 显示、DataScope/原厂上位机数据帧。

## 打开工程
Keil 工程：`source\USER\MiniBalance.uvprojx`

压缩包中包含 2026-07-09 的 Keil 构建结果；整理后将最后一次 `MiniBalance.hex` 和 `MiniBalance.bin` 单独放入 `firmware/`。`source\OBJ` 只作为本机构建目录，不进入 GitHub。重新编译时打开 Keil 执行 Build。

## 演示流程
1. 烧录 Keil 新编译出的精简版 hex。
2. 打开 `启动平衡串口精简版调参助手.bat`。
3. 连接 COM7，按 F3-F10 写入不同参数案例。
4. 手扶小车直立，确认 KEY2/enable 打开，按 F11 ARM。
5. 最终参数建议从 `PID 27000 110 400 2` 开始，这是从 WHEELTEC 源码提取的基准参数。

## 目录结构

- `source/`：精简后的完整 Keil 工程与源码。
- `docs/`：源码改动、答辩导读和调参流程。
- `tools/`：串口 PID 调参助手。
- `firmware/`：整理后保留的最终 hex/bin。
- `backup/`：本地烧录前备份，不进入 GitHub。

## 验证边界

本次整理完成了 ZIP 来源核对、关键源码保留检查和固件哈希记录；未在当前会话中连接实体小车重新做硬件验收。首次使用仍需按安全流程扶车验证。
