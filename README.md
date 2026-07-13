# WHEELTEC B585 平衡小车实验

本仓库整理 WHEELTEC B585 两轮平衡小车的五个实验。硬件主控为 STM32F103RCT6，姿态传感器为 MPU6050，工程主要使用 Keil MDK 构建。

当前仓库从整理后的工作区重新建立历史。EX0 已移除；EX3 已用「实验三平衡串口精简版」替换原先混杂且落后的工程，作为当前实验三主线。

## 实验目录

| 目录 | 内容 | Keil 工程入口 | 最终固件 |
|---|---|---|---|
| [EX1](EX1/) | 电机、编码器与速度 PI | `EX1/Projects/MDK-ARM/atk_f103.uvprojx` | `EX1/firmware/` |
| [EX2](EX2/) | MPU6050 姿态采集与姿态解算 | `EX2/Projects/MDK-ARM/atk_f103.uvprojx` | `EX2/firmware/` |
| [EX3](EX3/) | 角度环 PD + 速度环 PI、串口调参的平衡精简版 | `EX3/source/USER/MiniBalance.uvprojx` | `EX3/firmware/` |
| [EX4](EX4/) | 平衡控制、蓝牙遥控与 OLED 状态显示 | `EX4/USER/MiniBalance.uvprojx` | `EX4/firmware/` |
| [EX5](EX5/) | 超声波安全监督、动态减速与防撞 | `EX5/USER/MiniBalance.uvprojx` | `EX5/Firmware/` |

## 推荐阅读顺序

1. 阅读对应实验目录的 `README.md`。
2. 按 [构建与复现指南](BUILDING.md) 准备工具并校验仓库内容。
3. 用 Keil 打开表格中的 `.uvprojx` 工程。
4. 先检查硬件连接、电机方向和编码器方向，再编译、烧录。
5. 调试 EX3 时，先阅读 [精简版源码改动说明](EX3/docs/精简版源码改动说明.md) 和 [验收答辩代码导读](EX3/docs/验收答辩代码导读.md)。
6. 如只需烧录已整理的构建结果，使用各实验的固件目录。

所有保留固件的校验值见 [FIRMWARE_SHA256SUMS.txt](FIRMWARE_SHA256SUMS.txt)。

## EX3 当前主线

EX3 来自指定文件 `实验三平衡串口精简版.zip`，不是此前 GitHub 上的旧 EX3。当前版本保留：

- MPU6050 / DMP 姿态角；
- 编码器速度反馈；
- 电机 PWM；
- 角度环 PD 与速度环 PI；
- USART1 文本调参命令：`BAL`、`VEL`、`PID`、`MID`、`MODE`、`ARM`、`STOP`、`STATUS`；
- KEY2 使能、电池电压保护、超角度停机与拎起/放下检测。

蓝牙、OLED、雷达、超声波、PS2、巡线和原厂上位机协议等与实验三验收无关的功能已从该精简版移除。基准参数为：

```text
PID 27000 110 400 2
```

参数仅作为已有源码中的起点；实际使用前仍应在有人扶车、可随时断电的条件下逐步验证。

## 仓库与本地资料边界

仓库只跟踪可复现工程所需的源码、Keil 项目配置、实验文档、数据和最终固件。以下内容不得放入项目目录或上传 GitHub；如确需保留，应放在项目目录之外的独立位置：

- `_local/`：课程资料、原厂参考源码、FLYMCU、Ghidra、JDK 等大文件或本机工具；
- `实验报告/`：可能包含姓名、学号、文档作者属性等个人信息；
- `Output/`、`OBJ/`：可重新生成的 Keil 中间产物；
- `backup/`、`运行记录/`：本机烧录备份和临时日志；
- `.vscode/`、`__pycache__/`、Keil 用户配置等机器相关文件。

## 安全提示

- 首次上电或修改 PID 后必须扶住车体，并准备立即断电。
- 先确认电机与编码器方向，再启用闭环控制。
- EX3 使用 `ARM` 前确认 KEY2/enable 状态；异常时发送 `STOP` 或直接断电。
- 不要在车轮悬空、人员靠近轮组或供电不稳定时进行高增益测试。

## 相关文档

- [构建与复现指南](BUILDING.md)
- [参与贡献](CONTRIBUTING.md)
- [项目背景与硬件说明](docs/项目背景与硬件说明.md)
- [EX1–EX3 面试知识点](docs/EX1-EX3_面试知识点.md)
- [文档索引](docs/README.md)
- [仓库整理记录（2026-07-10）](docs/仓库整理记录_2026-07-10.md)
- [公开仓库边界与安全检查](SECURITY.md)
