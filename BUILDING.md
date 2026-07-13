# 构建与复现指南

本文说明如何从一次干净克隆中检查仓库、打开 Keil 工程、生成固件并运行现有的软件测试。源码可构建、固件可校验，不等同于已经在每一套实车硬件上完成验证；未记录的实车结果统一视为「待实车确认」。

## 1. 环境

- Windows 10 或 Windows 11；
- Keil MDK / µVision；
- STM32F1 Device Family Pack；
- Windows PowerShell 5.1 或 PowerShell 7，用于运行公开仓库检查和固件校验；
- 可选：GCC，用于运行 EX5 的纯 C 单元测试。

工程文件记录的 Device Family Pack 版本如下：

| 实验 | 工程记录的 Pack 版本 |
|---|---|
| EX1、EX2 | `Keil.STM32F1xx_DFP.1.0.5` |
| EX3、EX4、EX5 | `Keil.STM32F1xx_DFP.2.4.1` |

可通过 Keil 的 Pack Installer 安装 [STM32F1 Device Family Pack](https://www.keil.arm.com/packs/stm32f1xx_dfp-keil/)；µVision 的 Pack Installer 入口见 [Keil 官方说明](https://www.keil.com/support/man/docs/uv4/uv4_ui_manage.asp)。若现有 Pack Installer 不再提供 1.0.5，请先在副本中验证新版 Pack 的编译兼容性，不要在未验证时直接改写工程版本。

## 2. 干净克隆后的检查

```powershell
git clone https://github.com/zhang-chenhao-creator/Balance-Car.git
Set-Location Balance-Car
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/check_public_repo.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/verify_firmware_hashes.ps1
```

两条脚本都应以退出码 0 结束。第一条检查意外提交的本地资料、构建产物和常见凭据；第二条将已发布固件与 `FIRMWARE_SHA256SUMS.txt` 逐项比对。

## 3. Keil 构建入口

| 实验 | 工程文件 | Target | 输出名 |
|---|---|---|---|
| EX1 | `EX1/Projects/MDK-ARM/atk_f103.uvprojx` | `Template` | `B585_Motor_Encoder_PI` |
| EX2 | `EX2/Projects/MDK-ARM/atk_f103.uvprojx` | `Template` | `B585_MPU6050_Attitude` |
| EX3 | `EX3/source/USER/MiniBalance.uvprojx` | `MiniBalance` | `MiniBalance` |
| EX4 | `EX4/USER/MiniBalance.uvprojx` | `EX4_Bluetooth_OLED` | `EX4_Bluetooth_OLED` |
| EX5 | `EX5/USER/MiniBalance.uvprojx` | `EX5_Ultrasonic_Avoid` | `EX5_Ultrasonic_Avoid` |

对每个实验：

1. 用 µVision 打开工程文件。
2. 确认表格中的 Target 已选中。
3. 执行 `Rebuild all target files`。
4. 先解决所有 Error；Warning 也应逐项检查，不应直接忽略。
5. 将经过验证、需要发布的 HEX/BIN 复制到对应 `firmware/` 或 `Firmware/` 目录。
6. 更新 `FIRMWARE_SHA256SUMS.txt`，再运行固件校验脚本。

`Output/`、`OBJ/` 等中间目录被有意排除，不应提交到仓库。

## 4. EX5 软件测试

EX5 的超声波安全监督器可脱离单片机工程进行纯 C 测试：

```powershell
gcc -std=c99 -Wall -Wextra -Werror `
  -IEX5/MiniBalance/CONTROL `
  EX5/Tools/obstacle_guard_test.c `
  EX5/MiniBalance/CONTROL/obstacle_guard.c `
  -o obstacle_guard_test.exe
./obstacle_guard_test.exe
```

预期输出为 `All obstacle guard tests passed`。完整台架和实车步骤见 [EX5 测试流程](EX5/Docs/test_flow.md)。

## 5. 实车安全边界

- 编译成功和单元测试通过不能替代台架、传感器和实车验证。
- 首次烧录或修改 PID 后必须扶住车体，并准备立即断电。
- 先确认电机方向、编码器方向、供电和保护逻辑，再启用闭环控制。
- 烧录会改写设备；执行前应确认硬件型号、目标固件和恢复方案。
