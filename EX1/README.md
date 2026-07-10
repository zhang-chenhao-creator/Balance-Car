# EX1 目录结构说明

本实验已跑通，整理原则是：不改源码、不改 Keil 工程配置、不移动会影响运行路径的脚本。

## 主要入口

- Keil 工程：`Projects/MDK-ARM/atk_f103.uvprojx`
- 实验源码：`Users/`
- 编译输出：`Output/`
- 烧录备份：`backup/`
- 实验数据：`实验数据/`
- 实验报告：仅本地保存在 `_local/实验报告/EX1/`，不上传 GitHub
- 操作说明：`文档/`
- 构建与烧录记录：`运行记录/`

## 根目录保留项

- `串口实验助手.pyw`
- `启动串口实验助手.bat`
- `keilkill.bat`

这些文件保留在根目录，是为了避免改变它们原来的相对路径行为。

## 最终固件

- `firmware/B585_Motor_Encoder_PI.hex`
- `firmware/B585_Motor_Encoder_PI.bin`

`Output/` 是本地构建目录，不进入 GitHub；需要重新生成时直接在 Keil 中 Build。
