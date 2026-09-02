# Day 2 四路灰度巡线版本

本目录保存短学期项目第二天使用的四路灰度自动巡线版本，源码快照基于本地 Gitee 工程提交 `836da12`（`feat: default to IRDM patrol and increase speed`），并加入本轮转向速度平滑优化。原仓库中的 EX1-EX5 内容保持不变。

## 本轮内容

- `source/`：完整 Keil 工程源码快照，包含 `MDK-ARM/miniBlance.uvprojx`；
- `source/MiniBalance_HARDWARE/TrackModule/`：四路灰度读取、状态编码、转向差速和弯道变速逻辑；
- 转向速度采用 `400 / 375 / 350 / 325 / 200 mm/s` 分级目标，并以 `5 / 10 mm/s` 每周期的加减速斜坡限制平滑切换；
- `source/Src/main.c`：上电默认进入 IRDM 自动巡线模式并初始化巡线输入；
- `source/MiniBalance_HARDWARE/KEY/key.c`：巡线模式下不再通过 KEY1 切换到其他模式；
- `firmware/miniBlance_day2_ir_line_following.hex`：本轮 Keil 构建生成的 HEX；
- `firmware/miniBlance_day2_ir_line_following.bin`：同一构建生成的 BIN。

## 线路状态约定

状态编码书写顺序为 `O1 O2 O3 O4`，探头物理顺序从左到右为 `O4 O3 O2 O1`。

```text
1001        -> 保持直行
0011 / 0111 -> 向左调节
1100 / 1110 -> 向右调节
```

## 构建记录

使用 Keil MDK-ARM / Arm Compiler 5.06 update 6 重新构建 `source/MDK-ARM/miniBlance.uvprojx`，结果为 `0 Error(s), 3 Warning(s)`；警告来自原有 `filter.c` 和 `show.c`，本轮 TrackModule 修改未引入编译错误。本目录中的 HEX 和 BIN 与该构建对应；首次烧录或修改固件后仍需在实车上扶车确认电机方向、编码器方向和安全断电条件。

## 固件校验

```powershell
Get-FileHash .\firmware\miniBlance_day2_ir_line_following.hex -Algorithm SHA256
Get-FileHash .\firmware\miniBlance_day2_ir_line_following.bin -Algorithm SHA256
```
