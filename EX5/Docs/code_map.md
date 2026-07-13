# EX5 代码地图

## 测距层

- `MiniBalance_HARDWARE/ULTRASONIC/ultrasonic.c`
  - TIM2_CH2 捕获 Echo 上升沿和下降沿。
  - 30 ms 比较中断终止异常测量，65 ms 服务周期发布一次结果。
  - 双缓冲 `UltrasonicSnapshot` 完整提交距离、有效位、丢帧数和样本编号。

## 安全监督层

- `MiniBalance/CONTROL/obstacle_guard.c`
  - 纯状态机和速度裁剪，不访问 MCU 外设。
  - 实现动态停车距离、线性减速、阻挡锁定、三样本释放和无回波降级。
- `MiniBalance/CONTROL/control.c`
  - 蓝牙标志先生成 `requested_speed_mm_s`。
  - `ObstacleGuard_Update()` 生成 `allowed_speed_mm_s`，再换算为速度 PI 的编码器目标。
  - 进入 `BLOCKED` 时清一次历史前进积分，但保留速度闭环制动和平衡角度环。

## 人机接口

- `SYSTEM/usart/usart.c`：`STATUS`、`UGUARD ON/OFF` 及兼容命令。
- `MiniBalance/OLED_STATUS/oled_status.c`：显示 `OFF/CLR/SLW/BLK/DGD`。
- `MiniBalance_HARDWARE/USART3/usart3.c`：蓝牙只修改驾驶意图，不能关闭安全监督器。

## 测试与交付

- `Tools/obstacle_guard_test.c`：状态机边界和时序测试。
- `Docs/test_flow.md`：台架与实车验收流程。
- `Firmware/`：编译后的 HEX、BIN 和构建日志。
