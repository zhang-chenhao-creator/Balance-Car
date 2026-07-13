# 参与贡献

欢迎提交问题和改进。为了让修改可复查、可复现，请保持每次变更范围明确，并附上实际执行过的验证。

## 提交前

1. 从最新 `main` 创建分支。
2. 不要提交 `_local/`、实验报告、个人信息、凭据、工具安装包、Keil 用户配置或可重新生成的中间产物。
3. 运行：

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File scripts/check_public_repo.ps1
   powershell -NoProfile -ExecutionPolicy Bypass -File scripts/verify_firmware_hashes.ps1
   ```

4. 修改 EX5 安全监督逻辑时，同时运行 [EX5 软件测试](BUILDING.md#4-ex5-软件测试)。
5. Pull Request 中写明修改原因、影响的实验、验证命令及结果；未完成的硬件验证标为「待实车确认」。

## 固件和控制参数

- 只有与当前源码对应且经过验证的最终 HEX/BIN 才能进入固件目录。
- 更新固件时必须同步更新 `FIRMWARE_SHA256SUMS.txt`。
- PID、停车距离、速度上限和保护阈值的修改应附测试条件与结果；没有实测依据时不要宣称已改善实车性能或安全性。

## 安全问题

不要在公开 Issue 或 Pull Request 中粘贴密钥、令牌、个人信息或其他敏感数据。发现安全问题时按 [安全策略](SECURITY.md) 处理。
