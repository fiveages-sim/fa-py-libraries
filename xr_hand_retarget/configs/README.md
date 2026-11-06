# xr_hand_retarget 配置

完整说明见 [docs/dev/CONFIG.md](../docs/dev/CONFIG.md)。

```
仓库 configs/local.yaml             User 白名单（type / command / teleop）
{o6,l6,o7,wuji_hand2,xhand1}.yaml   Product 索引 + 库默认 safety.command
hands/*                             T_robot
users/* + calib/*                   T_person
standard_hand.yaml                  T_standard
```

本机体感只改 `configs/local.yaml`（模板：`configs/local.yaml.example`）。
