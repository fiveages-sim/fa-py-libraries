# xr_hand_retarget 配置

完整说明见 [docs/dev/CONFIG.md](../docs/dev/CONFIG.md)。

```
standard_hand.yaml     T_standard
users/* + calib/*      T_person（curl 也算 user）
hands/*                T_robot
{o6,l6,o7,wuji_hand2,xhand1}.yaml   索引
仓库 configs/local.yaml             source + user + type + method
```
