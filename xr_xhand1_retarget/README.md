# xr_xhand1_retarget（一期）

| 手指 | 映射 |
|------|------|
| 食 / 中 / 无名 / 小 | 骨链屈曲 → `j2/j3`（或食 `j2/j3`） |
| 食指 | 额外 `index_joint1` 侧移（近节横向，握拳时衰减） |
| 拇指 | tip 方位 → `j1`；tip 骨链屈曲 → `j2/j3` |

## 运行

```bash
source /opt/ros/jazzy/setup.bash
PYTHONUNBUFFERED=1 python -m xr_xhand1_retarget --side right
```

## 调参

- `finger_chain`：四指（含食指屈伸）量程
- `index_abduction`：侧移；反了 → `abduction_sign: -1`
- `thumb_tip`：对掌反 → `invert_cmc: true`；屈伸弱 → 降 `mcp_fist_ref_rad`
