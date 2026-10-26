# 算法 method 目录

手型号（backend）和求解器（algorithm）分开。实现住在 `xr_hand_retarget.algorithms`。

运行时：`from xr_hand_retarget.algorithms import CATALOG, normalize`。

## 算法（local.yaml `method`）

| method | 来源 | 捏合 | 合掌/拳 | 算力 | 何时用 |
|--------|------|------|---------|------|--------|
| **dexpilot** | dex-retargeting 迁入 | 最好 | 合不了掌 | 高（解析雅可比 + L-BFGS） | 日常遥操 |
| **dexpilot + vector** | `dexpilot.mix.w_bone` / `w_fist` > 0 | 仍强 | 能合拳 | 更高 | 要拳；易自碰 |
| **vector** | SomeHand 骨链 | 一般 | 跟手较好 | 中 | 少标定、跟姿态 |
| **curl** | 本仓标定插值；O6/L6 j2/j1 主用 tip−巢 `nest_lat`/`nest_r`（az 回退） | 指槽稳 | 标定到位则到底 | 低 | Linker / XHand 可复现遥操 |
| **mp_curl** | **VR26→MP21 四指 curl**（O6/L6/O7 同路径）；**O7 拇指** PalmTip curl + j1 attract；**O7 规划层** `pinch_q` 天花板 + FK 垫球投影（`curl.pinch`） | 四指跟手；O7 防穿模 | 四指 `mp_curl.yaml`；O7 拇指 `calib/o7.yaml` | 低 | O6/L6/O7 |
| **nest** | O6/L6 第二条：RViz 捏合点当巢，指尖→巢向量 | 两锚点地板 | 拳走 envelope | 低 | `method: nest`；不必为 nest 重采 |
| **thumb_ik** | 拇指笛卡尔 IK | 拇 tip 准 | 四指几何 | 中 | 只抠拇指 |

`method` 在 `configs/local.yaml`，覆盖包装 yaml 的 `retargeting.type`。以前只写 `curl`、包装文件却是 `dexpilot` 时，实际跑的是 dexpilot。

dexpilot+vector **不是** 新的 method 名：`method: dexpilot`，再改 `xr_hand_retarget/configs/hands/xhand1.yaml`（或索引指向的明细）里的 mix：

```yaml
dexpilot:
  mix:
    w_dexpilot: 1.0
    w_bone: 0.45    # >0 混骨链，能合拳
    w_fist: 1.8     # >0 关节握拳先验；易撞
```

共用 curl：O6 palm-tip 与 XHand1 开掌/握拳插值都是「人手特征 → 机器人端点 lerp」，不要合成一个加权优化。DexPilot tip 投影也不要和 curl 混权。

## O6 `cspace` 安全集

离线用垫球把关节网格标成 safe/unsafe；运行时把不安全的 curl \(q\) 投影到**加权最近**安全格（拇指权重大），替代 `fist_inside` 规则机与 FK 轴搜索。速度限仍接在投影前。

```bash
# 在 xr_hand_retarget 根目录；改 pad / clearance 后必须重跑
python tools/build_o6_cspace.py --side both
# 或: python -m xr_hand_retarget.tools.build_o6_cspace --side both
```

```yaml
# configs/hands/o6.yaml → safety.position.fk
cspace:
  enabled: true
  table: assets/cspace/o6_{side}.npz
  weights: [4.0, 10.0, 0.2, 0.2, 0.2, 0.2]  # j1,j2,index…pinky
```

`weights` 是**移动偏好**（越大越优先改该关节）；内部取倒数作路径边权，故默认会先收 j2/j1。标签策略（与运行 mid 跳过 pack 不同）：rake/pack/palm **始终**启用；捏合走廊内用 `pinch_clearance`。表缺失时回退 legacy FK。

侧角采集步骤见 [../user/CURL_CALIB.md](../user/CURL_CALIB.md)。
