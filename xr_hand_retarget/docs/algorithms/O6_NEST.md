# O6 第二条技术路线：巢向量（nest）

`method: curl` 仍是日常路径（特征插值 + attract）。  
`method: nest` 是并列路线：**不新采一套人手**，用已有 RViz 捏合 q 定义「四指能到的中心」，用人手 **指尖→巢** 向量驱动拇指 2 轴。四指仍 1-DOF curl。不要和 curl 混权优化。

O7 日常走 **curl**（az 垫带）。试验路径 `method: veccurl` 才把 `v = tip−nest` 插到 j3/j2/j1。`method: nest` 在 O7 上回退 curl。

## 1. 「四指能到的中心」是什么

不是掌心几何质心，不是腕原点，也不是开掌时四指尖的平均。

O6 四指各 1-DOF，垫只能沿一条弧往掌面收。拇指 j2 决定这条弧扫到哪一指。所谓中心，是 **这些弧和拇指垫能对上的公共区域**（巢），在配置里用已经测过的两个捏合点来钉住：

| 锚点 | 已有 `hands/o6.yaml` | 语义 |
|------|----------------------|------|
| \(C_{\text{index}}\) | `j2=1.0`，`pinch_q.index`：j1=0.41，index=0.8 | 食指垫对上拇指垫 |
| \(C_{\text{middle}}\) | `j2=1.3`，`pinch_q.middle`：j1=0.41，middle=0.83 | 中指垫对上拇指垫 |

无名指/小指没有独立槽，和中指同一条 j2=1.3 弧。四指「中心」因此是：

- 一条 **巢曲线** \(C(j_2)\)：在 q 空间插值 \(C_{\text{index}} \leftrightarrow C_{\text{middle}}\)（j2 从 1.0 到 1.3）。
- 握拳时的 **拳心**：四指都卷、拇指进掌时，落在曲线上 j2 较低的一侧（现有 `fist_envelope.j2_split: 0.6` 附近），j1 有屋顶 `0.44`，避免沿捏合点把拇指捅穿掌面。

没有可靠垫 FK 时，**不要把 C 先变成笛卡尔再 IK**。锚点就是这两份 q。有垫 FK 之后，同一对 q 可以算出掌系里的两点 \(p(C_{\text{index}})\)、\(p(C_{\text{middle}})\)，向量残差再落到笛卡尔。

人手侧对应的不是机器人 XYZ，而是标准手掌面上的 **同一语义点**（四指 PIP/垫朝向的公共点，经 `T_person`）。光学特征仍用现有 `calib/o6.yaml`：curl / az / h / pinch_d。

## 2. 已有食指/中指捏合点怎么用

三件事，不要当「整手目标姿态」去抄：

**槽位（j2）**  
\(C_{\text{index}}\)、\(C_{\text{middle}}\) 给出两条合法捏合走廊。人手 `pinch_d` 或朝巢方位靠近哪条，j2 就驶向 1.0 或 1.3。这是现在 `blend_pinch_j2` 已经在做的；nest 只是把目标说成「驶向哪个 C」而不是「插值 az」。

**接触深度（j1 + 该指 curl）**  
每个 C 里的 j1=0.41 和指关节 0.8/0.83 是 **接触地板**（attract 的下限），不是锁死姿态。人两尖靠近才把 q 往地板抬；已经更弯的 VR 不要往回拉。无名/小指没有独立 RViz 点，沿用 `pinch_q.ring/pinky` 的指关节 0.8，j2 跟中指槽。

**安全屋顶**  
捏合点是「垫对上」；握拳是「拇指进掌」。同一 j1=0.41 在 j2=0.3 会穿模。所以 \(C(j_2)\) 必须带 `fist_envelope`：j2 低时 j1 封顶 0.44，只有 j2 进入捏合走廊才允许到 0.41 附近。

不要为 nest 再采一套 pinch。人手 pinch×4 仍只说明「现在对着哪根手指」；机器人槽位永远是这两份 RViz q。

## 3. 掌侧还是掌内

curl 难处：`thumb_az` 是掌面内掌骨朝向。下面三种光学 az 容易糊在一起：

| 人 | 机器人该去哪 | az 为何不够 |
|----|----------------|-------------|
| 掌侧 / along | j2→0，j1 小，四指可握 | 拇指贴尺侧，方位「沿着四指」 |
| 掌内 / 拳 | j2 中低，j1 被屋顶拦住 | 拇指已经跨过掌面，az 像对掌 |
| 对掌捏合 | j2→1.0 或 1.3，j1 到接触地板 | 同样跨掌，但尖对垫、四指未握满 |

`thumb_h`（近节相对掌面的高度）曾用来补「竖起 vs 压下」，光学一矮就和拳、捏混。

nest 用 **拇指相对巢的平面径向 + 半径**，不再把巢方位塞进掌骨 az 的 7° 窗：

```
v = p_thumb - C(j2)     （人：tip−巢；机：垫−C，或直接看 q 离锚点的距离）
```

- **掌侧**：\(\|v\|\) 大，v 指向尺侧/开掌，四指 curl 低 → 停在 along（j2≈0），不要吸向 C。
- **掌内**：四指 curl 过 `fist_envelope.finger_t`（0.92），\(\|v\|\) 变小且拇指在掌面法向内侧 → 走拳心，j1 只允许屋顶，**禁止**再按 pinch_d 吸到 0.41。
- **捏合**：四指未进拳门，某一 \(\|p_{\text{thumb}}-p_{\text{finger}}\|\) 短（现有 pinch_d）→ 才允许驶向 \(C_{\text{index}}\) 或 \(C_{\text{middle}}\)。

这就是 DexPilot「短向量升权」的 1D 版，但投影目标是 **巢锚点** 不是 URDF tip。拳门用四指 curl，不用 az，用来拆开掌内和对掌。

## 4. 安全

能处理，而且应该 **后置、与求解器无关**（和现在一样）。nest 只减少「命令本身往墙上开」；穿模仍靠层。

| 层 | 已有 | nest 怎么用 |
|----|------|-------------|
| 关节盒 | URDF limits | 照旧 |
| 巢屋顶 | `fist_envelope`：j1_max(j2) | 这就是掌内安全；nest 的拳心必须走这条，不能 IK 穿屋顶去够笛卡尔掌心 |
| 捏合走廊 | `safety.position.fk.pinch_j2`：j2≈1.0 或 1.3 ±0.25 才当 pinch 对 | 走廊外 thumb–index 当 rake（扫到 PIP），走廊内才允许尖距变小 |
| 速度/加速度 | `safety.motion` 默认关 | 需要再开 |
| mesh FCL | 关；O6 垫接触不是 tip mesh | 不要当主安全；没有垫 mesh 时 FCL 会放行真碰撞或误挡捏合 |
| **cspace 表** | `fk.cspace`：离线垫球网格 + 运行时拇指偏好投影 | 可选；开则替换 fist_inside / 轴搜索；见 [METHODS.md](METHODS.md) |

nest **不要**用「尖点 IK + 碰撞惩罚」替代屋顶。O6 空/简化 URDF 的 tip 不是垫。主约束永远是：j2 在走廊外时 j1 不得超过屋顶；走廊内才允许到 `pinch_q` 的 j1。

软吸引（attract）只抬低于地板的关节，不写 j2。硬安全在 attract 之后：envelope → FK 投影（若打开）。

## 5. 运行时

```
四指 curl     → 1-DOF（map_palm_tip）
j2            → 尖−巢掌面径向；结点来自 along / 食指 / 中指 nest_lat（标定，不写死 0.42）
                 拇指伸直且远离巢（along / good）→ j2→0，四指握拳不把拇指锁在对掌
pinch 时      → 再用 pinch_d 往 C_index / C_middle 软吸 j2，不锁死
j1            → nest_r（远→0，近→close_j1）；thumb_curl 只加 curl_span
拳门          → 只经 fist_envelope 限制 j1_max(j2)，不把 j2 钉在 0.6
attract       → 仅 pinch
```

切换：

```yaml
# configs/local.yaml
hands:
  type: o6
  method: nest    # 或 curl
```

```bash
FA_HAND=o6 FA_HAND_RETARGETING=nest ./run.sh vr-xrt wrist
```

启动日志应有 `type=nest` 和 `solver=nest`。模式切换时打印 `nest mode=along|fist|pinch/index|pinch/middle`。

O7 若写 nest 会回退 curl。试验 `method: veccurl` 才用 tip−nest。L6 可用 nest，锚点读 `hands/l6.yaml` 的 pinch_q。

## 6. 要不要继续标定？

**不必为 nest 新采一套。** 机器人巢就是现有 RViz `pinch_q`；人手仍是 `calib/o6.yaml` 的 open / together / fist / good / along / pinch×4。

| 已有 | nest 是否够用 |
|------|----------------|
| `hands/o6.yaml` 食指/中指 pinch_q + j2 | 是，就是 \(C_{\text{index}}\) / \(C_{\text{middle}}\) |
| `calib/o6.yaml` 四指 curl、pinch_d、拳门 | 是，用来分 along / 拳 / 对掌 |
| 人手 nest 半径 / 径向（tip−PIP 中心） | 运行时现算；量程在 `hands/o6.yaml` `workspace.nest` |

只有这些情况才重采 **同一套** 人手姿态（不是新 pose）：

- 换操作员 / 换 Pico 会话（不要用 O7 的 `users/pico.yaml`）
- 现有 o6 calib 握拳或 pinch 明显偏了
- 以后有垫 FK，要把 C 从 q 映到笛卡尔（那是机器人侧一次测量，不是人手重采）

不要为 nest 再采一遍 pinch_q。那是工作空间，不是人。

