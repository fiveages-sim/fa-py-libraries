# 手部映射分层与仓库边界

目标：`source → 标准手 21 点 → 灵巧手 q`。  
人只标定到标准手；灵巧手只标定一次到标准手。VR 是一种 source，不是映射核心。

## 1. 运行时分层

```
┌─────────────────────────────────────────────────────────┐
│  sources（可插拔，可有 SDK / 串口 / pkl）                  │
│  xrt OpenXR26 │ dexcap 角+FK │ wuji_glove EMF+IK │ replay │
└──────────────────────────┬──────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────┐
│  HandFrame（契约，无 ROS、无厂商 SDK）                     │
│  xyz (21,3) m, MediaPipe 点序, wrist 原点, active         │
│  T_person：骨长 → 标准手（users/<id>.yaml）                 │
└──────────────────────────┬──────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────┐
│  robots（每手一份 yaml + URDF + 求解器）                   │
│  wuji: tip+骨方向  │  xhand1: 四指曲率+拇 tip  │  o6: 协同 │
│  T_robot 写在 hands/<type>.yaml，与人无关                   │
└──────────────────────────┬──────────────────────────────┘
                           ▼
              ROS Float64MultiArray / 其它 sink
```

禁止：source 直接出目标机器人 q；Wuji **20 维关节**当中间层；把 `segment_scaling` 和人焊在同一份 overlay 里。
标准手是 **Wuji 21 点**（MediaPipe 点序，见 §5），不是 20-DOF。20 轴盖不住侧摆/指尖笛卡尔，手套遥操本就是 21 点，再经 20 轴转一圈不合理。

## 2. 仓库怎么拆

### 现在的问题

`fa-py-libraries` 把 **VR 产品线** 和 **手映射算法** 缠在一起：

- `vr_pose_publisher` 直接 `import xr_hand_retarget`
- 早期 pipeline 从包名到实现都假设 OpenXR 26
- DexCap / 手套没有 VR 时也要装整仓 ROS+XRT

手映射 **不必** 住在 `fa-py-libraries`。VR **应当** 继续住在这里。

### 本仓过渡（已选）

不新开 `fa-hand`。映射核心唯一家在 `xr_hand_retarget`（`algorithms/` / `backends/` / `sources/`）。VR 仍在本仓 `vr_pose_publisher`。布局见 [LAYOUT.md](LAYOUT.md)；method 表见 [../algorithms/METHODS.md](../algorithms/METHODS.md)。

```
fa-py-libraries/
  xr_hand_retarget/
    algorithms/          # curl / nest / vector / dexpilot / …
    backends/            # xhand1, linker, wuji
    sources/             # OpenXR26 → 21
  vr_pose_publisher/     # 腕 + DualHand
  configs/local.yaml     # type + method
  third_party/wuji-retargeting/   # 官方源（可选）
```

以后若抽独立仓：等 `HandFrame` + `step(frame)` 稳定再 `git subtree`。**下一里程碑**仍是契约解耦（§3 P0），不是立刻两仓。

## 3. 现有代码怎么改（分阶段）

### P0 — 契约（本仓已落地）

1. `HandFrame`（`sources/frame.py`）：`joints26` + `xyz`(21×3) + `active` + 可选 timestamp。
2. `openxr26_to_mediapipe21` 在 `sources/`；XRT 取帧 `sources.xrt.read_hand_frame`。
3. Backend / `SidePipeline.step(frame)`；构造函数**不再收** `xrt`（Linker j2 锁可选用绑定 `xrt=` 轮询按键）。
4. `node.py` / `DualHandRetargetNode`：循环里 source 取帧，再 `step(frame)`。

DexCap 以后只写 source，不必改 O6/XHand。验证命令见 [RUN.md](RUN.md)。

### P1 — 配置拆开（本仓已落地）

| 文件 | 内容 |
|------|------|
| `standard_hand.yaml` | 固定骨长 / 休息比例（`person.load_standard_hand`） |
| `users/<id>.yaml` + `calib/*` | T_person（人→标准手；curl 端点也是 user） |
| `hands/{wuji,xhand1,o6,…}.yaml` | T_robot（URDF、捏合槽、尺度、权重） |
| `local.yaml` | `source` / `user` / `type` / `method` |

Wuji：`hands/wuji.yaml` 持 robot `segment_scaling`；操作员覆盖进 `users/<id>.yaml` 的 `person.segment_scaling`。详见 [CONFIG.md](CONFIG.md)。

### P2 — `T_xhand` 一次标定（本仓已落地）

- 开合限位、捏合槽：优先 XHand URDF（`robot.use_urdf_limits`；curl pinch 扫 j1）。
- 小指过长等：`hands/xhand1.yaml` → `robot.finger_scale`，不写 user/calib。
- 转储：`python -m xr_hand_retarget.txhand --side right`。
- 参考员必须先 T_person（curl calib）再调 T_xhand。

详见 [TXHAND.md](TXHAND.md)。XHand：四指曲率 + 拇指笛卡尔，不要单走 q 抄录。

### P3 — 多源

每个 source 实现：

```python
class HandSource(Protocol):
    def start(self) -> None: ...
    def read(self) -> dict[str, HandFrame]:  # left/right
        ...
    def close(self) -> None: ...
```

XRT 独占 PC Service 的约束留在 `source_xrt`，不要写进 `fa_hand.frame`。

## 4. VR 与手映射的关系

| | VR（本仓） | 手映射（fa_hand） |
|--|-----------|-------------------|
| 负责 | 头显/腕 `/teleop/*`、FSM 热键、bag | 21 点 → q |
| 不负责 | 灵巧手 IK、人骨长 | 臂笛卡尔、Pico Input 模式 |

`./run.sh vr-xrt wrist` = XRT source + 腕 EE + `fa_hand` 双手。  
手套/DexCap 单开 `python -m fa_hand.ros` 即可，不启动 `vr_pose_publisher`。

## 5. 标准手 = Wuji 21（不是 20-DOF）

标准手是 **21 点笛卡尔**（MediaPipe 点序，与官方 wuji-retargeting / 光学手套同一套），不是 Hand 2 的 20 个主动关节。

Linker **`mp_curl`**：`VR 26 → openxr26_to_mediapipe21 → MP21 骨角 curl knots → O6/L6/O7 q`（与默认 PalmTip `curl` 并列；标定见 `configs/calib/mp_curl.yaml`，机器人端点见 `hands/{o6,l6,o7}.yaml`）。

```
VR 26 / 手套 / DexCap  →  T_person（手势手册 + users/*.yaml）
                       →  标准手 xyz (21,3)     ← 用户只标定到这里
                       →  T_robot + attract     ← 捏合吸引、指槽、拳包络
                       →  Wuji 20 / O6 / O7 / XHand1 q
```

- **21 点**才覆盖指尖位置、骨方向、侧摆；20-DOF 是某一只机器人的命令，不能当契约。
- 手套已经在 21 点上遥操 Wuji；若标准手改成 20 轴，手套还要再映射一次，多一层还丢信息。
- 手势手册 `configs/poses/handbook.yaml` 描述的是人在 21 点上的姿态（open / pinch / `index_abd`…）。某款灵巧手没有对应关节就忽略 extra。
- 捏合吸引属于 **T_robot**（21 点标准手 → 该设备），不要写在 VR 原始 26 点里，也不要写在 20 轴 q 上当「标准」。
- 光学换视角：pose 用判定域（lo/hi），不要把 OpenXR 26 当标定文件。

