# 顺畅度标定（外层 LPF + vmax）

P1 节拍已收口后，体感主战场是 **阶梯 / 滞后**，不是 EE Hz。

塑形只走发布路径一层：

```
q_target → OnlineCommandFilter(filter_cutoff_hz, max_velocity) @ publish_hz → q_des
```

旋钮写在 **`configs/local.yaml` → `command:`**（用户层）。不要改 `hands/*.yaml` 内层 EMA（已关）。

## 推荐起点（xhand1）

| 键 | 库默认（索引 yaml） | 本机 example |
|----|---------------------|--------------|
| `filter_cutoff_hz` | 8.0 | **10.0**（略跟手） |
| `max_velocity` | 6.0 | **6.0** |
| `teleop.publish_hz` | 120 | **120** |

Linker / Wuji：同一 `command:` 键；vmax 库默认 4 / 1.5，先别为顺畅度改 solver。

## A/B 协议

1. 固定动作：张合掌 5s + 捏合 5s（左右各一轮）。
2. 每次只改一个旋钮（fc 或 vmax）。
3. 探针：

```bash
python3 scripts/teleop_probe.py --sec 5 --csv /tmp/smooth_a.csv
```

4. 记录：`|d2q| p99`、`|dq| p99`、体感（台阶 / 拖尾）。
5. 验收：`|d2q| p99 ≤ 0.05`；**无明显台阶** 优先于再压 `|d2q|`。

方向直觉：

- `fc`↑（→12）更跟手，易毛刺；`fc`↓（→6）更滑，易滞后。
- `vmax`↓ 少台阶但拖尾；`vmax`↑ 更跟手易抖。
- 不要把 `publish_hz` 拉回 250 来「治」顺畅度（会抢 EE GIL）。
- 解算过密会把 spawn worker 打到数百分 CPU：保持 `retarget_hz≈80`，需要再省再用 `max_iterations`。

定参后写回 `configs/local.yaml` 的 `command:` / `teleop.*`。

## Worker CPU（与顺畅度正交）

Process B 默认：`OMP/OPENBLAS_NUM_THREADS=1` + 绑最后 2 核。  
验收：htop 里 `multiprocessing.spawn` 子进程宜 **≲150–200%**（约 1–2 核），EE 主进程仍可单独占一核。
