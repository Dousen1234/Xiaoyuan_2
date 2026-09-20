#!/usr/bin/env python3
"""
@file plot_motor_log.py
@brief 将 motor_log.csv 数据绘制成曲线图。

规则：
  - 每个物理量（位置/速度/电流/错误）各生成一张图；
  - 不同 CAN ID 的电机用不同颜色区分；
  - 所有图的横轴均为时间（秒）。

特殊处理：
  - 位置图：叠加各电机的目标位置曲线（与实测同色的虚线）做对照，目标轨迹复现自
    examples/taihu_singleleg_step.cpp 的单腿步态；
  - 电流图：额外绘制四个电机瞬时电流之和的曲线。

用法:
  python3 tools/plot_motor_log.py [输入CSV] [-o 输出目录]

默认:
  输入 record/motor_log.csv，输出图片到 record/ 目录。

依赖: python3 + matplotlib
"""

import argparse
import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")  # 无界面后端
import matplotlib.pyplot as plt

# 每个物理量：名称、单位、在 vals（[pos, vel, cur, vol, err]）中的索引
FIELDS = [
    ("position", "rad",   0),   # 位置
    ("velocity", "rad/s", 1),   # 速度
    ("current",  "mA",    2),   # 电流
    ("error",    "",      4),   # 错误状态位
]

# 不同电机 ID 的曲线颜色（循环使用）
COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]

# —— 目标轨迹参数（复现自 examples/taihu_singleleg_step.cpp） ——
STEP_SEC = 10.0                                 # 每个阶段时长 t (s)，须与 cpp 中 kStepSec 一致
# 6 个阶段的关键帧序列（度，ID1/2/3/4）：单段阶段 2 个关键帧，往复阶段 4 个关键帧
PHASE_KEYFRAMES = [
    [(0.0, 0.0, 0.0, 0.0),                      # 阶段 0：回零保持
     (0.0, 0.0, 0.0, 0.0)],
    [(0.0,    0.0,   0.0,   0.0),               # 阶段 1：ID2 -180°，ID3 +360°
     (0.0, -180.0, 360.0,   0.0)],
    [(0.0, -180.0, 360.0, 0.0),                 # 阶段 2：ID1 往复 0→60→-60→0
     (60.0, -180.0, 360.0, 0.0),
     (-60.0, -180.0, 360.0, 0.0),
     (0.0, -180.0, 360.0, 0.0)],
    [(0.0, -180.0, 360.0, 0.0),                 # 阶段 3：ID2/3 反向回零
     (0.0,    0.0,   0.0, 0.0)],
    [(0.0, 0.0, 0.0,   0.0),                    # 阶段 4：ID4 往复 0→90→-90→0
     (0.0, 0.0, 0.0,  90.0),
     (0.0, 0.0, 0.0, -90.0),
     (0.0, 0.0, 0.0,   0.0)],
    [(0.0, 0.0, 0.0, 0.0),                      # 阶段 5：回零保持
     (0.0, 0.0, 0.0, 0.0)],
]
NUM_PHASES = len(PHASE_KEYFRAMES)
TOTAL_SEC = STEP_SEC * NUM_PHASES
DEG_TO_RAD = math.pi / 180.0


def quintic_smoothstep(t):
    """五次多项式平滑：s(0)=0、s(1)=1，且一阶、二阶导数在两端为 0。"""
    return 6.0 * t ** 5 - 15.0 * t ** 4 + 10.0 * t ** 3


def compute_target(t_rel):
    """复现 taihu_singleleg_step.cpp 的 computeTarget，返回 4 元组目标角度（度）。"""
    if t_rel < 0.0:
        return PHASE_KEYFRAMES[0][0]
    if t_rel >= TOTAL_SEC:
        return PHASE_KEYFRAMES[-1][-1]
    phase = int(t_rel / STEP_SEC)
    kf = PHASE_KEYFRAMES[phase]
    n_seg = len(kf) - 1
    seg_sec = STEP_SEC / n_seg
    t_phase = min(t_rel - phase * STEP_SEC, STEP_SEC - 1e-9)
    seg = min(int(t_phase / seg_sec), n_seg - 1)
    frac = (t_phase - seg * seg_sec) / seg_sec
    s = quintic_smoothstep(frac)
    a, b = kf[seg], kf[seg + 1]
    return tuple(a[i] + (b[i] - a[i]) * s for i in range(4))


def main() -> int:
    parser = argparse.ArgumentParser(description="将 motor_log.csv 绘制成曲线图")
    parser.add_argument("input", nargs="?",
                        default="record/motor_log.csv",
                        help="CSV 文件路径（默认 record/motor_log.csv）")
    parser.add_argument("-o", "--output-dir", default="record",
                        help="输出图片目录（默认 record）")
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"[错误] 找不到文件 {args.input}", file=sys.stderr)
        return 1

    os.makedirs(args.output_dir, exist_ok=True)

    # 解析 CSV：series[id] = [(time, [pos, vel, cur, vol, err]), ...]
    series = {}
    with open(args.input, newline="") as f:
        reader = csv.reader(f)
        next(reader, None)  # 跳过表头
        for row in reader:
            if not row or row[0] == "":
                continue
            t = float(row[0])
            n = (len(row) - 1) // 6
            for i in range(n):
                base = 1 + i * 6
                mid = int(row[base])
                vals = [float(row[base + j]) for j in range(1, 6)]
                series.setdefault(mid, []).append((t, vals))

    motor_ids = sorted(series.keys())
    if not motor_ids:
        print("[错误] CSV 中没有解析到任何电机数据", file=sys.stderr)
        return 1
    print(f"[信息] 解析到电机 ID: {motor_ids}")

    # 时间轴（各电机同一次采样对齐，取第一个电机的时间轴）
    ts = [p[0] for p in series[motor_ids[0]]]

    for name, unit, idx in FIELDS:
        fig, ax = plt.subplots(figsize=(10, 5))

        if name == "position":
            # 实测位置（实线）+ 目标位置（同色虚线）对照
            for k, mid in enumerate(motor_ids):
                color = COLORS[k % len(COLORS)]
                ys = [p[1][idx] for p in series[mid]]
                ax.plot(ts, ys, color=color, label=f"ID {mid}", linewidth=1.2)
                # 目标轨迹按 ID 顺序 1/2/3/4 对应索引 0/1/2/3
                ti = mid - 1
                tgt = [compute_target(t)[ti] * DEG_TO_RAD for t in ts]
                ax.plot(ts, tgt, color=color, linestyle="--",
                        linewidth=1.0, label=f"ID {mid} target")
        elif name == "current":
            # 各电机电流（实线）+ 四电机瞬时电流之和（加粗黑线）
            for k, mid in enumerate(motor_ids):
                ys = [p[1][idx] for p in series[mid]]
                ax.plot(ts, ys, color=COLORS[k % len(COLORS)],
                        label=f"ID {mid}", linewidth=1.2)
            total = [sum(series[mid][i][1][idx] for mid in motor_ids)
                     for i in range(len(ts))]
            ax.plot(ts, total, color="black", linewidth=2.0, label="Total")
        else:
            for k, mid in enumerate(motor_ids):
                ys = [p[1][idx] for p in series[mid]]
                ax.plot(ts, ys, color=COLORS[k % len(COLORS)],
                        label=f"ID {mid}", linewidth=1.2)

        ax.set_xlabel("Time (s)")
        ylabel = f"{name.capitalize()} ({unit})" if unit else name.capitalize()
        ax.set_ylabel(ylabel)
        ax.set_title(f"{name.capitalize()} vs Time")
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.legend()
        fig.tight_layout()

        out = os.path.join(args.output_dir, f"{name}.png")
        fig.savefig(out, dpi=150)
        plt.close(fig)
        print(f"[信息] 已生成 {out}")

    print("[信息] 绘图完成")
    return 0


if __name__ == "__main__":
    sys.exit(main())
