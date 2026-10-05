#!/usr/bin/env python3
"""
@file plot_motor_log.py
@brief 将 motor_log.csv 数据绘制成曲线图。

规则：
  - 每个物理量（位置/速度/电流/错误）各生成一张图；
  - 不同 CAN ID 的电机用不同颜色区分；
  - 所有图的横轴均为时间（秒）。

特殊处理：
  - 位置图：叠加各电机的目标位置曲线（与实测同色的虚线）做对照。目标轨迹支持两种步态模型：
      singleleg：复现 examples/taihu_singleleg_step.cpp 的 6 阶段单腿步态（每阶段 10 s）；
      xcan     ：复现 examples/taihu_singleleg_step_xCAN.cpp 的一步 3 阶段步态
                 （零位保持 / ID2,3 0→+180° / ID2,3 +180°→0，每阶段 5 s，重复 N 步）；
  - 步态模型默认自动识别：文件名含 "_can"（如 motor_log_can0.csv）用 xcan，否则用 singleleg；
  - 电流图：额外绘制四个电机瞬时电流之和的曲线。

用法:
  python3 tools/plot_motor_log.py [输入CSV] [-o 输出目录] [--gait {auto,singleleg,xcan}]
                                  [--phase-sec 秒] [--num-steps N] [--total-sec 秒]

示例:
  # 单总线单腿步态日志（自动识别为 singleleg）
  python3 tools/plot_motor_log.py record/motor_log.csv
  # xCAN 多总线日志（自动识别为 xcan，默认 N=1）；运行时走了 3 步则指定 --num-steps 3
  python3 tools/plot_motor_log.py record/motor_log_can0.csv --num-steps 3

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

# —— 目标轨迹参数 ——
DEG_TO_RAD = math.pi / 180.0

# 模型 1：singleleg（复现自 examples/taihu_singleleg_step.cpp）
# 6 个阶段 × 10 s；每个阶段的关键帧序列（度，ID1/2/3/4）：单段阶段 2 个关键帧，往复阶段 4 个关键帧
SINGLELEG_PHASE_SEC = 10.0                      # 每个阶段时长 (s)，须与 cpp 中 kStepSec 一致
SINGLELEG_PHASES = [
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

# 模型 2：xcan（复现自 examples/taihu_singleleg_step_xCAN.cpp）
# 一步 = 3 个等长阶段：零位保持 / ID2,3 0→+180° / ID2,3 +180°→0；多步循环复用，重复 N 次
XCAN_PHASE_SEC = 5.0                            # 每个阶段时长 (s)，须与 cpp 中 kPhaseSec 一致
XCAN_STEP_PHASES = [
    [(0.0,   0.0,   0.0, 0.0),                  # 阶段 0：四电机零位保持不动
     (0.0,   0.0,   0.0, 0.0)],
    [(0.0,   0.0,   0.0, 0.0),                  # 阶段 1：ID1/4 不动，ID2/3 0°→+180°
     (0.0, 180.0, 180.0, 0.0)],
    [(0.0, 180.0, 180.0, 0.0),                  # 阶段 2：ID1/4 不动，ID2/3 +180°→0°
     (0.0,   0.0,   0.0, 0.0)],
]
XCAN_PHASES_PER_STEP = len(XCAN_STEP_PHASES)    # 每步阶段数（3）


def quintic_smoothstep(t):
    """五次多项式平滑：s(0)=0、s(1)=1，且一阶、二阶导数在两端为 0。"""
    return 6.0 * t ** 5 - 15.0 * t ** 4 + 10.0 * t ** 3


def interp_phases(phase_list, phase_sec, total_sec, t_rel):
    """通用关键帧插值：复现两个 cpp 的 computeTarget 逻辑，返回 4 元组目标角度（度）。

    phase_list: 阶段列表，每个阶段是关键帧列表（4 元组，度）；
    phase_sec : 每个阶段时长 (s)；total_sec: 总时长 (s)。
    """
    if t_rel < 0.0:
        return phase_list[0][0]
    if t_rel >= total_sec:
        return phase_list[-1][-1]
    phase = min(int(t_rel / phase_sec), len(phase_list) - 1)
    kf = phase_list[phase]
    n_seg = len(kf) - 1
    seg_sec = phase_sec / n_seg
    t_phase = min(t_rel - phase * phase_sec, phase_sec - 1e-9)
    seg = min(int(t_phase / seg_sec), n_seg - 1)
    frac = (t_phase - seg * seg_sec) / seg_sec
    s = quintic_smoothstep(frac)
    a, b = kf[seg], kf[seg + 1]
    return tuple(a[i] + (b[i] - a[i]) * s for i in range(4))


def resolve_gait(gait, phase_sec_arg, num_steps, total_sec_arg):
    """按步态模型构造目标轨迹函数，返回 (target_fn, phase_sec, total_sec)。

    target_fn: t_rel -> 4 元组目标角度（度）。
    phase_sec_arg / total_sec_arg 为 None 时使用模型默认值。
    """
    num_steps = max(1, num_steps)
    if gait == "singleleg":
        phase_sec = SINGLELEG_PHASE_SEC if phase_sec_arg is None else phase_sec_arg
        phase_list = SINGLELEG_PHASES
    else:  # xcan：一步的 3 个阶段重复 num_steps 次（首尾姿态重合，衔接连续）
        phase_sec = XCAN_PHASE_SEC if phase_sec_arg is None else phase_sec_arg
        phase_list = XCAN_STEP_PHASES * num_steps
    total_sec = phase_sec * len(phase_list)
    if total_sec_arg is not None:
        total_sec = total_sec_arg
    return (lambda t: interp_phases(phase_list, phase_sec, total_sec, t)), phase_sec, total_sec


def main() -> int:
    parser = argparse.ArgumentParser(description="将 motor_log.csv 绘制成曲线图")
    parser.add_argument("input", nargs="?",
                        default="record/motor_log.csv",
                        help="CSV 文件路径（默认 record/motor_log.csv）")
    parser.add_argument("-o", "--output-dir", default="record",
                        help="输出图片目录（默认 record）")
    parser.add_argument("--gait", choices=["auto", "singleleg", "xcan"], default="auto",
                        help="目标轨迹步态模型（默认 auto：文件名含 _can 用 xcan，否则 singleleg）")
    parser.add_argument("--phase-sec", type=float, default=None,
                        help="每个阶段时长 (s)；默认 singleleg=10.0、xcan=5.0，须与 cpp 中 kStepSec/kPhaseSec 一致")
    parser.add_argument("--num-steps", type=int, default=1,
                        help="xcan 步态的步数 N（默认 1，须与运行时输入一致）")
    parser.add_argument("--total-sec", type=float, default=None,
                        help="显式指定总时长 (s)，优先于阶段时长×阶段数计算")
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"[错误] 找不到文件 {args.input}", file=sys.stderr)
        return 1

    os.makedirs(args.output_dir, exist_ok=True)

    # 解析步态模型（auto：motor_log_canN.csv 之类的多总线日志用 xcan，其余用 singleleg）
    gait = args.gait
    if gait == "auto":
        gait = "xcan" if "_can" in os.path.basename(args.input) else "singleleg"
    compute_target, phase_sec, total_sec = resolve_gait(
        gait, args.phase_sec, args.num_steps, args.total_sec)
    print(f"[信息] 步态模型: {gait}，阶段时长 {phase_sec:g} s，总时长 {total_sec:g} s")

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
