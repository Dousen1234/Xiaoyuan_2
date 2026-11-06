#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成 3588_control 用的示例轨迹文件 step_sample.xlsx

表头格式（17 列，与 upper_control 期望完全一致）：
    Time | A1 A2 A3 A4 | B1 B2 B3 B4 | C1 C2 C3 C4 | D1 D2 D3 D4

单位：时间 = 秒，关节角 = 度（deg）。
若想用弧度，可把表头写成 A1_rad 之类，或用 --unit rad 强制。

用法：
    python3 make_sample_xlsx.py
    python3 make_sample_xlsx.py --out my_traj.xlsx --hz 20 --samples 200
"""

import argparse
import math
import sys

try:
    from openpyxl import Workbook
except ImportError:
    print("[错误] 需要 openpyxl：pip3 install openpyxl", file=sys.stderr)
    sys.exit(1)

LEGS = ["A", "B", "C", "D"]
DEFAULT_HOME = {
    "J1": 0.0,
    "J2": 0.0,
    "J3": 0.0,
    "J4": 0.0,
}


def build_trajectory(hz: float, samples: int, amp_deg: float):
    """生成一条温和的正弦摆动轨迹（所有关节同步、相位按腿臂错开）。"""
    dt = 1.0 / hz
    rows = []
    for i in range(samples):
        t = i * dt
        # 前 1 秒静止在原位，便于观察使能保持是否稳定
        ramp = min(1.0, max(0.0, (t - 1.0) / 1.5))
        s = 6 * ramp**5 - 15 * ramp**4 + 10 * ramp**3

        row = [round(t, 4)]
        for li, leg in enumerate(LEGS):
            phase = li * math.pi / 4.0
            for ji, jname in enumerate(["J1", "J2", "J3", "J4"]):
                # J3 的限位不对称（-30~+180），只用正向小幅摆动
                if ji == 2:
                    val = DEFAULT_HOME[jname] + s * amp_deg * (0.5 + 0.5 * math.sin(2 * math.pi * 0.25 * t + phase))
                else:
                    val = DEFAULT_HOME[jname] + s * amp_deg * math.sin(2 * math.pi * 0.25 * t + phase + ji * math.pi / 4.0)
                row.append(round(val, 4))
        rows.append(row)
    return rows


def main():
    ap = argparse.ArgumentParser(description="生成 3588_control 示例轨迹 xlsx")
    ap.add_argument("--out", default="step_sample.xlsx", help="输出文件名")
    ap.add_argument("--hz", type=float, default=10.0, help="采样频率（默认 10Hz）")
    ap.add_argument("--samples", type=int, default=100, help="样点数量（默认 100）")
    ap.add_argument("--amp", type=float, default=10.0, help="摆幅（度，默认 10）")
    args = ap.parse_args()

    header = ["Time"]
    for leg in LEGS:
        for j in range(1, 5):
            header.append(f"{leg}{j}")

    rows = build_trajectory(args.hz, args.samples, args.amp)

    wb = Workbook()
    ws = wb.active
    ws.title = "Joint samples"
    ws.append(header)
    for r in rows:
        ws.append(r)

    wb.save(args.out)
    dur = (args.samples - 1) / args.hz
    print(f"[完成] {args.out}")
    print(f"       样点 {args.samples} 条，时长 {dur:.2f}s，频率 {args.hz}Hz，摆幅 ±{args.amp}°")
    print(f"       表头 {len(header)} 列：{' | '.join(header)}")


if __name__ == "__main__":
    main()