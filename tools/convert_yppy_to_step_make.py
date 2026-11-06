#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把「YPPY 关节轨迹」格式的 xlsx 转换成上位机 up_walk_control_V1.py 可直接读取的
step_make 格式（step_make_V2.xlsx）。

用法：
    python3 tools/convert_yppy_to_step_make.py
    python3 tools/convert_yppy_to_step_make.py \
        --src step_position/YPPY_joint_trajectories_0p1s_with_10s_transition.xlsx \
        --dst step_position/step_make_V2.xlsx

输入格式（源文件自带两张表，本脚本用 Angles_rad）：
    表名 Angles_rad
    Time_s | LF_Y1_rad LF_P2_rad LF_P3_rad LF_Y4_rad | RF_… | LR_… | RR_…

输出格式（与 step_position/step_make.xlsx 完全一致）：
    表名 Joint samples
    Time (s) | L1_J1 L1_J2 L1_J3 L1_J4 | L2_… | L3_… | L4_J1 … L4_J4
    单位：关节端 rad，正方向 = 逆时针（ccw）

两处必要转换
------------
1) 表头改名：源文件用 LF/RF/LR/RR + Y/P 命名，上位机只认 L{腿}_J{关节}。
   腿序映射按现场接线确认：LF→L1、RF→L2、LR→L3、RR→L4。
   关节序：Y1→J1、P2→J2、P3→J3、Y4→J4。

2) J1 限幅：源轨迹的 Y1（髋偏航）峰值到 ±139°，而本机 J1 是直驱、软件限位
   ±120°（见 examples/walk_control_V1.cpp 的 kJointLimitDeg）。下位机对越限帧
   逐帧丢弃，不裁剪的话 J1 会在 t≈11 s 后一直卡在限位附近。这里按用户确认的
   方案**饱和裁剪**到 ±120°（超出的采样点削平在限位上，位置不变、时间不改）。

运行结束后会打印每列的裁剪点数与限位核对结果。
"""

import argparse
import math
import os
import sys

try:
    import openpyxl
except ImportError:
    sys.exit("缺少 openpyxl，请先安装：pip3 install --user --break-system-packages openpyxl")

# ---------------------------------------------------------------------------
# 常量：与 examples/walk_control_V1.cpp / tools/up_walk_control_V1.py 保持一致
# ---------------------------------------------------------------------------

#: 源文件前缀 → 下位机腿号（按现场接线：LF→L1、RF→L2、LR→L3、RR→L4）
LEG_MAP = {"LF": 1, "RF": 2, "LR": 3, "RR": 4}

#: 源文件关节标识 → 关节序号
JOINT_MAP = {"Y1": 1, "P2": 2, "P3": 3, "Y4": 4}

#: 关节端软件限位（度），与下位机 kJointLimitDeg 严格一致
JOINT_LIMIT_DEG = {
    1: (-120.0, 120.0),   # J1 Y 直驱
    2: (-120.0, 120.0),   # J2 P 锥齿轮 1:2
    3: (-30.0, 180.0),    # J3 P 锥齿轮 1:2
    4: (-90.0, 90.0),     # J4 Y 直驱
}

#: 输出表名与表头（与 step_make.xlsx 一致，up_walk_control_V1.py 按名识别）
OUT_SHEET = "Joint samples"
OUT_TIME_HDR = "Time (s)"


def build_header():
    """生成 ['Time (s)', 'L1_J1', …, 'L4_J4'] 共 17 个表头。"""
    hdr = [OUT_TIME_HDR]
    for leg in range(1, 5):
        for joint in range(1, 5):
            hdr.append("L%d_J%d" % (leg, joint))
    return hdr


def parse_src_header(header):
    """
    解析源表头，返回 (time_col, [(目标列序号, 腿号, 关节号, 源表头名), …])。

    源表头形如 'LF_Y1_rad'：按 '_' 切开，第 1 段是腿、第 2 段是关节标识。
    无法识别的列（如 'Time_s'）跳过。
    """
    time_col = None
    cols = []
    for ci, name in enumerate(header):
        if not name:
            continue
        text = str(name).strip()
        if time_col is None and text.lower().startswith(("time", "t", "时间")):
            time_col = ci
            continue
        parts = text.split("_")
        if len(parts) < 2:
            continue
        leg_key = parts[0].upper()
        joint_key = parts[1].upper()
        if leg_key not in LEG_MAP or joint_key not in JOINT_MAP:
            continue
        if len(parts) >= 3 and parts[2].lower() == "deg":
            continue                       # 度制列跳过，输出统一用 rad
        cols.append((ci, LEG_MAP[leg_key], JOINT_MAP[joint_key], text))
    if time_col is None:
        time_col = 0
    if not cols:
        raise RuntimeError("源表头里没有识别到任何 YPPY 关节列，实际表头：%s"
                           % (" | ".join(str(h) for h in header)))
    return time_col, cols


def convert(src, dst, src_sheet="Angles_rad"):
    """读取源文件、换表头、裁剪 J1、写出 step_make 格式。返回统计字典。"""
    if not os.path.isfile(src):
        raise RuntimeError("找不到源文件：%s" % src)

    wb_in = openpyxl.load_workbook(src, data_only=True, read_only=True)
    if src_sheet not in wb_in.sheetnames:
        raise RuntimeError("源文件没有表 %r，现有表：%s"
                           % (src_sheet, ", ".join(wb_in.sheetnames)))
    rows = list(wb_in[src_sheet].iter_rows(values_only=True))
    wb_in.close()
    if len(rows) < 2:
        raise RuntimeError("源表 %r 没有数据行" % src_sheet)

    header = rows[0]
    time_col, cols = parse_src_header(header)

    # 目标关节 ID（1~16）→ (源列序号, 源列名)
    slot = {}
    for ci, leg, joint, name in cols:
        jid = (leg - 1) * 4 + joint
        if jid in slot:
            raise RuntimeError("源表头冲突：%r 与 %r 都映射到 L%d_J%d"
                               % (name, slot[jid][1], leg, joint))
        slot[jid] = (ci, name)
    missing = [jid for jid in range(1, 17) if jid not in slot]

    # 逐行换算
    out_times = []
    out_frames = []
    clip_count = {jid: 0 for jid in range(1, 17)}
    for r_i, row in enumerate(rows[1:], start=2):
        if row is None:
            continue
        t_val = row[time_col] if time_col < len(row) else None
        if t_val is None or (isinstance(t_val, str) and not t_val.strip()):
            continue                       # 跳过空行
        try:
            t = float(t_val)
        except (TypeError, ValueError):
            raise RuntimeError("第 %d 行时间列无法解析为数字：%r" % (r_i, t_val))

        frame = []
        for jid in range(1, 17):
            if jid not in slot:
                frame.append(0.0)          # 源文件缺的关节补 0（零位）
                continue
            ci, name = slot[jid]
            if ci >= len(row) or row[ci] is None or \
                    (isinstance(row[ci], str) and not row[ci].strip()):
                frame.append(0.0)
                continue
            try:
                val = float(row[ci])
            except (TypeError, ValueError):
                raise RuntimeError("第 %d 行 %s（第 %d 列）无法解析为数字：%r"
                                   % (r_i, name, ci + 1, row[ci]))

            # 限位饱和裁剪（关节端，度）
            joint = (jid - 1) % 4 + 1
            lo_deg, hi_deg = JOINT_LIMIT_DEG[joint]
            deg = math.degrees(val)
            if deg < lo_deg:
                val = math.radians(lo_deg)
                clip_count[jid] += 1
            elif deg > hi_deg:
                val = math.radians(hi_deg)
                clip_count[jid] += 1
            frame.append(val)

        out_times.append(t)
        out_frames.append(frame)

    if not out_times:
        raise RuntimeError("源表 %r 没有解析到任何采样点" % src_sheet)

    # 写出
    wb_out = openpyxl.Workbook()
    ws = wb_out.active
    ws.title = OUT_SHEET
    ws.append(build_header())
    for t, frame in zip(out_times, out_frames):
        ws.append([t] + frame)
    wb_out.save(dst)
    wb_out.close()

    return {
        "n": len(out_times),
        "t0": out_times[0],
        "t1": out_times[-1],
        "dt": out_times[1] - out_times[0] if len(out_times) > 1 else 0.0,
        "missing": missing,
        "clip_count": clip_count,
        "minmax": [(min(f[i] for f in out_frames),
                    max(f[i] for f in out_frames)) for i in range(16)],
    }


def main():
    ap = argparse.ArgumentParser(
        description="YPPY 关节轨迹 xlsx → step_make 格式（上位机可直读）")
    ap.add_argument("--src", default="step_position/"
                    "YPPY_joint_trajectories_0p1s_with_10s_transition.xlsx",
                    help="源文件路径")
    ap.add_argument("--sheet", default="Angles_rad",
                    help="源工作表名（默认 Angles_rad；Angles_deg 也可，但输出会被"
                         "当作 rad 使用，必须用 rad 表）")
    ap.add_argument("--dst", default="step_position/step_make_V2.xlsx",
                    help="输出文件路径（默认 step_position/step_make_V2.xlsx）")
    args = ap.parse_args()

    if "deg" in args.sheet.lower():
        sys.exit("错误：源表必须是弧度制（Angles_rad）。Angles_deg 表里是度数，"
                 "直接当 rad 用会导致关节目标被放大 57 倍。")

    try:
        st = convert(args.src, args.dst, args.sheet)
    except RuntimeError as exc:
        sys.exit("转换失败：%s" % exc)

    print("源文件：%s [表 %s]" % (args.src, args.sheet))
    print("输出  ：%s [表 %s]" % (args.dst, OUT_SHEET))
    print("采样点：%d 个，t = %.1f → %.1f s，时间步 %.3f s（≈ %.2f Hz）"
          % (st["n"], st["t0"], st["t1"], st["dt"],
             (1.0 / st["dt"]) if st["dt"] > 0 else 0.0))
    if st["missing"]:
        print("源文件缺列（已补 0.0 rad）：%s"
              % ", ".join("ID%d" % j for j in st["missing"]))
    else:
        print("源文件 16 个关节列齐全。")

    print("\n限位核对（裁剪后）：")
    print("  %-7s %10s %10s %16s %8s" % ("列", "min(°)", "max(°)", "限位(°)", "裁剪点"))
    ok = True
    for i in range(16):
        jid = i + 1
        leg = (jid - 1) // 4 + 1
        joint = (jid - 1) % 4 + 1
        lo, hi = JOINT_LIMIT_DEG[joint]
        mn = math.degrees(st["minmax"][i][0])
        mx = math.degrees(st["minmax"][i][1])
        if mn < lo - 1e-6 or mx > hi + 1e-6:
            ok = False
        print("  L%d_J%-4d %10.3f %10.3f %16s %8d"
              % (leg, joint, mn, mx, "[%g, %g]" % (lo, hi), st["clip_count"][jid]))
    print("\n限位检查：%s" % ("✓ 全部目标都在软件限位内" if ok else "✗ 仍有越限"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())