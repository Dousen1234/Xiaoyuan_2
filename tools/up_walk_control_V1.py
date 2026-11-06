#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    up_walk_control_V1.py
@brief   上位机调试程序：读取 step_make.xlsx 的关节轨迹，以 10Hz 通过 CAN 发送给下位机
         `examples/walk_control_V1`，由下位机拆分到 can0~can3 的 16 个电机执行。

轨迹文件格式（表头 L{腿}_J{关节}，单位 = 关节端 rad，正 = 逆时针 ccw）：
    Time (s) | L1_J1 L1_J2 L1_J3 L1_J4 | L2_J1 … | L3_J1 … | L4_J1 … L4_J4
列→帧 ID 映射由表头自动识别：ID = (腿-1)×4 + 关节，与下位机拆分规则一致。

角色分工（与 docs/step_protocol.pdf V2.0 一致）：
    上位机（本程序，另一台电脑或本机另一路 CAN）
        └─ 读 step_make.xlsx → 10Hz 发 ID 1~16 关节目标 → 收反馈/心跳
    下位机（build/walk_control_V1）
        └─ 收 ID 1~16 → 拆分到 4 条电机总线 → 5ms 重发 → 电机固件内 PD 执行

帧格式（与钛虎电机写命令帧同构）：
    上位机 → 下位机   ID 1~16, DLC=5  [0x1E][int32 小端 = 关节端角(rad) × 10000]
                     ID 1~16, DLC=1  [0x02]  退出流模式（电机保持，不抱闸）
                     ID 1~16, DLC=1  [0x03]  清除软限位报警锁定
                     ID 1~16, DLC=1  [0x41]  状态请求 → 下位机立即回该 ID 实际角
    下位机 → 上位机   ID 1~16, DLC=5  [0x41][int32 小端 = 实际关节端角(rad) × 10000]，10Hz
                     ID 0x161, DLC=1  心跳 200ms（bit0=流模式中，bit1=最近 1s 有样点，
                                                bit2=软限位报警锁定中）
                     ID 0x162, DLC=8  软限位报警 200ms（锁定期间重发）
                                       [0x62][关节ID 1~16][侧别][0][int32=当前实际角]
                                       侧别：0x01=上限，0x02=下限

软限位保护（下位机行为）：
    目标越限不再丢弃，而是**钳位到限位值**下发（关节走到限位并停住）；
    当**实际角**到达限位时，下位机整机锁定并通过 ID 0x162 上报报警，
    本程序收到后立即停止发送（可加 --keep-on-limit 改为仅告警继续）。
    解除方式：下位机键盘 clear，或本程序启动新一次回放时自动发 [0x03]。

使用方法（两个终端）：
    终端 A（下位机）：
        sudo ./build/kcanctl up can0 1000000    # 按实际接线开 can0~can3
        sudo ./build/kcanctl up can4 1000000    # 上位机链路（独立接口）
        ./build/walk_control_V1                 # 腿号回车=全部；上位机接口输 can4
    终端 B（上位机，本程序）：
        python3 tools/up_walk_control_V1.py --can can4

    启动后本程序**先只做链路检测**（被动收心跳 + 主动 ping），确认通讯打通并等用户
    按回车后，才开始发送关节目标；发送完毕自动发 0x02 退出流模式。

无硬件自测：
        python3 tools/up_walk_control_V1.py --dry-run      # 只解析 xlsx + 模拟节拍

依赖：python3 标准库（socket/select/struct，原生 SocketCAN，**不需要 python-can**）
      + openpyxl（读 xlsx：`pip3 install --user openpyxl`）
"""

import argparse
import os
import re
import select
import signal
import socket
import struct
import sys
import threading
import time

# =============================================================================
# 协议常量（★ 必须与 examples/walk_control_V1.cpp 严格一致，改动需两边同步）
# =============================================================================

ANGLE_SCALE = 10000.0        # int32 定点标度：协议值 = 关节端角(rad) × 10000
CMD_SET_POS = 0x1E           # 上位机→下位机：目标位置（DLC=5）
CMD_STOP = 0x02              # 上位机→下位机：退出流模式（DLC=1）
CMD_CLEAR_LIMIT = 0x03       # 上位机→下位机：清除软限位报警锁定（DLC=1）
CMD_GET_STAT = 0x41          # 双向：状态请求（DLC=1）/ 状态反馈（DLC=5）
HEARTBEAT_ID = 0x161         # 下位机→上位机：心跳帧 ID
HEARTBEAT_MS = 200           # 下位机心跳周期（ms）

# 软限位报警（下位机→上位机）
LIMIT_ALARM_ID = 0x162       # 报警帧 ID（避开关节 ID 1~16 与心跳 0x161）
LIMIT_ALARM_CODE = 0x62      # 报警帧 data[0] 功能码
LIMIT_ALARM_MS = 200         # 报警帧重发周期（ms）
LIMIT_SIDE_MAX = 0x01        # data[2]：撞上限
LIMIT_SIDE_MIN = 0x02        # data[2]：撞下限
HB_BIT_STREAMING = 0x01      # 心跳 bit0：流模式中
HB_BIT_SAMPLE_ONLINE = 0x02  # 心跳 bit1：最近 1s 有样点
HB_BIT_LIMIT_LOCKED = 0x04   # 心跳 bit2：软限位报警锁定中

LEG_COUNT = 4                # 腿数
JOINT_PER_LEG = 4            # 每腿关节数
TOTAL_JOINTS = LEG_COUNT * JOINT_PER_LEG    # 全局关节数 = 帧 ID 数 = 16
JOINT_ID_BASE = 1            # 帧 ID 起始值（ID = 基址 + (腿-1)×4 + 关节-1）

DEG_PER_RAD = 57.29577951308232   # 180/π，仅用于打印显示
DEFAULT_HZ = 10.0            # 目标流频率（协议约定 10Hz）
HOST_STREAM_TIMEOUT_S = 1.0  # 下位机断流门限：超时无样点即退出流模式
HB_LOST_S = 1.0              # 上位机判定心跳丢失的门限（= 5 个心跳周期）

# 软件限位（**关节端**，度）：★ 与下位机 kJointLimitDeg 严格一致，用于**离线预检查**，
# 提前发现会被下位机丢弃的越限目标（下位机对越限帧直接丢弃并计数）。
#
# 关节端 → 电机端换算（下位机 kMotorRadPerJointRad = {1, 2, 2, 1}）：
#   J1 直驱      ×1：电机端 ±120°
#   J2 锥齿轮1:2 ×2：关节端 ±120°      ↔ 电机端 ±240°
#   J3 锥齿轮1:2 ×2：关节端 −60°~+180° ↔ 电机端 −120°~+360°
#   J4 直驱      ×1：电机端 ±90°
JOINT_LIMIT_DEG = {
    1: (-120.0, 120.0),   # J1（Y 直驱）
    2: (-120.0, 120.0),   # J2（P 锥齿轮 1:2，电机端 ±240°）
    3: (-60.0, 180.0),    # J3（P 锥齿轮 1:2，电机端 −120°~+360°）
    4: (-90.0, 90.0),     # J4（Y 直驱）
}

# 关节方向符号（+1 = 关节端正方向对应电机端正方向）。
# ★ 正方向定义与 taihu_singleleg_step_xCAN 一致：正角 = 逆时针（ccw）。
#   该 demo 的关键帧 {0, 180, 180, 0} 中 ID2/ID3 同为 **+180°**，说明 P 型
#   锥齿轮（J2/J3）在电机端同向；Y 型直驱（J1/J4）无需换算。
#   与下位机 kJointDirSign = {1,1,1,1} 一致；上位机不做符号翻转，只做限位预检查。
JOINT_DIR_SIGN = {1: 1.0, 2: 1.0, 3: 1.0, 4: 1.0}

# =============================================================================
# SocketCAN 底层（struct can_frame：__u32 can_id + __u8 len + 3B 填充 + 8B data）
# =============================================================================

CAN_FRAME_FMT = "<IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)   # = 16 字节

CAN_EFF_FLAG = getattr(socket, "CAN_EFF_FLAG", 0x80000000)  # 扩展帧
CAN_RTR_FLAG = getattr(socket, "CAN_RTR_FLAG", 0x40000000)  # 远程帧
CAN_ERR_FLAG = getattr(socket, "CAN_ERR_FLAG", 0x20000000)  # 内核错误帧
CAN_EFF_MASK = getattr(socket, "CAN_EFF_MASK", 0x1FFFFFFF)
CAN_SFF_MASK = getattr(socket, "CAN_SFF_MASK", 0x000007FF)


def pack_i32le(value):
    """int → 4 字节小端（与下位机 put_i32le 一致）。"""
    return struct.pack("<i", int(value))


def unpack_i32le(data):
    """4 字节小端 → int（符号扩展，与下位机 get_i32le 一致）。"""
    return struct.unpack("<i", bytes(data[:4]).ljust(4, b"\x00"))[0]


def rad_to_proto(rad):
    """关节端角(rad) → 协议定点值（× ANGLE_SCALE，四舍五入并饱和到 int32）。"""
    v = rad * ANGLE_SCALE
    if v > 2147483647.0:
        return 2147483647
    if v < -2147483648.0:
        return -2147483647 - 1
    return int(round(v))


def proto_to_rad(value):
    """协议定点值 → 关节端角(rad)。"""
    return value / ANGLE_SCALE


def joint_id_name(jid):
    """全局关节 ID（1~16）→ 可读名，如 3 → '腿1 J3'。"""
    leg = (jid - JOINT_ID_BASE) // JOINT_PER_LEG + 1
    j = (jid - JOINT_ID_BASE) % JOINT_PER_LEG + 1
    return "腿%d J%d" % (leg, j)


def leg_of_id(jid):
    """全局关节 ID → 腿号（1~4）。"""
    return (jid - JOINT_ID_BASE) // JOINT_PER_LEG + 1


# =============================================================================
# CAN 链路封装
# =============================================================================

class CanLink(object):
    """
    原生 SocketCAN 收发（PF_CAN + SOCK_RAW + CAN_RAW），一个实例独占一个接口。

    与下位机 `taihu::SocketCanInterface` 等价：bind 到接口名，波特率由
    `ip link set canX type can bitrate 1000000` 事先配置（本类不设置波特率）。
    """

    def __init__(self, ifname):
        self.ifname = ifname
        self.tx_count = 0
        self.rx_count = 0
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.bind((ifname,))
        except OSError as e:
            self.sock.close()
            raise OSError("绑定接口 %s 失败：%s" % (ifname, e))

    def send(self, can_id, payload):
        """发送一帧标准帧。payload 为 bytes（长度即 DLC，不足 8 字节自动补 0）。"""
        payload = bytes(payload)
        dlc = min(len(payload), 8)
        data = payload.ljust(8, b"\x00")
        frame = struct.pack(CAN_FRAME_FMT, can_id & CAN_SFF_MASK, dlc, data)
        self.sock.send(frame)
        self.tx_count += 1

    def recv(self, timeout_s):
        """
        收一帧，返回 (can_id, payload_bytes) 或 None（超时）。
        内核错误帧（CAN_ERR_FLAG）与远程帧自动跳过，与下位机 receive() 行为一致。
        """
        while True:
            rlist, _, _ = select.select([self.sock], [], [], timeout_s)
            if not rlist:
                return None
            try:
                raw = self.sock.recv(CAN_FRAME_SIZE)
            except OSError:
                return None
            if len(raw) < CAN_FRAME_SIZE:
                continue
            can_id, dlc = struct.unpack("<IB", raw[:5])
            if can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG):
                continue                      # 跳过错误帧/远程帧，继续等
            if can_id & CAN_EFF_FLAG:
                continue                      # 本协议只用标准帧，扩展帧忽略
            self.rx_count += 1
            return can_id & CAN_SFF_MASK, raw[8:8 + min(dlc, 8)]

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


# =============================================================================
# 后台接收监视线程（只收不发，维护心跳与 16 路反馈缓存）
# =============================================================================

class LinkMonitor(threading.Thread):
    """
    持续接收下位机帧，解析并缓存：
      - 心跳 0x161：计数、bit0（流模式中）、bit1（样点在线）、bit2（软限位锁定）、
        最后到达时刻；
      - 反馈 ID 1~16 [0x41][int32]：每关节实际角(rad) 与帧计数；
      - 软限位报警 0x162 [0x62][关节ID][侧别][0][int32 实际角]：报警明细与计数。

    所有字段用锁保护，主线程随时可读快照（链路检测阶段与回放阶段的实时显示都靠它）。
    """

    def __init__(self, link):
        super(LinkMonitor, self).__init__(name="LinkMonitor", daemon=True)
        self.link = link
        self.stop_event = threading.Event()
        self._lock = threading.Lock()
        self.hb_count = 0
        self.hb_last_t = 0.0
        self.hb_streaming = False       # bit0
        self.hb_sample_online = False   # bit1
        self.hb_limit_locked = False    # bit2
        self.fb_rad = [0.0] * TOTAL_JOINTS
        self.fb_count = [0] * TOTAL_JOINTS
        self.fb_last_t = 0.0
        # —— 软限位报警 ——
        self.alarm_count = 0            # 收到的 0x162 帧数
        self.alarm_first_t = 0.0        # 首帧到达时刻（monotonic）
        self.alarm_jid = 0              # 报警关节 ID（1~16），0 = 尚未报警
        self.alarm_side = 0             # 0x01=上限，0x02=下限
        self.alarm_rad = 0.0            # 报警帧里的当前实际角
        self.alarm_last_t = 0.0         # 最近一帧到达时刻
        self.other_frames = 0           # 非协议帧计数（排查误接总线用）
        self.other_ids = set()

    # ---- 主循环 ----
    def run(self):
        while not self.stop_event.is_set():
            item = self.link.recv(0.05)     # 50ms 超时，保证能及时响应 stop
            if item is None:
                continue
            can_id, payload = item
            now = time.monotonic()

            if can_id == HEARTBEAT_ID and len(payload) >= 1:
                bits = payload[0]
                with self._lock:
                    self.hb_count += 1
                    self.hb_last_t = now
                    self.hb_streaming = bool(bits & HB_BIT_STREAMING)
                    self.hb_sample_online = bool(bits & HB_BIT_SAMPLE_ONLINE)
                    self.hb_limit_locked = bool(bits & HB_BIT_LIMIT_LOCKED)
                continue

            # 软限位报警帧（DLC=8）：[0x62][关节ID][侧别][0][int32 实际角]
            if can_id == LIMIT_ALARM_ID and len(payload) >= 8:
                if payload[0] == LIMIT_ALARM_CODE:
                    jid = payload[1]
                    side = payload[2]
                    rad = proto_to_rad(unpack_i32le(payload[4:8]))
                    with self._lock:
                        if self.alarm_count == 0:
                            self.alarm_first_t = now
                        self.alarm_count += 1
                        self.alarm_jid = jid
                        self.alarm_side = side
                        self.alarm_rad = rad
                        self.alarm_last_t = now
                    continue

            if JOINT_ID_BASE <= can_id < JOINT_ID_BASE + TOTAL_JOINTS:
                if len(payload) >= 5 and payload[0] == CMD_GET_STAT:
                    rad = proto_to_rad(unpack_i32le(payload[1:5]))
                    idx = can_id - JOINT_ID_BASE
                    with self._lock:
                        self.fb_rad[idx] = rad
                        self.fb_count[idx] += 1
                        self.fb_last_t = now
                continue

            with self._lock:
                self.other_frames += 1
                if len(self.other_ids) < 8:
                    self.other_ids.add(can_id)

    # ---- 快照读取 ----
    def heartbeat_alive(self):
        """心跳是否在 HB_LOST_S 内到达过。"""
        with self._lock:
            if self.hb_count == 0:
                return False
            return (time.monotonic() - self.hb_last_t) <= HB_LOST_S

    def snapshot(self):
        """返回 (hb_count, hb_age_s, streaming, sample_online, fb_rad副本, fb_count副本)。"""
        with self._lock:
            age = (time.monotonic() - self.hb_last_t) if self.hb_count else float("inf")
            return (self.hb_count, age, self.hb_streaming, self.hb_sample_online,
                    list(self.fb_rad), list(self.fb_count))

    def alarm_snapshot(self):
        """返回 (报警帧计数, 关节ID, 侧别, 实际角rad)；无报警时关节ID=0。"""
        with self._lock:
            return (self.alarm_count, self.alarm_jid, self.alarm_side, self.alarm_rad)

    def limit_locked(self):
        """下位机是否处于软限位报警锁定（心跳 bit2 / 已收到报警帧）。"""
        with self._lock:
            return self.hb_limit_locked or self.alarm_count > 0

    def replied_ids(self):
        """已收到过至少一帧反馈的关节 ID 列表。"""
        with self._lock:
            return [JOINT_ID_BASE + i for i in range(TOTAL_JOINTS) if self.fb_count[i] > 0]

    def stop(self):
        self.stop_event.set()


# =============================================================================
# 轨迹数据加载
#
# ★★★ 列识别规则（只认表头名，**绝不按列序猜测**）★★★
# 轨迹文件标准格式 = step_position/step_make.xlsx：
#     Time (s) | L1_J1 L1_J2 L1_J3 L1_J4 | L2_J1 … | L3_J1 … | L4_J1 … L4_J4
# 表头 L{腿}_J{关节} 直接换算成协议帧 ID：
#     ID = JOINT_ID_BASE + (腿-1)×JOINT_PER_LEG + (关节-1)
#     例：L1_J1 → ID1、L2_J3 → ID7、L4_J4 → ID16（与下位机拆分规则完全一致）
# 单位一律为【关节端弧度】，正方向 = 逆时针（ccw），与 taihu_singleleg_step_xCAN
# 的电机端正方向、以及下位机 kJointDirSign = {+1,+1,+1,+1} 一致。
#
# 之所以不再用固定列序映射：错误的列序会让目标角落到别的关节上（例如把 J2 的
# ±70° 送给限位只有 -60°~+180° 的 J3），下位机会逐帧丢弃甚至驱动关节撞限位。
# 表头名对不上时直接报错退出，比"猜一个能跑的映射"安全得多。
# =============================================================================

# 表头名 → 关节列。允许 L1_J1 / L1J1 / L1-J1 / L1_J1 (rad) 等写法；
# 带 (deg) 后缀的列一律跳过（协议本身即 rad，避免单位混用）。
_HDR_LJ_RE = re.compile(
    r"^L\s*([1-4])\s*[_\-.]?\s*J\s*([1-4])\s*(?:\((rad|deg)\))?$", re.IGNORECASE)
# 表头名 → 直接给全局 ID。允许 ID1 / ID 1 / Joint ID 1 / ID1 (rad)。
_HDR_ID_RE = re.compile(
    r"^(?:ID|Joint\s*ID)\s*[:#]?\s*(\d{1,2})\s*(?:\((rad|deg)\))?$", re.IGNORECASE)
# 时间列表头（找不到就用第 0 列）。
_HDR_TIME_RE = re.compile(r"^(time|t)\b|^时间", re.IGNORECASE)

# 未出现在轨迹文件中的关节所下发的角度（rad）。0 = 标零位。
# 注意：下位机收到即进入流模式并以 PD 追踪，若该关节当前不在 0 位会运动过去。
UNMAPPED_JOINT_RAD = 0.0


def detect_time_column(header):
    """返回时间列序号（按表头名找 Time/t/时间，找不到则默认第 0 列）。"""
    for ci, name in enumerate(header):
        if name and _HDR_TIME_RE.match(name):
            return ci
    return 0


def detect_column_map(header):
    """
    按表头名识别「列序号 → 全局关节 ID」映射。

    支持两种表头写法（可混用）：
      1) L{腿}_J{关节}          例 L1_J1、L2_J3、L4_J4  ← step_make.xlsx 格式
      2) ID{N} / Joint ID {N}   例 ID1、ID 16（N = 1~16）

    @return list[(列序号, 全局关节ID, 表头名)]，按列序号升序。
    @raise RuntimeError 一个关节列都识别不到，或两列映射到同一个 ID。
    """
    found = {}                     # 关节ID → (列序号, 表头名)
    for ci, name in enumerate(header):
        if not name:
            continue
        jid = None
        m = _HDR_LJ_RE.match(name)
        if m:
            if m.group(3) and m.group(3).lower() == "deg":
                continue           # 跳过角度制列
            leg, joint = int(m.group(1)), int(m.group(2))
            jid = JOINT_ID_BASE + (leg - 1) * JOINT_PER_LEG + (joint - 1)
        else:
            m = _HDR_ID_RE.match(name)
            if m:
                if m.group(2) and m.group(2).lower() == "deg":
                    continue
                cand = int(m.group(1))
                if JOINT_ID_BASE <= cand < JOINT_ID_BASE + TOTAL_JOINTS:
                    jid = cand
        if jid is None:
            continue
        if jid in found:
            prev_ci, prev_name = found[jid]
            raise RuntimeError(
                "表头冲突：%r（第 %d 列）与 %r（第 %d 列）都映射到关节 ID%d"
                % (name, ci + 1, prev_name, prev_ci + 1, jid))
        found[jid] = (ci, name)

    if not found:
        raise RuntimeError(
            "表头里识别不到任何关节列。\n"
            "       实际表头：%s\n"
            "       期望格式：Time (s) + L1_J1 … L4_J4（16 列，单位 rad），"
            "或 Time + ID1 … ID16。\n"
            "       参考文件：step_position/step_make.xlsx"
            % (" | ".join(h or "(空)" for h in header)))

    return sorted((ci, jid, name) for jid, (ci, name) in found.items())


class SampleSet(object):
    """一组等间隔（或按时间列）的 16 关节目标采样点。"""

    def __init__(self, times, angles, source, mapped_ids):
        self.times = times          # list[float]，秒（相对起点）
        self.angles = angles        # list[list[float]]，每项 16 个关节角(rad)
        self.source = source        # 数据来源描述（打印用）
        self.mapped_ids = mapped_ids  # 实际由 xlsx 驱动的关节 ID 列表

    def __len__(self):
        return len(self.times)

    @property
    def duration(self):
        return self.times[-1] if self.times else 0.0

    @property
    def mean_dt(self):
        if len(self.times) < 2:
            return 0.0
        return (self.times[-1] - self.times[0]) / (len(self.times) - 1)


def load_samples(xlsx_path, sheet=None):
    """
    读取 xlsx → SampleSet。

    列映射**由表头名自动识别**（见 detect_column_map），标准格式为
    step_position/step_make.xlsx：
        Time (s) | L1_J1 L1_J2 L1_J3 L1_J4 | L2_J1 … | L3_J1 … | L4_J1 … L4_J4
    单位为关节端 rad，正方向 = 逆时针（ccw）。文件里缺的关节按
    UNMAPPED_JOINT_RAD 补齐（默认 0 = 标零位）。

    @param xlsx_path xlsx 路径
    @param sheet     工作表名，None = 第一个表
    @raise RuntimeError 文件缺失 / 表头无法识别 / 单元格非数字等，报错均带行列号。
    """
    try:
        import openpyxl
    except ImportError:
        raise RuntimeError("缺少 openpyxl，请先安装：pip3 install --user "
                           "--break-system-packages openpyxl")

    if not os.path.isfile(xlsx_path):
        raise RuntimeError("找不到轨迹文件：%s" % xlsx_path)

    wb = openpyxl.load_workbook(xlsx_path, data_only=True, read_only=True)
    try:
        ws = wb[sheet] if sheet else wb.worksheets[0]
        rows = list(ws.iter_rows(values_only=True))
        ws_title = ws.title
    finally:
        wb.close()
    if len(rows) < 2:
        raise RuntimeError("表 %s 数据不足（只有 %d 行）" % (ws_title, len(rows)))

    header = [str(c).strip() if c is not None else "" for c in rows[0]]
    col_map = detect_column_map(header)      # [(列序号, 关节ID, 表头名)]
    t_col = detect_time_column(header)
    max_col = max(ci for ci, _, _ in col_map)

    times = []
    angles = []
    for r_i, row in enumerate(rows[1:], start=2):
        if row is None:
            continue
        t_val = row[t_col] if t_col < len(row) else None
        if t_val is None or (isinstance(t_val, str) and not t_val.strip()):
            continue                      # 跳过空行
        try:
            t = float(t_val)
        except (TypeError, ValueError):
            raise RuntimeError("第 %d 行时间列无法解析为数字：%r" % (r_i, t_val))

        if len(row) <= max_col:
            raise RuntimeError("第 %d 行列数不足（需 %d 列，实到 %d 列）"
                               % (r_i, max_col + 1, len(row)))

        frame = [UNMAPPED_JOINT_RAD] * TOTAL_JOINTS
        for ci, jid, name in col_map:
            cell = row[ci]
            if cell is None or (isinstance(cell, str) and not cell.strip()):
                continue                  # 空单元格 = 保持默认（标零位）
            try:
                val = float(cell)
            except (TypeError, ValueError):
                raise RuntimeError("第 %d 行 %s（第 %d 列）无法解析为数字：%r"
                                   % (r_i, name or ("列%d" % (ci + 1)), ci + 1, cell))
            frame[jid - JOINT_ID_BASE] = val
        times.append(t)
        angles.append(frame)

    if not times:
        raise RuntimeError("表 %s 没有解析到任何采样点" % ws_title)

    src = "%s [表 %s]：%d 点，时间列 %r，识别到 %d 个关节列（%s）" % (
        os.path.basename(xlsx_path), ws_title, len(times),
        header[t_col] or ("列%d" % (t_col + 1)), len(col_map),
        "、".join("%s→ID%d" % (n, j) for _, j, n in col_map))
    return SampleSet(times, angles, src, [j for _, j, _ in col_map])


def check_limits(samples, ids):
    """
    离线限位预检查：扫描将要发送的关节，报告超出下位机软件限位的目标。

    下位机的处理策略是**钳位**：越限目标被改成限位值下发（关节走到限位并停住），
    随后当**实际角**到达限位时会触发软限位报警（ID 0x162）→ 整机锁定停止接收目标。
    所以越限本身不会让关节卡在原地不动，但会导致轨迹变形并最终触发报警，
    此处在**上机前**用 JOINT_LIMIT_DEG 提前暴露问题。
    返回 list[(jid, 关节序号, min_deg, max_deg, 实际最小deg, 实际最大deg, 越限点数)]，
    只含越限的关节。
    """
    problems = []
    for jid in ids:
        i = jid - JOINT_ID_BASE
        joint = i % JOINT_PER_LEG + 1        # 关节序号 1~4（决定限位）
        lo, hi = JOINT_LIMIT_DEG[joint]
        over = 0
        vmin, vmax = float("inf"), float("-inf")
        for frame in samples.angles:
            deg = frame[i] * DEG_PER_RAD
            if deg < vmin:
                vmin = deg
            if deg > vmax:
                vmax = deg
            if deg < lo - 1e-6 or deg > hi + 1e-6:
                over += 1
        if over > 0:
            problems.append((jid, joint, lo, hi, vmin, vmax, over))
    return problems


# =============================================================================
# 通讯阶段：链路检测 / 主动 ping / 回放
# =============================================================================

def print_link_status(monitor, prefix="      "):
    """打印一行链路状态摘要（心跳 + 反馈覆盖 + 软限位锁定）。"""
    hb_count, hb_age, streaming, online, fb_rad, fb_count = monitor.snapshot()
    n_fb = sum(1 for c in fb_count if c > 0)
    if hb_count == 0:
        hb_txt = "心跳 ✗ 未收到"
    else:
        hb_txt = "心跳 ✓ %d 次（%.0fms 前，流模式=%s 样点在线=%s）" % (
            hb_count, hb_age * 1000.0, "是" if streaming else "否",
            "是" if online else "否")
    print("%s%s | 反馈 %d/%d 个 ID 有数据 | 收 %d 帧"
          % (prefix, hb_txt, n_fb, TOTAL_JOINTS, monitor.link.rx_count))
    alarm_n, alarm_jid, alarm_side, alarm_rad = monitor.alarm_snapshot()
    if alarm_n > 0:
        print("%s[软限位锁] 下位机锁定中：%s"
              % (prefix, describe_limit_alarm(alarm_jid, alarm_side, alarm_rad)))


def wait_for_link(monitor, wait_s, quiet=False):
    """
    被动等待下位机心跳。返回 True 表示链路已通。
    期间每 0.5s 打印一次状态，超时则给出排查建议。
    """
    deadline = time.monotonic() + wait_s
    if not quiet:
        print("[链路] 等待下位机心跳（ID 0x%X，%dms 周期），最多 %.0f s …"
              % (HEARTBEAT_ID, HEARTBEAT_MS, wait_s))
    while time.monotonic() < deadline:
        if monitor.heartbeat_alive():
            if not quiet:
                print_link_status(monitor, prefix="[链路] ✓ ")
            return True
        time.sleep(0.2)

    if not quiet:
        print("[链路] ✗ %.0f s 内未收到下位机心跳，请检查：" % wait_s)
        print("        1) 下位机 walk_control_V1 是否已启动，且上位机接口名与本程序 --can 一致；")
        print("        2) 该通道是否已 up：sudo ./build/kcanctl up <接口名> 1000000；")
        print("        3) 两端是否接在**同一条**总线上、120Ω 终端电阻是否到位；")
        print("        4) 用 candump <接口名> 看是否有 161 号帧到达。")
        if monitor.other_frames:
            print("        （另收到 %d 帧非协议帧，ID=%s —— 可能误接到了电机总线）"
                  % (monitor.other_frames,
                     " ".join("%X" % i for i in sorted(monitor.other_ids))))
    return False


def ping_joints(link, monitor, ids=None, wait_s=0.6):
    """
    主动握手：给指定关节 ID 发 [0x41]（DLC=1）状态请求，统计下位机的回复。

    这是**双向**验证：请求能到达下位机、下位机的回复能回到本程序，才算通讯打通。
    返回 (回复的 ID 列表, 请求的 ID 列表)。
    """
    ids = ids or list(range(JOINT_ID_BASE, JOINT_ID_BASE + TOTAL_JOINTS))
    before = set(monitor.replied_ids())
    counts_before = {}
    _, _, _, _, _, fb_before = monitor.snapshot()
    for i, jid in enumerate(ids):
        counts_before[jid] = fb_before[i]

    for jid in ids:
        link.send(jid, bytes([CMD_GET_STAT]))

    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        _, _, _, _, _, fb_now = monitor.snapshot()
        if all(fb_now[jid - JOINT_ID_BASE] > counts_before[jid] for jid in ids):
            break
        time.sleep(0.02)

    _, _, _, _, _, fb_after = monitor.snapshot()
    replied = [jid for jid in ids if fb_after[jid - JOINT_ID_BASE] > counts_before[jid]]
    return replied, ids


def print_feedback_table(monitor, ids):
    """打印指定关节的实际角（来自下位机反馈缓存），便于与下位机 `pos` 对照。"""
    _, _, _, _, fb_rad, fb_count = monitor.snapshot()
    print("      关节实际角（下位机反馈）：")
    for jid in ids:
        i = jid - JOINT_ID_BASE
        if fb_count[i] > 0:
            print("        ID%-3d %-8s %+8.2f°  (%+.4f rad, %d 帧)"
                  % (jid, joint_id_name(jid), fb_rad[i] * DEG_PER_RAD,
                     fb_rad[i], fb_count[i]))
        else:
            print("        ID%-3d %-8s     ——     (无反馈：该腿未在下位机启用或电机离线)"
                  % (jid, joint_id_name(jid)))


def stream_samples(link, monitor, samples, hz, stop_event, send_ids, progress_s=1.0,
                   stop_on_limit=True):
    """
    按 hz 频率回放采样点：每个 tick 给 send_ids 里每个关节发一帧
    [0x1E][int32 = rad × 10000]（DLC=5）。

    节拍基于绝对时间轴（t0 + i/hz），避免累积漂移；下位机断流门限 1s，
    故单次 tick 延迟超过 0.5s 会告警。

    stop_on_limit=True 时，一旦收到下位机的软限位报警（0x162）立即停止发送，
    返回的第三个值为报警信息（无报警为 None）。

    返回 (已发采样点数, 是否被中断, 报警信息或 None)。
    """
    period = 1.0 / hz
    n = len(samples)
    t0 = time.monotonic()
    next_progress = t0 + progress_s
    late_ticks = 0

    for i in range(n):
        if stop_event.is_set():
            return i, True, None

        # 软限位报警：下位机已整机锁定，继续发目标毫无意义且会被丢弃
        if stop_on_limit:
            alarm_n, alarm_jid, alarm_side, alarm_rad = monitor.alarm_snapshot()
            if alarm_n > 0:
                return (i, True,
                        (alarm_jid, alarm_side, alarm_rad, i, samples.times[i]))

        frame = samples.angles[i]
        for jid in send_ids:
            rad = frame[jid - JOINT_ID_BASE]
            link.send(jid, bytes([CMD_SET_POS]) + pack_i32le(rad_to_proto(rad)))

        now = time.monotonic()
        if now >= next_progress:
            _, hb_age, streaming, online, fb_rad, _ = monitor.snapshot()
            shown = []
            for jid in send_ids[:4]:
                shown.append("ID%d %+.1f°" % (jid, fb_rad[jid - JOINT_ID_BASE]
                                             * DEG_PER_RAD))
            print("      t=%6.2fs  进度 %3d/%-3d  发 %d 帧  下位机流模式=%s  %s"
                  % (samples.times[i], i + 1, n, link.tx_count,
                     "是" if streaming else "否", " ".join(shown)))
            next_progress = now + progress_s

        # 节拍对齐（绝对时间轴）
        delay = t0 + (i + 1) * period - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        elif delay < -0.5:
            late_ticks += 1
            if late_ticks <= 3:
                print("      [警告] tick %d 落后 %.0fms（下位机断流门限 %dms，可能已退出流模式）"
                      % (i + 1, -delay * 1000.0, int(HOST_STREAM_TIMEOUT_S * 1000)))

    return n, False, None


def send_stream_stop(link, ids=None):
    """发 [0x02]（DLC=1）通知下位机退出流模式（电机保持当前目标，不抱闸）。"""
    for jid in (ids or [JOINT_ID_BASE]):
        link.send(jid, bytes([CMD_STOP]))


def send_limit_clear(link, ids=None):
    """发 [0x03]（DLC=1）通知下位机清除软限位报警锁定（整机恢复接收目标）。"""
    for jid in (ids or [JOINT_ID_BASE]):
        link.send(jid, bytes([CMD_CLEAR_LIMIT]))


def describe_limit_alarm(alarm_jid, alarm_side, alarm_rad):
    """把报警三元组格式化成可读文本，含限位值与越限量。"""
    if not (JOINT_ID_BASE <= alarm_jid < JOINT_ID_BASE + TOTAL_JOINTS):
        return "未知关节 ID %s" % alarm_jid
    joint = (alarm_jid - JOINT_ID_BASE) % JOINT_PER_LEG + 1
    lo, hi = JOINT_LIMIT_DEG[joint]
    side_txt = "上限" if alarm_side == LIMIT_SIDE_MAX else "下限"
    limit_deg = hi if alarm_side == LIMIT_SIDE_MAX else lo
    deg = alarm_rad * DEG_PER_RAD
    return ("ID%-3d %-8s 到达%s %+.1f°（实际 %+.2f°，越限 %+.2f°）"
            % (alarm_jid, joint_id_name(alarm_jid), side_txt, limit_deg,
               deg, deg - limit_deg))


# =============================================================================
# 主流程
# =============================================================================

def build_parser():
    p = argparse.ArgumentParser(
        prog="up_walk_control_V1.py",
        description="上位机调试程序：读 step_make.xlsx，10Hz 发关节目标给下位机 walk_control_V1",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""示例：
  python3 tools/up_walk_control_V1.py --can can4              # 标准用法（先检测链路，回车后发送）
  python3 tools/up_walk_control_V1.py --can can4 --dry-run    # 无硬件自测（只解析 xlsx + 模拟节拍）
  python3 tools/up_walk_control_V1.py --can can4 --auto       # 跳过确认直接发送（脚本化）
  python3 tools/up_walk_control_V1.py --can can4 --only-mapped  # 只发 xlsx 里有的关节，其余不动
  python3 tools/up_walk_control_V1.py --can can4 --loop 0     # 循环回放直到 Ctrl+C
  python3 tools/up_walk_control_V1.py --can can4 --keep-on-limit  # 触限报警后仍继续发
""")
    p.add_argument("--can", default="can4",
                   help="上位机 CAN 接口名（须与下位机启动时输入的一致，默认 can4）")
    p.add_argument("--xlsx", default="step_position/step_make.xlsx",
                   help="轨迹文件路径（默认 step_position/step_make.xlsx，"
                        "表头 L{腿}_J{关节} 或 ID{N}，单位 rad）")
    p.add_argument("--sheet", default=None, help="工作表名（默认第一个表）")
    p.add_argument("--hz", type=float, default=DEFAULT_HZ,
                   help="发送频率 Hz（默认 %(default)s，须与协议一致）")
    p.add_argument("--loop", type=int, default=1,
                   help="回放次数，0 = 无限循环直到 Ctrl+C（默认 %(default)s）")
    p.add_argument("--only-mapped", action="store_true",
                   help="只发送 xlsx 中实际有数据的关节 ID，其余关节完全不发"
                        "（默认发全部 16 个 ID，未映射关节目标为 %.1f rad）" % UNMAPPED_JOINT_RAD)
    p.add_argument("--wait-link", type=float, default=10.0,
                   help="链路检测最长等待秒数（默认 %(default)s）")
    p.add_argument("--auto", action="store_true",
                   help="跳过回车确认，链路通了就立即发送（调试脚本化用）")
    p.add_argument("--dry-run", action="store_true",
                   help="不打开 CAN 接口，只解析 xlsx 并模拟发送节拍（无硬件自测）")
    p.add_argument("--keep-on-limit", action="store_true",
                   help="收到下位机软限位报警（0x162）后仍继续发送；"
                        "默认是立即停发（下位机会整机锁定，继续发也会被丢弃）")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    g_stop = threading.Event()

    def on_sigint(signum, frame):
        if g_stop.is_set():
            print("\n[退出] 再次收到中断，强制退出")
            sys.exit(130)
        print("\n[退出] 收到 Ctrl+C，停止发送并通知下位机退出流模式 …")
        g_stop.set()

    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    print("=" * 78)
    print("  上位机调试程序 up_walk_control_V1 —— 关节位置流发送（协议 V2.0）")
    print("=" * 78)
    print("  角色  ：上位机（本程序） → CAN → 下位机 walk_control_V1 → can0~can3 → 16 电机")
    print("  接口  ：%s%s" % (args.can, "（dry-run，不打开真实接口）" if args.dry_run else ""))
    print("  帧格式：ID 1~16，DLC=5，[0x1E][int32 小端 = 关节端角(rad) × %.0f]" % ANGLE_SCALE)
    print("  频率  ：%.1f Hz（每 tick 发 %d 帧）" % (args.hz, TOTAL_JOINTS))
    print("-" * 78)

    # ---- 1. 加载轨迹 ----
    print("[1/4] 加载轨迹文件 %s …" % args.xlsx)
    try:
        samples = load_samples(args.xlsx, args.sheet)
    except Exception as e:
        print("[错误] %s" % e)
        return 1
    print("      %s" % samples.source)
    print("      采样点 %d 个，时长 %.2f s，平均间隔 %.4f s（≈ %.2f Hz）"
          % (len(samples), samples.duration, samples.mean_dt,
             (1.0 / samples.mean_dt) if samples.mean_dt > 0 else 0.0))
    if samples.mean_dt > 0:
        dev = abs(1.0 / samples.mean_dt - args.hz) / args.hz
        if dev > 0.05:
            print("      [提示] 文件采样率 %.2fHz 与 --hz %.2fHz 相差 %.0f%%，"
                  "回放将按 --hz 重新定时（时长会变为 %.2f s）"
                  % (1.0 / samples.mean_dt, args.hz, dev * 100.0, len(samples) / args.hz))
    if samples.duration > 0:
        max_step = 0.0
        for i in range(1, len(samples)):
            for j in range(TOTAL_JOINTS):
                max_step = max(max_step, abs(samples.angles[i][j] - samples.angles[i - 1][j]))
        print("      相邻采样点最大关节变化 %.4f rad（%.2f°）"
              % (max_step, max_step * DEG_PER_RAD))
        if max_step > 0.035:
            print("      [提示] 超过协议建议的 ≤0.035 rad(2°)/帧，电机 PD 追踪会有滞后")

    send_ids = samples.mapped_ids if args.only_mapped else \
        list(range(JOINT_ID_BASE, JOINT_ID_BASE + TOTAL_JOINTS))
    print("      本次发送 %d 个关节 ID：%s" % (len(send_ids),
          ", ".join(str(i) for i in send_ids)))
    if not args.only_mapped:
        print("      [注意] 未映射的 %d 个关节目标为 %.1f rad（标零位）；"
              "若它们当前不在零位，开始发送后会运动到零位。"
              % (TOTAL_JOINTS - len(samples.mapped_ids), UNMAPPED_JOINT_RAD))
        print("             只想动 xlsx 里的关节请加 --only-mapped。")

    # 离线限位预检查：提前暴露会被下位机钳位的越限目标
    problems = check_limits(samples, send_ids)
    if problems:
        print("      [限位警告] 以下关节有目标超出下位机软件限位，"
              "下位机会**钳位到限位值**下发（关节走到限位停住），")
        print("                 并且当实际角到达限位时触发软限位报警（0x162）"
              "→ 下位机整机锁定，本程序默认随即停发：")
        for jid, _joint, lo, hi, vmin, vmax, over in problems:
            print("        ID%-3d %s 范围 [%+.1f°, %+.1f°] 超出限位 [%+.0f°, %+.0f°]，%d/%d 点越限"
                  % (jid, joint_id_name(jid), vmin, vmax, lo, hi, over, len(samples)))
        print("        → 请在 xlsx 里把这些列裁到限位内；J3 关节端限位是 -60°~+180°，尤其容易越限。")
    else:
        print("      限位检查：✓ 全部目标都在软件限位内")

    # ---- 2. 打开链路 + 启动监视线程 ----
    link = None
    monitor = None
    if args.dry_run:
        print("[2/4] dry-run：跳过 CAN 接口打开")
        print("[3/4] dry-run：跳过链路检测")
    else:
        print("[2/4] 打开 CAN 接口 %s …" % args.can)
        try:
            link = CanLink(args.can)
        except OSError as e:
            print("[错误] %s" % e)
            print("       排查：① 接口名是否正确（ip -brief link show | grep can）；")
            print("             ② 通道是否已 up：sudo ./build/kcanctl up %s 1000000；" % args.can)
            print("             ③ 是否与下位机启动时输入的上位机接口名一致。")
            return 1
        print("      ✓ 已打开 %s" % args.can)

        monitor = LinkMonitor(link)
        monitor.start()

        # ---- 3. 链路检测（被动收心跳 + 主动 ping），等用户确认 ----
        print("[3/4] 链路检测")
        link_ok = wait_for_link(monitor, args.wait_link)

        if link_ok:
            print("      主动握手：向 %d 个关节 ID 发 [0x41] 状态请求 …" % TOTAL_JOINTS)
            replied, requested = ping_joints(link, monitor)
            print("      ✓ %d/%d 个 ID 有回复（双向通讯打通）" % (len(replied), len(requested)))
            if len(replied) < len(requested):
                missing = [j for j in requested if j not in replied]
                legs = sorted(set(leg_of_id(j) for j in missing))
                print("        无回复：%s（腿 %s）—— 通常是这些腿未在下位机启用或电机离线，"
                      "不影响其余关节" % (", ".join(str(j) for j in missing),
                                           ", ".join(str(l) for l in legs)))
            print_feedback_table(monitor, samples.mapped_ids)
        else:
            print("      主动握手：链路未通，仍尝试发 [0x41] 看是否有回复 …")
            replied, _ = ping_joints(link, monitor, wait_s=1.0)
            if replied:
                print("      ✓ 收到 %d 个 ID 的回复，链路实际可用（心跳可能刚启动）" % len(replied))
                link_ok = True
            else:
                print("      ✗ 无任何回复")

        if not link_ok:
            print("\n[终止] 通讯未打通，未发送任何关节目标。修好后重新运行本程序即可。")
            monitor.stop()
            link.close()
            return 2

        # 清掉上一轮遗留的软限位报警锁定（[0x03]），否则下位机会丢弃所有新目标
        if monitor.limit_locked():
            print("      [软限位] 下位机正处于报警锁定，发送 [0x03] 尝试解除 …")
        send_limit_clear(link, send_ids[:1])
        time.sleep(0.2)
        if monitor.limit_locked():
            print("      [警告] 仍处于锁定状态，可能是报警条件持续满足（关节仍压在限位上）。"
                  "请先把关节退离限位，或在键盘输入 clear 后重试。")

        print("\n      持续监视中（Ctrl+C 退出）。当前链路状态：")
        print_link_status(monitor)

    # ---- 4. 等确认 → 回放 ----
    loop_txt = "无限循环" if args.loop == 0 else "%d 遍" % args.loop
    if args.auto:
        print("[4/4] --auto：跳过确认，直接开始发送")
    elif args.dry_run:
        print("[4/4] dry-run：模拟发送（不写 CAN）")
    else:
        print("-" * 78)
        try:
            ans = input("[4/4] 按回车开始发送关节目标（%s，共 %d 点 × %.0fHz）；"
                        "输入 q 回车退出：" % (loop_txt, len(samples), args.hz)).strip().lower()
        except EOFError:
            ans = "q"
        if ans in ("q", "quit", "exit"):
            print("[退出] 用户取消，未发送任何关节目标")
            if monitor:
                monitor.stop()
            if link:
                link.close()
            return 0

    print("\n[发送] 开始回放：%.1f Hz × %d 点，发送 ID %s"
          % (args.hz, len(samples), ",".join(str(i) for i in send_ids)))
    t_start = time.monotonic()
    total_points = 0
    rounds = 0
    limit_alarm = None      # 软限位报警信息（收到后按 --keep-on-limit 决定是否继续）
    try:
        while not g_stop.is_set():
            if args.loop and rounds >= args.loop:
                break
            rounds += 1
            if args.loop != 1:
                print("      —— 第 %d 遍 ——" % rounds)
            if args.dry_run:
                # 无硬件：只做节拍模拟，不写 CAN
                period = 1.0 / args.hz
                for i in range(len(samples)):
                    if g_stop.is_set():
                        break
                    time.sleep(period)
                    if (i + 1) % max(1, int(args.hz)) == 0:
                        print("      [dry-run] t=%6.2fs 进度 %3d/%d"
                              % (samples.times[i], i + 1, len(samples)))
                total_points += len(samples)
            else:
                sent, interrupted, alarm = stream_samples(
                    link, monitor, samples, args.hz, g_stop, send_ids,
                    stop_on_limit=not args.keep_on_limit)
                total_points += sent
                if alarm is not None:
                    limit_alarm = alarm
                    alarm_jid, alarm_side, alarm_rad, idx, t_s = alarm
                    print("\n" + "!" * 78)
                    print("[软限位报警] 下位机上报：%s"
                          % describe_limit_alarm(alarm_jid, alarm_side, alarm_rad))
                    print("             触发时刻 t=%.2fs（第 %d/%d 点）"
                          % (t_s, idx + 1, len(samples)))
                    if args.keep_on_limit:
                        print("             --keep-on-limit：继续发送（下位机已整机锁定，"
                              "目标会被丢弃）")
                    else:
                        print("             已停止发送（下位机整机锁定，"
                              "需键盘 clear 或本程序下次启动时发 [0x03] 解除）")
                    print("!" * 78)
                    if not args.keep_on_limit:
                        break
                if interrupted:
                    break
    finally:
        elapsed = time.monotonic() - t_start
        if link:
            try:
                send_stream_stop(link, send_ids[:1])
                print("\n[发送] 已发 [0x02] 通知下位机退出流模式（电机保持当前目标，不抱闸）")
            except OSError as e:
                print("\n[警告] 发送 0x02 失败：%s" % e)

    print("-" * 78)
    print("[统计] 回放 %d 遍共 %d 个采样点，耗时 %.2f s（平均 %.2f Hz）"
          % (rounds, total_points, elapsed,
             (total_points / elapsed) if elapsed > 0 else 0.0))
    if link:
        print("       CAN 发 %d 帧，收 %d 帧" % (link.tx_count, link.rx_count))
    if monitor:
        hb_count, hb_age, streaming, online, fb_rad, fb_count = monitor.snapshot()
        print("       下位机心跳 %d 次（最后 %.0fms 前），流模式=%s，样点在线=%s"
              % (hb_count, hb_age * 1000.0, "是" if streaming else "否",
                 "是" if online else "否"))
        alarm_n, alarm_jid, alarm_side, alarm_rad = monitor.alarm_snapshot()
        if alarm_n > 0:
            print("       软限位报警 %d 帧：%s"
                  % (alarm_n, describe_limit_alarm(alarm_jid, alarm_side, alarm_rad)))
            print("       下位机处于报警锁定，解除方式：下位机键盘输入 clear，"
                  "或重启本程序（启动时自动发 [0x03]）")
        else:
            print("       软限位报警：无 ✓")
        print("       收到反馈的关节：%s"
              % (", ".join("ID%d(%d帧)" % (jid, fb_count[jid - JOINT_ID_BASE])
                           for jid in monitor.replied_ids()) or "无"))
        monitor.stop()
    if link:
        link.close()
    if limit_alarm is not None:
        print("[完成] 上位机退出（因软限位报警提前结束）。"
              "下位机仍处报警锁定，需 clear 解除。")
    else:
        print("[完成] 上位机退出。下位机可继续用键盘命令调试（pos/stat/estop/quit）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
