#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    limit_test.py
@brief   软限位专项测试（**上位机**运行）：让一条腿的 1 号关节故意越限，
         其余关节在限位内小范围摆动，验证下位机的软限位钳位 / 报警 / 锁定是否正常。

场景（默认，只动腿 1）：
    ID1  L1_J1 ：从**当前实际角**走**五次多项式**平滑轨迹（默认 10 s）到限位 +120°，
                 首尾速度和加速度均为 0（起停无冲击）；再用 1/3 时长（≈3.3 s）
                 追越到 +125°
                 → 下位机把目标**钳位到 +120°**下发，关节走到 120° 并停住
                 → 实际角进入限位 0.5° 内时，下位机**整机锁定**并上报 0x162
    ID2  L1_J2 ：围绕启动时的实际角，幅度 ±10°，周期 5 s 正弦摆动（限位 ±120°）
    ID3  L1_J3 ：同上（限位 −60°~+180°，若实际角靠近下限会自动收窄幅度）
    ID4  L1_J4 ：同上（限位 ±90°）

预期现象：
    1. 上位机日志里 ID1 的目标一直是 +125°，下位机日志打出 `[钳位]`，实际角停在 +120° 附近；
    2. ID2~ID4 连续摆动，心跳 bit0（流模式）/bit1（样点在线）常亮；
    3. 实际角进入限位 0.5° 后：下位机打印 `[软限位][报警]`，心跳 bit2 置位，
       ID 0x162 报警帧 200 ms 重发，本程序**立即停发并退出**；
    4. 下位机保持锁定（需键盘 `clear` 或 `[0x03]` 解除），本程序默认不清除，
       便于你用 `stat` 观察锁定状态；加 `--clear-on-exit` 可在退出前自动解除。

⚠ 安全提示：
    - J1 会真的走到 +120° 软限位附近并顶住，请确认该方向运动范围内无干涉、无负载伤人；
    - 首次跑建议先 `--dry-run` 看波形，再上机；上机时手放在急停附近；
    - 下位机 CAN 通道必须由你手动 up：`sudo ./build/kcanctl up can0 1000000`。

使用方法（两个终端）：
    终端 A（下位机）：
        sudo ./build/kcanctl up can0 1000000          # 腿1 总线（按实际接线）
        sudo ./build/kcanctl up can4 1000000          # 上位机链路（独立接口）
        ./build/walk_control_V1                       # 腿号回车 = 全部；上位机接口输 can4
    终端 B（上位机，本程序）：
        python3 tools/limit_test.py --can can4

    链路检测 + 主动 ping + 打印将要下发的目标 → 回车确认后才开始发送。

无硬件自测：
    python3 tools/limit_test.py --dry-run

依赖：python3 标准库（socket/select/struct，原生 SocketCAN，**不需要 python-can**）
"""

import argparse
import math
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

LIMIT_ALARM_ID = 0x162       # 下位机→上位机：软限位报警帧 ID
LIMIT_ALARM_CODE = 0x62      # 报警帧 data[0] 功能码
LIMIT_ALARM_MS = 200         # 报警帧重发周期（ms）
LIMIT_SIDE_MAX = 0x01        # data[2]：撞上限
LIMIT_SIDE_MIN = 0x02        # data[2]：撞下限
HB_BIT_STREAMING = 0x01      # 心跳 bit0：流模式中
HB_BIT_SAMPLE_ONLINE = 0x02  # 心跳 bit1：最近 1s 有样点
HB_BIT_LIMIT_LOCKED = 0x04   # 心跳 bit2：软限位报警锁定中

JOINT_PER_LEG = 4            # 每腿关节数
JOINT_ID_BASE = 1            # 帧 ID 起始值（ID = (腿-1)×4 + 关节号）

DEG_PER_RAD = 57.29577951308232   # 180/π
RAD_PER_DEG = 1.0 / DEG_PER_RAD

# 软件限位（**关节端**，度）—— ★ 与下位机 kJointLimitDeg 严格一致，仅用于本程序
# 提前告知「哪些目标会被钳位」「摆动会不会自己撞限位」。
#   J1 直驱      ±120°
#   J2 锥齿轮1:2 ±120°
#   J3 锥齿轮1:2 −60° ~ +180°   ← 下限由 −30° 放宽到 −60°
#   J4 直驱      ±90°
JOINT_LIMIT_DEG = {
    1: (-120.0, 120.0),
    2: (-120.0, 120.0),
    3: (-60.0, 180.0),
    4: (-90.0, 90.0),
}

# 下位机触发报警的"进入限位余量"（kLimitTriggerDeg = 0.5°）：实际角进入限位 0.5° 内
# 即判定到达限位。本程序用它判断"还差多少才能触发"。
LOWER_TRIGGER_MARGIN_DEG = 0.5

# 下位机防抖退让量（kLimitRearmDeg = 2°）：报警解除后，实际角必须先退离限位
# 2° 才会重新武装。因此「上次已把关节压到 120°」的场景直接重跑，
# 关节不退离就不会再报警 —— 本程序在起始角过于靠近限位时会先自动"预退"。
REARM_MARGIN_DEG = 2.0

SWING_MARGIN_DEG = 5.0       # 摆动时至少距离限位多远（避免摆动自身误触限位）
SWING_FADE_IN_S = 1.0        # 摆动渐入时间（避免从中心突然起振）
BACKOFF_TRIGGER_DEG = 5.0    # 起始角距限位小于该值 → 先预退到限位内 15°
BACKOFF_DEPTH_DEG = 15.0     # 预退深度（从限位往限位内侧退多少度）


# =============================================================================
# 关节 ID 工具
# =============================================================================

def joint_to_id(leg, joint):
    """(腿号 1~4, 关节号 1~4) → 全局关节 ID 1~16。"""
    return JOINT_ID_BASE + (leg - 1) * JOINT_PER_LEG + (joint - 1)


def id_to_leg(jid):
    """全局关节 ID → 腿号 1~4。"""
    return (jid - JOINT_ID_BASE) // JOINT_PER_LEG + 1


def id_to_joint(jid):
    """全局关节 ID → 关节号 1~4。"""
    return (jid - JOINT_ID_BASE) % JOINT_PER_LEG + 1


def joint_name(jid):
    """全局关节 ID → 可读名，如 1 → 'L1_J1'。"""
    return "L%d_J%d" % (id_to_leg(jid), id_to_joint(jid))


def leg_ids(jid):
    """与给定 ID 同一条腿的 4 个关节 ID（升序）。"""
    leg = id_to_leg(jid)
    return [joint_to_id(leg, j) for j in range(1, JOINT_PER_LEG + 1)]


# =============================================================================
# SocketCAN 底层（struct can_frame：__u32 can_id + __u8 len + 3B 填充 + 8B data）
# =============================================================================

CAN_FRAME_FMT = "<IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)   # = 16 字节

CAN_EFF_FLAG = getattr(socket, "CAN_EFF_FLAG", 0x80000000)
CAN_RTR_FLAG = getattr(socket, "CAN_RTR_FLAG", 0x40000000)
CAN_ERR_FLAG = getattr(socket, "CAN_ERR_FLAG", 0x20000000)
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


class CanLink(object):
    """
    原生 SocketCAN 收发（PF_CAN + SOCK_RAW + CAN_RAW），一个实例独占一个接口。
    波特率由事先的 `ip link set canX type can bitrate 1000000` 配置（本类不设置）。
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
        """发送一帧标准帧（payload 长度即 DLC）。"""
        payload = bytes(payload)
        dlc = min(len(payload), 8)
        frame = struct.pack(CAN_FRAME_FMT, can_id & CAN_SFF_MASK, dlc,
                            payload.ljust(8, b"\x00"))
        self.sock.send(frame)
        self.tx_count += 1

    def recv(self, timeout_s):
        """收一帧，返回 (can_id, payload) 或 None（超时）。跳过错误帧/远程帧/扩展帧。"""
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
            if can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG):
                continue
            self.rx_count += 1
            return can_id & CAN_SFF_MASK, raw[8:8 + min(dlc, 8)]

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


# =============================================================================
# 后台接收监视线程：心跳 / 实际角反馈 / 软限位报警
# =============================================================================

class Monitor(threading.Thread):
    """
    只收不发，维护：
      - 心跳 0x161：到达计数、bit0 流模式、bit1 样点在线、bit2 软限位锁定；
      - 反馈 ID 1~16 [0x41][int32]：各关节实际角（rad）与帧计数；
      - 软限位报警 0x162 [0x62][ID][侧别][0][int32 实际角]：明细、计数、首帧时刻。
    """

    def __init__(self, link):
        super(Monitor, self).__init__(name="Monitor", daemon=True)
        self.link = link
        self.stop_event = threading.Event()
        self._lock = threading.Lock()
        self.hb_count = 0
        self.hb_last_t = 0.0
        self.hb_streaming = False
        self.hb_sample_online = False
        self.hb_limit_locked = False
        self.fb_rad = [0.0] * (JOINT_ID_BASE + JOINT_PER_LEG * 4)
        self.fb_count = [0] * (JOINT_ID_BASE + JOINT_PER_LEG * 4)
        self.alarm_count = 0
        self.alarm_first_t = 0.0
        self.alarm_jid = 0
        self.alarm_side = 0
        self.alarm_rad = 0.0
        self.alarm_last_t = 0.0
        self.other_ids = set()

    def run(self):
        while not self.stop_event.is_set():
            item = self.link.recv(0.05)
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

            if can_id == LIMIT_ALARM_ID and len(payload) >= 8:
                if payload[0] == LIMIT_ALARM_CODE:
                    with self._lock:
                        if self.alarm_count == 0:
                            self.alarm_first_t = now
                        self.alarm_count += 1
                        self.alarm_jid = payload[1]
                        self.alarm_side = payload[2]
                        self.alarm_rad = proto_to_rad(unpack_i32le(payload[4:8]))
                        self.alarm_last_t = now
                continue

            if JOINT_ID_BASE <= can_id < JOINT_ID_BASE + JOINT_PER_LEG * 4:
                if len(payload) >= 5 and payload[0] == CMD_GET_STAT:
                    idx = can_id
                    with self._lock:
                        self.fb_rad[idx] = proto_to_rad(unpack_i32le(payload[1:5]))
                        self.fb_count[idx] += 1
                continue

            with self._lock:
                if len(self.other_ids) < 8:
                    self.other_ids.add(can_id)

    # ---- 快照 ----
    def heartbeat_alive(self, tol_s=1.0):
        with self._lock:
            return self.hb_count > 0 and (time.monotonic() - self.hb_last_t) <= tol_s

    def hb_age(self):
        with self._lock:
            return (time.monotonic() - self.hb_last_t) if self.hb_count else float("inf")

    def flags(self):
        """返回 (流模式, 样点在线, 软限位锁定)。"""
        with self._lock:
            return self.hb_streaming, self.hb_sample_online, self.hb_limit_locked

    def fb_deg(self, jid):
        """某关节的实际角（度）；无反馈返回 None。"""
        with self._lock:
            if self.fb_count[jid] == 0:
                return None
            return self.fb_rad[jid] * DEG_PER_RAD

    def fb_deg_all(self, ids):
        with self._lock:
            return [(jid,
                     self.fb_rad[jid] * DEG_PER_RAD if self.fb_count[jid] else None)
                    for jid in ids]

    def alarm(self):
        """返回 (报警帧数, ID, 侧别, 实际角rad, 首帧 monotonic 时刻)。"""
        with self._lock:
            return (self.alarm_count, self.alarm_jid, self.alarm_side,
                    self.alarm_rad, self.alarm_first_t)

    def limit_locked(self):
        with self._lock:
            return self.hb_limit_locked or self.alarm_count > 0

    def stop(self):
        self.stop_event.set()


# =============================================================================
# 目标波形：J1 越限轨迹（两段五次多项式）+ 其余关节正弦摆动
# =============================================================================

def quintic(x):
    """
    0→1 的五次多项式平滑曲线，x 需已归一化到 [0,1]。

        s(x) = 10x³ - 15x⁴ + 6x⁵

    相比三次的 smoothstep，五次曲线额外约束了**端点加速度为 0**：
    首尾的速度和加速度都为 0，起停没有任何冲击（jerk 连续），
    适合在真机上慢慢把关节推到限位附近观察。

    峰值斜率（=1.875×平均速度）出现在 x=0.5：s'(0.5) = 1.875。
    峰值加速度 |s''|max = 5.7735，出现在 x≈0.2113 / 0.7887。
    """
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x))


class Plan(object):
    """
    目标波形生成器（全部按**关节端度**计算，发送时再转 rad）。

      test_id ：越限关节。**两段五次多项式**：
                ① 主段：start_deg → 限位值（aka 被钳位的那个角），耗时 ramp_s；
                ② 次段：限位值 → target_deg，耗时 ramp_s/3。
                因此“用 ramp_s 秒跑到限位”是精确成立的，越限那一步是短促的
                追加段，便于观察钳位。两段的端点速度/加速度均为 0。
      swings  ：其余关节，围绕各自 center_deg 以 amp_deg / period_s 正弦摆动。
    """

    def __init__(self, test_id, start_deg, target_deg, ramp_s,
                 swing_ids, centers_deg, amp_deg, period_s, stop_deg=None):
        self.test_id = test_id
        self.start_deg = start_deg
        self.target_deg = target_deg
        self.ramp_s = max(0.01, ramp_s)
        # 主段的终点：默认为目标值（不限幅时就是直达）
        self.stop_deg = target_deg if stop_deg is None else stop_deg
        # 越限追加段时长：若有越限量则用 ramp_s/3，否则为 0
        self.extra_s = (self.ramp_s / 3.0
                        if abs(target_deg - self.stop_deg) > 1e-9 else 0.0)
        self.swing_ids = list(swing_ids)
        self.centers_deg = dict(centers_deg)
        self.amp_deg = dict(amp_deg)
        self.period_s = max(0.01, period_s)

    @property
    def ids(self):
        return [self.test_id] + self.swing_ids

    @property
    def total_s(self):
        """越限关节走完全部轨迹所需时长。"""
        return self.ramp_s + self.extra_s

    def swing_amp_at(self, jid, t):
        """摆动幅度（含渐入），渐入期内线性放大到设定值。"""
        a = self.amp_deg.get(jid, 0.0)
        if SWING_FADE_IN_S <= 0.0:
            return a
        k = min(1.0, t / SWING_FADE_IN_S)
        return a * k

    def deg_at(self, jid, t):
        """t 秒时的目标角（度）。"""
        if jid == self.test_id:
            # 段①：start_deg → stop_deg（限位值），耗时 ramp_s
            if t < self.ramp_s:
                s = quintic(t / self.ramp_s)
                return self.start_deg + (self.stop_deg - self.start_deg) * s
            # 段②：stop_deg → target_deg（越限），耗时 extra_s
            if self.extra_s <= 0.0:
                return self.stop_deg
            s = quintic((t - self.ramp_s) / self.extra_s)
            return self.stop_deg + (self.target_deg - self.stop_deg) * s
        center = self.centers_deg.get(jid, 0.0)
        amp = self.swing_amp_at(jid, t)
        if amp <= 0.0:
            return center
        return center + amp * math.sin(2.0 * math.pi * t / self.period_s)

    def targets_at(self, t):
        """t 秒时全部要下发的 (jid, 目标角deg) 列表。"""
        return [(jid, self.deg_at(jid, t)) for jid in self.ids]


def plan_swing(jid, center_deg, amp_deg):
    """
    把摆动幅度限制在该关节限位内（保留 SWING_MARGIN_DEG 余量）。

    返回 (可用幅度, 说明)。说明非空表示幅度被收窄或该关节不摆动 ——
    必须收窄，否则摆动自己撞到限位会触发软限位报警，干扰本次测试。
    """
    joint = id_to_joint(jid)
    lo, hi = JOINT_LIMIT_DEG[joint]
    room_up = hi - SWING_MARGIN_DEG - center_deg
    room_dn = center_deg - (lo + SWING_MARGIN_DEG)
    room = min(room_up, room_dn)
    if room <= 0.0:
        return 0.0, "中心 %.2f° 距限位不足 %.0f°，不摆动（保持该角）" % (
            center_deg, SWING_MARGIN_DEG)
    if amp_deg > room:
        return room, "幅度 %.1f° → 收窄到 %.1f°（限位 [%+.0f°, %+.0f°]）" % (
            amp_deg, room, lo, hi)
    return amp_deg, ""


# =============================================================================
# 打印辅助
# =============================================================================

def side_text(side):
    if side == LIMIT_SIDE_MAX:
        return "上限"
    if side == LIMIT_SIDE_MIN:
        return "下限"
    return "未知(%d)" % side


def describe_alarm(jid, side, rad):
    """报警明细文本（含限位值、越限量）。"""
    if not (JOINT_ID_BASE <= jid < JOINT_ID_BASE + JOINT_PER_LEG * 4):
        return "未知关节 ID %s" % jid
    joint = id_to_joint(jid)
    lo, hi = JOINT_LIMIT_DEG[joint]
    limit_deg = hi if side == LIMIT_SIDE_MAX else lo
    deg = rad * DEG_PER_RAD
    return ("ID%-3d %-7s 到达%s %+.1f°（实际 %+.2f°，越限 %+.2f°）"
            % (jid, joint_name(jid), side_text(side), limit_deg, deg, deg - limit_deg))


def print_link_line(mon, prefix="      "):
    """链路/状态一行摘要。"""
    streaming, online, locked = mon.flags()
    print("%s心跳 %d 次（%.0fms 前）流模式=%s 样点在线=%s 软限位锁=%s | 收 %d 帧"
          % (prefix, mon.hb_count, mon.hb_age() * 1000.0,
             "是" if streaming else "否", "是" if online else "否",
             "是" if locked else "否", mon.link.rx_count))


def wait_link(mon, wait_s):
    """被动等心跳，返回是否通。"""
    print("[链路] 等待下位机心跳（ID 0x%X，%dms 周期），最多 %.0f s …"
          % (HEARTBEAT_ID, HEARTBEAT_MS, wait_s))
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        if mon.heartbeat_alive():
            print_link_line(mon, prefix="[链路] ✓ ")
            return True
        time.sleep(0.2)
    print("[链路] ✗ %.0f s 内未收到心跳，请检查：" % wait_s)
    print("        1) 下位机 walk_control_V1 是否已启动，上位机接口名与 --can 是否一致；")
    print("        2) 该通道是否已 up：sudo ./build/kcanctl up <接口名> 1000000；")
    print("        3) 两端是否接在**同一条**总线上、120Ω 终端电阻是否到位；")
    print("        4) 用 candump <接口名> 看是否有 161 号帧到达。")
    if mon.other_ids:
        print("        （另收到非协议帧，ID=%s —— 可能误接到了电机总线）"
              % " ".join("%X" % i for i in sorted(mon.other_ids)))
    return False


def ping(link, mon, ids, wait_s=0.8):
    """向指定 ID 发 [0x41] 状态请求，返回有回复的 ID 列表。"""
    before = {}
    for jid in ids:
        before[jid] = mon.fb_count[jid]
    for jid in ids:
        link.send(jid, bytes([CMD_GET_STAT]))
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        if all(mon.fb_count[jid] > before[jid] for jid in ids):
            break
        time.sleep(0.02)
    return [jid for jid in ids if mon.fb_count[jid] > before[jid]]


def send_stream_stop(link, jid):
    """发 [0x02] 让下位机退出流模式（电机保持当前目标，不抱闸）。"""
    link.send(jid, bytes([CMD_STOP]))


def send_limit_clear(link, jid):
    """发 [0x03] 清除下位机软限位报警锁定。"""
    link.send(jid, bytes([CMD_CLEAR_LIMIT]))


def send_target(link, jid, deg):
    """
    发一帧 [0x1E][int32 小端 = 关节端角(rad) × ANGLE_SCALE]。

    ★ 必须走 rad_to_proto 完成定点缩放（×10000）。直接把度换算成 rad 后交给
      pack_i32le 是错的：int(1.658) = 1，下位机解析成 1/10000 rad ≈ 0.006°，
      表现为「目标发出去但关节不动、也不越限报警」。
    """
    link.send(jid, bytes([CMD_SET_POS]) + pack_i32le(rad_to_proto(deg * RAD_PER_DEG)))


def run_backoff(link, mon, jid, from_deg, to_deg, dur_s, hz, stop_event):
    """
    预退阶段：把注视关节从 from_deg 平滑退到 to_deg 并保持。

    用途：下位机有 2° 防抖（clear 后必须退离限位 2° 才重新武装）。若上一次
    测试已把关节压在限位上，直接重跑不会再次报警，必须先退离。

    返回 (是否走完, 结束时的实际角或 None)。
    """
    period = 1.0 / hz
    n = max(1, int(round(dur_s * hz)))
    t0 = time.monotonic()
    for i in range(n):
        if stop_event.is_set():
            return False, mon.fb_deg(jid)
        n_alarm, a_jid, a_side, a_rad, _ = mon.alarm()
        if n_alarm > 0:
            print("      [警告] 预退过程中收到 0x162：%s"
                  % describe_alarm(a_jid, a_side, a_rad))
            return False, mon.fb_deg(jid)
        t = time.monotonic() - t0
        deg = from_deg + (to_deg - from_deg) * quintic(t / max(0.01, dur_s))
        send_target(link, jid, deg)
        delay = t0 + (i + 1) * period - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    return True, mon.fb_deg(jid)


# =============================================================================
# 主流程
# =============================================================================

def build_parser():
    p = argparse.ArgumentParser(
        prog="limit_test.py",
        description="软限位专项测试：越限关节触发下位机钳位 + 0x162 报警，其余关节摆动便于观察",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""示例：
  python3 tools/limit_test.py --can can4                   # 标准用法（先检测链路，回车后发送）
  python3 tools/limit_test.py --can can4 --dry-run          # 无硬件自测：只打印波形与目标角
  python3 tools/limit_test.py --can can4 --target-deg 130   # 越限更深（仍会被钳位到 120°）
  python3 tools/limit_test.py --can can4 --ramp-s 20        # 更慢：20 s 走完五次多项式轨迹
  python3 tools/limit_test.py --can can4 --duration 60      # 跑 60 s
  python3 tools/limit_test.py --can can4 --amp-deg 5        # 摆动幅度改为 ±5°
""")
    p.add_argument("--can", default="can4",
                   help="上位机 CAN 接口名（须与下位机启动时输入一致，默认 can4）")
    p.add_argument("--test-id", type=int, default=1,
                   help="越限测试的全局关节 ID（默认 1 = L1_J1，限位 ±120°）")
    p.add_argument("--target-deg", type=float, default=125.0,
                   help="越限关节的目标角（度，默认 %(default)s；超过限位即被下位机钳位）")
    p.add_argument("--ramp-s", type=float, default=10.0,
                   help="越限关节从当前角爬到**限位值**的时长（秒，默认 %(default)s）；"
                        "走五次多项式平滑轨迹，首尾速度/加速度均为 0；"
                        "越限那一步另用 1/3 时长完成")
    p.add_argument("--amp-deg", type=float, default=10.0,
                   help="其余关节摆动幅度（度，默认 ±%(default)s）")
    p.add_argument("--period-s", type=float, default=5.0,
                   help="摆动周期（秒，默认 %(default)s）")
    p.add_argument("--duration", type=float, default=45.0,
                   help="发送总时长（秒，默认 %(default)s；需大于 --ramp-s 才留有观察余量）")
    p.add_argument("--hz", type=float, default=10.0,
                   help="目标流频率 Hz（默认 %(default)s，须与协议一致）")
    p.add_argument("--wait-link", type=float, default=10.0,
                   help="链路检测最长等待秒数（默认 %(default)s）")
    p.add_argument("--auto", action="store_true",
                   help="跳过回车确认，链路通 + 目标打印完立即开始（脚本化用）")
    p.add_argument("--dry-run", action="store_true",
                   help="不打开 CAN 接口，只打印将要下发的波形（无硬件自测）")
    p.add_argument("--clear-on-exit", action="store_true",
                   help="退出前发 [0x03] 解除下位机软限位锁定（默认不清除，便于用 stat 观察）")
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

    test_id = args.test_id
    if not (JOINT_ID_BASE <= test_id < JOINT_ID_BASE + JOINT_PER_LEG * 4):
        print("[错误] --test-id 必须在 1~16 之间")
        return 1
    swing_ids = [jid for jid in leg_ids(test_id) if jid != test_id]
    test_joint = id_to_joint(test_id)
    lo, hi = JOINT_LIMIT_DEG[test_joint]

    print("=" * 78)
    print("  软限位专项测试 limit_test —— 验证下位机钳位 / 0x162 报警 / 整机锁定")
    print("=" * 78)
    print("  角色  ：上位机（本程序） → CAN → 下位机 walk_control_V1 → 电机总线")
    print("  接口  ：%s%s" % (args.can, "（dry-run，不打开真实接口）" if args.dry_run else ""))
    print("  帧格式：ID 1~16，DLC=5，[0x1E][int32 小端 = 关节端角(rad) × %.0f]" % ANGLE_SCALE)
    print("  频率  ：%.1f Hz，时长 %.0f s" % (args.hz, args.duration))
    print("-" * 78)
    print("  测试关节：ID%-3d %-7s 限位 [%+.0f°, %+.0f°]，目标 %+.1f°（越限 → 下位机钳位）"
          % (test_id, joint_name(test_id), lo, hi, args.target_deg))
    print("  摆动关节：%s（围绕各自启动时的实际角，幅度 ±%.1f°，周期 %.1f s）"
          % (", ".join("ID%d %s" % (j, joint_name(j)) for j in swing_ids),
             args.amp_deg, args.period_s))
    print("  报警反应：收到 0x162 立即停发并退出（%s）"
          % ("退出前发 [0x03] 解锁" if args.clear_on_exit else "保留锁定，需手动 clear"))
    print("-" * 78)

    # ---- 打开链路 / dry-run ----
    link = None
    mon = None
    dry = args.dry_run
    if dry:
        print("[1/4] dry-run：跳过 CAN 接口打开与链路检测")
        # 无硬件时统一用 0° 作为起始角，便于观察波形
        start_deg = 0.0
        centers = {jid: 0.0 for jid in swing_ids}
    else:
        print("[1/4] 打开 CAN 接口 %s …" % args.can)
        try:
            link = CanLink(args.can)
        except OSError as e:
            print("[错误] %s" % e)
            print("       排查：① 接口名是否正确（ip -brief link show | grep can）；")
            print("             ② 通道是否已 up：sudo ./build/kcanctl up %s 1000000；" % args.can)
            print("             ③ 是否与下位机启动时输入的上位机接口名一致。")
            return 1
        print("      ✓ 已打开 %s" % args.can)

        mon = Monitor(link)
        mon.start()

        print("[2/4] 链路检测")
        if not wait_link(mon, args.wait_link):
            print("\n[终止] 通讯未打通，未发送任何关节目标。")
            mon.stop()
            link.close()
            return 2

        print("      主动握手：向 ID %s 发 [0x41] 状态请求 …"
              % ",".join(str(j) for j in leg_ids(test_id)))
        replied = ping(link, mon, leg_ids(test_id))
        print("      ✓ %d/4 个 ID 有回复" % len(replied))
        if test_id not in replied:
            print("[终止] 测试关节 ID%d %s 无反馈，无法确定起始角，未发送任何目标。"
                  % (test_id, joint_name(test_id)))
            print("       请确认该腿已在下位机启用、电机在线（下位机启动命令输 1 或回车）。")
            mon.stop()
            link.close()
            return 3
        missing = [j for j in leg_ids(test_id) if j not in replied]
        if missing:
            print("      [注意] %s 无反馈，将不参与摆动（不影响限位测试）"
                  % ", ".join("ID%d %s" % (j, joint_name(j)) for j in missing))

        start_deg = mon.fb_deg(test_id)
        centers = {jid: mon.fb_deg(jid) for jid in swing_ids}

        # 清掉上一轮的锁定，否则下位机会丢弃所有目标
        if mon.limit_locked():
            print("      [软限位] 下位机正处于报警锁定，发送 [0x03] 尝试解除 …")
        send_limit_clear(link, test_id)
        time.sleep(0.2)
        if mon.limit_locked():
            print("      [警告] 仍处锁定：可能是关节还压在限位上。请先退离限位 / 键盘 clear。")
        print("      当前实际角：%s"
              % "  ".join("ID%d %s %+.2f°" % (jid, joint_name(jid), d)
                          for jid, d in mon.fb_deg_all(leg_ids(test_id)) if d is not None))

        # —— 预退：上次测试可能把关节压在限位上，而 2° 防抖会让本次重启后
        #    不再报警。此处先把关节退到限位内侧，保证报警可复现。
        back_deg = None
        if start_deg is not None:
            near_hi = hi - start_deg
            near_lo = start_deg - lo
            if near_hi < BACKOFF_TRIGGER_DEG:
                back_deg = hi - BACKOFF_DEPTH_DEG
                side_name = "上限"
            elif near_lo < BACKOFF_TRIGGER_DEG:
                back_deg = lo + BACKOFF_DEPTH_DEG
                side_name = "下限"
            else:
                side_name = None
            if back_deg is not None:
                print("      [预退] ID%d %s 当前 %+.2f° 距%s不足 %.0f°（防抖需退离 %.1f°），"
                      "先退到 %+.1f°"
                      % (test_id, joint_name(test_id), start_deg, side_name,
                         BACKOFF_TRIGGER_DEG, REARM_MARGIN_DEG, back_deg))
                ok, cur = run_backoff(link, mon, test_id, start_deg, back_deg,
                                      min(args.ramp_s, 3.0), args.hz, g_stop)
                if not ok:
                    print("[终止] 预退未完成，未进入测试阶段。")
                    send_stream_stop(link, test_id)
                    mon.stop()
                    link.close()
                    return 5
                if cur is not None:
                    print("      [预退] 完成，实际角 %+.2f°，可重新触发报警" % cur)
                    start_deg = cur

    # ---- 构建波形 ----
    swing_amps = {}
    for jid in swing_ids:
        c = centers.get(jid)
        if c is None:
            swing_amps[jid] = 0.0
            print("      [摆动] ID%d %s 无反馈，跳过" % (jid, joint_name(jid)))
            continue
        amp, note = plan_swing(jid, c, args.amp_deg)
        swing_amps[jid] = amp
        print("      [摆动] ID%d %s 中心 %+.2f° 幅度 ±%.1f°%s"
              % (jid, joint_name(jid), c, amp, ("（%s）" % note) if note else ""))
        centers[jid] = c

    # ---- 确定主段的终点（限位值）与越限段的起点 ----
    over = args.target_deg > hi + 1e-9 or args.target_deg < lo - 1e-9
    clamped = hi if args.target_deg > hi else lo

    plan = Plan(test_id=test_id,
                start_deg=start_deg if start_deg is not None else 0.0,
                target_deg=args.target_deg,
                # 越限时：主段只到限位值，越限量用短促的追加段完成
                stop_deg=clamped if over else args.target_deg,
                ramp_s=args.ramp_s,
                # 幅度为 0（无反馈 / 距限位太近）的关节**完全不发**，
                # 保持原姿态，避免因为它们被拉向 0° 而干扰观察
                swing_ids=[j for j in swing_ids if swing_amps.get(j, 0.0) > 0.0],
                centers_deg={j: centers.get(j, 0.0) for j in swing_ids},
                amp_deg=swing_amps,
                period_s=args.period_s)

    # ---- 打印将要下发的目标（人工确认前） ----
    dist = abs(clamped - plan.start_deg)
    # 五次多项式峰值斜率 = 1.875×平均速度（出现于 t = ramp_s/2）
    peak_speed = 1.875 * dist / max(0.01, args.ramp_s)
    print("-" * 78)
    print("[目标] ID%-3d %-7s：当前 %+.2f° → 目标 %+.2f°（%s）"
          % (test_id, joint_name(test_id), plan.start_deg, args.target_deg,
             "越限，下位机将钳位到 %+.1f°" % clamped if over else "限位内，不会被钳位"))
    if over:
        # 五次曲线的峰值加速度（归一化）= 5.7735
        peak_acc = 5.7735 * dist / max(0.01, args.ramp_s) ** 2
        print("       段①：五次多项式 %.1f s 从 %+.2f° 平滑走到限位 %+.1f°（%.1f°），"
              "峰值 %.1f°/s ≈ %.1f°/帧，峰值加速度 %.2f°/s²"
              % (args.ramp_s, plan.start_deg, clamped, dist,
                 peak_speed, peak_speed / args.hz, peak_acc))
        print("       （首尾速度、加速度均为 0，起停无冲击）")
        print("       段②：再用 %.1f s 从 %+.1f° 推到 %+.2f°（越限量 %.1f°，被下位机钳位）"
              % (plan.extra_s, clamped, args.target_deg,
                 abs(args.target_deg - clamped)))
        if peak_speed / args.hz > 2.0:
            # 要让峰值 ≤2°/帧：ramp_s = 1.875 × 距离 / (2 × hz)
            suggest = 1.875 * dist / (2.0 * args.hz)
            print("       [提示] 峰值超过协议建议的 ≤2°/帧，电机 PD 会有追踪滞后；"
                  "想更平缓可加 --ramp-s %.1f" % suggest)
        print("       目标保持越限值 → 实际角停在下位机限位 %+.1f°，"
              "进入 %.1f° 内即触发报警" % (clamped, LOWER_TRIGGER_MARGIN_DEG))
    print("       预期：段①结束（%.1f s）到达限位、段②走完（%.1f s）后持续越限，"
          "随即收到 ID 0x%X 报警帧" % (args.ramp_s, plan.total_s, LIMIT_ALARM_ID))

    # ---- 等确认 ----
    if args.auto:
        print("[3/4] --auto：跳过确认，直接开始发送")
    elif dry:
        print("[3/4] dry-run：模拟发送（不写 CAN）")
    else:
        try:
            ans = input("[3/4] 确认上述目标后按回车开始发送（%.0f s，共约 %d 帧）；"
                        "输入 q 退出："
                        % (args.duration, int(args.duration * args.hz) * len(plan.ids))
                        ).strip().lower()
        except EOFError:
            ans = "q"
        if ans in ("q", "quit", "exit"):
            print("[退出] 用户取消，未发送任何关节目标")
            send_limit_clear(link, test_id)
            mon.stop()
            link.close()
            return 0

    # ---- 发送循环 ----
    print("[4/4] 开始发送：%.1f Hz，时长 %.1f s，ID %s"
          % (args.hz, args.duration, ",".join(str(j) for j in plan.ids)))
    period = 1.0 / args.hz
    n_ticks = max(1, int(round(args.duration * args.hz)))
    t0 = time.monotonic()
    sent_ticks = 0
    alarm_info = None
    hb_warned = False
    next_report = t0 + 1.0
    exit_code = 0

    try:
        for i in range(n_ticks):
            if g_stop.is_set():
                break
            now_mono = time.monotonic()
            t = now_mono - t0

            # 报警：立即停发并退出
            if mon is not None:
                n_alarm, a_jid, a_side, a_rad, a_first = mon.alarm()
                if n_alarm > 0:
                    alarm_info = (n_alarm, a_jid, a_side, a_rad, a_first - t0, t)
                    break

            # 心跳丢失保护
            if mon is not None and i > int(1.5 * args.hz) and not mon.heartbeat_alive(2.0):
                if not hb_warned:
                    print("\n      [警告] 超过 2 s 未收到下位机心跳，链路可能已断")
                    hb_warned = True
                if not mon.heartbeat_alive(4.0):
                    print("      [终止] 心跳中断 4 s，停止发送")
                    exit_code = 4
                    break

            for jid, deg in plan.targets_at(t):
                if dry:
                    continue
                send_target(link, jid, deg)
            sent_ticks += 1

            # 1Hz 状态
            if now_mono >= next_report:
                next_report = now_mono + 1.0
                parts = []
                for jid, tgt in plan.targets_at(t):
                    act = mon.fb_deg(jid) if mon is not None else None
                    parts.append("ID%d %+7.2f/%s"
                                 % (jid, tgt,
                                    ("%+7.2f" % act) if act is not None else "   ——  "))
                streaming, online, locked = mon.flags() if mon is not None else (False, False, False)
                print("      t=%5.1fs 目标/实际 %s | 流=%s 样点=%s 锁=%s"
                      % (t, "  ".join(parts),
                         "是" if streaming else "否", "是" if online else "否",
                         "是" if locked else "否"))

            delay = t0 + (i + 1) * period - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    finally:
        if link is not None:
            try:
                send_stream_stop(link, test_id)
                print("\n[发送] 已发 [0x02] 通知下位机退出流模式（电机保持当前目标，不抱闸）")
            except OSError as e:
                print("\n[警告] 发送 0x02 失败：%s" % e)
            if alarm_info is not None and args.clear_on_exit:
                send_limit_clear(link, test_id)
                print("[发送] 已发 [0x03] 清除软限位锁定")

    elapsed = time.monotonic() - t0
    print("-" * 78)
    print("[统计] 发送 %d 个 tick（%.2f s，平均 %.2f Hz）"
          % (sent_ticks, elapsed, (sent_ticks / elapsed) if elapsed > 0 else 0.0))
    if link is not None:
        print("       CAN 发 %d 帧，收 %d 帧" % (link.tx_count, link.rx_count))
    if mon is not None:
        streaming, online, locked = mon.flags()
        print("       下位机心跳 %d 次（最后 %.0fms 前），流模式=%s，样点在线=%s，软限位锁=%s"
              % (mon.hb_count, mon.hb_age() * 1000.0,
                 "是" if streaming else "否", "是" if online else "否",
                 "是" if locked else "否"))
        final = mon.fb_deg_all(leg_ids(test_id))
        print("       最终实际角：%s"
              % "  ".join("ID%d %s %+7.2f°" % (jid, joint_name(jid), d)
                          for jid, d in final if d is not None))

    # ---- 结论 ----
    print("-" * 78)
    if alarm_info is not None:
        n_alarm, a_jid, a_side, a_rad, a_t, stop_t = alarm_info
        print("[结果] ✓ 软限位报警已触发并正确上报")
        print("       报警明细：%s" % describe_alarm(a_jid, a_side, a_rad))
        print("       首帧到达：t=%.2f s（五次多项式轨迹设定 %.1f s）；累计收到 %d 帧（%dms 重发≈%.1f 帧/秒）"
              % (a_t, args.ramp_s, n_alarm, LIMIT_ALARM_MS,
                 1000.0 / LIMIT_ALARM_MS))
        print("       本程序在 t=%.2f s 停止发送（报警即停）" % stop_t)
        if a_jid != test_id:
            print("       [注意] 报警关节是 ID%d %s，不是预期的 ID%d %s —— "
                  "请检查是否有摆动关节自身撞到了限位（减小 --amp-deg）"
                  % (a_jid, joint_name(a_jid), test_id, joint_name(test_id)))
        if mon is not None and mon.limit_locked():
            print("       下位机当前处于**整机锁定**：%s"
                  % ("已随退出发送 [0x03] 解除" if args.clear_on_exit
                     else "需在键盘输入 clear，或下次运行时自动发 [0x03] 解除"))
    elif exit_code == 0:
        print("[结果] ✗ 未收到 0x162 软限位报警")
        if mon is not None:
            act = mon.fb_deg(test_id)
            if act is not None:
                print("       ID%d %s 最终实际角 %+.2f°，限位 %+.1f°，"
                      "触发门限 %+.1f°（距门限 %.2f°）"
                      % (test_id, joint_name(test_id), act, clamped,
                         clamped - (LOWER_TRIGGER_MARGIN_DEG if clamped > 0 else -LOWER_TRIGGER_MARGIN_DEG),
                         abs(clamped - act)))
                print("       常见原因：① PD 静差使实际角停在门限外 —— 可加大 --target-deg"
                      "（仍被钳位，只影响不到门限）无助于此，"
                      "实际应提高下位机 kPositionKp 或检查负载/摩擦；")
                print("                   ② 该关节未使能或电机离线（下位机启动时应有"
                      " [腿X][使能] 日志）；")
                print("                   ③ 目标被下位机丢弃（下位机是否在急停锁定 / 上位机接口不一致）。")
        print("       可再看下位机日志：是否出现 [钳位] 与 [软限位][报警] 行。")
    else:
        print("[结果] 异常结束（exit=%d），详见上方日志" % exit_code)

    if mon is not None:
        mon.stop()
    if link is not None:
        link.close()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())