#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成上位机 ⇄ 下位机（本机）软件通讯协议文档 PDF。

输出：docs/step_protocol.pdf
依赖：reportlab（pip install --user --break-system-packages reportlab）
字体：Noto Sans CJK（系统自带 .ttc，reportlab 需 subfontIndex 选择子字体）
"""
import os

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    HRFlowable, KeepTogether, PageBreak, Paragraph, SimpleDocTemplate,
    Spacer, Table, TableStyle,
)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "docs", "step_protocol.pdf")

# ---------------------------------------------------------------- 字体
FONT = "DroidCJK"
FONT_BOLD = "DroidCJK-Bold"
# Noto .ttc 为 PostScript 轮廓（reportlab 不支持）；Droid Sans Fallback
# 为 TrueType 轮廓且完整覆盖 CJK，含常规与粗体近似的单字重。
_DROID = "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"
if os.path.exists(_DROID):
    pdfmetrics.registerFont(TTFont(FONT, _DROID))
    pdfmetrics.registerFont(TTFont(FONT_BOLD, _DROID))
else:
    raise SystemExit("未找到 DroidSansFallbackFull.ttf（fonts-droid-fallback）")

# ---------------------------------------------------------------- 样式
styles = getSampleStyleSheet()
H1 = ParagraphStyle("H1", parent=styles["Heading1"], fontName=FONT_BOLD,
                    fontSize=16, spaceAfter=6, textColor=colors.HexColor("#1a3a5c"))
H2 = ParagraphStyle("H2", parent=styles["Heading2"], fontName=FONT_BOLD,
                    fontSize=13, spaceBefore=10, spaceAfter=4,
                    textColor=colors.HexColor("#1a3a5c"))
H3 = ParagraphStyle("H3", parent=styles["Heading3"], fontName=FONT_BOLD,
                    fontSize=11, spaceBefore=8, spaceAfter=3)
BODY = ParagraphStyle("BODY", parent=styles["BodyText"], fontName=FONT,
                      fontSize=9.5, leading=14)
CODE = ParagraphStyle("CODE", parent=BODY, fontName=FONT, fontSize=8.5, leading=11,
                      backColor=colors.HexColor("#f5f5f5"), borderPadding=4,
                      leftIndent=2 * mm)
TCELL = ParagraphStyle("TCELL", parent=BODY, fontSize=8.5, leading=11,
                       spaceBefore=0, spaceAfter=0)
THEAD = ParagraphStyle("THEAD", parent=TCELL, fontName=FONT_BOLD,
                       textColor=colors.white)
TITLE = ParagraphStyle("TITLE", parent=H1, fontSize=20, alignment=TA_CENTER,
                       spaceAfter=2)
SUB = ParagraphStyle("SUB", parent=BODY, alignment=TA_CENTER, fontSize=10,
                     textColor=colors.HexColor("#555555"))


def P(text, style=BODY):
    return Paragraph(text, style)


def code(lines):
    return Paragraph("<br/>".join(lines).replace(" ", "&nbsp;"), CODE)


def table(head, rows, widths):
    data = [[Paragraph(h, THEAD) for h in head]]
    data += [[Paragraph(c, TCELL) for c in r] for r in rows]
    t = Table(data, colWidths=widths, repeatRows=1, hAlign="LEFT")
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1a3a5c")),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1),
         [colors.white, colors.HexColor("#eef2f6")]),
        ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#9aa5b1")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
        ("LEFTPADDING", (0, 0), (-1, -1), 4),
        ("RIGHTPADDING", (0, 0), (-1, -1), 4),
    ]))
    return t


# ================================================================ 文档
doc = SimpleDocTemplate(
    OUT, pagesize=A4,
    leftMargin=18 * mm, rightMargin=18 * mm,
    topMargin=16 * mm, bottomMargin=16 * mm,
    title="四足机器人 上位机⇄下位机 轨迹流通讯协议",
    author="TaiHu_motor_control")

story = []

# ---------------- 封面
story += [
    Spacer(1, 30 * mm),
    Paragraph("四足机器人步态轨迹流", TITLE),
    Paragraph("上位机 ⇄ 下位机（本机）软件通讯协议与接口 API", TITLE),
    Spacer(1, 6 * mm),
    HRFlowable(width="60%", thickness=1, color=colors.HexColor("#1a3a5c")),
    Spacer(1, 6 * mm),
    Paragraph("项目：TaiHu_motor_control / walk_control_V1", SUB),
    Paragraph("协议版本：V1.0　|　物理层：CAN 2.0A（标准帧，1&nbsp;Mbps）"
              "　|　通讯频率：10&nbsp;Hz", SUB),
    Spacer(1, 200 * mm),
    Paragraph("本文档描述上位机（轨迹规划层）与下位机（本机，执行 "
              "walk_control_V1 程序）之间的 CAN 通讯协议。协议设计与钛虎"
              "（DM/TaiHu）电机控制协议<b>同构</b>——上位机发送 0x1E/0x41/"
              "0x02 功能码帧，与直接控制钛虎电机的代码完全复用。", BODY),
    PageBreak(),
]

# ---------------- 1 系统架构
story += [
    P("1. 系统架构与角色划分", H1),
    code([
        "┌──────────────────┐   10Hz 轨迹流    ┌─────────────────────────────┐",
        "│      上位机       │  CAN 2.0A 1Mbps │      下位机（本电脑）        │",
        "│  （步态规划/遥操） │ ───────────────▶│  运行 walk_control_V1       │",
        "│                  │   0x141~0x144    │  ┌───────────────────────┐  │",
        "│  鲲弘 USB-CAN 卡  │ ◀─────────────── │  │ 协议线程：收样点/心跳  │  │",
        "└──────────────────┘   0x161/0x41回复  │  ├───────────────────────┤  │",
        "                                      │  │ 执行线程：5ms 下发     │  │",
        "                                      │  ├───────────────────────┤  │",
        "                                      │  │ can0~can3 → 4×4 电机  │  │",
        "                                      │  └───────────────────────┘  │",
        "                                      └─────────────────────────────┘",
    ]),
    Spacer(1, 3 * mm),
    P("职责划分："),
    table(["层级", "设备/程序", "职责"], [
        ["上位机", "步态规划电脑（鲲弘 USB-CAN 卡）",
         "步态/轨迹规划，输出 10Hz 关节角样点流；断链检测；急停下发"],
        ["下位机", "本电脑 + walk_control_V1",
         "接收样点流；软件限位保护；换算为电机端角并以 5ms 周期下发到 "
         "can0~can3 上的 16 个钛虎电机；反馈状态与心跳"],
    ], [22 * mm, 52 * mm, 98 * mm]),
    Spacer(1, 2 * mm),
    P("<b>物理连接</b>：上位机 CAN 卡的 CAN_H/CAN_L 接入本机 can0 所在总线"
      "（与腿 1 电机同一条总线）。总线两端各并联 120Ω 终端电阻。波特率 "
      "1 Mbps（`sudo ./build/kcanctl up can0 1000000`）。"),
    P("<b>总线负载估算</b>：上位机 4 帧/周期 × 10Hz + 下位机心跳 5Hz + "
      "腿 1 电机自身报文 ≈ 50 帧/s，远低于 1Mbps 总线容量（约 8000 帧/s）"
      "，无冲突风险。"),
]

# ---------------- 2 协议总览
story += [
    P("2. 协议总览：虚拟电机设计", H2),
    P("核心思想：把下位机的 4 条腿抽象为 4 个<b>虚拟电机</b>。帧 ID "
      "0x141~0x144 对应腿 1~4（避开每条总线上真电机 ID 1~4）。上位机"
      "发来的帧与控制真钛虎电机的帧格式完全同构，上位机代码无需为新"
      "协议写专用编码——控制真实电机与控制本下位机使用同一套收发逻辑。"),
    table(["方向", "帧 ID", "DLC", "功能码", "含义"], [
        ["上位机→下位机", "0x141~0x144", "8", "0x1E",
         "整腿目标位置帧：J1~J4 关节角（详见 §3.1）"],
        ["上位机→下位机", "0x141~0x144", "1", "0x02",
         "退出流模式：电机保持当前目标，不抱闸"],
        ["上位机→下位机", "0x141~0x144", "1", "0x41",
         "状态请求：下位机立即回同 ID 反馈帧（§3.3）"],
        ["下位机→上位机", "0x141~0x144", "8", "0x41",
         "状态反馈：J1~J4 当前目标角（§3.3）"],
        ["下位机→上位机", "0x161", "1", "0x3F",
         "心跳，200ms 周期：bit0=流模式中，bit1=样点在线"],
    ], [30 * mm, 26 * mm, 12 * mm, 18 * mm, 86 * mm]),
    Spacer(1, 2 * mm),
    P("帧 ID 与腿号换算：<b>腿号 = 帧ID − 0x140</b>（0x141→腿1，…，"
      "0x144→腿4）。全部为 CAN 2.0A 标准帧（11 位 ID），数据字段多字节"
      "一律<b>小端序</b>（低字节在前，与钛虎电机协议一致）。"),
]

# ---------------- 3 帧格式
story += [
    P("3. 帧格式详解", H2),

    P("3.1 目标位置帧（0x1E，上位机→下位机）", H3),
    P("一帧携带<b>一条腿四个关节</b>的目标角（整腿帧），10Hz 周期发送。"
      "收到首帧即进入流模式；之后每个样点收到即生效（覆盖式，始终追踪"
      "最新目标）。"),
    table(["字节", "0", "1~2", "3~4", "5~6", "7~8"], [
        ["内容", "0x1E", "J1 角度", "J2 角度", "J3 角度", "J4 角度"],
        ["类型", "uint8", "int16 LE", "int16 LE", "int16 LE", "int16 LE"],
        ["单位", "功能码", "0.01°/LSB", "0.01°/LSB", "0.01°/LSB", "0.01°/LSB"],
    ], [16 * mm, 14 * mm, 26 * mm, 26 * mm, 26 * mm, 26 * mm]),
    Spacer(1, 2 * mm),
    P("角度编码：<b>定点 1 LSB = 0.01°</b>，int16 有符号小端，范围 "
      "±327.67°（覆盖全部关节行程）。例：J2 = +45.32° → 4532 = 0x11B4 "
      "→ 字节序 B4 11。负角按补码：−30.00° → −3000 = 0xF448 → 48 F4。"),
    P("<b>语义</b>：角度为关节端<b>绝对角</b>（度），正值 = 逆时针（ccw）。"
      "下位机内部自动完成关节端 → 电机端换算（J2/J3 锥齿轮 ×2）并下发。"),

    P("3.2 退出流模式帧（0x02，上位机→下位机）", H3),
    P("DLC=1，数据 [0x02]。收到后下位机退出流模式，各电机<b>保持当前</b>"
      "目标位置（电机内部位置环维持，不抱闸）。用于正常结束轨迹回放。"
      "同语义帧可发到任意 0x141~0x144 ID。"),

    P("3.3 状态请求与反馈帧（0x41）", H3),
    P("上位机发 [0x41]（DLC=1，ID=0x141~0x144），下位机立即在同 ID 回 8 "
      "字节反馈："),
    table(["字节", "0", "1~2", "3~4", "5~6", "7~8"], [
        ["内容", "0x41", "J1 角度", "J2 角度", "J3 角度", "J4 角度"],
        ["类型", "uint8", "int16 LE", "int16 LE", "int16 LE", "int16 LE"],
        ["单位", "功能码", "0.01°/LSB", "0.01°/LSB", "0.01°/LSB", "0.01°/LSB"],
    ], [16 * mm, 14 * mm, 26 * mm, 26 * mm, 26 * mm, 26 * mm]),
    Spacer(1, 2 * mm),
    P("反馈内容为该腿最近一次<b>下发</b>的目标角（关节端度）。上位机"
      "周期性（如 1Hz）轮询 0x41 并与心跳比对，可确认下位机存活且指令"
      "通道畅通。"),

    P("3.4 心跳帧（0x3F，下位机→上位机）", H3),
    table(["字段", "帧 ID", "DLC", "数据"], [
        ["值", "0x161", "1", "状态位：bit0=流模式中；bit1=最近1s有新样点"],
    ], [22 * mm, 30 * mm, 12 * mm, 104 * mm]),
    Spacer(1, 2 * mm),
    P("200ms 固定周期发送。上位机超过 600ms 未收到心跳 → 判定下位机"
      "失联，应停止轨迹规划并进入安全策略。"),
]

# ---------------- 4 时序与执行语义
story += [
    P("4. 10Hz 时序与执行语义", H2),
    code([
        "上位机时刻   0ms     100ms    200ms    300ms  ......",
        "0x1E帧      ──▶     ──▶      ──▶      ──▶     10Hz 稳定周期",
        "             样点k    样点k+1  样点k+2  样点k+3",
        "",
        "下位机协议线程（收到即存入各腿缓冲）",
        "             ↓push    ↓push    ↓push",
        "下位机执行线程（5ms 周期 streamTick）",
        "             ─────────────────────────────▶  持续重发最新目标",
        "             重发q_k 20次 → 重发q_k+1 20次 → ...",
        "",
        "真电机（钛虎，KP=4000 KD=60 位置环）",
        "             平滑追踪 10Hz 阶梯目标（样点间过渡由电机 PD 完成）",
    ]),
    Spacer(1, 2 * mm),
    P("执行策略（<b>覆盖式最新样点</b>，非队列回放）："),
    P("① 样点不排队——执行线程始终取「最新」样点作为目标，5ms 周期重发；"
      "上位机 10Hz 更新目标、电机 200ms 内跟上，天然无累积延迟。"
      "这是实时遥操/步态流的正确语义（丢帧只影响平滑度，不产生滞后堆积）。"),
    P("② 样点间过渡由电机固件内位置环（KP=4000/KD=60）完成，下位机不做"
      "额外软件插值，控制延迟最小。"),
    P("③ 上位机应保证样点序列本身平滑（相邻样点角变化 ≤ 2°/帧），"
      "10Hz 阶梯目标即被电机 PD 平滑追踪。"),
    P("典型周期分配：上位机侧规划+发送 ≤ 5ms；下位机协议线程入缓冲 "
      "&lt;1ms；执行线程 5ms 周期下发 16 电机 × 0x1E 写帧（纯写不等回复）"
      "约 1ms 总线占用。"),
]

# ---------------- 5 数据换算
story += [
    P("5. 角度换算与限位", H2),
    P("协议角度一律为<b>关节端</b>绝对角（度）。下位机内部换算："),
    code([
        "电机输出端角 = 关节角 × kMotorRadPerJointRad × kJointDirSign",
        "  J1（Y 直驱）  ：×1     J2（P 锥齿轮1:2）：×2",
        "  J3（P 锥齿轮1:2）：×2   J4（Y 直驱）    ：×1",
        "  方向符号：kJointDirSign = {+1, +1, +1, +1}（正角=逆时针）",
    ]),
    Spacer(1, 2 * mm),
    P("<b>软件限位</b>（关节端度，越限样点整帧丢弃并计数，电机保持上一"
      "目标）："),
    table(["关节", "构型", "电机 CAN ID", "限位（关节端）", "关节端总减速比"], [
        ["J1", "Y 直驱", "1", "±90°", "101"],
        ["J2", "P 锥齿轮 1:2", "2", "±120°", "81×2 = 162"],
        ["J3", "P 锥齿轮 1:2", "3", "−30° ~ +180°", "81×2 = 162"],
        ["J4", "Y 直驱", "4", "±90°", "101"],
    ], [18 * mm, 34 * mm, 26 * mm, 44 * mm, 44 * mm]),
]

# ---------------- 6 异常与保护
story += [
    P("6. 异常处理与安全保护", H2),
    table(["场景", "检测方", "判定条件", "动作"], [
        ["轨迹流断流", "下位机", "流模式中 1s 无新样点",
         "自动退出流模式，电机保持当前目标（不抱闸），打印告警"],
        ["上位机失联", "上位机", "600ms 无 0x161 心跳",
         "上位机停止规划，走安全策略（如发 0x02 后重启）"],
        ["样点越限", "下位机", "任一关节角超限位表",
         "丢弃该帧（电机保持），stat 命令可查丢弃计数"],
        ["下位机急停", "人工", "键盘 estop",
         "全部 16 电机发 0x02 抱闸 + 退出流模式（流模式中亦可响应）"],
        ["程序退出", "人工", "键盘 quit",
         "全部电机抱闸后关闭总线"],
        ["move/goto 冲突", "下位机", "流模式中收到 move/goto",
         "提示暂缓（流模式优先），不执行轨迹命令"],
    ], [28 * mm, 22 * mm, 44 * mm, 78 * mm]),
]

# ---------------- 7 上位机接口 API
story += [
    P("7. 上位机接口 API（C 参考实现）", H2),
    P("以下 C 函数封装协议帧编解码，可直接嵌入上位机工程（不依赖本仓库）："),
    code([
        "/* ---------- 轨迹流发送（10Hz 周期调用） ---------- */",
        "/* joint_deg[4]：J1~J4 关节端目标角（度，ccw 为正）",
        " * leg：1~4。帧 ID = 0x140 + leg，标准帧 DLC=8 小端 int16。 */",
        "int32_t host_send_leg_target(can_handle_t h, int leg,",
        "                              const double joint_deg[4]) {",
        "    uint8_t d[8] = {0};",
        "    d[0] = 0x1E;",
        "    for (int j = 0; j < 4; j++) {",
        "        int32_t v = (int32_t)lround(joint_deg[j] * 100.0);",
        "        if (v >  32767) v =  32767;      /* 越限饱和编码 */",
        "        if (v < -32768) v = -32768;",
        "        d[1+j*2]   = (uint8_t)(v & 0xFF);       /* 小端 */",
        "        d[2+j*2]   = (uint8_t)((v >> 8) & 0xFF);",
        "    }",
        "    return can_send(h, 0x140 + leg, 8, d);   /* 标准帧 */",
        "}",
        "",
        "/* ---------- 退出流模式（轨迹回放结束时调用） ---------- */",
        "int32_t host_stream_stop(can_handle_t h) {",
        "    uint8_t d[1] = {0x02};",
        "    return can_send(h, 0x141, 1, d);          /* 任意腿 ID 均可 */",
        "}",
        "",
        "/* ---------- 状态请求（可选，1Hz 轮询） ---------- */",
        "int32_t host_query_leg(can_handle_t h, int leg) {",
        "    uint8_t d[1] = {0x41};",
        "    return can_send(h, 0x140 + leg, 1, d);",
        "}",
        "/* 收反馈帧：ID∈{0x141..0x144}, DLC=8, data[0]==0x41：",
        " *   joint_deg[j] = (int16)(data[1+j*2] | data[2+j*2]<<8) / 100.0;",
        " * 收心跳帧：ID==0x161, DLC=1：",
        " *   stream_armed = data[0] & 0x01;  stream_online = data[0] & 0x02; */",
    ]),
    Spacer(1, 3 * mm),
    P("<b>上位机主循环调用流程</b>："),
    code([
        "can_init(1000000);                       /* 1. 初始化 CAN 卡    */",
        "for (t = 0; ; t += 0.1) {                /* 2. 10Hz 步进        */",
        "    plan_gait(t, joint_deg);             /*    步态规划         */",
        "    for (leg = 1; leg <= 4; leg++)       /*    逐腿发整腿帧     */",
        "        host_send_leg_target(can, leg, joint_deg[leg]);",
        "    if (heartbeat_timeout_600ms()) {     /* 3. 心跳监护         */",
        "        emergency();  return;             /*    失联→安全策略    */",
        "    }",
        "}",
        "host_stream_stop(can);                   /* 4. 结束：退出流模式 */",
    ]),
]

# ---------------- 8 下位机使用说明
story += [
    P("8. 下位机（本机）使用说明", H2),
    code([
        "# 1. 启动 CAN 接口（1Mbps）",
        "sudo ./build/kcanctl up can0 1000000     # 其余 can1~can3 同理",
        "",
        "# 2. 编译并启动下位机程序",
        "cmake --build build --target walk_control_V1",
        "./build/walk_control_V1",
        "#   → 选择启用腿号（直接回车 = 全部 4 腿）",
        "#   → 程序进入命令模式，同时协议线程监听 can0 上位机帧",
        "",
        "# 3. 运行期键盘命令",
        "pos [腿1-4]     # 查询关节当前位置",
        "stat            # 流模式统计（各腿收帧/丢弃/缓冲）",
        "estop           # 急停：全部电机抱闸 + 退出流模式",
        "quit            # 退出：电机抱闸后关闭总线",
    ]),
    Spacer(1, 2 * mm),
    P("下位机收到首帧 0x1E 后自动进入流模式（终端打印 `[流] 收到首帧 "
      "0x1E：进入流模式`）；1s 断流或收到 0x02 自动退出。流模式期间 "
      "move/goto 命令暂缓，pos/estop 始终可用。"),
    P("<b>测试建议</b>：先用 `candump can0` 观察上位机帧是否到达本机；"
      "下位机终端 `stat` 命令核对各腿收帧率应为 ≈10 帧/s（10Hz × 整腿帧"
      "×1）；腿 1 的真电机 0x141 反馈与心跳 0x161 同时出现在总线上属正常"
      "（不同 ID 不冲突）。"),
]

doc.build(story)
print("生成完成：", OUT)
