#!/usr/bin/env bash
# =====================================================================
#  鲲弘 KCAN-USB FD 六路适配器 —— 固定 can0~can5 与物理端口的映射
# =====================================================================
#
#  用法：
#      sudo ./install_kcan_persistent_name.sh            # 安装
#      sudo ./install_kcan_persistent_name.sh --check    # 只检查当前映射
#      sudo ./install_kcan_persistent_name.sh --uninstall # 卸载
#
#  背景：
#      kcan 驱动注册 netdevice 时是"抢空位"命名，不由物理端口决定。
#      RK3588 板载 CAN 先枚举抢走 can0，随后被 10-can-mcu.link 改名为
#      can_mcu，把 can0 空出来；接着枚举的 USB 第 5 个口恰好拿到 can0。
#      结果是 can0 ≠ 第 1 个物理口，且每次开机都可能漂移，导致上层
#      程序"勾选 can0、实际拉起别的通道"。
#
#  本脚本做两件事：
#      1) 安装 6 个 systemd .link 文件，按 USB 物理路径强制命名；
#      2) 立即重新触发命名，无需重启即可生效。
#
#  映射目标（物理口顺序 → 接口名）：
#      2-1.4.2.1 → can0
#      2-1.4.2.2 → can1
#      2-1.4.2.3 → can2
#      2-1.4.2.4 → can3
#      2-1.4.2.5 → can4
#      2-1.4.2.6 → can5
#
# =====================================================================

set -u

# ---- 配色（非终端环境自动降级）---------------------------------------
if [ -t 1 ]; then
    C_RED=$'\033[31m'; C_GRN=$'\033[32m'; C_YEL=$'\033[33m'
    C_CYA=$'\033[36m'; C_BLD=$'\033[1m';  C_RST=$'\033[0m'
else
    C_RED=''; C_GRN=''; C_YEL=''; C_CYA=''; C_BLD=''; C_RST=''
fi

ok()   { printf '%s[ OK ]%s %s\n' "$C_GRN" "$C_RST" "$*"; }
info() { printf '%s[INFO]%s %s\n' "$C_CYA" "$C_RST" "$*"; }
warn() { printf '%s[WARN]%s %s\n' "$C_YEL" "$C_RST" "$*"; }
err()  { printf '%s[FAIL]%s %s\n' "$C_RED" "$C_RST" "$*" >&2; }

# ---- 常量 ------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LINK_DIR="/etc/systemd/network"
LINK_PREFIX="60-kcan-persistent-name"
USB_CTRL_PATH="platform-fc880000.usb"
# 物理端口号（2-1.4.2.N 的 N）→ 目标接口名
PHYS_PORTS="1 2 3 4 5 6"

# ---- 前置检查 --------------------------------------------------------
require_root()
{
    if [ "$(id -u)" -ne 0 ]; then
        err "本脚本需要 root 权限。请用： sudo $0 $*"
        exit 1
    fi
}

have_cmd()
{
    command -v "$1" >/dev/null 2>&1
}

# 查询某个接口的 udev ID_PATH（用于核对映射）
get_id_path()
{
    udevadm info -q property -p "/sys/class/net/$1" 2>/dev/null \
        | sed -n 's/^ID_PATH=//p' | head -1
}

# 把 ID_PATH 里的 USB 端口号抠出来，例如 ...-usb-0:1.4.2.5:1.0 → 5
get_phys_port()
{
    printf '%s' "$1" | sed -n 's/.*1\.4\.2\.\([0-9]\+\):1\.0$/\1/p'
}

# ---- 显示当前映射 ----------------------------------------------------
show_mapping()
{
    printf '\n%s当前接口 ↔ 物理端口映射%s\n' "$C_BLD" "$C_RST"
    printf '%-8s %-9s %-6s %s\n' 接口 ifindex 物理口 状态
    printf '%s\n' "------------------------------------------------------------"

    local n port ifx stat line
    for n in can0 can1 can2 can3 can4 can5 can_mcu; do
        [ -e "/sys/class/net/$n" ] || continue
        ifx=$(cat "/sys/class/net/$n/ifindex" 2>/dev/null)
        port=$(get_phys_port "$(get_id_path "$n")")
        [ -n "$port" ] || port="—"

        stat=$(ip -details link show "$n" 2>/dev/null \
               | grep -oE 'state [A-Z_-]+' | head -1 | sed 's/^state //')
        [ -n "$stat" ] || stat="—"

        printf '%-8s %-9s %-6s %s\n' "$n" "$ifx" "$port" "$stat"
    done
    printf '\n'
}

# ---- 一致性校验 ------------------------------------------------------
verify_mapping()
{
    local n i port expect bad=0
    for i in $PHYS_PORTS; do
        expect="can$((i-1))"
        port=$(get_phys_port "$(get_id_path "$expect")")
        if [ -z "$port" ]; then
            warn "$expect 不存在或没有 ID_PATH（适配器没插好？）"
            bad=$((bad+1))
            continue
        fi
        if [ "$port" != "$i" ]; then
            err "$expect 实际对应物理口 2-1.4.2.$port，期望 2-1.4.2.$i"
            bad=$((bad+1))
        else
            ok "$expect ↔ 物理口 2-1.4.2.$i"
        fi
    done

    # can_mcu 必须仍在位（板载 CAN）
    if [ ! -e /sys/class/net/can_mcu ]; then
        warn "can_mcu 不存在（板载 CAN 可能未加载）"
    else
        ok "can_mcu 在位（板载 CAN）"
    fi

    return $bad
}

# ---- 安装 ------------------------------------------------------------
do_install()
{
    info "目标路径：$LINK_DIR"
    mkdir -p "$LINK_DIR" || { err "无法创建 $LINK_DIR"; exit 1; }

    # 备份可能存在的旧规则
    local bak
    bak="/root/kcan-persistent-name.bak.$(date +%Y%m%d-%H%M%S)"
    if compgen -G "$LINK_DIR/${LINK_PREFIX}-*.link" >/dev/null; then
        mkdir -p "$bak"
        cp -a "$LINK_DIR/${LINK_PREFIX}"-*.link "$bak"/ 2>/dev/null
        info "已备份旧规则到 $bak"
    fi

    local i idx dst
    for i in $PHYS_PORTS; do
        idx=$((i-1))
        dst="$LINK_DIR/${LINK_PREFIX}-${idx}.link"
        cat > "$dst" <<EOF
# 鲲弘 KCAN-USB FD —— 物理端口 2-1.4.2.$i 固定为 can$idx
# 由 install_kcan_persistent_name.sh 生成于 $(date '+%Y-%m-%d %H:%M:%S')
# 目的：消除 kcan 驱动"抢空位"命名导致的 canN 与物理口错位/漂移。
[Match]
Path=${USB_CTRL_PATH}-usb-0:1.4.2.${i}:1.0

[Link]
Name=can$idx
EOF
        [ -f "$dst" ] || { err "写入失败：$dst"; exit 1; }
        ok "已写入 $dst"
    done

    chmod 644 "$LINK_DIR/${LINK_PREFIX}"-*.link

    info "重新触发热插拔命名（无需重启）……"
    # 让 systemd-udevd 重新处理这些 netdevice 的命名规则
    udevadm trigger --action=add --subsystem-match=net || true
    udevadm settle --timeout=15 || true

    # 如果 udev 触发后名字没变（部分驱动不支持在线改名），回退到重载驱动
    sleep 1
    local need_reload=0 i port
    for i in $PHYS_PORTS; do
        port=$(get_phys_port "$(get_id_path "can$((i-1))")")
        [ "$port" = "$i" ] || need_reload=1
    done

    if [ "$need_reload" -eq 1 ]; then
        warn "在线改名未完全生效，尝试重载 kcan 驱动……"
        # 重载前先把通道都 down 掉，避免总线残留
        local n
        for n in can0 can1 can2 can3 can4 can5; do
            ip link set "$n" down 2>/dev/null || true
        done
        modprobe -r kcan 2>/dev/null || true
        sleep 1
        modprobe kcan 2>/dev/null || true
        udevadm settle --timeout=15 || true
        sleep 1
    fi

    printf '\n'
    show_mapping

    printf '%s校验结果%s\n' "$C_BLD" "$C_RST"
    if verify_mapping; then
        printf '\n'
        ok "映射已固定：物理口 2-1.4.2.N ↔ can(N-1)"
        info "注意：接口已重新枚举，之前的 bitrate/up 配置已丢失，"
        info "      需要重新执行 ip link set ... type can bitrate ... 或点 UI 的【应用】。"
        return 0
    else
        printf '\n'
        err "仍有不一致，请检查适配器是否插在原 USB 口上。"
        info "可用以下命令核对实际拓扑："
        info "  udevadm info -q property -p /sys/class/net/can0 | grep ID_PATH"
        return 1
    fi
}

# ---- 卸载 ------------------------------------------------------------
do_uninstall()
{
    local f found=0
    for f in "$LINK_DIR/${LINK_PREFIX}"-*.link; do
        [ -e "$f" ] || continue
        rm -f "$f" && ok "已删除 $f" && found=1
    done
    if [ "$found" -eq 0 ]; then
        info "没有找到已安装的规则，无需卸载。"
        return 0
    fi
    info "重新触发命名（恢复驱动默认的抢空位命名）……"
    udevadm trigger --action=add --subsystem-match=net || true
    udevadm settle --timeout=15 || true
    show_mapping
    warn "已卸载。canN 与物理口的对应关系将再次变得不确定。"
}

# ---- 入口 ------------------------------------------------------------
case "${1:---install}" in
    --install|install)
        require_root "$@"
        do_install
        ;;
    --check|check)
        # 只读检查，普通用户即可，但拿不到部分 udev 属性时提示用 sudo
        show_mapping
        verify_mapping
        ;;
    --uninstall|uninstall)
        require_root "$@"
        do_uninstall
        ;;
    -h|--help|help)
        sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'
        ;;
    *)
        err "未知参数：$1"
        printf '用法：sudo %s [--install|--check|--uninstall]\n' "$0"
        exit 1
        ;;
esac