#!/usr/bin/env bash
# =====================================================================
# TaiHu_motor_control —— 一键安装本项目所需的全部内核驱动
#
# 本项目一共用到【三类】硬件接口，对应三个驱动：
#
#   ① 鲲弘 KH-UCANFDX6-Mini CAN FD 扩展模块（4 路电机 + 1 路扭矩传感器）
#        设备节点 : can0 / can1 / ...（SocketCAN）
#        驱动     : kcan.ko（成都鲲弘电子，需编译）
#
#   ② 金钢 MPT 磁编码器（经 FT232H / FT2232H USB 转 RS485，2.5 Mbps）
#        设备节点 : /dev/ttyUSB*
#        驱动     : ftdi_sio（Linux 内核自带）
#
#   ③ 键盘（examples/taihu_keyboardctl.cpp 直接读 /dev/input/event*）
#        设备节点 : /dev/input/event*
#        驱动     : evdev（Linux 内核自带）
#
# 脚本做的事（10 个步骤，全部幂等，可反复执行）：
#   步骤 1   环境自检（root / 架构 / 发行版 / 内核 / ip 命令）
#   步骤 2   APT 准备与 DKMS（按需 apt-get update，安装 dkms）
#   步骤 3   驱动①kcan（头文件检查 → 自动 apt → 源码编译 + DKMS → 或退回预编译件）
#   步骤 4   驱动②ftdi_sio（检查 / 加载 / 黑名单冲突排查）
#   步骤 5   驱动③evdev（内核配置检查 / /dev/input 可读性）
#   步骤 6   用户组权限（dialout 串口组 / input 输入设备组）
#   步骤 7   brltty 冲突排查（它会把 FTDI/CH340 串口抢走当盲文显示器）
#   步骤 8   设备稳定命名（可选：给 USB-RS485 建 /dev/taihu-rs485 软链）
#   步骤 9   状态总览
#   步骤 10  后续手动操作提示
#
# 用法：
#   sudo ./deploy/install_drivers.sh              # 全流程（推荐）
#   ./deploy/install_drivers.sh --check           # 只检查，不改系统
#   sudo ./deploy/install_drivers.sh --dry-run    # 看要做什么，但不执行
#   sudo ./deploy/install_drivers.sh --kcan=skip  # 只装串口和输入驱动
#   sudo ./deploy/install_drivers.sh --uninstall  # 卸载（只卸 kcan，系统自带的没法卸）
#   ./deploy/install_drivers.sh --help
#
# ==================== 三条硬性安全约束 ====================
#   * 【绝不】自动 `ip link set up` 任何 CAN 通道。
#     通道必须按现场接线手动打开：
#         sudo ./build/kcanctl up can0 1000000
#     （接口编号随 USB 插入顺序变化，先 `./build/kcanctl list` 确认）
#
#   * 【绝不】自动运行任何 demo / 主程序。本脚本只装驱动。
#
#   * 【绝不】使用厂商的 `build.sh -rules` / `-dkms-rules`。
#     那两个参数会写入 udev 规则 `NAME="kcan%n"`，把 can0 改名成
#     kcan0，导致本项目所有硬编码 "can0" 的程序全部失效。
#     本脚本只用 `-dkms`（不触碰接口名）。
# =====================================================================

set -euo pipefail

# ---------------------------------------------------------------------
# 全局常量
# ---------------------------------------------------------------------
readonly SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}")"
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

readonly KREL="$(uname -r)"
readonly KARCH="$(uname -m)"

readonly KCAN_INSTALL_SCRIPT="${SCRIPT_DIR}/install_kcan.sh"
readonly KCAN_SYSFS="/sys/module/kcan"

# 厂商 udev 规则会被 build.sh -rules 安装到这两个位置，本脚本会主动检测并告警
readonly VENDOR_UDEV_45="/etc/udev/rules.d/45-kcan.rules"
readonly VENDOR_RENAME_RULE='NAME="kcan%n"'

# 本项目自建的 RS485 设备软链规则（只加 SYMLINK，绝不改 NAME）
readonly RS485_UDEV_RULE="/etc/udev/rules.d/99-taihu-rs485.rules"

# FTDI USB 转串口的 USB VID/PID（FT232H / FT2232H / FT232R / FT4232H）
readonly FTDI_VID="0403"
readonly FTDI_PIDS_DEFAULT=("6014" "6010" "6001" "6011")

# 颜色（非 tty 或设置了 NO_COLOR 时自动关闭）
if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
    C_RED=$'\033[31m'; C_GREEN=$'\033[32m'; C_YELLOW=$'\033[33m'
    C_BLUE=$'\033[34m'; C_BOLD=$'\033[1m'; C_DIM=$'\033[2m'; C_RESET=$'\033[0m'
else
    C_RED=''; C_GREEN=''; C_YELLOW=''; C_BLUE=''; C_BOLD=''; C_DIM=''; C_RESET=''
fi

# 统计
declare -i N_OK=0 N_WARN=0 N_FAIL=0 N_SKIP=0

# 命令行开关
OPT_CHECK_ONLY=0
OPT_DRY_RUN=0
OPT_ASSUME_YES=0
OPT_UNINSTALL=0
OPT_KCAN_MODE="auto"        # auto | compile | prebuilt | skip
OPT_KO_PATH=""
OPT_WITH_DKMS=1             # 默认启用 DKMS（内核升级后自动重编）
OPT_SKIP_KCAN=0
OPT_SKIP_FTDI=0
OPT_SKIP_EVDEV=0
OPT_SKIP_GROUPS=0
OPT_FIX_BRLTTY=0
OPT_WITH_RS485_SYMLINK=0
OPT_NO_APT_UPDATE=0

# 运行期探测到的状态
HAVE_KHDR=0
KHDR_DIR=""
KHDR_PKG_CANDIDATE=""
SDK_ROOT=""
PREBUILT_KO=""
GROUP_CHANGED=0
RENAME_RULE_CHECKED=0
KCAN_EXPECT_COUNT=6         # KH-UCANFDX6-Mini 是 6 通道；仅用于提示

# =====================================================================
# 输出辅助
# =====================================================================
log_info() { printf '%s[信息]%s %s\n' "${C_BLUE}"   "${C_RESET}" "$*"; }
log_ok()   { printf '%s[完成]%s %s\n' "${C_GREEN}"  "${C_RESET}" "$*"; N_OK+=1; }
log_warn() { printf '%s[警告]%s %s\n' "${C_YELLOW}" "${C_RESET}" "$*" >&2; N_WARN+=1; }
log_fail() { printf '%s[失败]%s %s\n' "${C_RED}"    "${C_RESET}" "$*" >&2; N_FAIL+=1; }
log_skip() { printf '%s[跳过]%s %s\n' "${C_YELLOW}" "${C_RESET}" "$*"; N_SKIP+=1; }
log_hint() { printf '%s         %s%s\n' "${C_DIM}" "$*" "${C_RESET}"; }

section() {
    printf '\n%s=====================================================================%s\n' "${C_BOLD}" "${C_RESET}"
    printf '%s %s%s\n' "${C_BOLD}" "$*" "${C_RESET}"
    printf '%s=====================================================================%s\n' "${C_BOLD}" "${C_RESET}"
}

die() {
    printf '\n%s[中止]%s %s\n' "${C_RED}" "${C_RESET}" "$*" >&2
    exit 1
}

# 执行命令（dry-run 时只打印，不执行）
run() {
    if (( OPT_DRY_RUN )); then
        printf '%s[试运行]%s %s\n' "${C_YELLOW}" "${C_RESET}" "$*"
        return 0
    fi
    "$@"
}

# 写文件（dry-run 安全）
write_file() {
    local path="$1" content="$2"
    if (( OPT_DRY_RUN )); then
        printf '%s[试运行]%s 写入 %s:\n' "${C_YELLOW}" "${C_RESET}" "${path}"
        printf '%s\n' "${content}" | sed 's/^/           /'
        return 0
    fi
    printf '%s\n' "${content}" > "${path}"
}

# 写操作前的 root 检查（--check / --dry-run 不要求 root）
need_root() {
    if (( OPT_CHECK_ONLY || OPT_DRY_RUN )); then
        return 0
    fi
    if [[ "${EUID}" -ne 0 ]]; then
        die "该步骤需要 root 权限，请用 sudo 重新运行：sudo ./deploy/${SCRIPT_NAME}"
    fi
    return 0
}

# 是否允许真正改动系统（--check 时全为假）
can_write() { (( ! OPT_CHECK_ONLY )); }

confirm() {
    if (( OPT_ASSUME_YES )); then
        return 0
    fi
    if (( OPT_CHECK_ONLY )); then
        return 1
    fi
    # --dry-run 不读 stdin：无人值守场景下管道会阻塞，
    # 且试运行本来就只打印不执行，无需确认。
    if (( OPT_DRY_RUN )); then
        return 0
    fi
    local reply
    read -r -p "$1 [y/N] " reply || true
    if [[ "${reply}" =~ ^[Yy]$ ]]; then
        return 0
    fi
    return 1
}

# =====================================================================
# 通用探测函数
# =====================================================================

# 需要被加入 dialout / input 组的真实用户（不是 root）
target_user() {
    if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
        printf '%s\n' "${SUDO_USER}"
        return 0
    fi
    if [[ "${EUID}" -ne 0 && -n "${USER:-}" && "${USER}" != "root" ]]; then
        printf '%s\n' "${USER}"
        return 0
    fi
    return 1
}

# 内核模块是否已加载
module_loaded() {
    local m="$1"
    [[ -d "/sys/module/${m}" ]]
}

# 模块是否存在于当前内核的模块目录
module_available() {
    local m="$1"
    command -v modinfo >/dev/null 2>&1 || return 1
    modinfo "${m}" >/dev/null 2>&1
}

# 模块是否被 modprobe.d 里的 blacklist 掉
module_blacklisted() {
    local m="$1"
    [[ -d /etc/modprobe.d ]] || return 1
    grep -rqsE "^[[:space:]]*blacklist[[:space:]]+${m}([[:space:]]|$)" /etc/modprobe.d/ 2>/dev/null
}

# 打印命中黑名单的文件（便于用户自己判断）
module_blacklist_source() {
    local m="$1"
    grep -rlsE "^[[:space:]]*blacklist[[:space:]]+${m}([[:space:]]|$)" /etc/modprobe.d/ 2>/dev/null | tr '\n' ' '
}

# 内核构建树（可编译外部模块的前提）
kernel_headers_dir() {
    local d r
    for d in "/lib/modules/${KREL}/build" "/lib/modules/${KREL}/source"; do
        [[ -e "${d}" ]] || continue
        r="$(readlink -f "${d}" 2>/dev/null || printf '%s' "${d}")"
        [[ -d "${r}" ]] || continue
        if [[ -f "${r}/Makefile" ]] && [[ -d "${r}/include" ]] && \
           { [[ -d "${r}/scripts" ]] || [[ -d "${r}/arch" ]]; }; then
            printf '%s\n' "${r}"
            return 0
        fi
    done
    return 1
}

# 是否 aarch64 ELF
is_aarch64_elf() {
    local f="$1"
    [[ -f "${f}" ]] || return 1
    if command -v file >/dev/null 2>&1; then
        file -b "${f}" 2>/dev/null | grep -qi 'aarch64'
    else
        # ELF e_machine @ offset 18 (小端) = 0x00B7
        [[ "$(od -An -tx1 -j18 -N2 "${f}" 2>/dev/null | tr -d ' \n')" == "b700" ]]
    fi
}

# 定位厂商 SDK（含 build.sh 才算可用）
find_sdk_root() {
    local c
    for c in "${PROJECT_ROOT}/.kunhong_sdk"/KH-UCANFD_Linux_SDK-*; do
        [[ -d "${c}" ]] || continue
        [[ -x "${c}/build.sh" ]] || continue
        [[ -f "${c}/driver/Makefile" ]] || continue
        printf '%s\n' "${c}"
        return 0
    done
    return 1
}

# 定位已就绪的 aarch64 kcan.ko（预编译件）
find_prebuilt_aarch64_ko() {
    local c
    local -a cands=(
        "${SCRIPT_DIR}/kcan-driver/kcan.ko"
        "${SCRIPT_DIR}/kcan-driver/driver/kcan.ko"
        "${PROJECT_ROOT}/kcan-driver/kcan.ko"
    )
    [[ -n "${OPT_KO_PATH}" ]] && cands=("${OPT_KO_PATH}" "${cands[@]}")
    for c in "${cands[@]}"; do
        [[ -n "${c}" ]] || continue
        [[ -f "${c}" ]] || continue
        if is_aarch64_elf "${c}"; then
            printf '%s\n' "${c}"
            return 0
        fi
    done
    return 1
}

# 列出当前 canN 接口
list_can_ifaces() {
    command -v ip >/dev/null 2>&1 || return 0
    ip -o link show 2>/dev/null | awk -F': ' '/: can[0-9]+/{print $2}' | tr '\n' ' ' || true
}

# 列出 /dev/ttyUSB*
list_ttyusb() {
    local -a v=(/dev/ttyUSB*)
    if [[ ! -e "${v[0]}" ]]; then
        return 0
    fi
    printf '%s ' "${v[@]##*/}"
    return 0
}

# 检测当前插着的 FTDI 设备的 PID（用于生成精确的 udev 规则）
detect_ftdi_pids() {
    local -a pids=()
    if command -v lsusb >/dev/null 2>&1; then
        # lsusb 行格式: Bus 001 Device 003: ID 0403:6014 Future Technology ...
        # 取 "0403:6014" 这一段的后半，即 PID；一个 PID 只记一次
        while IFS= read -r id; do
            if [[ -n "${id}" ]]; then
                pids+=("${id}")
            fi
        done < <(lsusb 2>/dev/null \
            | awk -v vid="${FTDI_VID}" '
                {
                    for (i = 1; i <= NF; i++) {
                        if ($i ~ ("^" vid ":")) {
                            pid = $i
                            sub("^" vid ":", "", pid)
                            if (!(pid in seen)) { seen[pid] = 1; print pid }
                            break
                        }
                    }
                }')
    fi
    if (( ${#pids[@]} == 0 )); then
        printf '%s\n' "${FTDI_PIDS_DEFAULT[@]}"
    else
        printf '%s\n' "${pids[@]}"
    fi
    return 0
}

# =====================================================================
# 参数解析
# =====================================================================
usage() {
    cat <<'EOF'
TaiHu_motor_control —— 一键安装本项目所需的全部内核驱动

本项目用到三类硬件接口，本脚本一次装齐：

  ① kcan       鲲弘 KH-UCANFDX6-Mini CAN FD  → can0 / can1 / ...
  ② ftdi_sio   金钢 MPT 磁编码器（FT232H 转 RS485）→ /dev/ttyUSB*
  ③ evdev      键盘（taihu_keyboardctl）→ /dev/input/event*

用法:
  sudo ./deploy/install_drivers.sh [选项]

选项:
  --check                 只做检查，不修改系统（不需要 root）
  --dry-run               打印将要执行的命令，但不实际执行
  -y, --yes               所有确认自动回答 yes

  --kcan=<模式>           ① kcan 驱动处理方式
                            auto      自动决策（默认）：有头文件就编译，没有就找预编译件
                            compile   强制从厂商 SDK 源码编译（需要内核头文件）
                            prebuilt  强制使用现成的 aarch64 kcan.ko
                            skip      完全跳过 kcan
  --ko <路径>             指定 aarch64 kcan.ko 文件（等价于 --kcan=prebuilt）
  --no-dkms               不启用 DKMS（默认启用：内核升级后自动重编驱动）

  --skip-kcan             等价于 --kcan=skip
  --skip-ftdi             跳过 ftdi_sio（USB 转 RS485）
  --skip-evdev            跳过 evdev（键盘输入）
  --skip-groups           不修改 dialout / input 用户组
  --fix-brltty            自动屏蔽 brltty 服务（默认只告警，不动它）
  --with-rs485-symlink    建立 /dev/taihu-rs485 稳定软链（默认不建）
  --no-apt-update         不执行 apt-get update

  --uninstall             卸载 kcan 驱动（转交 install_kcan.sh），并清理本脚本建的规则
  -h, --help              显示本帮助

────────────────────────────────────────────────────────────────
kcan 驱动的三条路（按可行性自动选择）:

  A) 板上有内核头文件 / 内核源码树
         → 用厂商 SDK 源码编译 + DKMS 安装（推荐，内核升级后自动重编）

  B) 板上没有头文件，但内核版本能在 apt 里找到
         → 脚本自动 apt-get install linux-headers-$(uname -r)，然后走 A

  C) 以上都不行（RK3588 厂商内核 5.10.160 属于这种情况）
         → 需要厂商提供的 aarch64 预编译 kcan.ko，放到
              deploy/kcan-driver/kcan.ko
           然后用 --ko 指定，或让脚本自动发现

  ⚠ SDK 里自带的 driver/kcan.ko 是 x86-64 的，拷到 3588 会
    "Exec format error"，脚本会自动跳过它。

────────────────────────────────────────────────────────────────
⚠ 本脚本【不会】做的事（有意为之）:

   * 不会 `ip link set up` 任何 CAN 通道
   * 不会运行任何 demo / 主程序
   * 不会使用厂商的 build.sh -rules / -dkms-rules
     （那两个参数会写入 udev 规则 NAME="kcan%n"，把 can0 改名成
       kcan0，导致本项目所有硬编码 "can0" 的程序失效）

开 CAN 通道请手动执行:
      ./build/kcanctl list
      sudo ./build/kcanctl up can0 1000000
EOF
}

parse_args() {
    while (($#)); do
        case "$1" in
            --check)              OPT_CHECK_ONLY=1 ;;
            --dry-run)            OPT_DRY_RUN=1 ;;
            -y|--yes)             OPT_ASSUME_YES=1 ;;
            --no-dkms)            OPT_WITH_DKMS=0 ;;
            --skip-kcan)          OPT_SKIP_KCAN=1 ;;
            --skip-ftdi)          OPT_SKIP_FTDI=1 ;;
            --skip-evdev)         OPT_SKIP_EVDEV=1 ;;
            --skip-groups)        OPT_SKIP_GROUPS=1 ;;
            --fix-brltty)         OPT_FIX_BRLTTY=1 ;;
            --with-rs485-symlink) OPT_WITH_RS485_SYMLINK=1 ;;
            --no-apt-update)      OPT_NO_APT_UPDATE=1 ;;
            --uninstall)          OPT_UNINSTALL=1 ;;
            --kcan=*)             OPT_KCAN_MODE="${1#*=}" ;;
            --kcan)
                [[ $# -ge 2 ]] || die "--kcan 需要一个参数（auto|compile|prebuilt|skip）"
                OPT_KCAN_MODE="$2"; shift ;;
            --ko)
                [[ $# -ge 2 ]] || die "--ko 需要一个参数"
                OPT_KO_PATH="$2"; OPT_KCAN_MODE="prebuilt"; shift ;;
            --ko=*)
                OPT_KO_PATH="${1#*=}"; OPT_KCAN_MODE="prebuilt" ;;
            -h|--help)            usage; exit 0 ;;
            *)                    usage; die "未知参数: $1" ;;
        esac
        shift
    done

    case "${OPT_KCAN_MODE}" in
        auto|compile|prebuilt|skip) ;;
        *) die "--kcan 只接受 auto / compile / prebuilt / skip，收到: ${OPT_KCAN_MODE}" ;;
    esac
    if [[ "${OPT_KCAN_MODE}" == "skip" ]]; then
        OPT_SKIP_KCAN=1
    fi
    return 0
}

# =====================================================================
# 步骤 1 —— 环境自检
# =====================================================================
step1_env() {
    section "步骤 1 / 10  环境自检"

    # 1.1 架构
    log_info "系统架构       : ${KARCH}"
    case "${KARCH}" in
        aarch64|arm64)
            log_ok "架构为 aarch64，符合本项目目标平台（RK3588）"
            ;;
        x86_64)
            log_warn "当前是 x86_64，不是 RK3588 目标平台"
            log_hint "本脚本可以在这台机器上跑通检查逻辑，但 kcan 驱动必须"
            log_hint "在 aarch64 板子上编译，x86 上编出来的 .ko 不能拿去 3588 用"
            ;;
        *)
            log_warn "架构 ${KARCH} 未验证过，请自行确认兼容性"
            ;;
    esac

    # 1.2 内核
    log_info "内核版本       : ${KREL}"
    if [[ -r "/boot/config-${KREL}" ]]; then
        log_ok "找到内核配置文件 /boot/config-${KREL}"
    elif [[ -r /proc/config.gz ]]; then
        log_ok "找到内核配置 /proc/config.gz"
    else
        log_warn "找不到内核配置文件（/boot/config-${KREL} 与 /proc/config.gz 都没有）"
        log_hint "这不影响装驱动，但会影响步骤 5 的 evdev 内核选项检查"
    fi

    # 1.3 发行版
    if [[ -r /etc/os-release ]]; then
        # shellcheck disable=SC1091
        local pretty; pretty="$( . /etc/os-release; printf '%s' "${PRETTY_NAME:-unknown}" )"
        log_info "操作系统       : ${pretty}"
    else
        log_warn "读不到 /etc/os-release，无法确认发行版"
    fi

    # 1.4 root 权限
    if (( OPT_CHECK_ONLY )); then
        log_info "运行身份       : EUID=${EUID}（--check 模式，不需要 root）"
    elif (( OPT_DRY_RUN )); then
        log_info "运行身份       : EUID=${EUID}（--dry-run 模式，不需要 root）"
    elif [[ "${EUID}" -eq 0 ]]; then
        log_ok "以 root 身份运行"
    else
        log_fail "未以 root 运行，安装步骤会失败"
        log_hint "请用: sudo ./deploy/${SCRIPT_NAME}"
    fi

    # 1.5 必要命令
    local miss=() c
    for c in ip modprobe depmod udevadm lsusb; do
        command -v "${c}" >/dev/null 2>&1 || miss+=("${c}")
    done
    if (( ${#miss[@]} == 0 )); then
        log_ok "ip / modprobe / depmod / udevadm / lsusb 均可用"
    else
        for c in "${miss[@]}"; do
            case "${c}" in
                ip)       log_fail "缺少 ip 命令（iproute2），本项目 kcanctl 硬依赖它" ;;
                lsusb)    log_warn "缺少 lsusb（usbutils），不影响装驱动，但排查 USB 设备时会用到" ;;
                *)        log_fail "缺少 ${c}（kmod/udev 基础组件，通常随系统自带）" ;;
            esac
        done
    fi

    # 1.6 apt 是否可用
    if command -v apt-get >/dev/null 2>&1; then
        if (( OPT_NO_APT_UPDATE )); then
            log_skip "已指定 --no-apt-update，跳过更新索引"
        else
            log_ok "apt-get 可用，可按需安装内核头文件"
        fi
    else
        log_warn "没有 apt-get，无法自动安装内核头文件"
    fi
}

# =====================================================================
# 步骤 2 —— APT 准备与 DKMS
# =====================================================================
step2_apt_prep() {
    section "步骤 2 / 10  APT 准备与 DKMS"

    if (( OPT_SKIP_KCAN )); then
        log_skip "kcan 已跳过，无需 APT 准备与 DKMS"
        return 0
    fi

    # 2.1 DKMS 只在「编译源码」路线下才是必需的
    if (( ! OPT_WITH_DKMS )); then
        log_skip "已指定 --no-dkms，跳过 DKMS 检查"
        return 0
    fi

    if command -v dkms >/dev/null 2>&1; then
        log_ok "dkms 已安装：$(dkms --version 2>/dev/null | head -1 || echo '版本未知')"
        return 0
    fi

    log_warn "dkms 未安装（DKMS 能让内核升级后自动重编 kcan 驱动）"
    if (( OPT_CHECK_ONLY )); then
        log_hint "安装时会执行: apt-get install -y dkms"
        return 0
    fi
    if ! command -v apt-get >/dev/null 2>&1; then
        log_warn "没有 apt-get，无法安装 dkms —— 将退回普通模块安装方式"
        OPT_WITH_DKMS=0
        return 0
    fi

    need_root
    if (( ! OPT_NO_APT_UPDATE )); then
        log_info "更新 apt 索引（可用 --no-apt-update 跳过）"
        run apt-get update || log_warn "apt-get update 失败（离线环境正常，继续尝试安装）"
    fi
    if run apt-get install -y --no-install-recommends dkms; then
        if command -v dkms >/dev/null 2>&1; then
            log_ok "dkms 安装完成"
        else
            log_warn "dkms 安装命令成功但命令仍不可用，将退回普通模块安装方式"
            OPT_WITH_DKMS=0
        fi
    else
        log_warn "dkms 安装失败，将退回普通模块安装方式（内核升级后需手动重装驱动）"
        OPT_WITH_DKMS=0
    fi
}

# =====================================================================
# 步骤 3 —— 驱动①：kcan
# =====================================================================
step3_kcan() {
    section "步骤 3 / 10  驱动①：kcan（鲲弘 KH-UCANFDX6-Mini CAN FD）"

    if (( OPT_SKIP_KCAN )); then
        log_skip "按 --kcan=skip / --skip-kcan 跳过"
        return 0
    fi

    # ---- 3.1 现状 ----
    if module_loaded kcan; then
        log_ok "kcan 模块已加载（${KCAN_SYSFS} 存在）"
        local ifs; ifs="$(list_can_ifaces)"
        if [[ -n "${ifs// }" ]]; then
            log_info "当前 CAN 接口 : ${ifs}"
        else
            log_warn "模块已加载，但没有任何 canN 接口 —— 检查 USB 是否插好"
            log_hint "lsusb | grep -i kunhong    或    sudo kcanosdiag.sh"
        fi
    else
        log_info "kcan 模块尚未加载"
    fi

    # ---- 3.2 厂商 udev 规则：这是本项目的地雷，必须先查 ----
    check_vendor_rename_rule

    # ---- 3.3 探测内核头文件 ----
    local need_compile=0
    case "${OPT_KCAN_MODE}" in
        prebuilt) need_compile=0 ;;
        compile)  need_compile=1 ;;
        auto)     need_compile=1 ;;
    esac

    if (( need_compile )); then
        if KHDR_DIR="$(kernel_headers_dir)"; then
            HAVE_KHDR=1
            log_ok "内核构建树   : ${KHDR_DIR}"
        else
            HAVE_KHDR=0
            log_warn "未找到内核构建树（/lib/modules/${KREL}/build 不存在）"
            try_install_kernel_headers
        fi
    fi

    # ---- 3.4 探测厂商 SDK 源码 ----
    if SDK_ROOT="$(find_sdk_root)"; then
        log_ok "厂商 SDK 源码: ${SDK_ROOT}"
    else
        SDK_ROOT=""
        log_warn "未找到厂商 SDK（.kunhong_sdk/KH-UCANFD_Linux_SDK-*/）"
        log_hint "获取: git clone https://gitee.com/ChengDu-KunHong/KH-UCANFD_Linux_SDK.git .kunhong_sdk"
    fi

    # ---- 3.5 探测预编译 aarch64 .ko ----
    if PREBUILT_KO="$(find_prebuilt_aarch64_ko)"; then
        log_ok "预编译 aarch64 kcan.ko: ${PREBUILT_KO}"
    else
        PREBUILT_KO=""
        log_info "未找到可用的 aarch64 预编译 kcan.ko"
    fi

    # ---- 3.6 按模式决策 ----
    case "${OPT_KCAN_MODE}" in
        prebuilt) kcan_use_prebuilt ;;
        compile)  kcan_compile ;;
        auto)     kcan_auto ;;
    esac
}

# 3.x 子流程：检测 udev 规则是否会改名 canN
# 步骤 3 与 kcan_compile 都会调用，用 RENAME_RULE_CHECKED 保证只报告一次
check_vendor_rename_rule() {
    if (( RENAME_RULE_CHECKED )); then
        return 0
    fi
    RENAME_RULE_CHECKED=1

    local hit=0 f
    for f in "${VENDOR_UDEV_45}" "${RS485_UDEV_RULE}"; do
        [[ -f "${f}" ]] || continue
        if grep -qF "${VENDOR_RENAME_RULE}" "${f}" 2>/dev/null; then
            hit=1
            log_fail "发现会改接口名的 udev 规则: ${f}"
            log_hint "规则内容含 ${VENDOR_RENAME_RULE}，会把 can0 改名成 kcan0，"
            log_hint "导致本项目所有硬编码 \"can0\" 的程序失效"
            log_hint "处理: sudo rm -f ${f} && sudo udevadm control --reload-rules"
            log_hint "然后拔插一次 USB-CAN 模块，接口名会恢复成 canN"
        fi
    done
    # 顺带扫一遍 /etc/udev/rules.d，防止用户手工加过
    if (( ! hit )) && [[ -d /etc/udev/rules.d ]]; then
        local bad
        bad="$(grep -rlsF "${VENDOR_RENAME_RULE}" /etc/udev/rules.d/ 2>/dev/null | tr '\n' ' ' || true)"
        if [[ -n "${bad// }" ]]; then
            log_warn "以下 udev 规则会把 CAN 接口改名为 kcanN，请确认是否要保留: ${bad}"
        fi
    fi
    if (( hit )); then
        return 0
    fi
    log_ok "未发现会改接口名的 udev 规则（canN 命名安全）"
    return 0
}

# 3.x 子流程：尝试 apt 安装内核头文件
try_install_kernel_headers() {
    KHDR_PKG_CANDIDATE="linux-headers-${KREL}"

    if ! command -v apt-get >/dev/null 2>&1; then
        log_warn "没有 apt-get，无法自动安装 ${KHDR_PKG_CANDIDATE}"
        return 0
    fi

    log_info "尝试从 apt 安装内核头文件: ${KHDR_PKG_CANDIDATE}"

    if (( OPT_CHECK_ONLY )); then
        if apt-cache policy "${KHDR_PKG_CANDIDATE}" 2>/dev/null | grep -qE 'Candidate:[[:space:]]+[0-9]'; then
            log_ok "apt 源里有 ${KHDR_PKG_CANDIDATE}，安装时可以装"
        else
            log_fail "apt 源里没有 ${KHDR_PKG_CANDIDATE}"
            log_hint "RK3588 的 ${KREL} 是 Rockchip 厂商内核，Ubuntu 官方源不提供对应头文件"
            log_hint "这是预期结果，请走「预编译 .ko」路线（把 aarch64 kcan.ko 放到"
            log_hint "deploy/kcan-driver/ 后重跑，或加 --ko <路径>）"
        fi
        return 0
    fi

    need_root
    if (( ! OPT_NO_APT_UPDATE )); then
        run apt-get update || log_warn "apt-get update 失败（离线环境正常，继续尝试安装）"
    fi

    if run apt-get install -y --no-install-recommends "${KHDR_PKG_CANDIDATE}"; then
        if KHDR_DIR="$(kernel_headers_dir)"; then
            HAVE_KHDR=1
            log_ok "内核头文件安装成功 : ${KHDR_DIR}"
        else
            log_warn "apt 报告安装成功，但 ${KHDR_DIR:-/lib/modules/${KREL}/build} 仍不可用"
        fi
    else
        log_warn "apt 安装 ${KHDR_PKG_CANDIDATE} 失败（RK3588 厂商内核的常见情况）"
        log_hint "apt 源里通常没有 Rockchip 5.10.x 的内核头文件"
        log_hint "可行的替代方案:"
        log_hint "  a) 向成都鲲弘索取本机内核版本对应的 aarch64 kcan.ko"
        log_hint "     （support@cdkhdz.com），放到 deploy/kcan-driver/ 后重跑本脚本"
        log_hint "  b) 从板卡厂商拿到 ${KREL} 的完整内核源码树，用"
        log_hint "     --kcan=compile 指定编译"
    fi
}

# 3.x 子流程：auto 决策
kcan_auto() {
    # 优先：已加载且接口正常 → 什么都不用做
    if module_loaded kcan && [[ -n "$(list_can_ifaces)" ]]; then
        log_ok "kcan 已就绪，无需操作"
        return 0
    fi

    # 有头文件 + 有 SDK 源码 → 编译（最优路径）
    if (( HAVE_KHDR )) && [[ -n "${SDK_ROOT}" ]]; then
        log_info "决策: 有内核头文件 + 有厂商源码 → 源码编译"
        kcan_compile
        return 0
    fi

    # 有预编译 aarch64 .ko → 用它
    if [[ -n "${PREBUILT_KO}" ]]; then
        log_info "决策: 无内核头文件，但有 aarch64 预编译件 → 直接安装"
        kcan_use_prebuilt
        return 0
    fi

    # 什么都没有
    log_fail "kcan 驱动无法安装：既没有内核头文件，也没有 aarch64 预编译 kcan.ko"
    log_hint "这是 RK3588 + 厂商内核（${KREL}）的典型情况，请二选一："
    log_hint "  ① 向成都鲲弘索取对应内核版本的 aarch64 kcan.ko"
    log_hint "     support@cdkhdz.com / sales@cdkhdz.com"
    log_hint "     拿到后放到 deploy/kcan-driver/kcan.ko，再重跑本脚本"
    log_hint "  ② 从板卡厂商获取 ${KREL} 完整内核源码树，然后:"
    log_hint "     sudo ./deploy/${SCRIPT_NAME} --kcan=compile"
    log_hint ""
    log_hint "⚠ SDK 自带的 .kunhong_sdk/**/driver/kcan.ko 是 x86-64 的，"
    log_hint "  拷到 3588 会 Exec format error，不能用。"
}

# 3.x 子流程：源码编译（+ DKMS）
kcan_compile() {
    if [[ -z "${SDK_ROOT}" ]]; then
        log_fail "没有厂商 SDK 源码，无法编译"
        log_hint "获取: git clone https://gitee.com/ChengDu-KunHong/KH-UCANFD_Linux_SDK.git .kunhong_sdk"
        return 0
    fi
    if (( ! HAVE_KHDR )); then
        log_fail "没有内核构建树，无法编译 kcan"
        log_hint "编译内核模块必须有 /lib/modules/${KREL}/build"
        if [[ -n "${PREBUILT_KO}" ]]; then
            log_info "已找到 aarch64 预编译件，可改用: --kcan=prebuilt"
        fi
        return 0
    fi

    local -a args=()
    if (( OPT_WITH_DKMS )) && command -v dkms >/dev/null 2>&1; then
        args+=("-dkms")
        log_info "安装方式     : 源码编译 + DKMS（内核升级后自动重编）"
    else
        log_info "安装方式     : 源码编译 + 普通模块安装（内核升级后需手动重跑）"
        if (( OPT_WITH_DKMS )); then
            log_hint "dkms 不可用，已自动退回普通安装"
        fi
    fi

    # 明确不用 -rules / -dkms-rules：会写入 NAME="kcan%n"，把 can0 改名成 kcan0
    log_warn "注意: 本脚本不会传 -rules / -dkms-rules"
    log_hint "那两个参数会写入 NAME=\"kcan%n\"，把 can0 改名成 kcan0，"
    log_hint "直接破坏本项目所有硬编码 \"can0\" 的程序"

    log_info "SDK 目录     : ${SDK_ROOT}"
    log_info "内核构建树   : ${KHDR_DIR}"
    log_info "将执行       : ${SDK_ROOT}/build.sh ${args[*]:-}"

    if (( OPT_CHECK_ONLY )); then
        log_hint "--check 模式：未实际编译"
        return 0
    fi

    need_root

    if (( OPT_DRY_RUN )); then
        printf '%s[试运行]%s (cd %s && ./build.sh %s)\n' \
            "${C_YELLOW}" "${C_RESET}" "${SDK_ROOT}" "${args[*]:-}"
        return 0
    fi

    # 编译在 SDK 目录内进行（make 会在原地生成 kcan.ko）
    local rc=0
    ( cd "${SDK_ROOT}" && ./build.sh "${args[@]}" ) || rc=$?

    if (( rc != 0 )); then
        log_fail "build.sh 返回 ${rc}，编译或安装失败"
        log_hint "常见原因与处理:"
        log_hint "  · gcc 版本与内核编译版本不一致"
        log_hint "    → 看内核用哪个 gcc: grep -a 'Linux version' /proc/version"
        log_hint "    → sudo CC=gcc-12 ./build.sh ${args[*]:-}"
        log_hint "  · 内核头文件不完整（只有部分头文件，没有 Module.symvers）"
        log_hint "  · Secure Boot 拒绝加载未签名模块（服务器上常见，开发板少见）"
        log_hint "详见 SDK 的 release_notes.txt 与厂商文档"
        return 0
    fi

    # 复核
    if module_loaded kcan; then
        log_ok "kcan 模块已加载"
    else
        log_warn "编译安装完成，但模块未加载"
        log_hint "手动加载: sudo modprobe kcan"
    fi

    # 编译完立刻复核有没有被装上改名的 udev 规则
    check_vendor_rename_rule

    local ifs; ifs="$(list_can_ifaces)"
    if [[ -n "${ifs// }" ]]; then
        log_ok "CAN 接口已出现: ${ifs}"
        if (( $(printf '%s\n' ${ifs} | wc -l) < KCAN_EXPECT_COUNT )); then
            log_info "当前 $(printf '%s\n' ${ifs} | wc -l) 个通道，KH-UCANFDX6-Mini 满配应为 ${KCAN_EXPECT_COUNT} 个"
        fi
    else
        log_warn "模块已加载但没看到 canN 接口"
        log_hint "USB-CAN 模块是否插好？lsusb | grep -i -e kunhong -e 2d80"
    fi
}

# 3.x 子流程：安装现成的 aarch64 .ko（转交 install_kcan.sh）
kcan_use_prebuilt() {
    if [[ -z "${PREBUILT_KO}" ]]; then
        log_fail "没有可用的 aarch64 预编译 kcan.ko"
        log_hint "请把厂商提供的 aarch64 kcan.ko 放到: deploy/kcan-driver/kcan.ko"
        log_hint "或指定路径: sudo ./deploy/${SCRIPT_NAME} --ko /path/to/kcan.ko"
        log_hint ""
        log_hint "⚠ SDK 自带的 .kunhong_sdk/**/driver/kcan.ko 是 x86-64，不能用"
        return 0
    fi

    if [[ ! -x "${KCAN_INSTALL_SCRIPT}" ]]; then
        log_fail "找不到 ${KCAN_INSTALL_SCRIPT}，无法完成安装"
        log_hint "该脚本负责 .ko 的架构/vermagic 校验与落盘，是安装链的一环"
        return 0
    fi

    local -a kargs=( --ko "${PREBUILT_KO}" )
    if (( OPT_ASSUME_YES || OPT_DRY_RUN )); then kargs+=( -y ); fi
    if (( OPT_DRY_RUN ));    then kargs+=( --dry-run ); fi

    log_info "转交 ${KCAN_INSTALL_SCRIPT##*/} 执行安装"
    log_info "转交参数     : ${kargs[*]}"

    if (( OPT_CHECK_ONLY )); then
        "${KCAN_INSTALL_SCRIPT}" --check || true
        return 0
    fi

    local rc=0
    "${KCAN_INSTALL_SCRIPT}" "${kargs[@]}" || rc=$?
    if (( rc == 0 )); then
        log_ok "kcan 驱动安装完成"
    else
        log_fail "install_kcan.sh 返回 ${rc}，详见上方输出"
    fi
}

# =====================================================================
# 步骤 4 —— 驱动②：ftdi_sio（USB 转 RS485）
# =====================================================================
step4_ftdi() {
    section "步骤 4 / 10  驱动②：ftdi_sio（金钢 MPT 磁编码器 USB 转 RS485）"

    if (( OPT_SKIP_FTDI )); then
        log_skip "按 --skip-ftdi 跳过"
        return 0
    fi

    # 4.1 模块是否存在（内核自带）
    if ! module_available ftdi_sio; then
        log_fail "当前内核没有 ftdi_sio 模块"
        log_hint "检查: find /lib/modules/${KREL} -name 'ftdi_sio*'"
        log_hint "若确实没有，说明内核裁剪掉了 USB 串口支持，需要换内核或"
        log_hint "向板卡厂商索取带 CONFIG_USB_SERIAL_FTDI_SIO 的内核"
        return 0
    fi
    log_ok "ftdi_sio 模块存在（内核自带）"

    # 4.2 黑名单冲突
    if module_blacklisted ftdi_sio; then
        log_warn "ftdi_sio 被 modprobe 黑名单屏蔽，文件: $(module_blacklist_source ftdi_sio)"
        log_hint "取消屏蔽: 删除/注释掉上面文件里的 blacklist ftdi_sio"
    else
        log_ok "ftdi_sio 未被黑名单屏蔽"
    fi

    # 4.3 是否已加载
    if module_loaded ftdi_sio; then
        log_ok "ftdi_sio 已加载"
    else
        log_info "ftdi_sio 尚未加载（插上设备通常会自动加载）"
        if (( OPT_CHECK_ONLY )); then
            log_hint "安装时会执行: modprobe ftdi_sio"
        else
            need_root
            if run modprobe ftdi_sio; then
                if module_loaded ftdi_sio; then
                    log_ok "ftdi_sio 加载成功"
                else
                    log_warn "modprobe 成功但模块未出现在 /sys/module"
                fi
            else
                log_fail "modprobe ftdi_sio 失败"
                log_hint "若被黑名单屏蔽，先取消屏蔽再试: sudo modprobe -v ftdi_sio"
            fi
        fi
    fi

    # 4.4 设备节点
    local tty; tty="$(list_ttyusb)"
    if [[ -n "${tty// }" ]]; then
        log_ok "检测到串口设备: ${tty}"
        local d
        for d in ${tty}; do
            local perm owner grp
            perm="$(stat -c '%A %U %G' "/dev/${d}" 2>/dev/null || echo '?')"
            log_info "  /dev/${d}  权限 ${perm}"
        done
    else
        log_info "当前没有 /dev/ttyUSB*（设备未插入时属正常）"
        log_hint "插上 USB 转 485 后确认: ls /dev/ttyUSB*"
        log_hint "物理解析确认: lsusb | grep -i 0403"
    fi

    # 4.5 波特率能力说明
    log_info "说明: 本项目用 termios2 + BOTHER 设置任意波特率（2.5 Mbps）"
    log_hint "ftdi_sio 支持任意波特率，无需额外配置"
    log_hint "⚠ USB 转 485 必须用 FT232H / FT2232H；"
    log_hint "  CP2102 / CH340 达不到 2.5 Mbps，会通信失败"
}

# =====================================================================
# 步骤 5 —— 驱动③：evdev（键盘输入）
# =====================================================================
step5_evdev() {
    section "步骤 5 / 10  驱动③：evdev（taihu_keyboardctl 键盘输入）"

    if (( OPT_SKIP_EVDEV )); then
        log_skip "按 --skip-evdev 跳过"
        return 0
    fi

    # 5.1 内核是否编译了 CONFIG_INPUT_EVDEV
    local cfg=""
    if [[ -r "/boot/config-${KREL}" ]]; then
        cfg="/boot/config-${KREL}"
    elif [[ -r /proc/config.gz ]]; then
        cfg="/proc/config.gz"
    fi

    if [[ -n "${cfg}" ]]; then
        local opt
        if [[ "${cfg}" == *.gz ]]; then
            opt="$(zcat "${cfg}" 2>/dev/null | grep -E '^CONFIG_INPUT_EVDEV=' | head -1 || true)"
        else
            opt="$(grep -E '^CONFIG_INPUT_EVDEV=' "${cfg}" 2>/dev/null | head -1 || true)"
        fi
        case "${opt}" in
            CONFIG_INPUT_EVDEV=y) log_ok "内核已内建 evdev（CONFIG_INPUT_EVDEV=y）" ;;
            CONFIG_INPUT_EVDEV=m) log_ok "内核以模块形式提供 evdev（CONFIG_INPUT_EVDEV=m）" ;;
            "")                   log_warn "内核配置里未找到 CONFIG_INPUT_EVDEV（可能被裁剪）" ;;
            *)                    log_warn "CONFIG_INPUT_EVDEV 未启用: ${opt}" ;;
        esac
    else
        log_info "读不到内核配置，跳过 CONFIG_INPUT_EVDEV 检查"
    fi

    # 5.2 evdev 若为模块，确保已加载
    if [[ ! -d /sys/class/input ]]; then
        log_fail "/sys/class/input 不存在，输入子系统不可用"
        return 0
    fi
    log_ok "/sys/class/input 存在（输入子系统正常）"

    if module_loaded evdev; then
        log_ok "evdev 模块已加载"
    elif module_available evdev; then
        log_info "evdev 编译为模块但未加载"
        if (( OPT_CHECK_ONLY )); then
            log_hint "安装时会执行: modprobe evdev"
        else
            need_root
            run modprobe evdev && log_ok "evdev 加载成功" \
                               || log_warn "modprobe evdev 失败（内建内核时无此模块属正常）"
        fi
    else
        log_info "evdev 为内核内建或不可作为模块加载（正常）"
    fi

    # 5.3 设备节点与可读性
    local -a events=(/dev/input/event*)
    if [[ ! -e "${events[0]}" ]]; then
        log_warn "/dev/input/event* 不存在 —— 没有任何输入设备被识别"
        log_hint "接上键盘后重新执行本脚本；或检查 dmesg | grep -i input"
        return 0
    fi
    log_ok "检测到 $((${#events[@]})) 个输入设备节点（/dev/input/event*）"

    # 5.4 判定是否有可读的键盘类设备
    local u readable=0 d
    u="$(target_user || true)"
    for d in "${events[@]}"; do
        [[ -r "${d}" ]] || continue
        if [[ -n "${u}" ]]; then
            if sudo -u "${u}" test -r "${d}" 2>/dev/null; then readable=1; fi
        else
            readable=1
        fi
        if (( readable )); then break; fi
    done

    if (( readable )); then
        log_ok "当前身份可读 /dev/input/event*，taihu_keyboardctl 可以直接用 evdev"
    else
        log_warn "当前身份无法读 /dev/input/event*，taihu_keyboardctl 会退回 stdin 模式"
        log_hint "修复: 把用户加入 input 组（步骤 6 会处理），然后【重新登录】生效"
        log_hint "临时验证: sudo ./build/taihu_keyboardctl"
    fi
}

# =====================================================================
# 步骤 6 —— 用户组权限
# =====================================================================
step6_groups() {
    section "步骤 6 / 10  用户组权限（dialout / input）"

    if (( OPT_SKIP_GROUPS )); then
        log_skip "按 --skip-groups 跳过"
        return 0
    fi

    local u
    if ! u="$(target_user)"; then
        log_skip "无法确定要授权的普通用户（当前直接以 root 运行）"
        log_hint "如需为非 root 用户授权，请用 sudo 运行，或手动："
        log_hint "  sudo usermod -aG dialout,input <用户名>"
        return 0
    fi

    log_info "目标用户       : ${u}"

    # 组是否存在于系统上
    local g need_groups=()
    for g in dialout input; do
        if ! getent group "${g}" >/dev/null 2>&1; then
            log_warn "系统里没有 ${g} 组（异常，Debian 系通常自带）"
            continue
        fi
        if id -nG "${u}" 2>/dev/null | tr ' ' '\n' | grep -qx "${g}"; then
            log_ok "${u} 已在 ${g} 组"
        else
            case "${g}" in
                dialout) log_warn "${u} 不在 dialout 组 —— 读写 /dev/ttyUSB* 会 Permission denied" ;;
                input)   log_warn "${u} 不在 input 组 —— 读 /dev/input/event* 会 Permission denied" ;;
            esac
            need_groups+=("${g}")
        fi
    done

    if (( ${#need_groups[@]} == 0 )); then
        return 0
    fi

    local joined; joined="$(IFS=,; printf '%s' "${need_groups[*]}")"
    if (( OPT_CHECK_ONLY )); then
        log_hint "安装时会执行: usermod -aG ${joined} ${u}"
        log_hint "执行后需要【注销重新登录】（或重启）才会生效"
        return 0
    fi

    need_root
    if run usermod -aG "${joined}" "${u}"; then
        log_ok "已把 ${u} 加入 ${joined} 组"
        log_warn "组变更需要【注销重新登录】（或重启）才生效"
        log_hint "立即验证（不改登录会话）: newgrp ${joined%%','*}"
        GROUP_CHANGED=1
    else
        log_fail "usermod 失败，请手动执行: sudo usermod -aG ${joined} ${u}"
    fi
}

# =====================================================================
# 步骤 7 —— brltty 冲突排查
# =====================================================================
step7_brltty() {
    section "步骤 7 / 10  brltty 冲突排查"

    local installed=0 running=0
    if command -v dpkg-query >/dev/null 2>&1; then
        dpkg-query -W -f='${Status}' brltty 2>/dev/null | grep -q "install ok installed" && installed=1
    elif command -v rpm >/dev/null 2>&1; then
        rpm -q brltty >/dev/null 2>&1 && installed=1
    fi

    if command -v systemctl >/dev/null 2>&1; then
        systemctl is-active --quiet brltty 2>/dev/null && running=1
        systemctl is-active --quiet brltty-udev 2>/dev/null && running=1
    fi

    # 检查是否有 brltty 相关服务被 mask/enable
    local masked=0
    if command -v systemctl >/dev/null 2>&1; then
        [[ "$(systemctl is-enabled brltty 2>/dev/null || true)" == "masked" ]] && masked=1
    fi

    if (( ! installed && ! running )); then
        log_ok "未安装/未运行 brltty，USB 串口不会被抢占"
        return 0
    fi

    if (( installed )); then
        log_warn "检测到 brltty 已安装"
        log_hint "brltty 是盲文显示器服务，它会通过 udev 规则抢占"
        log_hint "FTDI / CH340 / CP210x 等 USB 串口，表现为："
        log_hint "  · /dev/ttyUSB0 插上后几秒内消失"
        log_hint "  · dmesg 出现 \"brltty: ... ttyUSB0\" 或 USB 设备断开"
    fi
    if (( running )); then
        log_warn "brltty 服务正在运行"
    fi
    if (( masked )); then
        log_ok "brltty 已被 mask（不会再启动）"
        return 0
    fi

    if (( ! OPT_FIX_BRLTTY )); then
        log_info "默认不自动处理，避免误伤真正使用盲文设备的场景"
        log_hint "若确认不需要盲文支持，重新运行时加 --fix-brltty，本脚本会执行:"
        log_hint "  sudo systemctl mask --now brltty.service brltty-udev.service"
        log_hint "也可直接卸载: sudo apt-get remove --purge brltty"
        return 0
    fi

    if (( OPT_CHECK_ONLY )); then
        log_hint "安装时会执行: systemctl mask --now brltty.service brltty-udev.service"
        return 0
    fi

    need_root
    local svc
    for svc in brltty.service brltty-udev.service brltty.path; do
        if systemctl list-unit-files "${svc}" >/dev/null 2>&1; then
            run systemctl mask --now "${svc}" 2>/dev/null || true
        fi
    done
    log_ok "brltty 相关服务已 mask（如需恢复: sudo systemctl unmask brltty.service）"
}

# =====================================================================
# 步骤 8 —— 设备稳定命名（可选）
# =====================================================================
step8_stable_names() {
    section "步骤 8 / 10  设备稳定命名（可选）"

    if (( ! OPT_WITH_RS485_SYMLINK )); then
        log_skip "未指定 --with-rs485-symlink，跳过"
        log_hint "本项目默认用 /dev/ttyUSB0 即可；多路 485 需要固定名字时再加此参数"
        return 0
    fi

    local -a pids
    mapfile -t pids < <(detect_ftdi_pids)
    if (( ${#pids[@]} == 0 )); then
        log_warn "未检测到 FTDI 设备的 PID，无法生成精确规则"
        return 0
    fi

    local lines=()
    lines+=("# TaiHu_motor_control —— USB-RS485 稳定设备名")
    lines+=("# 由 deploy/${SCRIPT_NAME} 生成。只增加软链，不改设备名。")
    lines+=("# 删除本文件后执行 sudo udevadm control --reload-rules 即可恢复。")
    local p
    for p in "${pids[@]}"; do
        lines+=("SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${FTDI_VID}\", ATTRS{idProduct}==\"${p}\", SYMLINK+=\"taihu-rs485\", MODE=\"0660\", GROUP=\"dialout\"")
    done
    local content
    content="$(printf '%s\n' "${lines[@]}")"

    log_info "将写入 ${RS485_UDEV_RULE}（匹配 ${FTDI_VID}:${pids[*]}）"
    if (( OPT_CHECK_ONLY )); then
        printf '%s\n' "${content}" | sed 's/^/           /'
        return 0
    fi

    need_root
    write_file "${RS485_UDEV_RULE}" "${content}"
    run chmod 644 "${RS485_UDEV_RULE}"
    run udevadm control --reload-rules
    run udevadm trigger --subsystem-match=tty
    log_ok "已配置稳定软链 /dev/taihu-rs485"
    log_hint "插上设备后: ls -l /dev/taihu-rs485"
    log_hint "程序里可直接用: ./build/taihu_mpt_encoder /dev/taihu-rs485 2500000"
}

# =====================================================================
# 步骤 9 —— 状态总览
# =====================================================================
step9_status() {
    section "步骤 9 / 10  状态总览"

    # 中文是双宽字符，printf 的 %-Ns 按字符数补齐会错位，
    # 因此这里统一用「标签：值」形式，不做列对齐。
    line() { printf '  %s：%s\n' "$1" "$2"; }

    printf '%s① 鲲弘 CAN FD（kcan）%s\n' "${C_BOLD}" "${C_RESET}"
    if module_loaded kcan; then
        line "模块状态" "已加载"
    else
        line "模块状态" "未加载"
    fi
    local ifs; ifs="$(list_can_ifaces || true)"
    line "CAN 接口" "${ifs:-无}"
    if [[ -n "${ifs// /}" ]]; then
        local one; one="$(printf '%s\n' ${ifs} | head -1)"
        local st
        st="$(ip -d link show "${one}" 2>/dev/null | grep -o 'can state [A-Z]*' | head -1 || true)"
        line "通道状态" "${st:-未知（通道未 up 时可能无此信息）}"
    fi
    if command -v dkms >/dev/null 2>&1; then
        local ds; ds="$(dkms status 2>/dev/null | grep -i kunhong | head -3 || true)"
        line "DKMS" "${ds:-未注册}"
    else
        line "DKMS" "未安装"
    fi

    printf '\n%s② 金钢 MPT 磁编码器（ftdi_sio）%s\n' "${C_BOLD}" "${C_RESET}"
    if module_loaded ftdi_sio; then
        line "模块状态" "已加载"
    else
        line "模块状态" "未加载"
    fi
    local ttys; ttys="$(list_ttyusb || true)"
    line "串口设备" "${ttys:-无}"

    printf '\n%s③ 键盘输入（evdev）%s\n' "${C_BOLD}" "${C_RESET}"
    if [[ -d /sys/class/input ]]; then
        line "输入子系统" "可用"
    else
        line "输入子系统" "不可用"
    fi
    local -a ev=(/dev/input/event*)
    if [[ -e "${ev[0]}" ]]; then
        line "输入设备数" "${#ev[@]} 个"
    else
        line "输入设备数" "0 个"
    fi

    printf '\n%s④ 权限%s\n' "${C_BOLD}" "${C_RESET}"
    local u
    if u="$(target_user)"; then
        line "用户" "${u}"
        local g ok_g=()
        for g in dialout input; do
            if id -nG "${u}" 2>/dev/null | tr ' ' '\n' | grep -qx "${g}"; then
                ok_g+=("${g}✓")
            else
                ok_g+=("${g}✗")
            fi
        done
        line "组" "${ok_g[*]}"
    else
        line "用户" "以 root 运行，未检测"
    fi
}

# =====================================================================
# 步骤 10 —— 后续手动操作
# =====================================================================
step10_next_steps() {
    section "步骤 10 / 10  后续手动操作（本脚本不会自动执行）"

    cat <<EOF
  1) 确认 CAN 通道编号（每次插拔 USB 都要重新确认，免 root）:
         ./build/kcanctl list

  2) 手动打开所需通道并设 1 Mbps（必须逐个手动开，不要批量自动开）:
         sudo ./build/kcanctl up can0 1000000

     接口编号随 USB 插入顺序变化，不要假设固定就是 can0。

  3) 验证总线健康（bus-off 应为 0，berr-counter tx 应为 0）:
         ip -d link show can0 | grep "can state"

  4) 确认 USB 转 RS485:
         ls /dev/ttyUSB*
         ./build/taihu_mpt_encoder /dev/ttyUSB0 2500000

  5) 确认键盘输入:
         ./build/taihu_keyboardctl

  6) 跑下位机主程序:
         sudo ./build/walk_control_V1

  ⚠ 悬空（没接设备）的 CAN 通道不要跑真实流量，
     20 秒左右会 OSError: [Errno 105] No buffer space available。

  ⚠ 开机自加载只加载模块，不会 up 通道 —— 这是有意为之。
     通道必须在确认接线后手动打开，避免对错误通道下发 1Mbps 流量。

  ⚠ 若本步骤 6 修改了用户组，需要【注销重新登录】才生效。
EOF

    if (( GROUP_CHANGED )); then
        printf '\n  %s⟳ 提醒：%s用户组已变更，请注销重新登录后再验证权限%s\n' \
            "${C_YELLOW}" "${C_BOLD}" "${C_RESET}"
    fi
}

# =====================================================================
# 卸载
# =====================================================================
do_uninstall() {
    section "卸载模式"

    if (( OPT_CHECK_ONLY )); then
        log_info "--uninstall 与 --check 同时使用：只报告将要删除的内容"
    fi

    # 1) kcan：转交专用脚本
    if [[ -x "${KCAN_INSTALL_SCRIPT}" ]]; then
        local -a kargs=( --uninstall )
        if (( OPT_ASSUME_YES || OPT_DRY_RUN )); then kargs+=( -y ); fi
        if (( OPT_DRY_RUN ));    then kargs+=( --dry-run ); fi
        # --check 也透传，否则子脚本会因缺 root 直接中止
        if (( OPT_CHECK_ONLY ));  then kargs+=( --check ); fi
        log_info "转交 ${KCAN_INSTALL_SCRIPT##*/} 卸载 kcan 驱动"
        "${KCAN_INSTALL_SCRIPT}" "${kargs[@]}" || log_warn "install_kcan.sh 卸载返回非零"
    else
        log_warn "找不到 ${KCAN_INSTALL_SCRIPT}，无法自动卸载 kcan"
        log_hint "手动: sudo modprobe -r kcan && sudo rm -f /lib/modules/${KREL}/misc/kcan.ko"
    fi

    # 2) 本脚本建的 RS485 软链规则
    if [[ -f "${RS485_UDEV_RULE}" ]]; then
        log_info "删除 ${RS485_UDEV_RULE}"
        if (( OPT_CHECK_ONLY )); then
            log_hint "--check 模式：未实际删除"
        else
            need_root
            run rm -f "${RS485_UDEV_RULE}"
            run udevadm control --reload-rules
        fi
    else
        log_skip "没有本脚本生成的 ${RS485_UDEV_RULE}"
    fi

    # 3) 系统自带驱动不动
    log_info "ftdi_sio / evdev 属于内核自带模块，脚本不做卸载"
    log_hint "如需临时卸载: sudo modprobe -r ftdi_sio"

    # 4) 用户组不动
    log_info "用户组（dialout / input）不做回退，保留不影响系统"
}

# =====================================================================
# 汇总
# =====================================================================
print_summary() {
    section "汇总"
    printf '  %s完成%s %d    %s警告%s %d    %s失败%s %d    %s跳过%s %d\n' \
        "${C_GREEN}"  "${C_RESET}" "${N_OK}" \
        "${C_YELLOW}" "${C_RESET}" "${N_WARN}" \
        "${C_RED}"    "${C_RESET}" "${N_FAIL}" \
        "${C_YELLOW}" "${C_RESET}" "${N_SKIP}"

    if (( N_FAIL > 0 )); then
        printf '\n%s存在 %d 项失败，请按上面的 [失败] 提示逐条处理。%s\n' \
            "${C_RED}" "${N_FAIL}" "${C_RESET}"
        return 1
    elif (( N_WARN > 0 )); then
        printf '\n%s无致命错误，但有 %d 项警告需要确认。%s\n' \
            "${C_YELLOW}" "${N_WARN}" "${C_RESET}"
        return 0
    else
        printf '\n%s三类驱动检查/安装完成。%s\n' "${C_GREEN}" "${C_RESET}"
        return 0
    fi
}

# =====================================================================
# 主流程
# =====================================================================
main() {
    parse_args "$@"

    printf '%s\n' "${C_BOLD}=====================================================================${C_RESET}"
    printf '%s  TaiHu_motor_control —— 一键安装全部内核驱动%s\n' "${C_BOLD}" "${C_RESET}"
    printf '%s  目标: RK3588 / aarch64 · 内核 %s%s\n' "${C_BOLD}" "${KREL}" "${C_RESET}"
    printf '%s  覆盖: kcan (CAN FD) + ftdi_sio (RS485) + evdev (键盘)%s\n' "${C_BOLD}" "${C_RESET}"
    printf '%s=====================================================================%s\n' "${C_BOLD}" "${C_RESET}"

    if (( OPT_CHECK_ONLY )); then
        printf '%s[只检查模式]%s 不会修改任何文件，也不需要 root\n' "${C_YELLOW}" "${C_RESET}"
    elif (( OPT_DRY_RUN )); then
        printf '%s[试运行模式]%s 不会修改任何文件\n' "${C_YELLOW}" "${C_RESET}"
    fi

    if (( OPT_UNINSTALL )); then
        do_uninstall
        print_summary
        return $?
    fi

    step1_env
    step2_apt_prep
    step3_kcan
    step4_ftdi
    step5_evdev
    step6_groups
    step7_brltty
    step8_stable_names
    step9_status
    step10_next_steps

    print_summary
}

main "$@"