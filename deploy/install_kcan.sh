#!/usr/bin/env bash
# =====================================================================
# TaiHu_motor_control —— kcan 驱动安装 / 卸载 / 开机自加载
#
# 本脚本【只】管 kcan 驱动的安装与维护，不编译项目、不装编译器。
# 环境准备（gcc/cmake/can-utils 等）请用 deploy/install_env.sh。
#
# 为什么单独拆一个脚本：
#   - 驱动是可反复重装的（换内核、换设备、回滚），单独跑更快
#   - 与项目编译解耦，出问题时能快速定位是「驱动」还是「代码」
#   - 支持卸载 / 回滚，install_env.sh 不适合承担这种破坏性操作
#
# 脚本做的事（10 个阶段，全部幂等，可反复执行）：
#   步骤 1  环境检查（root / aarch64 / 内核版本 / 工具链）
#   步骤 2  定位 kcan.ko（多路径搜索 + 架构过滤）
#   步骤 3  严格校验 .ko（架构 / vermagic / 签名 / 依赖符号）
#   步骤 4  安装 .ko 到 /lib/modules/$(uname -r)/misc/
#   步骤 5  写入 modprobe 配置 /etc/modprobe.d/kcan.conf
#   步骤 6  写入开机自加载 /etc/modules-load.d/kcan.conf
#   步骤 7  安装厂商用户态工具与头文件（可选）
#   步骤 8  安装 udev 规则（可选，默认不装）
#   步骤 9  加载模块并验证 CAN 通道出现
#   步骤 10 汇总与后续提示
#
# 用法：
#   sudo ./deploy/install_kcan.sh                     # 全流程（推荐）
#   sudo ./deploy/install_kcan.sh --check             # 只检查，不改系统
#   sudo ./deploy/install_kcan.sh --ko /path/kcan.ko  # 指定驱动文件
#   sudo ./deploy/install_kcan.sh --reload            # 重装并热重载模块
#   sudo ./deploy/install_kcan.sh --uninstall         # 完整卸载
#   ./deploy/install_kcan.sh --help
#
# 重要约束：
#   * 本脚本【绝不】自动 up CAN 通道。通道必须按现场接线手动打开：
#         sudo ./build/kcanctl up can4 1000000
#     （接口编号随 USB 插入顺序变化，先用 ./build/kcanctl list 确认）
#   * 本脚本【不】编译内核模块。kcan.ko 必须是厂商为
#     本机内核版本 + aarch64 提供的预编译件。
#   * --reload / --uninstall 会先 down 掉所有 canN 接口（rmmod 的前置条件），
#     若此时有程序在用总线，请先停掉。
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

readonly MOD_INSTALL_DIR="/lib/modules/${KREL}/misc"
readonly MOD_INSTALL_PATH="${MOD_INSTALL_DIR}/kcan.ko"
readonly MODPROBE_CONF="/etc/modprobe.d/kcan.conf"
readonly MODULES_LOAD_CONF="/etc/modules-load.d/kcan.conf"
readonly UDEV_RULE_DST="/etc/udev/rules.d/45-kcan.rules"
readonly BIN_DIR="/usr/local/bin"
readonly INCLUDE_DIR="/usr/include"

# 厂商 SDK 配套文件（按需安装，缺失不算失败）
readonly VENDOR_TOOLS=(lskcan kcan_monitor kcan_make_devices kcanosdiag.sh)
readonly VENDOR_HEADERS=(kcan.h kcanfd.h)

# .ko 搜索路径（按优先级；已去重，同一文件不会重复报告）
DRIVER_SEARCH_PATHS=(
    "${SCRIPT_DIR}/kcan-driver"              # 推荐：随仓库分发
    "${PROJECT_ROOT}/kcan-driver"
    "${PROJECT_ROOT}/.kunhong_sdk"           # 厂商 SDK（通常是 x86-64，仅作发现）
)

# 颜色（非 tty 或 NO_COLOR 时自动关闭）
if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
    C_RED=$'\033[31m'; C_GREEN=$'\033[32m'; C_YELLOW=$'\033[33m'
    C_BLUE=$'\033[34m'; C_BOLD=$'\033[1m'; C_RESET=$'\033[0m'
else
    C_RED=''; C_GREEN=''; C_YELLOW=''; C_BLUE=''; C_BOLD=''; C_RESET=''
fi

# 统计
declare -i N_OK=0 N_WARN=0 N_FAIL=0 N_SKIP=0

# 驱动校验结果（与 N_FAIL 解耦：环境层面的失败不应阻断驱动安装）
VALIDATE_OK=0

# 命令行开关
OPT_CHECK_ONLY=0
OPT_DRY_RUN=0
OPT_ASSUME_YES=0
OPT_FORCE=0
OPT_UNINSTALL=0
OPT_RELOAD=0
OPT_NO_AUTOLOAD=0
OPT_NO_TOOLS=0
OPT_NO_HEADERS=0
OPT_WITH_UDEV=0
OPT_NO_LOAD=0
OPT_KO_PATH=""

# ---------------------------------------------------------------------
# 输出辅助
# ---------------------------------------------------------------------
log_info() { printf '%s[信息]%s %s\n' "${C_BLUE}"   "${C_RESET}" "$*"; }
log_ok()   { printf '%s[完成]%s %s\n' "${C_GREEN}"  "${C_RESET}" "$*"; N_OK+=1; }
log_warn() { printf '%s[警告]%s %s\n' "${C_YELLOW}" "${C_RESET}" "$*" >&2; N_WARN+=1; }
log_fail() { printf '%s[失败]%s %s\n' "${C_RED}"    "${C_RESET}" "$*" >&2; N_FAIL+=1; }
log_skip() { printf '%s[跳过]%s %s\n' "${C_YELLOW}" "${C_RESET}" "$*"; N_SKIP+=1; }

section() {
    printf '\n%s=====================================================================%s\n' "${C_BOLD}" "${C_RESET}"
    printf '%s %s%s\n' "${C_BOLD}" "$*" "${C_RESET}"
    printf '%s=====================================================================%s\n' "${C_BOLD}" "${C_RESET}"
}

die() {
    printf '\n%s[中止]%s %s\n' "${C_RED}" "${C_RESET}" "$*" >&2
    exit 1
}

# 执行命令（dry-run 时只打印）
run() {
    if (( OPT_DRY_RUN )); then
        printf '%s[试运行]%s %s\n' "${C_YELLOW}" "${C_RESET}" "$*"
        return 0
    fi
    "$@"
}

need_root() {
    # --dry-run 只打印命令、不改系统，因此不要求 root，方便普通用户预览全流程
    (( OPT_DRY_RUN )) && return 0
    if [[ "${EUID}" -ne 0 ]]; then
        die "该步骤需要 root 权限，请用 sudo 重新运行：sudo ./deploy/${SCRIPT_NAME}"
    fi
}

confirm() {
    (( OPT_ASSUME_YES )) && return 0
    local reply
    read -r -p "$1 [y/N] " reply || true
    [[ "${reply}" =~ ^[Yy]$ ]]
}

# 判断文件是否为 aarch64 ELF
is_aarch64_elf() {
    local f="$1"
    [[ -f "${f}" ]] || return 1
    if command -v file >/dev/null 2>&1; then
        file -b "${f}" 2>/dev/null | grep -qi 'aarch64\|ARM aarch64'
    else
        # 无 file 命令时读 ELF header：e_machine @ offset 18 (小端) = 0x00B7
        local m
        m="$(od -An -tx1 -j18 -N2 "${f}" 2>/dev/null | tr -d ' \n')"
        [[ "${m}" == "b700" ]]
    fi
}

# 判断文件是否为 x86-64 ELF
is_x86_64_elf() {
    local f="$1"
    [[ -f "${f}" ]] || return 1
    command -v file >/dev/null 2>&1 || return 1
    file -b "${f}" 2>/dev/null | grep -qi 'x86-64'
}

# 取 .ko 的 vermagic（优先 modinfo，退回 strings）
ko_vermagic() {
    local ko="$1"
    if command -v modinfo >/dev/null 2>&1; then
        local v
        v="$(modinfo -F vermagic "${ko}" 2>/dev/null || true)"
        [[ -n "${v}" ]] && { printf '%s\n' "${v}"; return 0; }
    fi
    strings "${ko}" 2>/dev/null | grep -m1 -E '^[0-9]+\.[0-9]+\.[0-9]+.*SMP' || true
}

# 模块是否已加载
module_loaded() {
    [[ -d "/sys/module/kcan" ]]
}

# 当前所有 canN 接口名
list_can_ifaces() {
    if command -v ip >/dev/null 2>&1; then
        ip -o link show 2>/dev/null \
            | awk -F': ' '/: can[0-9]+/{print $2}' | tr '\n' ' ' || true
    fi
}

# =====================================================================
# 参数解析
# =====================================================================
usage() {
    cat <<'EOF'
TaiHu_motor_control —— kcan 驱动安装 / 卸载 / 开机自加载

用法:
  sudo ./deploy/install_kcan.sh [选项]

选项:
  --ko <路径>      指定 kcan.ko（默认自动在 deploy/kcan-driver/ 等目录搜索）
  --reload         安装后热重载模块（会先 down 所有 canN 接口）
  --uninstall      完整卸载：rmmod + 删除 .ko/配置/工具
  --no-autoload    不写 /etc/modules-load.d/kcan.conf（即不设开机自加载）
  --no-tools       不安装厂商用户态工具（lskcan / kcan_monitor 等）
  --no-headers     不安装厂商头文件（kcan.h / kcanfd.h）
  --with-udev      安装厂商 udev 规则（/etc/udev/rules.d/45-kcan.rules）
  --no-load        装完不执行 modprobe（只落盘，不加载）
  --force          跳过 vermagic 严格校验（内核版本不完全一致时强行安装）
  --check          只做检查，不修改系统（不需要 root）
  --dry-run        打印将要执行的命令，但不实际执行
  -y, --yes        所有确认自动回答 yes
  -h, --help       显示本帮助

典型场景:
  # 1) 第一次装驱动
  sudo ./deploy/install_kcan.sh --ko /path/to/aarch64/kcan.ko

  # 2) 只想知道现在缺什么（不改系统）
  ./deploy/install_kcan.sh --check

  # 3) 换了新驱动文件，热重载
  sudo ./deploy/install_kcan.sh --ko ./deploy/kcan-driver/kcan.ko --reload

  # 4) 内核升级后需要重装
  sudo ./deploy/install_kcan.sh --reload

  # 5) 彻底卸载
  sudo ./deploy/install_kcan.sh --uninstall

驱动文件放哪里（脚本按顺序搜索，取第一个 aarch64 的）:
      deploy/kcan-driver/kcan.ko                  ← 推荐
      deploy/kcan-driver/driver/kcan.ko
      kcan-driver/kcan.ko
      .kunhong_sdk/**/kcan.ko                     ← 注意：SDK 自带的是 x86-64，会被跳过
      .kunhong_sdk/KH-UCANFD_Linux_SDK-*/driver/kcan.ko

⚠ 关于内核版本：
    kcan.ko 的 vermagic 必须与本机内核（当前 5.10.160）完全一致，
    否则 modprobe 会报 "invalid module format"。
    若本机是厂商定制内核（RK3588 Ubuntu 20.04 常见），apt 里没有匹配的
    linux-headers，无法自行编译，必须向厂商索取对应版本的 aarch64 .ko。

开 CAN 通道（本脚本不会自动执行）:
      ./build/kcanctl list
      sudo ./build/kcanctl up can4 1000000
EOF
}

parse_args() {
    while (($#)); do
        case "$1" in
            --ko)            [[ $# -ge 2 ]] || die "--ko 需要一个参数"
                             OPT_KO_PATH="$2"; shift ;;
            --ko=*)          OPT_KO_PATH="${1#*=}" ;;
            --reload)        OPT_RELOAD=1 ;;
            --uninstall)     OPT_UNINSTALL=1 ;;
            --no-autoload)   OPT_NO_AUTOLOAD=1 ;;
            --no-tools)      OPT_NO_TOOLS=1 ;;
            --no-headers)    OPT_NO_HEADERS=1 ;;
            --with-udev)     OPT_WITH_UDEV=1 ;;
            --no-load)       OPT_NO_LOAD=1 ;;
            --force)         OPT_FORCE=1 ;;
            --check)         OPT_CHECK_ONLY=1 ;;
            --dry-run)       OPT_DRY_RUN=1 ;;
            -y|--yes)        OPT_ASSUME_YES=1 ;;
            -h|--help)       usage; exit 0 ;;
            *)               usage; die "未知参数: $1" ;;
        esac
        shift
    done
}

# =====================================================================
# 步骤 1 —— 环境检查
# =====================================================================
step1_env() {
    section "步骤 1 / 10  环境检查"

    # —— 1.1 权限 ——
    if [[ "${EUID}" -eq 0 ]]; then
        log_ok "运行权限     : root"
    else
        if (( OPT_CHECK_ONLY )); then
            log_info "运行权限     : 普通用户（--check 模式，不需要 root）"
        else
            log_warn "运行权限     : 普通用户（uid=${EUID}），步骤 4/5/6/7/8/9 需要 root"
        fi
    fi

    # —— 1.2 架构 ——
    log_info "CPU 架构     : ${KARCH}"
    case "${KARCH}" in
        aarch64|arm64)
            log_ok "架构为 aarch64，与 RK3588 一致" ;;
        x86_64|amd64)
            log_warn "架构为 x86_64（这是开发机，不是 RK3588 板卡）"
            log_info "             本脚本设计为在板卡上本机执行；在开发机上运行仅适合做语法自检" ;;
        *)
            log_warn "架构为 ${KARCH}，非 aarch64，加载 ARM64 模块会失败" ;;
    esac

    # —— 1.3 内核版本 ——
    log_info "内核版本     : ${KREL}"

    # —— 1.4 发行版 ——
    local distro="" ver=""
    if [[ -r /etc/os-release ]]; then
        # shellcheck disable=SC1091
        distro="$(. /etc/os-release && printf '%s' "${NAME:-unknown}")"
        # shellcheck disable=SC1091
        ver="$(. /etc/os-release && printf '%s' "${VERSION_ID:-}")"
    fi
    log_info "发行版       : ${distro:-未知} ${ver:-}"

    if [[ "${ver}" == "20.04" ]]; then
        log_ok "发行版为 Ubuntu 20.04，与目标一致"
        # Ubuntu 20.04 官方内核是 5.4.x；5.10.x 说明是厂商定制内核
        if [[ "${KREL}" == 5.10.* ]]; then
            log_warn "内核 5.10.x 非 Ubuntu 20.04 官方版本（官方为 5.4.x）"
            log_info "             → 判定为厂商定制内核，apt 中没有匹配的 linux-headers，"
            log_info "               必须使用厂商提供的 aarch64 kcan.ko，无法自行编译"
        fi
    elif [[ -n "${distro}" ]]; then
        log_info "发行版不是 Ubuntu 20.04，本脚本仍可运行，请留意路径差异"
    fi

    # —— 1.5 内核头文件（仅影响「能否自行编译」，本脚本不编译） ——
    if [[ -d "/lib/modules/${KREL}/build" ]]; then
        log_info "内核头文件   : 存在（理论上可自行编译，但本脚本坚持用预编译 .ko）"
    else
        log_info "内核头文件   : 缺失 → 本脚本不编译，使用厂商预编译 .ko（这是预期路径）"
    fi

    # —— 1.6 必需工具 ——
    local tool missing=()
    for tool in depmod modprobe; do
        if command -v "${tool}" >/dev/null 2>&1; then
            log_ok "工具         : ${tool} 可用"
        else
            # /sbin 可能不在普通用户 PATH 里
            if [[ -x "/sbin/${tool}" || -x "/usr/sbin/${tool}" ]]; then
                log_ok "工具         : ${tool} 可用（在 /sbin）"
            else
                missing+=("${tool}")
                log_warn "工具         : ${tool} 缺失"
            fi
        fi
    done
    if ((${#missing[@]} > 0)); then
        log_info "             缺失的 ${missing[*]} 属于 kmod 包，Ubuntu 上执行："
        log_info "             sudo apt-get install -y kmod"
        log_info "             （离线环境请准备 kmod 的 .deb）"
    fi

    # —— 1.7 可选工具（仅影响诊断能力） ——
    for tool in lsmod modinfo file ip lsusb; do
        command -v "${tool}" >/dev/null 2>&1 \
            || log_info "可选工具     : ${tool} 未安装（影响诊断提示，不影响安装）"
    done

    # —— 1.8 Secure Boot 状态（未签名模块会被拒绝） ——
    if command -v mokutil >/dev/null 2>&1; then
        local sb
        sb="$(mokutil --sb-state 2>/dev/null || true)"
        if printf '%s' "${sb}" | grep -qi 'enabled'; then
            log_warn "Secure Boot  : 已启用（未签名内核模块会被拒绝加载）"
            log_info "             若 modprobe 报 'Key was rejected by service'，"
            log_info "             需要在 BIOS 中关闭 Secure Boot 或给模块签名"
        else
            log_ok "Secure Boot  : 未启用（可加载未签名模块）"
        fi
    else
        log_info "Secure Boot  : 无法检测（mokutil 未安装）"
    fi
}

# =====================================================================
# 步骤 2 —— 定位 kcan.ko
# =====================================================================
FOUND_KO=""

step2_locate() {
    section "步骤 2 / 10  定位 kcan.ko"
    FOUND_KO=""

    # —— 2.1 用户显式指定 ——
    if [[ -n "${OPT_KO_PATH}" ]]; then
        if [[ ! -f "${OPT_KO_PATH}" ]]; then
            log_fail "--ko 指定的文件不存在: ${OPT_KO_PATH}"
            return 0
        fi
        FOUND_KO="$(cd "$(dirname "${OPT_KO_PATH}")" && pwd)/$(basename "${OPT_KO_PATH}")"
        log_info "使用 --ko 指定的驱动: ${FOUND_KO}"
        if is_x86_64_elf "${FOUND_KO}"; then
            log_warn "该文件是 x86-64 架构，在 aarch64 板上无法加载（步骤 3 会再确认）"
        fi
        if (( OPT_CHECK_ONLY )); then
            log_ok "已定位驱动文件"
        fi
        return 0
    fi

    # —— 2.2 已安装的位置（重装/检查场景） ——
    if [[ -f "${MOD_INSTALL_PATH}" ]]; then
        log_info "已安装位置   : ${MOD_INSTALL_PATH}（将作为候选之一）"
    fi

    # —— 2.3 按路径优先级搜索，只接受 aarch64 ——
    local p ko candidates
    local -A rejected_map=()
    local -A seen=()
    for p in "${DRIVER_SEARCH_PATHS[@]}"; do
        [[ -d "${p}" ]] || continue
        # 先看目录根部与 driver/ 子目录，再递归兜底
        candidates=()
        [[ -f "${p}/kcan.ko" ]] && candidates+=("${p}/kcan.ko")
        [[ -f "${p}/driver/kcan.ko" ]] && candidates+=("${p}/driver/kcan.ko")
        while IFS= read -r ko; do
            [[ -n "${ko}" ]] && candidates+=("${ko}")
        done < <(find "${p}" -maxdepth 3 -name 'kcan.ko' -type f 2>/dev/null || true)

        for ko in "${candidates[@]}"; do
            # 规范化后去重（同一文件会被多个搜索路径覆盖到）
            local real
            real="$(readlink -f "${ko}" 2>/dev/null || printf '%s' "${ko}")"
            [[ -n "${seen[${real}]:-}" ]] && continue
            seen["${real}"]=1

            if is_aarch64_elf "${ko}"; then
                FOUND_KO="${ko}"
                break 2
            else
                rejected_map["${real}"]="${ko}"
            fi
        done
    done

    # —— 2.4 报告被跳过的非 aarch64 文件 ——
    if ((${#rejected_map[@]} > 0)); then
        log_warn "以下 kcan.ko 不是 aarch64，已跳过（在 RK3588 上无法加载）："
        local r
        for r in "${!rejected_map[@]}"; do
            log_info "             ${r}"
            log_info "               → $(file -b "${r}" 2>/dev/null | cut -d, -f1-2 || echo '未知类型')"
        done
    fi

    # —— 2.5 结论 ——
    if [[ -n "${FOUND_KO}" ]]; then
        log_ok "找到可用的 aarch64 驱动: ${FOUND_KO}"
        return 0
    fi

    log_fail "未找到任何 aarch64 版的 kcan.ko"
    printf '\n' >&2
    printf '%s  —— 你需要向厂商（成都昆虹电子 / Kunhong）索取以下文件 ——%s\n' "${C_BOLD}" "${C_RESET}" >&2
    cat >&2 <<EOF
  文件：kcan.ko
  架构：aarch64（ARM64）
  内核：${KREL}      ← 必须与本机完全一致（含 CONFIG_LOCALVERSION 后缀）
  用途：RK3588 板卡加载 USB-CAN 驱动

  拿到后的放置方式（二选一）：
      mkdir -p ${SCRIPT_DIR}/kcan-driver
      cp /path/to/aarch64/kcan.ko ${SCRIPT_DIR}/kcan-driver/
  或直接用参数指定：
      sudo ./deploy/${SCRIPT_NAME} --ko /path/to/aarch64/kcan.ko

  验证方式（在开发机上拷完就能验，不需要板卡）：
      file kcan.ko        # 必须含 "ARM aarch64"
      modinfo kcan.ko     # vermagic 必须含 ${KREL}
EOF
    printf '\n%s  为什么不能自己编译：%s\n' "${C_BOLD}" "${C_RESET}" >&2
    cat >&2 <<EOF
  kcan.ko 是内核模块，编译需要内核构建树 /lib/modules/${KREL}/build。

  若本机是 RK3588 厂商定制内核（如 5.10.160，Ubuntu 20.04 官方内核是 5.4.x），
  apt 源里没有匹配的 linux-headers 包，无法在板上自行编译；
  即便是官方内核，Ubuntu 20.04 的 linux-headers 也需与内核版本逐位对应。
  因此本脚本只走「厂商预编译 .ko」这一条路径。
EOF
    printf '\n' >&2
}

# =====================================================================
# 步骤 3 —— 严格校验 .ko
# =====================================================================
step3_validate() {
    section "步骤 3 / 10  校验驱动文件"
    VALIDATE_OK=0

    if [[ -z "${FOUND_KO}" ]]; then
        log_skip "无驱动文件可校验"
        return 0
    fi

    # —— 3.1 文件基本信息 ——
    log_info "文件路径     : ${FOUND_KO}"
    log_info "文件大小     : $(du -h "${FOUND_KO}" 2>/dev/null | cut -f1)"
    log_info "文件类型     : $(file -b "${FOUND_KO}" 2>/dev/null | cut -d, -f1-3 || echo 未知)"

    # —— 3.2 架构（硬性） ——
    if is_aarch64_elf "${FOUND_KO}"; then
        log_ok "架构校验通过（aarch64）"
    else
        log_fail "架构校验失败：该文件不是 aarch64 ELF，板上加载会报 Exec format error"
        return 0
    fi
    # —— 3.3 vermagic（硬性，除非 --force） ——
    local vm=""
    vm="$(ko_vermagic "${FOUND_KO}")"
    if [[ -n "${vm}" ]]; then
        log_info "模块 vermagic : ${vm}"
        local vm_krel="${vm%% *}"
        if [[ "${vm_krel}" == "${KREL}" ]]; then
            log_ok "内核版本完全匹配（${KREL}）"
        else
            if (( OPT_FORCE )); then
                log_warn "内核版本不匹配：本机 ${KREL}，模块要求 ${vm_krel}"
                log_info "             已指定 --force，将继续安装（大概率加载失败，仅供排错）"
            else
                log_fail "内核版本不匹配：本机 ${KREL}，模块要求 ${vm_krel}"
                log_info "             强行加载会报 'invalid module format'"
                log_info "             请向厂商索取 ${KREL} 版本的 aarch64 kcan.ko"
                log_info "             如确需尝试，加 --force 参数"
                return 0
            fi
        fi
    else
        log_warn "无法读取 vermagic（modinfo/strings 均失败），跳过内核版本校验"
    fi

    # —— 3.4 签名信息（信息性） ——
    local sig=""
    if command -v modinfo >/dev/null 2>&1; then
        sig="$(modinfo -F signer "${FOUND_KO}" 2>/dev/null || true)"
        if [[ -n "${sig}" ]]; then
            log_info "模块签名者   : ${sig}"
        else
            log_info "模块签名     : 未签名（Secure Boot 开启时会被拒绝）"
        fi
    fi

    # —— 3.5 依赖符号（信息性） ——
    if command -v modinfo >/dev/null 2>&1; then
        local deps
        deps="$(modinfo -F depends "${FOUND_KO}" 2>/dev/null || true)"
        log_info "依赖模块     : ${deps:-（无）}"
    fi

    # —— 3.6 版本字符串（信息性） ——
    if command -v modinfo >/dev/null 2>&1; then
        local mver
        mver="$(modinfo -F version "${FOUND_KO}" 2>/dev/null || true)"
        [[ -n "${mver}" ]] && log_info "驱动版本     : ${mver}"
        local mdesc
        mdesc="$(modinfo -F description "${FOUND_KO}" 2>/dev/null || true)"
        [[ -n "${mdesc}" ]] && log_info "驱动描述     : ${mdesc}"
    fi

    VALIDATE_OK=1
    log_ok "校验完成，可以安装"
}

# =====================================================================
# 步骤 4 —— 安装 .ko
# =====================================================================
step4_install_ko() {
    section "步骤 4 / 10  安装内核模块"

    if [[ -z "${FOUND_KO}" ]]; then
        log_skip "无驱动文件可安装"
        return 0
    fi

    # 校验未通过时不安装（架构/版本不符）
    if (( ! VALIDATE_OK )); then
        log_skip "校验未通过，跳过安装（先解决步骤 3 的 [失败] 项）"
        return 0
    fi

    need_root
    log_info "安装目标     : ${MOD_INSTALL_PATH}"

    # —— 4.1 已加载时需要先卸载 ——
    if module_loaded; then
        log_warn "kcan 模块当前已加载，覆盖 .ko 需要先 rmmod"
        if (( OPT_RELOAD )); then
            log_info "按 --reload 自动卸载旧模块"
            unload_module
        else
            log_info "             覆盖安装后需手动 rmmod + modprobe 才生效"
            log_info "             或下次运行时加 --reload"
        fi
    fi

    # —— 4.2 备份已有文件 ——
    if [[ -f "${MOD_INSTALL_PATH}" ]] && ! (( OPT_DRY_RUN )); then
        local bak="${MOD_INSTALL_PATH}.bak.$(date +%Y%m%d%H%M%S)"
        cp -f "${MOD_INSTALL_PATH}" "${bak}"
        log_info "已备份旧模块 : ${bak}"
    fi

    # —— 4.3 复制到位 ——
    run mkdir -p "${MOD_INSTALL_DIR}"
    run cp -f "${FOUND_KO}" "${MOD_INSTALL_PATH}"
    run chmod 644 "${MOD_INSTALL_PATH}"

    if (( OPT_DRY_RUN )); then
        log_info "[试运行] 跳过文件校验"
    else
        if [[ -f "${MOD_INSTALL_PATH}" ]] && cmp -s "${FOUND_KO}" "${MOD_INSTALL_PATH}"; then
            log_ok "模块已复制到位"
        else
            log_fail "模块复制失败：${MOD_INSTALL_PATH}"
            return 0
        fi
    fi

    # —— 4.4 depmod 更新依赖表 ——
    local depmod_bin=""
    depmod_bin="$(command -v depmod 2>/dev/null || true)"
    [[ -z "${depmod_bin}" && -x /sbin/depmod ]] && depmod_bin=/sbin/depmod
    [[ -z "${depmod_bin}" && -x /usr/sbin/depmod ]] && depmod_bin=/usr/sbin/depmod

    if [[ -n "${depmod_bin}" ]]; then
        if (( OPT_DRY_RUN )); then
            log_info "[试运行] ${depmod_bin} -a ${KREL}"
        elif "${depmod_bin}" -a "${KREL}" 2>/dev/null; then
            log_ok "depmod 完成（模块依赖表已更新）"
        else
            log_warn "depmod 返回非零（通常不影响 modprobe 直接加载）"
        fi
    else
        log_warn "depmod 不可用，跳过依赖表更新（modprobe 可能找不到模块）"
    fi

    # —— 4.5 安装厂商配套的 modprobe 配置 ——
    local base
    base="$(dirname "${FOUND_KO}")"
    run mkdir -p /etc/modprobe.d
    if [[ -f "${base}/kcan.conf" ]]; then
        if [[ -f "${MODPROBE_CONF}" ]] && ! (( OPT_DRY_RUN )); then
            cp -f "${MODPROBE_CONF}" "${MODPROBE_CONF}.bak.$(date +%Y%m%d%H%M%S)"
        fi
        log_info "modprobe 配置 : 使用厂商文件 ${base}/kcan.conf"
        run cp -f "${base}/kcan.conf" "${MODPROBE_CONF}"
        log_ok "modprobe 配置已安装"
    else
        log_info "modprobe 配置 : 厂商未提供 kcan.conf，写入最小配置"
        if (( OPT_DRY_RUN )); then
            log_info "[试运行] 写入 ${MODPROBE_CONF}"
        else
            cat > "${MODPROBE_CONF}" <<'KCONF_EOF'
# kcan 驱动配置（由 deploy/install_kcan.sh 生成）
# 完整选项见厂商 SDK 的 driver/kcan.conf
#
# 说明：本项目通过 SocketCAN 访问，波特率在打开通道时由
#       `kcanctl up <if> 1000000` 指定，不在这里固定。
#
# options kcan rxqsize=1024
# options kcan txqsize=1024
KCONF_EOF
            log_ok "最小 modprobe 配置已写入"
        fi
    fi
}

# =====================================================================
# 步骤 5 / 6 —— 开机自加载配置
# =====================================================================
step5_autoload() {
    section "步骤 5 / 10  开机自加载配置"

    if (( OPT_NO_AUTOLOAD )); then
        log_skip "按 --no-autoload 跳过（重启后需手动 modprobe kcan）"
        return 0
    fi

    need_root
    log_info "配置文件     : ${MODULES_LOAD_CONF}"
    log_info "作用         : 开机时由 systemd-modules-load 自动执行 modprobe kcan"
    log_info "不涉及       : 不会 up 任何 CAN 通道（通道仍需手动打开）"

    if (( OPT_DRY_RUN )); then
        log_info "[试运行] 写入 ${MODULES_LOAD_CONF} 内容: kcan"
        return 0
    fi

    run mkdir -p /etc/modules-load.d
    if [[ -f "${MODULES_LOAD_CONF}" ]] \
        && grep -qxF 'kcan' "${MODULES_LOAD_CONF}" 2>/dev/null; then
        log_ok "开机自加载已配置（无需改动）"
    else
        cat > "${MODULES_LOAD_CONF}" <<'LOAD_EOF'
# 开机自动加载 kcan 驱动（由 deploy/install_kcan.sh 生成）
# 注意：仅加载模块，不打开任何 CAN 通道。
#       通道需按现场接线手动执行：sudo ./build/kcanctl up <if> 1000000
kcan
LOAD_EOF
        log_ok "开机自加载已配置"
    fi

    # 校验 systemd-modules-load 服务存在（仅提示）
    if command -v systemctl >/dev/null 2>&1; then
        if systemctl is-enabled systemd-modules-load.service >/dev/null 2>&1; then
            log_ok "systemd-modules-load 服务已启用（开机生效）"
        else
            log_warn "systemd-modules-load 服务未启用，开机自加载不会生效"
            log_info "             启用命令: sudo systemctl enable systemd-modules-load"
        fi
    else
        log_info "无 systemctl（非 systemd 系统），/etc/modules-load.d 可能不被读取"
        log_info "             此时可改用 /etc/modules（老式方式），或手动 modprobe"
    fi
}

# =====================================================================
# 步骤 6 —— 厂商用户态工具与头文件
# =====================================================================
step6_tools() {
    section "步骤 6 / 10  厂商用户态工具与头文件"

    if [[ -z "${FOUND_KO}" ]]; then
        log_skip "无驱动目录可参考"
        return 0
    fi

    local base
    base="$(dirname "${FOUND_KO}")"
    need_root

    # —— 6.1 用户态工具 ——
    if (( OPT_NO_TOOLS )); then
        log_skip "按 --no-tools 跳过用户态工具安装"
    else
        local tool installed=0 skipped=0
        for tool in "${VENDOR_TOOLS[@]}"; do
            [[ -f "${base}/${tool}" ]] || continue
            # 若为 ELF 则必须 aarch64；纯脚本（无 ELF 头）直接装
            if command -v file >/dev/null 2>&1 \
                && file -b "${base}/${tool}" 2>/dev/null | grep -qi 'ELF' \
                && ! is_aarch64_elf "${base}/${tool}"; then
                log_warn "${tool} 不是 aarch64，跳过（PC 版工具在板上无法运行）"
                skipped+=1
                continue
            fi
            run cp -f "${base}/${tool}" "${BIN_DIR}/${tool}"
            run chmod 755 "${BIN_DIR}/${tool}"
            installed+=1
        done

        # kcan-settings 是 ELF，单独处理架构
        if [[ -f "${base}/kcan-settings" ]]; then
            if is_aarch64_elf "${base}/kcan-settings"; then
                run cp -f "${base}/kcan-settings" "${BIN_DIR}/kcan-settings"
                run chmod 755 "${BIN_DIR}/kcan-settings"
                installed+=1
            else
                log_warn "kcan-settings 不是 aarch64，跳过安装"
                skipped+=1
            fi
        fi

        if (( installed > 0 )); then
            log_ok "已安装 ${installed} 个厂商工具到 ${BIN_DIR}"
            log_info "             例如：lskcan / kcan_monitor / kcanosdiag.sh"
        fi
        if (( skipped > 0 )); then
            log_warn "跳过 ${skipped} 个架构不符的工具"
        fi
        if (( installed == 0 && skipped == 0 )); then
            log_info "驱动目录未附带用户态工具（不影响本项目，本项目用自带 kcanctl）"
        fi
    fi

    # —— 6.2 头文件 ——
    if (( OPT_NO_HEADERS )); then
        log_skip "按 --no-headers 跳过头文件安装"
    else
        local h installed_h=0
        for h in "${VENDOR_HEADERS[@]}"; do
            if [[ -f "${base}/${h}" ]]; then
                run cp -f "${base}/${h}" "${INCLUDE_DIR}/${h}"
                run chmod 644 "${INCLUDE_DIR}/${h}"
                installed_h+=1
            fi
        done
        if (( installed_h > 0 )); then
            log_ok "已安装 ${installed_h} 个头文件到 ${INCLUDE_DIR}"
            log_info "             仅编译厂商原生 API 时需要；本项目用 SocketCAN，不依赖它们"
        else
            log_info "驱动目录未附带头文件（不影响本项目）"
        fi
    fi
}

# =====================================================================
# 步骤 7 —— udev 规则（可选）
# =====================================================================
step7_udev() {
    section "步骤 7 / 10  udev 规则"

    if (( ! OPT_WITH_UDEV )); then
        log_skip "默认不安装 udev 规则（加 --with-udev 启用）"
        log_info "             说明：本项目按 canN 名称使用通道，不依赖 /dev 软链接"
        log_info "             若你希望 /dev/kcan-usb_x6/<adapter>/can<N> 这类稳定路径，再加 --with-udev"
        return 0
    fi

    local base rule=""
    base="$(dirname "${FOUND_KO}")"
    if [[ -f "${base}/udev/45-kcan.rules" ]]; then
        rule="${base}/udev/45-kcan.rules"
    elif [[ -f "${PROJECT_ROOT}/.kunhong_sdk/KH-UCANFD_Linux_SDK-1.4.2/driver/udev/45-kcan.rules" ]]; then
        rule="${PROJECT_ROOT}/.kunhong_sdk/KH-UCANFD_Linux_SDK-1.4.2/driver/udev/45-kcan.rules"
    fi

    if [[ -z "${rule}" ]]; then
        log_warn "未找到厂商 45-kcan.rules，跳过"
        return 0
    fi

    need_root
    log_info "规则来源     : ${rule}"
    run cp -f "${rule}" "${UDEV_RULE_DST}"
    run chmod 644 "${UDEV_RULE_DST}"
    log_ok "udev 规则已安装到 ${UDEV_RULE_DST}"

    if command -v udevadm >/dev/null 2>&1; then
        run udevadm control --reload-rules || log_warn "udevadm reload 失败"
        run udevadm trigger || log_warn "udevadm trigger 失败"
        log_ok "udev 规则已重新加载"
    else
        log_warn "udevadm 不可用，规则将在下次开机或重新插拔设备时生效"
    fi
}

# =====================================================================
# 步骤 8 —— 加载模块
# =====================================================================
unload_module() {
    if ! module_loaded; then
        log_info "kcan 模块未加载，无需卸载"
        return 0
    fi

    # rmmod 前必须 down 掉所有 canN，否则 EBUSY
    local cans
    cans="$(list_can_ifaces)"
    if [[ -n "${cans// /}" ]]; then
        log_info "卸载前需先 down 掉接口: ${cans}"
        local ch
        for ch in ${cans}; do
            run ip link set "${ch}" down || log_warn "down ${ch} 失败"
        done
    fi

    if (( OPT_DRY_RUN )); then
        log_info "[试运行] modprobe -r kcan"
        return 0
    fi

    if modprobe -r kcan 2>/dev/null; then
        log_ok "旧模块已卸载（modprobe -r kcan）"
    elif command -v rmmod >/dev/null 2>&1 && rmmod kcan 2>/dev/null; then
        log_ok "旧模块已卸载（rmmod kcan）"
    else
        log_warn "卸载模块失败（可能有程序仍在使用 CAN 通道）"
        log_info "             可用 lsof / fuser 排查，或重启后生效"
    fi
}

step8_load() {
    section "步骤 8 / 10  加载模块并验证"

    if (( OPT_NO_LOAD )); then
        log_skip "按 --no-load 跳过加载（模块已落盘，下次开机自动加载）"
        return 0
    fi

    if (( ! VALIDATE_OK )) || [[ ! -f "${MOD_INSTALL_PATH}" ]]; then
        log_skip "模块未成功安装，跳过加载"
        return 0
    fi

    # —— 8.1 加载 ——
    local modprobe_bin=""
    modprobe_bin="$(command -v modprobe 2>/dev/null || true)"
    [[ -z "${modprobe_bin}" && -x /sbin/modprobe ]] && modprobe_bin=/sbin/modprobe
    [[ -z "${modprobe_bin}" && -x /usr/sbin/modprobe ]] && modprobe_bin=/usr/sbin/modprobe

    if [[ -z "${modprobe_bin}" ]]; then
        log_fail "modprobe 不可用，无法加载模块（先装 kmod 包）"
        return 0
    fi

    if module_loaded; then
        log_ok "kcan 模块已在运行（跳过加载）"
    else
        need_root
        if (( OPT_DRY_RUN )); then
            log_info "[试运行] ${modprobe_bin} kcan"
            return 0
        fi
        log_info "执行         : ${modprobe_bin} kcan"
        if "${modprobe_bin}" kcan 2>/dev/null; then
            log_ok "modprobe kcan 成功"
        else
            log_fail "modprobe kcan 失败，按报错对照下表处理："
            cat >&2 <<'ERR_EOF'

  invalid module format
      → vermagic 与当前内核不匹配（.ko 内核版本不对）
        执行: modinfo /lib/modules/$(uname -r)/misc/kcan.ko | grep vermagic
        对比: uname -r

  Exec format error
      → .ko 架构不对（拿到 x86-64 版本了）
        执行: file /lib/modules/$(uname -r)/misc/kcan.ko
        期望: ELF 64-bit LSB relocatable, ARM aarch64

  Key was rejected by service
      → Secure Boot 拒绝未签名模块
        执行: mokutil --sb-state
        解决: 关闭 Secure Boot，或给模块签名

  Unknown symbol in module
      → 内核 API 不兼容（内核版本或配置不符）
        执行: dmesg | tail -20

  Module kcan not found in directory /lib/modules/...
      → depmod 未执行成功，或 .ko 未放到位
        执行: ls -l /lib/modules/$(uname -r)/misc/kcan.ko && sudo depmod -a
ERR_EOF
            log_info "详细错误: dmesg | tail -30"
            return 0
        fi
    fi

    # —— 8.2 lsmod 确认 ——
    if command -v lsmod >/dev/null 2>&1; then
        local line
        line="$(lsmod 2>/dev/null | grep -E '^kcan' | head -1 || true)"
        if [[ -n "${line}" ]]; then
            log_ok "lsmod 确认: ${line}"
        else
            log_warn "lsmod 中未见 kcan（可能加载后立即退出了）"
        fi
    fi

    # —— 8.3 等待并列出 CAN 接口 ——
    if ! command -v ip >/dev/null 2>&1; then
        log_warn "ip 命令不可用，无法验证 CAN 接口（装 iproute2）"
        return 0
    fi

    local cans="" i
    for i in 1 2 3 4 5 6 7 8 9 10; do
        cans="$(list_can_ifaces)"
        [[ -n "${cans// /}" ]] && break
        sleep 0.2
    done

    if [[ -n "${cans// /}" ]]; then
        log_ok "检测到 CAN 接口: ${cans}"
        printf '\n'
        local ch state
        for ch in ${cans}; do
            state="$(ip -d link show "${ch}" 2>/dev/null | grep -o 'can state [A-Z_]*' | head -1 || true)"
            printf '    %-8s %s %s\n' "${ch}" "$(ip -br link show "${ch}" 2>/dev/null | awk '{print $2}')" "${state}"
        done
        printf '\n'
        log_info "注意：接口编号随 USB 插入顺序变化，每次插拔都要重新确认"
    else
        log_warn "未检测到 canN 接口"
        log_info "             常见原因："
        log_info "               - USB-CAN 适配器没插好 / 供电不足（用 lsusb 确认能看到设备）"
        log_info "               - 模块加载了但 USB 枚举失败（dmesg | tail -30 查看）"
        log_info "               - 部分固件需要先执行厂商的 kcan_make_devices / kcanosdiag.sh"
    fi
}

# =====================================================================
# 步骤 9 —— 状态总览
# =====================================================================
step9_status() {
    section "步骤 9 / 10  当前状态总览"

    # 模块文件
    if [[ -f "${MOD_INSTALL_PATH}" ]]; then
        log_ok "模块文件     : ${MOD_INSTALL_PATH}"
        log_info "              $(file -b "${MOD_INSTALL_PATH}" 2>/dev/null | cut -d, -f1-2 || echo 未知)"
        local vm
        vm="$(ko_vermagic "${MOD_INSTALL_PATH}")"
        [[ -n "${vm}" ]] && log_info "              vermagic: ${vm}"
    else
        log_warn "模块文件     : 未安装（${MOD_INSTALL_PATH} 不存在）"
    fi

    # 加载状态
    if module_loaded; then
        log_ok "加载状态     : 已加载"
    else
        log_warn "加载状态     : 未加载"
    fi

    # 配置
    [[ -f "${MODPROBE_CONF}" ]] \
        && log_ok "modprobe 配置: ${MODPROBE_CONF}" \
        || log_info "modprobe 配置: 无（使用默认参数）"

    if [[ -f "${MODULES_LOAD_CONF}" ]] && grep -qxF 'kcan' "${MODULES_LOAD_CONF}" 2>/dev/null; then
        log_ok "开机自加载   : 已配置（${MODULES_LOAD_CONF}）"
    else
        log_info "开机自加载   : 未配置"
    fi

    [[ -f "${UDEV_RULE_DST}" ]] \
        && log_ok "udev 规则    : ${UDEV_RULE_DST}" \
        || log_info "udev 规则    : 未安装（本项目不需要）"

    # USB 设备
    if command -v lsusb >/dev/null 2>&1; then
        local n_usb
        n_usb="$(lsusb 2>/dev/null | grep -ci 'kunhong\|peak\|candle\|can' || true)"
        if (( n_usb > 0 )); then
            log_info "USB-CAN 设备 : 检测到 ${n_usb} 个疑似适配器（lsusb）"
        else
            log_info "USB-CAN 设备 : lsusb 未见明显匹配项"
        fi
    fi

    # CAN 接口
    local cans
    cans="$(list_can_ifaces)"
    if [[ -n "${cans// /}" ]]; then
        log_ok "CAN 接口     : ${cans}"
    else
        log_info "CAN 接口     : 无"
    fi
}

# =====================================================================
# 卸载流程
# =====================================================================
do_uninstall() {
    section "卸载 kcan 驱动"
    need_root

    log_warn "即将移除以下内容："
    log_info "    ${MOD_INSTALL_PATH}"
    log_info "    ${MODPROBE_CONF}"
    log_info "    ${MODULES_LOAD_CONF}"
    log_info "    ${UDEV_RULE_DST}"
    log_info "    ${BIN_DIR}/{${VENDOR_TOOLS[*]},kcan-settings}"
    log_info "    ${INCLUDE_DIR}/{${VENDOR_HEADERS[*]}}"
    log_info "    并从内核卸载 kcan 模块"

    if ! confirm "确认继续卸载？"; then
        die "已取消"
    fi

    # 1) 卸载模块（内部会先 down 接口）
    unload_module

    # 2) 删除文件
    run rm -f "${MOD_INSTALL_PATH}"
    run rm -f "${MODPROBE_CONF}"
    run rm -f "${MODULES_LOAD_CONF}"
    run rm -f "${UDEV_RULE_DST}"

    local f
    for f in "${VENDOR_TOOLS[@]}" kcan-settings; do
        [[ -f "${BIN_DIR}/${f}" ]] && run rm -f "${BIN_DIR}/${f}"
    done
    for f in "${VENDOR_HEADERS[@]}"; do
        [[ -f "${INCLUDE_DIR}/${f}" ]] && run rm -f "${INCLUDE_DIR}/${f}"
    done

    # 3) 更新依赖表
    local depmod_bin=""
    depmod_bin="$(command -v depmod 2>/dev/null || true)"
    [[ -z "${depmod_bin}" && -x /sbin/depmod ]] && depmod_bin=/sbin/depmod
    [[ -n "${depmod_bin}" ]] && run "${depmod_bin}" -a "${KREL}" || true

    if (( OPT_DRY_RUN )); then
        log_info "[试运行] 卸载流程演练完毕，未实际修改"
    else
        log_ok "卸载完成"
        log_info "提示：备份文件 *.bak.* 未被删除，如需清理请手动处理："
        log_info "      ls -l ${MOD_INSTALL_DIR}/*.bak.* /etc/modprobe.d/*.bak.* 2>/dev/null"
    fi
}

# =====================================================================
# 后续提示
# =====================================================================
print_next_steps() {
    printf '\n%s—— 后续手动操作（本脚本不会自动执行）——%s\n' "${C_BOLD}" "${C_RESET}"
    cat <<EOF
  1) 确认 CAN 通道编号（【每次插拔都要重新确认】，免 root）:
         ./build/kcanctl list

  2) 手动打开所需通道并设置 1 Mbps（必须逐个手动，不要批量自动开）:
         sudo ./build/kcanctl up can4 1000000

     接口编号随 USB 插入顺序变化，不要假设一定是 can4。

  3) 验证总线健康（berr-counter tx 应为 0）:
         ip -d link show can4 | grep "can state"

  4) 调试用总线监视:
         candump can4

  5) 跑下位机主程序:
         sudo ./build/walk_control_V1

  ⚠ 悬空（没接设备）的通道不要跑真实流量，
     20 秒左右会 OSError: [Errno 105] No buffer space available。

  ⚠ 开机自加载只加载模块，不会 up 通道 —— 这是有意为之。
     通道必须在确认接线后手动打开，避免对错误通道下发 1Mbps 流量。
EOF
}

# =====================================================================
# 汇总
# =====================================================================
print_summary() {
    section "汇总"
    printf '  %s完成%s %d 项    %s警告%s %d 项    %s失败%s %d 项    %s跳过%s %d 项\n' \
        "${C_GREEN}"  "${C_RESET}" "${N_OK}" \
        "${C_YELLOW}" "${C_RESET}" "${N_WARN}" \
        "${C_RED}"    "${C_RESET}" "${N_FAIL}" \
        "${C_YELLOW}" "${C_RESET}" "${N_SKIP}"

    if (( N_FAIL > 0 )); then
        printf '\n%s存在 %d 项失败，请按上面的 [失败] 提示逐条处理。%s\n' "${C_RED}" "${N_FAIL}" "${C_RESET}"
        return 1
    elif (( N_WARN > 0 )); then
        printf '\n%s无致命错误，但有 %d 项警告需要确认。%s\n' "${C_YELLOW}" "${N_WARN}" "${C_RESET}"
        return 0
    else
        printf '\n%s驱动配置完成。%s\n' "${C_GREEN}" "${C_RESET}"
        return 0
    fi
}

# =====================================================================
# 主流程
# =====================================================================
main() {
    parse_args "$@"

    printf '%s\n' "${C_BOLD}=====================================================================${C_RESET}"
    printf '%s  TaiHu_motor_control —— kcan 驱动安装 / 维护%s\n' "${C_BOLD}" "${C_RESET}"
    printf '%s  目标: RK3588 / aarch64 · Ubuntu 20.04 · 内核 %s%s\n' \
        "${C_BOLD}" "${KREL}" "${C_RESET}"
    printf '%s=====================================================================%s\n' "${C_BOLD}" "${C_RESET}"

    if (( OPT_DRY_RUN )); then
        printf '%s[试运行模式]%s 不会修改任何文件\n' "${C_YELLOW}" "${C_RESET}"
    fi

    # —— --uninstall 走独立流程 ——
    if (( OPT_UNINSTALL )); then
        do_uninstall
        print_summary
        return $?
    fi

    # —— --check 只做只读检查 ——
    if (( OPT_CHECK_ONLY )); then
        step1_env
        step2_locate
        step3_validate
        step9_status
        print_summary
        return $?
    fi

    # —— 完整安装流程 ——
    step1_env
    step2_locate
    step3_validate
    step4_install_ko
    step5_autoload
    step6_tools
    step7_udev
    step8_load
    step9_status
    print_next_steps

    print_summary
    return $?
}

main "$@"