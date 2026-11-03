#!/usr/bin/env bash
# =====================================================================
# TaiHu_motor_control —— 嵌入式设备（RK3588 / aarch64）环境安装脚本
#
# 目标环境特征（针对本脚本的默认设计）：
#   - Debian 系发行版（有 apt），但不是 Ubuntu（如 麒麟 / UOS / Armbian）
#   - aarch64 架构，在板上本机编译
#   - 板上不能访问外网（所有依赖需离线安装）
#   - kcan 驱动由厂商提供已编译好的 .ko（板上无内核头文件）
#
# 脚本做的事：
#   阶段 1  环境自检（架构 / 发行版 / root / 互联网）
#   阶段 2  基础构建依赖（gcc g++ cmake make ...，优先用离线 .deb）
#   阶段 3  运行时诊断工具（can-utils / iproute2 / usbutils / python3-serial）
#   阶段 4  Python 工具依赖（openpyxl）
#   阶段 5  编译本项目（cmake + make）
#   阶段 6  安装厂商 kcan 驱动（只装 .ko，不编译）
#   阶段 7  最终验收自检
#
# 用法：
#   sudo ./deploy/install_env.sh                  # 全流程（推荐）
#   sudo ./deploy/install_env.sh --skip-driver    # 跳过 kcan 驱动安装
#   sudo ./deploy/install_env.sh --skip-build     # 只装依赖，不编译项目
#   ./deploy/install_env.sh --check               # 只做自检，不改动系统
#   ./deploy/install_env.sh --help
#
# 重要约束：
#   * 本脚本【绝不】自动 up CAN 通道。开通道请手动执行：
#         sudo ./build/kcanctl up can4 1000000
#     （接口编号随 USB 插入顺序变化，先用 ./build/kcanctl list 确认）
#   * 本脚本不编译内核模块。kcan.ko 必须是厂商为本机内核版本提供的 ARM64 版本。
# =====================================================================

set -euo pipefail

# ---------------------------------------------------------------------
# 全局常量
# ---------------------------------------------------------------------
readonly SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}")"
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

readonly BUILD_DIR="${PROJECT_ROOT}/build"
readonly OFFLINE_DIR="${SCRIPT_DIR}/offline-debs"
readonly DRIVER_SEARCH_PATHS=(
    "${SCRIPT_DIR}/kcan-driver"
    "${PROJECT_ROOT}/.kunhong_sdk"
    "${PROJECT_ROOT}/kcan-driver"
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

# 命令行开关
OPT_SKIP_DEPS=0
OPT_SKIP_BUILD=0
OPT_SKIP_DRIVER=0
OPT_DRY_RUN=0
OPT_CHECK_ONLY=0
OPT_LOAD_DRIVER=0
OPT_ASSUME_YES=0
OPT_KO_PATH=""          # 透传给 install_kcan.sh 的 --ko

# ---------------------------------------------------------------------
# 输出辅助
# ---------------------------------------------------------------------
log_info()  { printf '%s[信息]%s %s\n' "${C_BLUE}"   "${C_RESET}" "$*"; }
log_ok()    { printf '%s[完成]%s %s\n' "${C_GREEN}"  "${C_RESET}" "$*"; N_OK+=1; }
log_warn()  { printf '%s[警告]%s %s\n' "${C_YELLOW}" "${C_RESET}" "$*" >&2; N_WARN+=1; }
log_fail()  { printf '%s[失败]%s %s\n' "${C_RED}"    "${C_RESET}" "$*" >&2; N_FAIL+=1; }
log_skip()  { printf '%s[跳过]%s %s\n' "${C_YELLOW}" "${C_RESET}" "$*"; N_SKIP+=1; }

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

# 需要 root 的写操作统一走这里，出错可定位
need_root() {
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

# ---------------------------------------------------------------------
# 参数解析
# ---------------------------------------------------------------------
usage() {
    cat <<'EOF'
TaiHu_motor_control —— 嵌入式设备（RK3588 / aarch64）环境安装脚本

用法:
  sudo ./deploy/install_env.sh [选项]

选项:
  --skip-deps      跳过阶段 2/3/4（基础依赖、诊断工具、Python 依赖）
  --skip-build     跳过阶段 5（编译本项目）
  --skip-driver    跳过阶段 6（安装厂商 kcan 驱动）
  --load-driver    安装驱动后立即 modprobe kcan（默认不加载）
  --ko <路径>      指定厂商 aarch64 kcan.ko（透传给 deploy/install_kcan.sh）
  --check          只做环境自检，不修改系统（不需要 root）
  --dry-run        打印将要执行的命令，但不实际执行
  -y, --yes        所有确认自动回答 yes
  -h, --help       显示本帮助

离线安装准备:
  板上不能上网时，请在联网的同类 aarch64 机器（或同版本系统）上预先下载 .deb：
      apt-get download gcc g++ make cmake build-essential libc6-dev \
          can-utils iproute2 usbutils python3-serial python3-openpyxl
  把 .deb 全部放进:
      deploy/offline-debs/
  脚本会自动优先使用该目录离线安装，缺失时再尝试联网 apt。

kcan 驱动放置位置（脚本会自动按顺序查找）:
      deploy/kcan-driver/
      .kunhong_sdk/
      kcan-driver/
  目录内需要包含: kcan.ko
  可选附带: kcan.conf  lskcan  kcan_monitor  kcan-settings  kcanosdiag.sh  kcan_make_devices

开 CAN 通道（脚本不会自动执行，请手动）:
      ./build/kcanctl list
      sudo ./build/kcanctl up can4 1000000
EOF
}

parse_args() {
    while (($#)); do
        case "$1" in
            --skip-deps)    OPT_SKIP_DEPS=1 ;;
            --skip-build)   OPT_SKIP_BUILD=1 ;;
            --skip-driver)  OPT_SKIP_DRIVER=1 ;;
            --load-driver)  OPT_LOAD_DRIVER=1 ;;
            --ko)           [[ $# -ge 2 ]] || die "--ko 需要一个参数"
                            OPT_KO_PATH="$2"; shift ;;
            --ko=*)         OPT_KO_PATH="${1#*=}" ;;
            --check)        OPT_CHECK_ONLY=1 ;;
            --dry-run)      OPT_DRY_RUN=1 ;;
            -y|--yes)       OPT_ASSUME_YES=1 ;;
            -h|--help)      usage; exit 0 ;;
            *)              usage; die "未知参数: $1" ;;
        esac
        shift
    done
}

# =====================================================================
# 阶段 1 —— 环境自检
# =====================================================================
phase1_selfcheck() {
    section "阶段 1 / 7  环境自检"

    # —— 1.1 架构 ——
    local arch
    arch="$(uname -m)"
    log_info "机器架构       : ${arch}"
    case "${arch}" in
        aarch64|arm64)
            log_ok "架构为 aarch64，符合 RK3588 目标" ;;
        *)
            log_warn "架构为 ${arch}，非 aarch64。本脚本按 ARM 板卡设计，请确认这是否是你想要的机器" ;;
    esac

    # —— 1.2 内核 ——
    local krel
    krel="$(uname -r)"
    log_info "内核版本       : ${krel}"

    # —— 1.3 发行版 ——
    local distro="" distro_like="" ver=""
    if [[ -r /etc/os-release ]]; then
        # shellcheck disable=SC1091
        distro="$(. /etc/os-release && printf '%s' "${NAME:-unknown}")"
        # shellcheck disable=SC1091
        distro_like="$(. /etc/os-release && printf '%s' "${ID_LIKE:-}")"
        # shellcheck disable=SC1091
        ver="$(. /etc/os-release && printf '%s' "${VERSION_ID:-}")"
    fi
    log_info "发行版         : ${distro:-未知} ${ver:-}"

    # —— 1.4 包管理器 ——
    if command -v apt-get >/dev/null 2>&1; then
        log_ok "包管理器       : apt-get 可用（Debian 系）"
    else
        log_fail "包管理器       : 未找到 apt-get。本脚本要求 Debian 系发行版（apt）"
        log_info "                若为 RPM 系（dnf/yum）或无包管理器（Buildroot/Yocto），"
        log_info "                请改用离线手动安装依赖，或告知以便另写脚本"
    fi

    # —— 1.5 权限 ——
    if [[ "${EUID}" -eq 0 ]]; then
        log_ok "运行权限       : root"
    else
        log_warn "运行权限       : 普通用户（uid=${EUID}）。阶段 2/3/4/6 需要 sudo"
    fi

    # —— 1.6 本机是否可编译 ——
    local tool
    for tool in gcc g++ cmake make; do
        if command -v "${tool}" >/dev/null 2>&1; then
            log_ok "已安装         : ${tool} ($("${tool}" --version 2>/dev/null | head -1))"
        else
            log_warn "缺失           : ${tool}（阶段 2 会尝试安装）"
        fi
    done

    # —— 1.7 内核头文件（仅提示，不影响流程） ——
    if [[ -d "/lib/modules/${krel}/build" ]]; then
        log_info "内核头文件     : /lib/modules/${krel}/build 存在（可自行编译内核模块）"
    else
        log_warn "内核头文件     : /lib/modules/${krel}/build 不存在 → 本脚本不编译 kcan 驱动，"
        log_info "                需使用厂商提供的、与本内核版本匹配的 ARM64 kcan.ko"
    fi

    # —— 1.8 联网状态 ——
    local online=0
    if command -v apt-get >/dev/null 2>&1; then
        if timeout 8 apt-get -s check >/dev/null 2>&1; then
            online=1
        fi
    fi
    if (( online )); then
        log_ok "网络 / apt 源  : 可用（会尝试在线安装缺失依赖）"
    else
        log_warn "网络 / apt 源  : 不可用或无法校验 → 依赖必须走离线 .deb"
        log_info "                请把 .deb 放进: ${OFFLINE_DIR}/"
    fi

    # —— 1.9 离线包目录 ——
    if [[ -d "${OFFLINE_DIR}" ]]; then
        local n_deb
        n_deb="$(find "${OFFLINE_DIR}" -maxdepth 1 -name '*.deb' 2>/dev/null | wc -l)"
        if (( n_deb > 0 )); then
            log_ok "离线包目录     : ${OFFLINE_DIR}（${n_deb} 个 .deb）"
        else
            log_warn "离线包目录     : ${OFFLINE_DIR} 存在但没有 .deb 文件"
        fi
    else
        log_info "离线包目录     : ${OFFLINE_DIR} 不存在（需要离线安装时请创建并放入 .deb）"
    fi

    # —— 1.10 已有 CAN 接口 ——
    if command -v ip >/dev/null 2>&1; then
        local cans
        cans="$(ip -o link show 2>/dev/null | awk -F': ' '/: can[0-9]+/{print $2}' | tr '\n' ' ' || true)"
        if [[ -n "${cans}" ]]; then
            log_ok "当前 CAN 接口  : ${cans}"
        else
            log_warn "当前 CAN 接口  : 无（kcan 驱动未加载或未插 USB-CAN）"
        fi
    else
        log_warn "iproute2       : 未找到 ip 命令（阶段 3 会尝试安装）"
    fi

    # —— 1.11 kcan 模块状态 ——
    if command -v lsmod >/dev/null 2>&1; then
        if lsmod 2>/dev/null | grep -q '^kcan'; then
            log_ok "kcan 模块      : 已加载"
        else
            log_info "kcan 模块      : 未加载"
        fi
    fi

    # —— 1.12 厂商驱动是否就位 ——
    local found_ko
    found_ko="$(find_driver_ko 2>/dev/null || true)"
    if [[ -n "${found_ko}" ]]; then
        log_ok "厂商 kcan.ko   : ${found_ko}"
        log_info "                $(file -b "${found_ko}" 2>/dev/null | cut -d, -f1-2)"
        if ! file -b "${found_ko}" 2>/dev/null | grep -q 'aarch64\|ARM aarch64'; then
            log_warn "                该 .ko 看起来不是 aarch64 架构！在板上加载会失败"
        fi
    else
        log_warn "厂商 kcan.ko   : 未找到（阶段 6 会跳过驱动安装）"
        local p
        for p in "${DRIVER_SEARCH_PATHS[@]}"; do
            log_info "                已查找: ${p}"
        done
    fi
}

# =====================================================================
# 阶段 2 —— 基础构建依赖
# =====================================================================
APT_BASE_PKGS=(
    build-essential   # gcc / g++ / make / libc6-dev
    cmake             # 构建系统（需要 >= 3.16）
    pkg-config
)

phase2_base_deps() {
    section "阶段 2 / 7  基础构建依赖"

    if (( OPT_SKIP_DEPS )); then
        log_skip "按 --skip-deps 跳过"
        return 0
    fi

    # —— 2.1 已满足则跳过 ——
    local all_ok=1 tool
    for tool in gcc g++ cmake make; do
        command -v "${tool}" >/dev/null 2>&1 || all_ok=0
    done
    if (( all_ok )); then
        log_ok "gcc / g++ / cmake / make 均已存在，无需安装"
    else
        install_packages "${APT_BASE_PKGS[@]}"
    fi

    # —— 2.2 cmake 版本必须 >= 3.16（CMakeLists.txt 要求） ——
    if command -v cmake >/dev/null 2>&1; then
        local cm_ver cm_major cm_minor
        cm_ver="$(cmake --version | head -1 | awk '{print $3}')"
        cm_major="${cm_ver%%.*}"
        cm_minor="$(printf '%s' "${cm_ver}" | cut -d. -f2)"
        log_info "cmake 版本     : ${cm_ver}"
        if (( cm_major > 3 || (cm_major == 3 && cm_minor >= 16) )); then
            log_ok "cmake 版本满足要求（>= 3.16）"
        else
            log_fail "cmake 版本过低（${cm_ver}），项目要求 >= 3.16"
            log_info "        离线环境请自行获取新版 cmake 的 aarch64 包"
        fi
    else
        log_fail "cmake 仍不可用"
    fi

    # —— 2.3 g++ 必须支持 C++17 ——
    if command -v g++ >/dev/null 2>&1; then
        local gxx_ver test_src test_bin
        gxx_ver="$(g++ -dumpversion)"
        log_info "g++ 版本       : ${gxx_ver}"
        test_src="$(mktemp /tmp/taihu_cxx17_XXXXXX.cpp)"
        test_bin="$(mktemp /tmp/taihu_cxx17_XXXXXX)"
        printf '#include <optional>\n#include <string_view>\nint main(){std::optional<int> o{1};return o.value()-1;}\n' > "${test_src}"
        if g++ -std=c++17 -O0 "${test_src}" -o "${test_bin}" 2>/dev/null; then
            log_ok "g++ 支持 C++17"
        else
            log_fail "g++ 无法编译 C++17 代码，需要 g++ >= 7"
        fi
        rm -f "${test_src}" "${test_bin}"
    fi

    # —— 2.4 线程库（项目大量使用 std::thread） ——
    log_info "说明           : 项目使用 std::thread，glibc >= 2.34 时 pthread 已并入 libc，"
    log_info "                无需额外 -lpthread；CMakeLists.txt 未显式链接 Threads"
}

# ---------------------------------------------------------------------
# 通用：安装软件包（优先离线 .deb，其次在线 apt）
# ---------------------------------------------------------------------
install_packages() {
    local pkgs=("$@")

    # 优先：离线 .deb
    if [[ -d "${OFFLINE_DIR}" ]] && compgen -G "${OFFLINE_DIR}/*.deb" >/dev/null; then
        log_info "发现离线 .deb，优先离线安装"
        need_root
        run dpkg -i "${OFFLINE_DIR}"/*.deb || {
            log_warn "dpkg -i 返回非零（可能存在依赖顺序问题），尝试 apt-get -f install 修复"
            run apt-get -f install -y || log_fail "apt-get -f install 失败"
        }
        # 离线装完仍缺依赖时，若有网络再补
        if command -v apt-get >/dev/null 2>&1; then
            run apt-get -f install -y || true
        fi
    else
        log_info "无离线 .deb，尝试在线 apt-get 安装: ${pkgs[*]}"
        need_root
        run apt-get update || log_warn "apt-get update 失败（离线环境正常）"
        run apt-get install -y --no-install-recommends "${pkgs[@]}" || {
            log_fail "apt-get 安装失败: ${pkgs[*]}"
            log_info "        离线环境请把对应 .deb 放入 ${OFFLINE_DIR}/"
        }
    fi
}

# =====================================================================
# 阶段 3 —— 运行时诊断工具
# =====================================================================
# 说明：
#   * can-utils  : candump / cansend / canbusload，调试 CAN 总线必需
#   * iproute2   : kcanctl 通过 system("ip link ...") 操作接口，运行时强依赖
#   * usbutils   : lsusb，排查 USB-CAN 是否被识别
#   * python3-serial : 若要走 USB 转 485 读 MPT 编码器，需要 pyserial 相关工具
APT_RUNTIME_PKGS=(
    can-utils
    iproute2
    usbutils
)

phase3_runtime_tools() {
    section "阶段 3 / 7  运行时诊断工具"

    if (( OPT_SKIP_DEPS )); then
        log_skip "按 --skip-deps 跳过"
        return 0
    fi

    local tool missing=()
    # 逐个检查，只装缺的
    command -v ip        >/dev/null 2>&1 || missing+=(iproute2)
    command -v candump   >/dev/null 2>&1 || missing+=(can-utils)
    command -v lsusb     >/dev/null 2>&1 || missing+=(usbutils)

    if ((${#missing[@]} == 0)); then
        log_ok "ip / candump / lsusb 均已存在"
    else
        log_info "缺失工具对应的包: ${missing[*]}"
        install_packages "${missing[@]}"
    fi

    # 复核
    for tool in ip candump lsusb; do
        if command -v "${tool}" >/dev/null 2>&1; then
            log_ok "${tool} 可用"
        else
            log_warn "${tool} 仍不可用（不影响编译，但影响现场调试）"
        fi
    done

    # 可选：ip 命令是 kcanctl 的运行时硬依赖，单独强调
    if ! command -v ip >/dev/null 2>&1; then
        log_fail "缺少 ip 命令！kcanctl 依赖它设置波特率/开关通道，必须解决"
    fi
}

# =====================================================================
# 阶段 4 —— Python 工具依赖
# =====================================================================
# 说明（已核对全部 .py 的实际 import）：
#   * openpyxl   : tools/up_walk_control_V1.py、convert_yppy_to_step_make.py 读写 xlsx —— 必需
#   * matplotlib : tools/plot_motor_log.py 绘图 —— 可选
#   * reportlab / fontTools : 仅 tools/gen_protocol_pdf.py 重新生成 PDF 时需要 —— 可选
#   * pyserial   : 若要用 Python 直接读 MPT 编码器 —— 可选（C++ 工具用 termios，不需要）

phase4_python_deps() {
    section "阶段 4 / 7  Python 工具依赖"

    if (( OPT_SKIP_DEPS )); then
        log_skip "按 --skip-deps 跳过"
        return 0
    fi

    # —— 4.1 python3 本体 ——
    if command -v python3 >/dev/null 2>&1; then
        log_ok "python3        : $(python3 --version 2>&1)"
    else
        log_warn "python3 未安装，尝试安装"
        install_packages python3
    fi

    # —— 4.2 openpyxl（虚拟上位机唯一硬依赖） ——
    if python3 -c 'import openpyxl' >/dev/null 2>&1; then
        log_ok "openpyxl       : 已安装（$(python3 -c 'import openpyxl;print(openpyxl.__version__)' 2>/dev/null)）"
    else
        log_warn "openpyxl 缺失（上位机读写 xlsx 必需）"
        # 优先发行版包（离线友好）
        install_packages python3-openpyxl
        if python3 -c 'import openpyxl' >/dev/null 2>&1; then
            log_ok "openpyxl       : 已通过发行版包安装"
        else
            log_warn "发行版包不可用，尝试 pip3"
            if command -v pip3 >/dev/null 2>&1; then
                need_root
                run pip3 install --break-system-packages openpyxl \
                    || run pip3 install openpyxl \
                    || log_fail "pip3 安装 openpyxl 失败（离线环境请手动传 wheel）"
            else
                log_fail "pip3 也不可用，无法安装 openpyxl"
            fi
        fi
    fi

    # —— 4.3 可选依赖：只报告，不强制安装 ——
    local mod
    for spec in "matplotlib:绘图（plot_motor_log.py）" \
                "reportlab:PDF 生成（gen_protocol_pdf.py）" \
                "fontTools:PDF 字体子集化（gen_protocol_pdf.py）" \
                "serial:pyserial（Python 读 MPT 编码器）"; do
        mod="${spec%%:*}"
        if python3 -c "import ${mod}" >/dev/null 2>&1; then
            log_ok "可选 ${mod} : 已安装"
        else
            log_info "可选 ${mod} : 未安装 —— ${spec#*:}"
        fi
    done
}

# =====================================================================
# 阶段 5 —— 编译本项目
# =====================================================================
phase5_build() {
    section "阶段 5 / 7  编译本项目"

    if (( OPT_SKIP_BUILD )); then
        log_skip "按 --skip-build 跳过"
        return 0
    fi

    if ! command -v cmake >/dev/null 2>&1; then
        log_fail "cmake 不可用，无法编译（先解决阶段 2）"
        return 0
    fi
    if ! command -v make >/dev/null 2>&1; then
        log_fail "make 不可用，无法编译（先解决阶段 2）"
        return 0
    fi

    log_info "项目根目录     : ${PROJECT_ROOT}"
    log_info "构建目录       : ${BUILD_DIR}"

    # —— 5.1 configure ——
    if (( OPT_DRY_RUN )); then
        log_info "[试运行] cmake -S ${PROJECT_ROOT} -B ${BUILD_DIR} -DCMAKE_BUILD_TYPE=Release"
    else
        mkdir -p "${BUILD_DIR}"
        if cmake -S "${PROJECT_ROOT}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release; then
            log_ok "cmake 配置成功"
        else
            log_fail "cmake 配置失败，请检查上面的报错"
            return 0
        fi
    fi

    # —— 5.2 编译 ——
    local jobs
    jobs="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"
    log_info "并行编译       : -j${jobs}"

    if (( OPT_DRY_RUN )); then
        log_info "[试运行] cmake --build ${BUILD_DIR} -j${jobs}"
    else
        if cmake --build "${BUILD_DIR}" -j"${jobs}"; then
            log_ok "编译成功"
        else
            log_fail "编译失败，请检查上面的报错"
            return 0
        fi
    fi

    # —— 5.3 检查产物（试运行时跳过，避免因旧 build 目录误报） ——
    if (( OPT_DRY_RUN )); then
        log_info "[试运行] 跳过产物校验"
        return 0
    fi

    local expected=(
        taihu_singleleg_step
        taihu_singleleg_step_xCAN
        taihu_single_motorctl
        taihu_single_motorctl_xCAN
        taihu_keyboardctl
        taihu_pd_leg
        walk_control_V1
        taihu_motor_tools
        taihu_torque_sensor
        taihu_mpt_encoder
        taihu_encoder_can_bridge
        kcanctl
    )
    local missing=() name
    for name in "${expected[@]}"; do
        [[ -x "${BUILD_DIR}/${name}" ]] || missing+=("${name}")
    done

    if ((${#missing[@]} == 0)); then
        log_ok "全部 ${#expected[@]} 个预期目标均已生成"
    else
        log_warn "以下目标未生成: ${missing[*]}"
    fi

    # Qt5 UI 是可选目标
    if [[ -x "${BUILD_DIR}/taihu_keyboardctl_ui" ]]; then
        log_ok "taihu_keyboardctl_ui 已生成（Qt5 可用）"
    else
        log_info "taihu_keyboardctl_ui 未生成 —— Qt5 Widgets 未安装（这是预期行为，CMakeLists 会自动跳过）"
    fi
}

# =====================================================================
# 阶段 6 —— 安装厂商 kcan 驱动（只装 .ko，不编译）
# =====================================================================
# 背景（已核对厂商 SDK 的 driver/Makefile）：
#   * 模块安装位置 : /lib/modules/$(uname -r)/misc/kcan.ko
#   * modprobe 配置 : /etc/modprobe.d/kcan.conf
#   * 用户态工具    : /usr/local/bin/{lskcan,kcan_monitor,kcan-settings,kcanosdiag.sh,kcan_make_devices}
#   * 头文件        : /usr/include/{kcan.h,kcanfd.h}
#   * udev 规则     : /etc/udev/rules.d/45-kcan.rules（可选）
#
#   本脚本【不】调用厂商 make install，因为那需要内核头文件编译；
#   这里等价地把厂商提供的 ARM64 .ko 与脚本工具装到位。

find_driver_ko() {
    local p
    for p in "${DRIVER_SEARCH_PATHS[@]}"; do
        [[ -d "${p}" ]] || continue
        # 直接放目录里，或放在 driver/ 子目录里
        if [[ -f "${p}/kcan.ko" ]]; then
            printf '%s\n' "${p}/kcan.ko"
            return 0
        fi
        if [[ -f "${p}/driver/kcan.ko" ]]; then
            printf '%s\n' "${p}/driver/kcan.ko"
            return 0
        fi
        # 兜底：递归找（厂商 SDK 常带 x86 预编译件，需靠架构判断）
        local f
        f="$(find "${p}" -name 'kcan.ko' -type f 2>/dev/null | head -1 || true)"
        if [[ -n "${f}" ]]; then
            printf '%s\n' "${f}"
            return 0
        fi
    done
    return 1
}

phase6_driver() {
    section "阶段 6 / 7  安装厂商 kcan 驱动"

    if (( OPT_SKIP_DRIVER )); then
        log_skip "按 --skip-driver 跳过"
        return 0
    fi

    # —— 6.0 转交专用脚本 ——
    # kcan 驱动的安装/卸载/开机自加载已独立成 deploy/install_kcan.sh，
    # 那里有更严格的架构与 vermagic 校验、可回滚、可重复执行。
    # 这里只负责转交，避免两处逻辑各自演化后不一致。
    local kcan_script="${SCRIPT_DIR}/install_kcan.sh"
    if [[ -x "${kcan_script}" ]]; then
        log_info "kcan 驱动安装已独立为专用脚本，转交执行"

        local -a kargs=()
        (( OPT_LOAD_DRIVER )) || kargs+=(--no-load)
        (( OPT_DRY_RUN ))     && kargs+=(--dry-run)
        (( OPT_ASSUME_YES ))  && kargs+=(-y)
        # 透传驱动路径（若用户用过 --ko，虽然 install_env.sh 暂无此选项）
        [[ -n "${OPT_KO_PATH:-}" ]] && kargs+=(--ko "${OPT_KO_PATH}")

        if (( ${#kargs[@]} > 0 )); then
            log_info "转交参数     : ${kargs[*]}"
        fi

        if (( OPT_DRY_RUN )); then
            log_info "[试运行] ${kcan_script} ${kargs[*]:-}"
            return 0
        fi

        if "${kcan_script}" "${kargs[@]}"; then
            log_ok "驱动安装脚本执行完成（详见上方输出）"
        else
            log_warn "驱动安装脚本返回非零，详见上方输出"
        fi
        return 0
    fi

    log_warn "未找到 ${kcan_script}，回退到脚本内置的简化驱动安装逻辑"
    phase6_driver_legacy
}

# ---------------------------------------------------------------------
# 阶段 6 的旧实现（仅在 install_kcan.sh 缺失时作为兜底）
# ---------------------------------------------------------------------
phase6_driver_legacy() {

    local ko
    if ! ko="$(find_driver_ko)"; then
        log_warn "未找到 kcan.ko，跳过驱动安装"
        log_info "请把厂商提供的 ARM64 kcan.ko 放到以下任一位置："
        local p
        for p in "${DRIVER_SEARCH_PATHS[@]}"; do
            log_info "    ${p}/kcan.ko"
        done
        log_info "然后重新运行: sudo ./deploy/${SCRIPT_NAME} --skip-build"
        return 0
    fi

    log_info "驱动文件       : ${ko}"

    # —— 6.1 架构校验：这是最关键的一步 ——
    local fdesc
    fdesc="$(file -b "${ko}" 2>/dev/null || echo '未知')"
    log_info "文件类型       : $(printf '%s' "${fdesc}" | cut -d, -f1-2)"

    if ! printf '%s' "${fdesc}" | grep -qi 'aarch64'; then
        log_fail "该 kcan.ko 不是 aarch64 架构，无法在本机加载！"
        log_info "        厂商 SDK 里自带的 kcan.ko 常是 x86-64 预编译件，"
        log_info "        必须向厂商索取与本机内核版本（$(uname -r)）匹配的 ARM64 版本。"
        return 0
    fi
    log_ok "架构校验通过（aarch64）"

    # —— 6.2 内核版本匹配（best effort） ——
    if command -v modinfo >/dev/null 2>&1; then
        local vermagic
        vermagic="$(modinfo -F vermagic "${ko}" 2>/dev/null || true)"
        if [[ -n "${vermagic}" ]]; then
            log_info "模块 vermagic : ${vermagic}"
            if printf '%s' "${vermagic}" | grep -q "$(uname -r)"; then
                log_ok "内核版本匹配（$(uname -r)）"
            else
                log_warn "内核版本不匹配！本机 $(uname -r)，模块要求 ${vermagic%% *}"
                log_info "        强行加载会报 \"invalid module format\"，请联系厂商换版本"
            fi
        fi
    else
        log_info "modinfo 不可用（kmod 未装），跳过 vermagic 校验"
    fi

    # —— 6.3 安装 .ko ——
    need_root
    local krel dst_dir dst_ko
    krel="$(uname -r)"
    dst_dir="/lib/modules/${krel}/misc"
    dst_ko="${dst_dir}/kcan.ko"

    log_info "安装目标       : ${dst_ko}"
    run mkdir -p "${dst_dir}"
    run cp -f "${ko}" "${dst_ko}"
    run chmod 644 "${dst_ko}"

    if (( OPT_DRY_RUN )); then
        log_info "[试运行] depmod -a ${krel}"
    else
        if command -v depmod >/dev/null 2>&1; then
            if depmod -a "${krel}"; then
                log_ok "depmod 完成"
            else
                log_warn "depmod 失败（不影响 modprobe 直接指定路径加载）"
            fi
        else
            log_warn "depmod 不可用，跳过模块依赖表更新"
        fi
    fi

    # —— 6.4 安装 modprobe 配置（厂商 kcan.conf，缺失则生成最小配置） ——
    local src_conf=""
    local base
    base="$(dirname "${ko}")"
    if [[ -f "${base}/kcan.conf" ]]; then
        src_conf="${base}/kcan.conf"
    fi

    run mkdir -p /etc/modprobe.d
    if [[ -n "${src_conf}" ]]; then
        log_info "modprobe 配置  : 来自 ${src_conf}"
        run cp -f "${src_conf}" /etc/modprobe.d/kcan.conf
    else
        log_info "modprobe 配置  : 未找到厂商 kcan.conf，写入最小配置"
        if (( OPT_DRY_RUN )); then
            log_info "[试运行] 写入 /etc/modprobe.d/kcan.conf"
        else
            cat > /etc/modprobe.d/kcan.conf <<'KCONF_EOF'
# kcan 驱动最小配置（由 deploy/install_env.sh 生成）
# 完整选项见厂商 SDK 的 driver/kcan.conf
# options kcan bitrate=1000000
# options kcan rxqsize=
# options kcan txqsize=
KCONF_EOF
        fi
    fi

    # —— 6.5 安装厂商用户态工具（若随 .ko 一起提供） ——
    local tool
    local tools_found=0
    for tool in lskcan kcan_monitor kcan_make_devices kcanosdiag.sh; do
        if [[ -f "${base}/${tool}" ]]; then
            run cp -f "${base}/${tool}" "/usr/local/bin/${tool}"
            run chmod 755 "/usr/local/bin/${tool}"
            tools_found+=1
        fi
    done
    if [[ -f "${base}/kcan-settings" ]]; then
        # kcan-settings 是 ELF，架构可能不对，先校验
        if file -b "${base}/kcan-settings" 2>/dev/null | grep -qi aarch64; then
            run cp -f "${base}/kcan-settings" /usr/local/bin/kcan-settings
            run chmod 755 /usr/local/bin/kcan-settings
            tools_found+=1
        else
            log_warn "kcan-settings 不是 aarch64 架构，跳过安装（也不常用）"
        fi
    fi
    if (( tools_found > 0 )); then
        log_ok "已安装 ${tools_found} 个厂商用户态工具到 /usr/local/bin"
    else
        log_info "未附带厂商用户态工具（不影响本项目，我们用自己的 kcanctl）"
    fi

    # —— 6.6 udev 规则（可选，不强制） ——
    if [[ -f "${base}/udev/45-kcan.rules" ]]; then
        log_info "发现 udev 规则: ${base}/udev/45-kcan.rules"
        log_info "        未自动安装（本项目按手动命名方式使用 canN）"
        log_info "        如需安装: sudo cp ${base}/udev/45-kcan.rules /etc/udev/rules.d/"
    fi

    # —— 6.7 加载模块（默认不加载，需 --load-driver） ——
    if (( OPT_LOAD_DRIVER )); then
        log_info "按 --load-driver 加载模块"
        if (( OPT_DRY_RUN )); then
            log_info "[试运行] modprobe kcan"
        else
            if modprobe kcan; then
                log_ok "modprobe kcan 成功"
            else
                log_fail "modprobe kcan 失败，常见原因："
                log_info "        - vermagic 不匹配（内核版本不对）"
                log_info "        - Secure Boot 拒绝未签名模块（Key was rejected by service）"
                log_info "        - .ko 架构不符"
            fi
        fi
    else
        log_info "默认不加载模块。手动加载请执行: sudo modprobe kcan"
    fi
}

# =====================================================================
# 阶段 7 —— 最终验收
# =====================================================================
phase7_verify() {
    section "阶段 7 / 7  最终验收"

    if (( OPT_DRY_RUN )); then
        log_info "[试运行] 跳过验收检查（未实际编译）"
        print_next_steps
        return 0
    fi

    # —— 7.1 编译产物 ——
    if [[ -x "${BUILD_DIR}/kcanctl" ]]; then
        log_ok "kcanctl 可执行（CAN 通道控制工具）"
        log_info "        当前接口列表（免 root 可读）:"
        "${BUILD_DIR}/kcanctl" list 2>/dev/null | sed 's/^/            /' || log_info "            (无 CAN 接口)"
    else
        log_warn "kcanctl 未生成（可能跳过了编译）"
    fi

    if [[ -x "${BUILD_DIR}/walk_control_V1" ]]; then
        log_ok "walk_control_V1 可执行（下位机主程序）"
    else
        log_warn "walk_control_V1 未生成"
    fi

    # —— 7.2 上位机脚本依赖 ——
    if python3 -c 'import openpyxl' >/dev/null 2>&1; then
        log_ok "虚拟上位机依赖就绪（openpyxl）"
    else
        log_warn "openpyxl 未就绪，tools/up_walk_control_V1.py 无法读写 xlsx"
    fi

    # —— 7.3 CAN 接口 ——
    if command -v ip >/dev/null 2>&1; then
        local cans
        cans="$(ip -o link show 2>/dev/null | awk -F': ' '/: can[0-9]+/{print $2}' | tr '\n' ' ' || true)"
        if [[ -n "${cans}" ]]; then
            log_ok "CAN 接口已就绪: ${cans}"
            # 显示每个接口的状态（是否 UP + 是否健康）
            local ch
            for ch in ${cans}; do
                local state
                state="$(ip -d link show "${ch}" 2>/dev/null | grep -o 'can state [A-Z-]*' | head -1 || true)"
                log_info "        ${ch}: ${state:-未知}"
            done
            log_warn "注意：接口编号随 USB 插入顺序变化，请用 kcanctl list 确认后再开通道"
        else
            log_info "CAN 接口: 无。若已插 USB-CAN，请执行 sudo modprobe kcan 后再看"
        fi
    fi

    # —— 7.4 后续步骤提示 ——
    print_next_steps
}

print_next_steps() {
    printf '\n%s—— 后续手动操作（脚本不会自动执行）——%s\n' "${C_BOLD}" "${C_RESET}"
    cat <<EOF
  1) 加载驱动（若尚未加载）:
         sudo modprobe kcan

  2) 确认 CAN 通道编号（免 root）:
         ./build/kcanctl list

  3) 手动打开所需通道并设置 1 Mbps（【必须】逐个手动，不要批量自动开）:
         sudo ./build/kcanctl up can4 1000000

  4) 验证总线健康（berr-counter tx 应为 0）:
         ip -d link show can4 | grep "can state"

  5) 跑下位机主程序:
         sudo ./build/walk_control_V1

  6) 跑虚拟上位机（另一个终端）:
         python3 tools/up_walk_control_V1.py --dry-run --xlsx step_position/step_make_V2.xlsx

  7) 调试用总线监视:
         candump can4
EOF
}

# =====================================================================
# 主流程
# =====================================================================
main() {
    parse_args "$@"

    printf '%s\n' "${C_BOLD}=====================================================================${C_RESET}"
    printf '%s  TaiHu_motor_control —— 嵌入式环境安装%s\n' "${C_BOLD}" "${C_RESET}"
    printf '%s  RK3588 / aarch64 · Debian 系 · 离线优先 · 本机编译%s\n' "${C_BOLD}" "${C_RESET}"
    printf '%s=====================================================================%s\n' "${C_BOLD}" "${C_RESET}"

    if (( OPT_DRY_RUN )); then
        printf '%s[试运行模式]%s 不会修改任何文件\n' "${C_YELLOW}" "${C_RESET}"
    fi

    # --check：只做自检
    if (( OPT_CHECK_ONLY )); then
        phase1_selfcheck
        print_summary
        return 0
    fi

    phase1_selfcheck
    phase2_base_deps
    phase3_runtime_tools
    phase4_python_deps
    phase5_build
    phase6_driver
    phase7_verify

    print_summary
}

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
        printf '\n%s环境配置完成。%s\n' "${C_GREEN}" "${C_RESET}"
        return 0
    fi
}

main "$@"