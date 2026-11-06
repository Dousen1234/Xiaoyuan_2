#!/usr/bin/env bash
# =============================================================================
# run_ui_sudo.sh —— 以 sudo 启动 3588 图形控制界面
#
# 为什么要这个脚本？
#   界面里「配置 CAN 接口」要做
#       ip link set canX down / type can bitrate N / up
#   这些操作必须是 root。如果每次点按钮都弹密码框，体验极差。
#   所以设计成：**用 sudo 启动整个界面**，之后界面内所有操作都不再需要密码。
#
# 为什么要处理 X 授权？
#   root 身份默认拿不到当前桌面用户的 X 授权文件：
#       /run/user/1000/gdm/Xauthority   权限是 -rwx------（只有 rk 能读）
#   直接 `sudo ./x3588_ui` 会报
#       qt.qpa.xcb: could not connect to display :1
#   本脚本会自动把 XAUTHORITY / DISPLAY / HOME 设到桌面用户的那一套上，
#   所以 Qt 能以 root 身份画到当前桌面上。
#
# 用法：
#   sudo ./run_ui_sudo.sh                   # 正常启动
#   sudo ./run_ui_sudo.sh --can can_mcu     # 透传参数给 x3588_ui
#   sudo ./run_ui_sudo.sh --help            # 看 x3588_ui 的参数
#
# 安全说明：
#   · 本脚本只做环境变量设置 + exec，不修改系统任何配置。
#   · 界面程序本身不会主动让电机运动，一切运动都由你点按钮触发。
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# 0. 定位自己，切到脚本所在目录（找 build/x3588_ui 需要）
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ---------------------------------------------------------------------------
# 1. 必须 root 运行
# ---------------------------------------------------------------------------
if [[ "$(id -u)" -ne 0 ]]; then
    cat >&2 <<'EOF'
错误：本脚本需要 root 权限。

请这样运行：

    sudo ./run_ui_sudo.sh

EOF
    exit 1
fi

# ---------------------------------------------------------------------------
# 2. 找出「真正的桌面用户」（不是 root）
#    sudo 会带来 SUDO_UID / SUDO_USER；直接 root 登录时它们为空。
# ---------------------------------------------------------------------------
TARGET_UID="${SUDO_UID:-1000}"
TARGET_USER="${SUDO_USER:-$(id -un -- "$TARGET_UID" 2>/dev/null || echo rk)}"
TARGET_HOME="$(getent passwd "$TARGET_UID" 2>/dev/null | cut -d: -f6)"
[[ -n "${TARGET_HOME:-}" ]] || TARGET_HOME="/home/$TARGET_USER"

echo "== 3588 控制界面启动器 =="
echo "   桌面用户 : $TARGET_USER (uid=$TARGET_UID)"
echo "   用户主目录: $TARGET_HOME"

# ---------------------------------------------------------------------------
# 3. 处理 X 授权
#    依次尝试：
#      a) /run/user/<uid>/gdm/Xauthority    （GDM + Xorg，3588 的实际情况）
#      b) /run/user/<uid>/Xauthority        （某些 display manager）
#      c) $HOME/.Xauthority                 （xinit / startx 场景）
#    找到可用且可读的就拷一份到 /tmp，让 root 能读。
# ---------------------------------------------------------------------------
XAUTH_SRC=""
for cand in \
    "/run/user/$TARGET_UID/gdm/Xauthority" \
    "/run/user/$TARGET_UID/Xauthority" \
    "$TARGET_HOME/.Xauthority"
do
    if [[ -f "$cand" ]]; then
        XAUTH_SRC="$cand"
        break
    fi
done

if [[ -n "$XAUTH_SRC" ]]; then
    # 原文件通常 0700 且属主是桌面用户；root 拷到 /tmp 后自己就能读了
    XAUTH_COPY="/tmp/.x3588_ui_xauth.$TARGET_UID"
    if cp -f "$XAUTH_SRC" "$XAUTH_COPY" 2>/dev/null; then
        chmod 600 "$XAUTH_COPY"
        export XAUTHORITY="$XAUTH_COPY"
        echo "   X 授权   : $XAUTH_SRC（已复制到 $XAUTH_COPY）"
    else
        export XAUTHORITY="$XAUTH_SRC"
        echo "   X 授权   : $XAUTH_SRC（直接使用）"
    fi
else
    echo "   X 授权   : ⚠ 没找到 Xauthority 文件，若界面报无法连接显示，请手动指定" >&2
    echo "               export XAUTHORITY=/run/user/$TARGET_UID/gdm/Xauthority" >&2
fi

# ---------------------------------------------------------------------------
# 4. DISPLAY
#    3588 上是 GDM + Xorg，实际可用的显示是 :1（:0 的 socket 存在但拒绝连接）。
#    先尊重用户已设的 DISPLAY，否则默认 :1。
# ---------------------------------------------------------------------------
if [[ -z "${DISPLAY:-}" ]]; then
    for d in ":1" ":0"; do
        if [[ -S "/tmp/.X11-unix/X${d#:}" ]]; then
            export DISPLAY="$d"
            break
        fi
    done
    export DISPLAY="${DISPLAY:-:1}"
fi
echo "   显示器   : $DISPLAY"

# ---------------------------------------------------------------------------
# 5. HOME / 用户级路径
#    root 的 HOME 是 /root，Qt 会在里面找不到字体缓存和配置，
#    所以指回桌面用户的 HOME，让 Qt / fontconfig 复用已有的缓存。
# ---------------------------------------------------------------------------
export HOME="$TARGET_HOME"
export USER="$TARGET_USER"
export LOGNAME="$TARGET_USER"

# ---------------------------------------------------------------------------
# 6. 清掉 snap 注入的路径
#    如果当前 shell 是从 VS Code（snap 版）里打开的，会带一堆 snap 沙箱路径，
#    那些路径 root 读不到，Qt 会加载失败。与项目里的 launch_ui.sh 做法一致。
# ---------------------------------------------------------------------------
unset GTK_EXE_PREFIX GTK_PATH GSETTINGS_SCHEMA_DIR LOCPATH \
      GTK_IM_MODULE_FILE GIO_MODULE_DIR LD_LIBRARY_PATH 2>/dev/null || true

# ---------------------------------------------------------------------------
# 7. 定位可执行文件
# ---------------------------------------------------------------------------
UI_BIN=""
for cand in \
    "$SCRIPT_DIR/build/x3588_ui" \
    "$SCRIPT_DIR/x3588_ui" \
    "$SCRIPT_DIR/build/bin/x3588_ui"
do
    if [[ -x "$cand" ]]; then
        UI_BIN="$cand"
        break
    fi
done

if [[ -z "$UI_BIN" ]]; then
    cat >&2 <<'EOF'
错误：找不到 x3588_ui 可执行文件。

请先编译：

    cd 3588_control
    mkdir -p build && cd build
    cmake .. && make -j4

编译前请确认已安装 Qt5 开发包：

    sudo apt install qtbase5-dev

EOF
    exit 1
fi

echo "   可执行   : $UI_BIN"
echo "==========================="
echo ""

# ---------------------------------------------------------------------------
# 8. 启动
#    exec 让界面直接取代本脚本，Ctrl-C 就能干净退出。
# ---------------------------------------------------------------------------
exec "$UI_BIN" "$@"