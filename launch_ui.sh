#!/bin/bash
# taihu_keyboardctl_ui 启动脚本
#
# 作用：清掉 snap 版 VS Code 终端注入的环境变量（GTK_EXE_PREFIX / GTK_PATH /
# LOCPATH / LD_LIBRARY_PATH 等），避免 Qt 程序错误加载 /snap/core20 里的旧版
# libpthread 而报 "undefined symbol: __libc_pthread_init"。
#
# 用法：
#   ./launch_ui.sh              # 正常启动图形界面
#   ./launch_ui.sh --offscreen  # 无显示器环境（CI/纯逻辑验证）
#
# 若在外部系统终端运行本程序，可直接执行 ./build/taihu_keyboardctl_ui，
# 无需本脚本。

set -e
cd "$(dirname "$0")"

BIN=./build/taihu_keyboardctl_ui
if [ ! -x "$BIN" ]; then
    echo "错误：未找到 $BIN，请先编译：cd build && cmake .. && make -j4" >&2
    exit 1
fi

# 清理 snap 沙箱注入、可能干扰动态链接/插件查找的环境变量
unset GTK_EXE_PREFIX GTK_PATH GSETTINGS_SCHEMA_DIR LOCPATH \
      GTK_IM_MODULE_FILE GIO_MODULE_DIR LD_LIBRARY_PATH || true

if [ "$1" = "--offscreen" ]; then
    exec env QT_QPA_PLATFORM=offscreen "$BIN"
fi

exec "$BIN"
