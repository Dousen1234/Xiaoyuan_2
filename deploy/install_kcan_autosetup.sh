#!/bin/bash
#
# 一键安装鲲弘 CAN 通道自动配置（udev 规则）
#
# 背景：snap 版 VS Code 的集成终端无法 sudo（NoNewPrivs 沙箱限制）。
#       安装本规则后，插上鲲弘设备即自动完成 can0~can5 的波特率配置与 up，
#       之后在 VS Code 终端里可直接运行项目程序（内核已允许普通用户打开 CAN raw socket）。
#
# 用法（在外部系统终端 Ctrl+Alt+T 中执行）：
#   bash ~/TaiHu_motor_control/deploy/install_kcan_autosetup.sh
#
# 卸载：
#   bash ~/TaiHu_motor_control/deploy/install_kcan_autosetup.sh --uninstall

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULE_SRC="$SCRIPT_DIR/99-kcan-autosetup.rules"
RULE_DST="/etc/udev/rules.d/99-kcan-autosetup.rules"

if [[ "$1" == "--uninstall" ]]; then
    echo "[信息] 卸载 udev 规则 ..."
    sudo rm -f "$RULE_DST"
    sudo udevadm control --reload-rules
    echo "[完成] 已卸载。之后需手动执行 ip link 配置 CAN 通道。"
    exit 0
fi

if [[ ! -f "$RULE_SRC" ]]; then
    echo "[错误] 找不到规则文件: $RULE_SRC"
    exit 1
fi

echo "[信息] 安装 udev 规则到 $RULE_DST ..."
sudo cp "$RULE_SRC" "$RULE_DST"
sudo udevadm control --reload-rules

echo "[信息] 重新加载 kcan 驱动以使规则立即生效 ..."
# 先 down 所有 kcan 接口再重载模块，触发 udev add 事件
sudo modprobe -r kcan 2>/dev/null || true
sudo modprobe kcan

echo ""
echo "[完成] 安装成功！当前 CAN 接口状态："
sleep 1
ip -brief link show | grep -E "^can" || echo "  (未发现 can 接口，请确认鲲弘设备已插入)"
echo ""
echo "提示：state UP 即为已自动配置。现在可以直接在 VS Code 终端运行项目程序，"
echo "      例如: ./build/taihu_motor_tools_scan"
