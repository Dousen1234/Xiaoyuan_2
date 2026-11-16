#!/usr/bin/env bash
# 一键编译。不想用 CMake 时执行这个更省事。
set -euo pipefail

cd "$(dirname "$0")"

CXX="${CXX:-g++}"
FLAGS=(-std=c++17 -O2 -Wall -Wextra)

echo "==> 编译 test_rotate"
"$CXX" "${FLAGS[@]}" -o test_rotate claw_can.cpp test_rotate.cpp

echo "==> 编译 test_positionctrl"
"$CXX" "${FLAGS[@]}" -o test_positionctrl claw_can.cpp test_positionctrl.cpp

echo "==> 编译 selftest_sim"
"$CXX" "${FLAGS[@]}" -o selftest_sim claw_can.cpp selftest_sim.cpp

echo "==> 编译 selftest_position"
"$CXX" "${FLAGS[@]}" -o selftest_position selftest_position.cpp

echo "==> 运行离线自测"
./selftest_sim >/dev/null && echo "      selftest_sim      ✓ 通过"
./selftest_position >/dev/null && echo "      selftest_position ✓ 通过"

echo "==> 完成。可执行文件："
echo "      ./test_rotate        （功能 1：三指往复旋转）"
echo "      ./test_positionctrl  （功能 2：单路或多路位置控制）"
echo "      ./selftest_sim       （离线自测：CAN 通信流程）"
echo "      ./selftest_position  （离线自测：位置控制安全逻辑）"