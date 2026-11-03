# 钛虎关节模组 CAN 控制库（C++）

基于 **Linux + 鲲弘 KH-UCANFDX6-Mini（USB-CANFD 六通道模块）** 的钛虎（Titan）关节模组 C++ 控制库。支持位置控制、单腿步态、编码器标零、刹车控制、CAN ID 管理、通信诊断、多总线多电机同步控制，以及运行数据日志记录与曲线绘图；所有电机调试工具整合为菜单式总入口 `taihu_motor_tools`。

- **目标模组**：`CRA-RI50-60-PRO-2-81-B-2E-EC`（双编码器，减速比 81）；亦支持减速比 101 的模组
- **通讯方式**：CAN，波特率 **1M**，出厂默认 CAN ID = 1
- **上位机接口**：鲲弘 KH-UCANFDX6-Mini（Linux 下以标准 SocketCAN 接口 `can0`~`can5` 呈现，最多 6 路独立 CAN 总线）
- **控制模式**：位置模式（双编码器 `262144 cnt = 360°`）
- **工具入口**：`taihu_motor_tools`（菜单式电机工具总入口）、`kcanctl`（CAN 通道开关与波特率配置）
- **运行数据**：记录到 `record/motor_log.csv`，可用 Python 脚本绘制曲线图

---

## 目录

> 按**用途**分组索引，点击直达对应章节。第一次上手建议按 ① → ② 的顺序读。

### ① 环境与上手

- [硬件接线](#硬件接线) —— 供电、`CAN_H`/`CAN_L`/`GND` 接线、终端电阻
- [鲲弘 CANFD 模块环境配置（首次使用必读）](#鲲弘-canfd-模块环境配置首次使用必读)
  - [0. 系统要求](#0-系统要求)
  - [1. 安装系统依赖](#1-安装系统依赖)（含 gcc 与内核版本对齐）
  - [2. 下载驱动](#2-下载驱动)
  - [3. 编译 & 安装](#3-编译--安装)
  - [4. 加载驱动](#4-加载驱动)
  - [5. 配置并打开 CAN 通道](#5-配置并打开-can-通道) ← **手动逐通道 up，本项目不做开机自动全开**
  - [6. 状态检测与测试工具（可选）](#6-状态检测与测试工具可选)
  - [7. 驱动开机自动加载（可选）](#7-驱动开机自动加载可选)
  - [8. 通道绑定（多设备/多路总线的固定映射，可选）](#8-通道绑定多设备多路总线的固定映射可选)
- [鲲弘 CANFD 设备使用与工具指南](#鲲弘-canfd-设备使用与工具指南)
  - [设备使用基本流程](#1-设备使用基本流程) / [波特率与 CAN FD 配置](#2-波特率与-can-fd-配置) / [设备状态检测](#3-设备状态检测) / [驱动测试与诊断脚本](#4-驱动测试与诊断脚本) / [can-utils 通用工具](#5-can-utils-通用工具抓包发帧分析) / [异常恢复（BUS-OFF 等）](#6-异常恢复bus-off-等) / [厂家技术支持](#7-厂家技术支持)
- [编译](#编译) —— `cmake` 构建，以及全部可执行文件/脚本清单
- [快速开始](#快速开始) —— 开通道 → 跑 demo → 标零 → 联调上位机轨迹，四步走
- [项目结构](#项目结构) —— 目录树与每个文件的职责

### ② 上位机联调（本机当下位机用）

- [上位机关节位置流协议（10Hz CAN）](#上位机关节位置流协议10hz-can) —— 帧格式、ID 映射、心跳、断流与越限保护
- **[虚拟上位机 `up_walk_control_V1.py` 使用说明](#虚拟上位机-up_walk_control_v1py-使用说明) ★** —— 安装、两个终端逐条命令、参数、轨迹格式、场景速查、排错
- [协议全文 PDF `docs/step_protocol.pdf`](docs/step_protocol.pdf) —— 系统架构图、字节表、时序图、换算公式、RK3588 C 参考实现与部署

### ③ 下位机 `walk_control_V1`

- [示例程序 `walk_control_V1`](#walk_control_v1下位机上位机关节位置流拆分--关节级命令控制) —— 功能概述
- [walk_control_V1 使用指南](#walk_control_v1-使用指南)
  - [1. 前置条件](#1-前置条件) / [2. 构型与约定](#2-构型与约定程序内置命令中无需重复指定) / [3. 启动流程（从零开始）](#3-启动流程从零开始) / [4. 命令接口详解](#4-命令接口详解) / [5. 执行流水线（内置行为）](#5-执行流水线内置行为) / [6. 按需定制](#6-按需定制源码常量集中在文件头) / [7. 首次上机建议与常见问题](#7-首次上机建议与常见问题)

### ④ API 与命令行

- [核心 API](#核心-api) —— `CanFrame` / `CanInterface` / `JointModule`
- [工具函数 API（taihu_tools）](#工具函数-apitaihu_tools) —— 诊断 / 改 ID / 恢复出厂 / 标零 / 回零 / 读状态
- [命令行工具](#命令行工具)
  - [`taihu_motor_tools`（工具总入口，菜单式）](#taihu_motor_tools工具总入口菜单式)
  - [`kcanctl`（鲲弘 CAN 通道控制）](#kcanctl鲲弘-can-通道控制)

### ⑤ 示例程序

- [`taihu_singleleg_step`（单腿步态，四电机）](#taihu_singleleg_step单腿步态四电机)
- [`taihu_singleleg_step_xCAN`（多 CAN 总线单腿同步）](#taihu_singleleg_step_xcan多-can-总线单腿步态同步控制)
- [`taihu_single_motorctl`（单电机交互式）](#taihu_single_motorctl单电机交互式控制)
- [`taihu_single_motorctl_xCAN`（多总线电机同步）](#taihu_single_motorctl_xcan多-can-总线电机同步控制)
- [`taihu_keyboardctl`（键盘控制单腿四电机）](#taihu_keyboardctl键盘交互式控制单腿四电机)
- [`taihu_keyboardctl_ui`（Qt5 图形界面版）](#taihu_keyboardctl_ui图形界面版单腿四电机控制面板)
- [`taihu_pd_leg`（单腿四关节纯 PD）](#taihu_pd_leg单腿四关节纯-pd-位置模式)

### ⑥ 传感器与外设

- [扭矩传感器（坤维 HIT-V2.0 协议）](#扭矩传感器坤维-hit-v20-协议)
  - [独立终端工具](#独立终端工具与任何主控制程序并行使用) / [ROS2 集成（opt-in）](#ros2-集成opt-in)
- [MPT 磁编码器（金钢，RS485 接口）](#mpt-磁编码器金钢rs485-接口)
  - [接线](#接线) / [USB 转 485 选型](#usb-转-485-选型) / [两台共总线操作流程（D 款）](#两台共总线操作流程d-款) / [独立终端工具](#独立终端工具) / [接入 CAN 总线网关](#把编码器接入-can-总线taihu_encoder_can_bridge-网关) / [ROS2 集成（独立节点）](#ros2-集成独立节点)

### ⑦ 日志、协议与排错

- [日志与绘图](#日志与绘图) —— CSV 记录、[采样周期](#采样周期)、[绘制曲线图](#绘制曲线图)
- [协议要点](#协议要点) —— 钛虎电机原生协议速查
- [常见问题排查](#常见问题排查)
- [ROS2 集成](#ros2-集成) —— [节点架构](#节点架构) / [使用流程](#使用流程)
- [参考资料](#参考资料)

---

## 硬件接线

1. **供电**：关节模组需独立接入额定直流电源（额定 36V，常用 48V 供电）。**CAN 总线不供电**。
2. **CAN 接线**：鲲弘模块的 `CAN_H` ↔ 电机 `CAN_H`，`CAN_L` ↔ 电机 `CAN_L`，`GND` 共地（连接器型号 CJT A1251WR-S-3P，PIN1=GND，PIN2=CAN_L，PIN3=CAN_H）。
3. **终端电阻**：模块每路 CAN 接口已板载 120Ω 终端电阻；总线另一端（最后一个电机）再接 120Ω。

## 鲲弘 CANFD 模块环境配置（首次使用必读）

> 本章按照厂家文档《鲲弘 CAN FD 系列产品 Linux 驱动安装说明 V1.3》整理，从零开始一步步执行即可。
> 全部流程：**装系统依赖 → 下载驱动 → 编译安装 → 加载驱动 → 配置 CAN 通道 → 验证**。

### 0. 系统要求

| 要求 | 说明 |
|------|------|
| 操作系统 | Linux 32/64 位内核（Ubuntu 20.04 及以上验证通过） |
| 编译工具 | `make`、`gcc`（**gcc 版本最好与编译内核时一致**）、`g++` 及 libstdc++ |
| 内核头文件 | `linux-headers-$(uname -r)`（或交叉编译内核源码树） |
| 依赖库 | `libpopt-dev` |

### 1. 安装系统依赖

```bash
sudo apt update
sudo apt install -y build-essential linux-headers-$(uname -r) libpopt-dev wget unzip can-utils
```

**检查 gcc 版本是否与内核一致**（不一致可能编译失败）：

```bash
# 查看内核是用哪个 gcc 版本编译的
cat /boot/config-$(uname -r) | grep -i gcc_version
# 例如输出 CONFIG_GCC_VERSION=130300，表示内核用 gcc-13 编译

# 查看当前系统 gcc 版本
gcc --version

# 若不一致，安装对应版本（以 gcc-13 为例）并临时指定
sudo apt install -y gcc-13
export CC=gcc-13
```

### 2. 下载驱动

```bash
# 从 gitee release 下载最新驱动源码包
wget https://gitee.com/ChengDu-KunHong/KH-UCANFD_Linux_SDK/releases/download/latest/KH-UCANFD_Linux_SDK.zip

# 解压并进入目录（x.y.z 为实际版本号，如 1.4.2）
unzip KH-UCANFD_Linux_SDK.zip
cd KH-UCANFD_Linux_SDK-x.y.z/
```

> 也可以用 git 克隆（但仓库内只有文档，驱动源码仍需从 release 下载）：
> `git clone https://gitee.com/ChengDu-KunHong/KH-UCANFD_Linux_SDK.git`

### 3. 编译 & 安装

```bash
sudo make clean
sudo make
sudo make install
```

若无报错，则驱动安装成功。`make install` 会：
- 把内核模块 `kcan.ko` 安装到 `/lib/modules/$(uname -r)/`
- 安装配套工具到 `/usr/local/bin/`（`lskcan`、`kcan_monitor`、`kcan-settings` 等）
- 安装测试/诊断脚本（`kcanfd_test.sh`、`kcanosdiag.sh`）

> 备选方式：仓库也提供一键脚本 `sudo ./build.sh`（等价于上面的 make 流程；`-rules` 额外加载 udev 命名规则，`-u` 卸载，`-h` 帮助）。

### 4. 加载驱动

```bash
sudo modprobe kcan
```

**验证是否加载成功**：

```bash
lsmod | grep kcan          # 应输出类似：kcan 204800 0
```

**插上鲲弘设备**（或重新拔插 USB），确认设备被识别：

```bash
ip -d link show            # 应能看到 can0~can5 共 6 个接口，且驱动信息为 kcan
lsusb -t                   # USB 设备树中应出现鲲弘设备
```

> ⚠️ **安全模式（Secure Boot）异常**：若 `modprobe kcan` 报错
> `modprobe: ERROR: could not insert 'kcan': Key was rejected by service`，
> 说明系统开启了 Secure Boot，拒绝了未签名的第三方内核模块。需重启进 BIOS 关闭 Secure Boot（具体步骤按主板型号搜索），之后重新执行 `sudo modprobe kcan`。

### 5. 配置并打开 CAN 通道

驱动加载成功后，`ip link show` 应能看到 `can0`~`can5` 共 6 个接口，每个通道对应一路独立 CAN 总线：

```bash
# 配置波特率（本项目 1Mbps）并打开
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 其余通道同理（can1~can5）；CAN FD 模式可加 `dbitrate ... fd on`，例如：
# sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
```

> 推荐直接用 `bitrate` 数值方式配置，驱动会自动计算较优的位时间参数；手动指定 tq/prop-seg 等参数需要 CAN 位时序基础，容易出错。
>
> 设备启动（up）后无法再修改波特率，需先 `down` 再重新配置。

**查看通道状态**：

```bash
ip -d link show can0       # state UP 表示已打开；可看波特率、错误计数 berr-counter 等
ifconfig -a                # 查看接口收发数据包统计
```

> ⚠️ 注意：鲲弘驱动**不支持** `berr-reporting on`，请勿开启。
>
> **BUS-OFF 等异常恢复**：若 `ip -d link show` 显示 `state BUS-OFF`（错误计数很高），执行以下命令恢复（或重新拔插设备）：
> ```bash
> sudo ip link set can0 down
> sudo ip link set can0 up
> ```

### 6. 状态检测与测试工具（可选）

驱动安装后自带若干检测/压测工具（`lskcan`、`kcan_monitor`、`kcan-settings`、`kcanfd_test.sh`、`kcanosdiag.sh`，另有 Linux 标准 `candump`/`cansend`），功能简介与用法速查统一见下一章[「鲲弘 CANFD 设备使用与工具指南」](#鲲弘-canfd-设备使用与工具指南)，此处不再展开。

### 7. 驱动开机自动加载（可选）

若驱动安装后开机未自动加载，可创建 systemd 服务：

```bash
sudo nano /etc/systemd/system/load-kcan-module.service
```

文件内容：

```ini
[Unit]
Description=Load kcan Kernel Module
After=systemd-modules-load.service
Before=network.target

[Service]
Type=oneshot
ExecStart=/sbin/modprobe kcan
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target
```

启用服务并重启验证：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now load-kcan-module.service
sudo reboot
lsmod | grep kcan          # 重启后确认已自动加载
```

### 8. 通道绑定（多设备/多路总线的固定映射，可选）

鲲弘 SDK 提供三种方式把**物理 USB 口 / 设备 ID 固定绑定到特定接口名**，保证拔插设备后接口名不变。单模块单通道（本项目默认 `can0`）无需配置；若是多路总线或多模块同时使用，推荐按需选用：

- **驱动绑定（最简单）**：安装时执行 `sudo ./build.sh -rules`，重启后驱动设备统一以 `kcan` 命名。

- **USB 绑定（按物理 USB 口）**：手动写 udev 规则，把某个 USB 端口固定命名为 `kcan0` / `kcan1` 等：
  ```bash
  # 1. 查看设备所在 USB 端口（KERNELS 值）
  udevadm info -a /sys/class/net/can0

  # 2. 创建规则文件 /etc/udev/rules.d/99_kcan_usb.rules
  SUBSYSTEM=="net", ACTION=="add", DRIVERS=="kcan", KERNELS=="5-1.3:1.0", NAME="kcan0"
  SUBSYSTEM=="net", ACTION=="add", DRIVERS=="kcan", KERNELS=="5-1.4:1.0", NAME="kcan1"

  # 3. 重载规则并重新拔插设备
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```

- **ID 绑定（按设备硬件 ID）**：给每路通道设固定设备 ID，驱动按 ID 分配接口名：
  ```bash
  # 1. 设置各通道设备 ID（示例：can0 通道 ID=66，can1 通道 ID=67）
  kcan-settings -f=/dev/kcanusbfd32 -d 66
  kcan-settings -f=/dev/kcanusbfd33 -d 67

  # 2. 编辑 /etc/modprobe.d/kcan.conf，启用按设备 ID 分配
  options kcan assign=devid

  # 3. 重新加载驱动
  sudo rmmod kcan && sudo modprobe kcan
  ```

> 完整说明见 SDK 内 `Channel_Binding.md`（`git clone` 后位于仓库根目录）。

---

## 鲲弘 CANFD 设备使用与工具指南

> 本章依据《鲲弘 CAN FD 系列产品 Linux 驱动安装说明 V1.3》（UM-040001）整理，
> 面向**已经完成驱动安装**的用户，介绍设备日常使用方式与随附工具。
> 驱动安装步骤见上一章「鲲弘 CANFD 模块环境配置」。
>
> 约定：下文 `canX` 指 `can0`~`can5` 中你实际使用的通道（本项目默认 `can0`）。
> 需要 root 的命令（`ip link`、测试脚本等）请加 `sudo`；抓包/发帧类工具普通用户即可运行。

### 1. 设备使用基本流程

每次使用设备只需四步：

```bash
# ① 确认驱动已加载（插拔设备后 kcan 会自动注册 can0~can5）
lsmod | grep kcan
ip -d link show | grep -A2 "^.*can0"

# ② 配置波特率（本项目电机为 1Mbps；设备 up 后不能再改，需先 down）
sudo ip link set can0 type can bitrate 1000000

# ③ 打开设备
sudo ip link set can0 up

# ④ 使用完毕后关闭（可选）
sudo ip link set can0 down
```

> 💡 本项目提供 `kcanctl` 工具（源码 `tools/kcanctl.c`，随 CMake 一起构建到 `build/kcanctl`），
> 把上面 ②③④ 步封装成简单命令（在外部终端用 `sudo` 运行）：
>
> ```bash
> ./build/kcanctl list                 # 列出所有通道状态与波特率
> sudo ./build/kcanctl down-all        # 关闭所有通道
> sudo ./build/kcanctl up can0 1000000 # 打开 can0 并设为 1Mbps
> sudo ./build/kcanctl down can0       # 关闭 can0
> sudo ./build/kcanctl fd can0 1000000 2000000  # 以 CAN FD 模式打开
> ```
>
> ⚠️ **通道默认策略**：本项目**不在**插入 USB 时自动打开任何通道（仓库已删除早期的
> `deploy/` 自动配置规则）。如果你之前把自动规则装进了系统，请手动清理：
>
> ```bash
> sudo rm /etc/udev/rules.d/99-kcan-autosetup.rules && sudo udevadm control --reload-rules
> ```
>
> 清理后重新拔插设备（或 `sudo rmmod kcan && sudo modprobe kcan`），
> 用 `./build/kcanctl list` 确认所有通道为 `DOWN`，需要哪个通道再用 `kcanctl up` 打开。

### 2. 波特率与 CAN FD 配置

```bash
# 经典 CAN：500kbps
sudo ip link set can0 type can bitrate 500000

# CAN FD：仲裁段 500k / 数据段 2M
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on

# 查看所有可用配置项
sudo ip link set can0 type can help
```

注意事项（厂家文档强调）：

- **推荐直接设置波特率**，驱动会自动计算较优的时间段参数；手动配 `tq/prop-seg/phase-seg` 等时间参数容易出错，除非有相关基础。
- **不要开启 `berr-reporting`**：部分教程会建议开启错误报告，但鲲弘设备不兼容该功能，开启会报错。
- 设备 **up 之后无法修改波特率**，必须先 `down` 再改。
- 其他可选开关：`loopback`（自环）、`listen-only`（只听不发）、`one-shot`（单次发送不重发）、`restart-ms`（BUS-OFF 自动重启间隔）等，详见 `help`。

### 3. 设备状态检测

```bash
ip -d link show can0    # state、can state、berr-counter、波特率等
ifconfig -a             # 各接口 RX/TX 包数与错误数
```

- `can state ERROR-ACTIVE` 为正常；`ERROR-WARNING / ERROR-PASSIVE` 表示错误计数偏高；`BUS-OFF` 为总线严重错误已离线（恢复见第 6 节）；
- `berr-counter tx N rx N` 为发送/接收错误计数，**无节点应答时 tx 涨到 128 属正常**（见第 6 节常见误区）。

厂家随附的检测工具（`make install` 后位于 `/usr/local/bin`）：

| 工具 | 功能简介 | 用法 |
|------|----------|------|
| `lskcan` | 一次性打印所有通道的总线状态、收发帧数、波特率配置、错误计数 | `lskcan` |
| `kcan_monitor` | 实时总线监控（交互式，统计通信数据/总线状态/错误计数） | `kcan_monitor` |
| `kcan-settings` | 查看/设置通道设备硬件 ID（配合「环境配置」第 8 节通道绑定用） | `kcan-settings -f=/dev/kcanusbfd32 -d 66` |

### 4. 驱动测试与诊断脚本

SDK 自带两个脚本（`make install` 后已安装到 `/usr/local/bin`）：

| 脚本 | 功能简介 | 用法 |
|------|----------|------|
| `kcanfd_test.sh` | 满负载收发压测，验证是否丢包；**测试前需把 CAN 通道两两对接**（如 can0↔can1） | `sudo kcanfd_test.sh -c 2 -b 1000000 -d 5000000 -t 500000`（`-c` 通道数 / `-b` 仲裁段波特率 / `-d` 数据段波特率 / `-t` 包数） |
| `kcanosdiag.sh` | 生成诊断日志，发厂家技术支持（support@cdkhdz.com）排查设备异常 | `sudo kcanosdiag.sh` |

### 5. can-utils 通用工具（抓包/发帧/分析）

Linux SocketCAN 标准工具集（`sudo apt-get install can-utils`），鲲弘设备完全兼容：

| 工具 | 功能简介 | 用法示例 |
|------|----------|----------|
| `candump` | 实时抓包；过滤器格式 `[can_id]:[can_mask]` | 只看电机 ID=2 的回复：`candump can0,0x002:0x7FF` |
| `cansend` | 发单帧（`ID#十六进制数据`） | 向 ID=2 电机发「读位置」（功能码 0x08）：`cansend can0 002#08` |
| `cansniffer` | 只显示数据变化的帧，变化字节高亮（协议逆向利器） | `cansniffer -c can0` |
| `cangen` | 按间隔生成随机报文（压测/仿真） | `cangen can0 -g 100` |
| `canbusload` | 实时总线负载率统计 | `canbusload can0@1000000` |

### 6. 异常恢复（BUS-OFF 等）

用 `ip -d link show` 或 `ifconfig -a` 发现 `state BUS-OFF`（如 `berr-counter tx 248 rx 127`）时，重新启用设备即可恢复：

```bash
sudo ip link set can0 down
sudo ip link set can0 up
```

若仍异常，重新拔插 USB 设备，或重载驱动：`sudo rmmod kcan && sudo modprobe kcan`。

> 常见误区：空总线（没接电机/电机没上电）时发帧，`tx` 错误计数会涨到 128 进入 ERROR-PASSIVE——这是**没有节点 ACK** 的正常表现，不是设备故障；接好电机并上电后即恢复 ERROR-ACTIVE。

### 7. 厂家技术支持

- 邮箱：销售 sales@cdkhdz.com ／ 技术 support@cdkhdz.com
- 文档：《鲲弘 CAN FD 系列产品 Linux 驱动安装说明 V1.3》（UM-040001）
- SDK：https://gitee.com/ChengDu-KunHong/KH-UCANFD_Linux_SDK

---

## 编译

```bash
cmake -S . -B build
cmake --build build
```

生成的可执行文件位于 `build/`：

| 可执行文件 | 类型 | 说明 |
|-----------|------|------|
| `taihu_singleleg_step` | 示例 | 四电机单腿步态控制 |
| `taihu_singleleg_step_xCAN` | 示例 | 多 CAN 总线单腿步态同步控制（运行时输入步数 N 与总线数，每总线一条腿） |
| `taihu_single_motorctl` | 示例 | 单电机交互式控制（位置/正弦） |
| `taihu_single_motorctl_xCAN` | 示例 | 多 CAN 总线电机同步控制（每总线一线程，时序对齐） |
| `walk_control_V1` | 示例 | **下位机：上位机关节位置流拆分 + 关节级命令控制**（ID 1~16 拆分到 `can0~can3` 各 4 电机，5ms 重发最新目标；另含 `move 1 J1 cw 90` 式语义命令 → 限位 → 平滑轨迹 → 到位确认，YPPY 构型） |
| `taihu_keyboardctl` | 示例 | **键盘交互式控制单腿四电机**（↑↓ 按住转动松手即停，Ctrl+V/1~4/P 切换与查询） |
| `taihu_pd_leg` | 示例 | **单腿四关节纯 PD 位置模式 demo**（200Hz 下发目标角，`hold` 姿态保持 / `goto a b c d [s]` 五次多项式平滑过渡，软限位+电流+掉线保护，急停抱闸） |
| `taihu_keyboardctl_ui` | 示例 | `taihu_keyboardctl` 的 **Qt5 图形界面版**（按住按钮转动、自动刷新电机状态、急停；需 qtbase5-dev，未装自动跳过） |
| `taihu_motor_tools` | 工具 | **工具总入口（菜单式）**：数字键选总线 + 数字键选工具（扫描/诊断/改ID/恢复出厂/标零/开刹车/低压阈值） |
| `taihu_mpt_encoder` | 工具 | **MPT 磁编码器调试**（自动探测/设零位/改地址，RS485 总线，独立于 CAN） |
| `taihu_encoder_can_bridge` | 工具 | **MPT 编码器→CAN 网关**（RS485 读数转 CAN 帧发到电机总线，共总线零冲突） |
| `up_walk_control_V1.py` | 工具 | **上位机调试程序**：读 `step_make.xlsx` 轨迹（表头 `L{腿}_J{关节}` 自动识别列→ID），10Hz 原生 SocketCAN 发关节目标给下位机 `walk_control_V1`（四阶段：加载→开链路→双向握手→确认后回放；`--dry-run` 无硬件自测，内置离线限位预检查） |
| `kcanctl` | 工具 | 鲲弘 CAN 通道控制（打开/关闭通道、配置波特率，需 sudo） |

---

## 快速开始

### 1. 打开 CAN 通道

```bash
# 先按上文「鲲弘 CANFD 模块环境配置」装好驱动并加载（sudo modprobe kcan），
# 然后用 kcanctl 配置波特率并打开需要的通道（需 sudo，外部终端执行）
sudo ./build/kcanctl up can0 1000000     # 打开 can0 并设 1Mbps
sudo ./build/kcanctl up can1 1000000     # 多总线时按需打开其它通道
./build/kcanctl list                     # 查看各通道状态（普通用户可用）
```

### 2. 运行 demo（单电机交互式控制）

```bash
./build/taihu_single_motorctl   # 通道已 up 后普通用户即可运行
```

交互输入电机 CAN ID 与减速比后选择模式：1 位置控制（五次多项式轨迹）／2 正弦轨迹测试。多总线多电机同步控制请用 `./build/taihu_single_motorctl_xCAN`。

### 3. 标零（首次使用前）

把模组摆到期望零点，运行工具总入口，选择对应总线后按 `5`（编码器标零），再输入电机 CAN ID：

```bash
./build/taihu_motor_tools
```

> 标零后，正弦/旋转等运动以该零点为基准。若需「打开刹车 → 手动旋转到新位置 → 重新标零」，在菜单中选 `6`（打开刹车并重新标零）。

### 4. 联调上位机下发轨迹（虚拟上位机）

没有真实策略上位机时，用 `up_walk_control_V1.py` 顶替上位机，把 `step_position/step_make.xlsx` 的轨迹按 10Hz 发给下位机：

```bash
sudo ./build/kcanctl up can4 1000000                 # 上位机独立链路
./build/walk_control_V1                              # 下位机：上位机接口输 can4
python3 tools/up_walk_control_V1.py --can can4       # 另一终端：虚拟上位机
```

完整命令、参数、轨迹格式与排错见 **[虚拟上位机 `up_walk_control_V1.py` 使用说明](#虚拟上位机-up_walk_control_v1py-使用说明)**。

---

## 项目结构

```
TaiHu_motor_control/
├── CMakeLists.txt
├── include/taihu/               # 头文件
│   ├── can_interface.h          #   CAN 抽象接口 CanInterface / CanFrame
│   ├── socket_can.h             #   SocketCAN 驱动（鲲弘 KH-UCANFDX6-Mini，can0~can5）
│   ├── joint_module.h           #   关节模组控制类 JointModule
│   ├── torque_sensor.h          #   坤维 HIT 扭矩传感器
│   ├── mpt_encoder.h            #   金钢 MPT RS485/BUS 编码器
│   ├── motor_logger.h           #   日志记录器 MotorLogger（CSV）
│   ├── config.h                 #   全局配置（控制周期/采样周期等）
│   └── taihu_tools.h            #   工具函数封装（诊断/标零/旋转等）
├── src/                         # 核心库实现
│   ├── socket_can.cpp
│   ├── joint_module.cpp
│   ├── torque_sensor.cpp
│   ├── mpt_encoder.cpp
│   ├── motor_logger.cpp
│   └── taihu_tools.cpp
├── docs/
│   ├── api.md                   # 核心 C++ / ROS 2 API 参考
│   └── manuals/                 # 厂家协议手册
├── examples/                    # 示例程序
│   ├── taihu_singleleg_step.cpp #   单腿步态（四电机，五次多项式轨迹）
│   ├── taihu_singleleg_step_xCAN.cpp # 多总线单腿步态同步控制（每总线一条腿）
│   ├── taihu_single_motorctl.cpp #  单电机交互式控制（位置/正弦）
│   ├── taihu_single_motorctl_xCAN.cpp # 多总线电机同步控制（每总线一线程）
│   ├── walk_control_V1.cpp     #   下位机：上位机 ID1~16 关节流拆分到 4 总线（+ 关节级语义命令）
│   ├── taihu_keyboardctl.cpp    #   键盘交互式控制单腿四电机（按住转动松手即停）
│   └── taihu_pd_leg.cpp         #   单腿四关节纯 PD 位置模式 demo（200Hz，hold/goto 命令）
├── ui/                          # Qt5 图形界面程序
│   └── taihu_keyboardctl_ui.cpp #   键盘控制 demo 的 GUI 版（worker 线程独占 CAN）
├── launch_ui.sh                 # GUI 启动脚本（清 snap 环境，VS Code 终端用）
├── tools/                       # 命令行工具入口 + 绘图/上位机脚本
│   ├── taihu_motor_tools.cpp    #   工具总入口（菜单式，选总线+选工具）
│   ├── taihu_torque_sensor.cpp  #   扭矩传感器独立查看工具
│   ├── taihu_mpt_encoder.cpp    #   MPT 磁编码器独立调试工具（设零位/改地址）
│   ├── taihu_encoder_can_bridge.cpp  #   MPT 编码器→CAN 软件网关
│   ├── kcanctl.c                #   鲲弘 CAN 通道控制（打开/关闭/波特率）
│   ├── up_walk_control_V1.py    #   上位机：读 step_make.xlsx，10Hz 发关节目标给下位机 walk_control_V1
│   ├── gen_protocol_pdf.py      #   上位机协议 PDF（docs/step_protocol.pdf）生成器
│   └── plot_motor_log.py        #   日志绘图脚本（Python + matplotlib）
├── step_position/               # 上位机轨迹源（`tools/up_walk_control_V1.py` 读取）
│   ├── step_make.xlsx           #   标准格式：Time (s) + L1_J1 … L4_J4（201 点 @0.1s = 10Hz，关节端 rad，正=ccw）
│   └── step.xlsx                #   旧格式（表头 Joint N (rad)/(deg)），现已不再被脚本接受
└── record/                      # 运行时生成的日志与图片（不入库）
    ├── motor_log.csv            #   运行数据日志
    ├── position.png             #   位置曲线
    ├── velocity.png             #   速度曲线
    ├── current.png              #   电流曲线
    └── error.png                #   错误状态曲线
```

核心实现编译为静态库 `taihu_core`，示例与工具统一链接。
供其他 C++ 项目复用的完整接口索引见 [API 参考](docs/api.md)。

---

## 核心 API

### [`CanFrame`](include/taihu/can_interface.h:13)（CAN 帧）

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | `uint32_t` | CAN 标识符 |
| `data[8]` | `uint8_t[8]` | 数据域（最多 8 字节） |
| `dlc` | `uint8_t` | 数据长度（0~8） |
| `is_extended` | `bool` | 是否扩展帧 |

### [`CanInterface`](include/taihu/can_interface.h:30)（抽象接口）

| 方法 | 说明 |
|------|------|
| `open(device, bitrate)` | 打开 CAN 设备 |
| `close()` | 关闭设备 |
| `send(frame)` | 发送一帧 |
| `receive(frame, timeout_ms)` | 接收一帧（`timeout_ms` 为超时，0 非阻塞，<0 无限） |
| `drain()` | 清空残留接收帧（读命令前调用，避免写命令响应干扰） |

实现类：
- [`SocketCanInterface`](include/taihu/socket_can.h)（默认，Linux 原生 SocketCAN，即鲲弘 KH-UCANFDX6-Mini）

### [`JointModule`](include/taihu/joint_module.h:22)（关节模组控制）

构造：`JointModule(CanInterface& can, uint32_t tx_id = 1, uint32_t rx_id = 1)`

**基本控制**

| 方法 | 功能码 | 说明 |
|------|--------|------|
| `stop()` | `0x02` | 立即停止并抱死刹车（去使能） |
| `clearError()` | `0x0B` | 清除锁存错误（欠压/过流等） |

**参数管理**

| 方法 | 功能码 | 说明 |
|------|--------|------|
| `setMotorId(id)` | `0x2E` | 修改 CAN ID（1~127，立刻生效） |
| `saveParams()` | `0x0E` | 保存参数到 Flash（掉电保留） |
| `resetFactory()` | `0x0F` | 恢复出厂设置 |

**编码器零点（双编码器标零）**

| 方法 | 功能码 | 说明 |
|------|--------|------|
| `setZeroPosition()` | `0x50` | 编码器归零 |
| `setPositionOffset(cnt)` | `0x53` | 设置位置偏移值（软标零） |
| `readPositionOffset(cnt)` | `0x54` | 读取位置偏移值 |
| `readPositionCnt(cnt)` | `0x08` | 读取原始编码器位置 cnt |

**位置 / 速度 / 电流控制**

| 方法 | 功能码 | 说明 |
|------|--------|------|
| `setTargetPosition(rad)` | `0x1E` | 设置目标位置（弧度，输出端） |
| `setTargetVelocity(rad_s)` | `0x1D` | 设置目标速度（rad/s） |
| `setTargetCurrent(ma)` | `0x1C` | 设置目标电流（mA）；设 0 即「使能 + 零转矩」，用于解除抱闸 |
| `setPositionKp(kp)` | `0x2B` | 位置环比例增益 |
| `setPositionKd(kd)` | `0x2D` | 位置环微分增益 |
| `setVelocityKp/Ki/Kd(...)` | `0x29/0x2A/0x33` | 速度环 PID 增益 |

**状态读取**

| 方法 | 功能码 | 说明 |
|------|--------|------|
| `readPosition(rad)` | `0x08` | 读当前位置（弧度） |
| `readVelocity(rad_s)` | `0x06` | 读当前速度（rad/s） |
| `readCurrent(ma)` | `0x04` | 读当前电流（mA） |
| `readCurrentVelocityPosition(...)` | `0x41` | 一次读回电流 + 速度 + 位置（8 字节回复，高频采样用） |
| `readErrorState(bits)` | `0x0A` | 读错误状态位 |

**其他**

| 方法 | 说明 |
|------|------|
| `setGearRatio(ratio)` | 设置减速比（默认 81） |

---

## 工具函数 API（[`taihu_tools`](include/taihu/taihu_tools.h:1)）

这些函数把常用操作封装成可直接调用的高层接口，均接收一个 `CanInterface&` 与电机 CAN ID。

### `diagnose`（诊断）

```cpp
void diagnose(CanInterface& can, uint32_t can_id);
```

读取并打印电机的**当前位置、位置偏移、错误状态**。

### `changeCanId`（修改 CAN ID）

```cpp
bool changeCanId(CanInterface& can, uint32_t cur_id, uint32_t new_id);
```

停止电机 → 修改 CAN ID（立刻生效）→ 用新 ID 保存到 Flash。`new_id` 范围 1~127。

### `resetFactory`（恢复出厂）

```cpp
bool resetFactory(CanInterface& can, uint32_t can_id);
```

停止 → 恢复出厂设置 → 保存。恢复后 CAN ID 回到默认 1。

### `setZeroPosition`（标零）

```cpp
bool setZeroPosition(CanInterface& can, uint32_t can_id);
```

采用「设置位置偏移值」软标零：读当前位置 + 旧偏移值 → 计算绝对位置 → 设置偏移值 = 绝对位置 → 保存到 Flash，使当前位置归零。

### `rotateFromZero`（回零位 + 旋转）★

```cpp
bool rotateFromZero(CanInterface& can, uint32_t can_id,
                    double angle_deg,      // 旋转角度（度，逆时针为正）
                    int32_t kp, int32_t kd);
```

单电机位置控制完整流程：清除错误 → 配置 KP/KD → **回到零位**（等待到位）→ **旋转指定角度**（等待到位）→ 停止。

### `readMotorStatus`（读完整状态）

```cpp
bool readMotorStatus(CanInterface& can, uint32_t can_id, double gear_ratio,
                     MotorStatus& status);
```

一次读取电机的 ID、位置、速度、电流、错误状态（用 `0x41` 三合一读优化）。

### 调用示例

```cpp
#include "taihu/socket_can.h"
#include "taihu/taihu_tools.h"

using namespace taihu;

SocketCanInterface can;
can.open("can0", 0);   // 鲲弘 KH-UCANFDX6-Mini 的 can0 通道

rotateFromZero(can, /*can_id=*/2, /*angle_deg=*/90.0, /*kp=*/5000, /*kd=*/60);
setZeroPosition(can, 2);        // 标零
diagnose(can, 2);               // 诊断

can.close();
```

---

## 命令行工具

### `taihu_motor_tools`（工具总入口，菜单式）

所有电机工具打包为一个交互程序，运行后：

1. 数字键选择电机所在的 CAN 总线（0~5 → can0~can5）；
2. 数字键选择要运行的工具；
3. 工具执行完回到菜单，可换工具、换总线（9）或退出（0）。

```bash
./build/taihu_motor_tools     # 普通用户即可运行（接口 up 后）
```

菜单一览：

| 按键 | 功能 | 对应工具函数 | 说明 |
|------|------|--------------|------|
| 1 | 扫描总线上的 CAN ID | — | 遍历 ID 1~127 发送读位置命令，打印所有有响应的设备 |
| 2 | 通信诊断 | [`diagnose()`](include/taihu/taihu_tools.h) | 打印位置/偏移/错误状态 |
| 3 | 修改 CAN ID | [`changeCanId()`](include/taihu/taihu_tools.h) | 新 ID 保存到 Flash，掉电保留 |
| 4 | 恢复出厂设置 | [`resetFactory()`](include/taihu/taihu_tools.h) | 有二次确认，防误操作 |
| 5 | 编码器标零 | [`setZeroPosition()`](include/taihu/taihu_tools.h) | 运行前先把模组摆到期望零点 |
| 6 | 打开刹车并重新标零 | — | 开刹车 → 手动旋转 → 按回车 → 保存新零位 → 关刹车抱闸 |
| 7 | 设置低压阈值 | [`setLowVoltageThreshold()`](include/taihu/taihu_tools.h) | 保存 Flash，断电重新上电生效 |
| 9 | 更换 CAN 总线 | — | 关闭当前接口，回到总线选择 |
| 0 | 退出 | — | |

各工具所需参数（CAN ID、新 ID、阈值等）在执行时交互输入。

### `kcanctl`（鲲弘 CAN 通道控制）

```bash
./build/kcanctl list                  # 列出所有通道状态与波特率（普通用户可用）
sudo ./build/kcanctl up can0 1000000  # 打开 can0 并设 1Mbps
sudo ./build/kcanctl down can0        # 关闭 can0
sudo ./build/kcanctl down-all         # 关闭所有通道
sudo ./build/kcanctl fd can0 1000000 2000000  # 以 CAN FD 模式打开
```

详见「鲲弘 CANFD 设备使用与工具指南」第 1 节。

---

## 示例程序

### `taihu_singleleg_step`（单腿步态，四电机）

```bash
./build/taihu_singleleg_step
```

四个电机（ID=1、4 减速比 101；ID=2、3 减速比 81）按 **6 阶段单腿步态**运动，每阶段 `kStepSec`（默认 10s，总时长 60s）：

| 阶段 | 动作 |
|------|------|
| 0 | 四电机回零并保持不动 |
| 1 | ID1/4 不动；ID2 转 -180°，ID3 转 +360° |
| 2 | ID2/3/4 不动；ID1 往复 0° → 60° → -60° → 0° |
| 3 | ID1/4 不动；ID2/3 按与阶段 1 相反方向回零 |
| 4 | ID1/2/3 不动；ID4 往复 0° → 90° → -90° → 0° |
| 5 | 四电机回零并保持不动 |

往复阶段在阶段内均分为 3 个子段（关键帧 0°/峰值/谷值/0°），相邻关键帧之间用**五次多项式**插值保证位置/速度/加速度连续、轨迹平滑。配置见 [`examples/taihu_singleleg_step.cpp`](examples/taihu_singleleg_step.cpp:44)。

### `taihu_singleleg_step_xCAN`（多 CAN 总线单腿步态同步控制）

```bash
./build/taihu_singleleg_step_xCAN
```

在 `taihu_singleleg_step` 基础上扩展为**多总线 + 多步**版本：

- **步态定义**：一步 = 3 个阶段（一步首尾姿态重合，多步之间衔接连续）：

| 阶段 | 动作 |
|------|------|
| 0 | 四电机在 0 位保持不动 |
| 1 | ID1/4 不动，ID2/3 从 0° 转到 +180° |
| 2 | ID1/4 不动，ID2/3 从 +180° 转回 0° |

- **回车确认（每一步开始前）**：**每一步的阶段 0 之前**都要按一次回车才开始这一步，共 N 次；一步之内阶段 0 → 1 → 2 连续执行、中途不停顿。多总线时所有总线都报告就绪后主线程才提示，回车后各总线**同步开跑**；
- **轨迹方式（二选一，由源码开头的 `kUsePdPointToPoint` 切换，默认纯 PD）**：

| 方式 | 阶段推进依据 | 适用场景 |
|------|--------------|----------|
| **纯 PD 点到点**（默认 `true`） | 直接下发阶段终点姿态，**实测到位**才推进（误差 ≤ `kSettleErrorRad`≈2° 且连续 `kSettleHoldSamples`=3 个采样周期），超时 `kSettleTimeoutMs`=15 s 强制推进并打印当时最大误差 | 快节奏（`kPhaseSec` 1 s 甚至更小），每阶段实际时长自适应 |
| 定时 + 五次多项式（`false`，原版） | 固定 `kPhaseSec` 秒，到点即切换 | 慢节奏、需要严格时间轴复现（绘图/上位机对拍） |

  > 为什么快节奏要用纯 PD：五次多项式在 T 秒内走完 Δ 角度，峰值速度为 $1.875\Delta/T$、峰值加速度为 $5.77\Delta/T^2$。Δ=180°、T=1.0 s 时约 5.9 rad/s / 18 rad/s²（折算到减速比 81 的电机端约 4500 rpm），电机会进入速度/电流饱和而严重滞后；但定时模式只看时钟，到点就切阶段，于是出现"轨迹没走完就进下一阶段"。纯 PD 把推进依据从**时钟**换成**实测到位**，从根上消除这种参考跳变。代价是阶跃瞬间位置误差最大、转矩指令一步拉满（电流冲击/可能过冲），若实机上太猛可把 `kPdRampSec` 从 0 改成 0.1~0.2 s，参考会在该时间内线性爬到终点（仍只靠 PD 跟随）。

- **运行时交互输入**：步数 N（1~1000，默认 1）、CAN 总线数量（1~6）；每条总线输入接口名（可输入 `can0` 或通道号 `0`，自动补全为 `canN`，默认 can0、can1、…）；
- **节奏参数**：由源码开头的 [`kPhaseSec`](examples/taihu_singleleg_step_xCAN.cpp) 单一变量控制（默认 3.0 s）——定时模式下即每阶段实际时长；纯 PD 模式下为**每阶段最短停留时间**（保证阶段 0 的零位保持不被塌缩），实际时长 = `max(kPhaseSec, 到位所需时间)`；
- 每条总线挂**一条腿**（电机 ID 1~4，减速比 101/81/81/101），每条总线由独立线程组控制（发送/接收/记录各一个线程）：发送线程按 `kLoopPeriodMs`=5 ms 下发目标位置，接收线程按 100 ms 采样状态入队并负责到位判定，记录线程写 CSV。所有总线先完成准备（清错、设增益、开日志），再由主线程设定**统一起跑时刻**（就绪后 +300ms）对齐线程启动；
- 日志按总线分文件：`record/motor_log_<接口名>.csv`（如 `record/motor_log_can0.csv`），绘图时作为输入传给 `tools/plot_motor_log.py`（文件名含 `_can` 会自动按本 demo 的步态模型画目标轨迹；走了多步时用 `--num-steps N` 指定，见[绘制曲线图](#绘制曲线图)）。**注意**：纯 PD 模式下每阶段实际时长是自适应的，绘图脚本用的是固定 `--phase-sec` 的名义时间轴，画出的目标曲线与实测会有相位差；要严格对拍请用定时模式。


典型场景：can0、can1 各接一条四电机腿，两腿严格同步连走 N 步。

### `taihu_single_motorctl`（单电机交互式控制）

```bash
./build/taihu_single_motorctl
```

交互式输入电机 CAN ID、减速比，然后选择模式：
- **1 位置控制**：再输入目标角度（度）与完成时间（秒），用五次多项式轨迹在指定时间内平滑到达目标角度；
- **2 正弦轨迹测试**：电机做正弦运动。

### `taihu_single_motorctl_xCAN`（多 CAN 总线电机同步控制）

```bash
./build/taihu_single_motorctl_xCAN
```

在 `taihu_single_motorctl` 基础上扩展为**多总线版本**：

- 交互输入 CAN 总线数量（1~6）；每条总线输入接口名（可输入 `can0` 或通道号 `0`，自动补全为 `canN`）、电机数量、各电机 CAN ID 与减速比；
- 每条总线由**独立线程**控制，单电机控制方法与 `taihu_single_motorctl` 完全一致；
- 所有线程先完成准备（清错、设增益、读起始位置），再由主线程设定**统一起跑时刻**（就绪后 +300ms），各线程自旋等待到该时刻后**同时开始发指令**，保证跨总线时序对齐；
- 控制模式与 `taihu_single_motorctl` 相同：1 位置控制（五次多项式轨迹，所有电机相同目标角度与时长）／2 正弦轨迹测试。

典型场景：can0、can1 各接一台电机，两电机严格同步转动同一角度。

### `walk_control_V1`（下位机：上位机关节位置流拆分 + 关节级命令控制）★

```bash
./build/walk_control_V1
```

本机作为**下位机**，另一台电脑（RL 策略/步态规划）作为**上位机**，4 条腿臂作为执行器：

**① 上位机关节位置流接收与拆分**（主功能）：上位机以 10Hz 发送帧 ID 1~16 的关节预期位置，帧结构与钛虎写命令帧**同构**——`DLC=5`，`[0x1E][int32 小端 = 关节端 rad × 10000]`。下位机按 ID 拆分：**ID 1~4 → `can0` 电机 1~4，ID 5~8 → `can1` 电机 1~4**，以此类推（腿号 = (ID−1)/4+1，电机号 = (ID−1)%4+1）。换算为电机端角后以 5ms 周期重发最新目标，**PD 由电机固件内部执行**（KP=4000/KD=60），下位机不做软件插值。同时 10Hz 回发 16 个关节的实际位置，200ms 发心跳 `0x161`。协议全文详见 **[docs/step_protocol.pdf](docs/step_protocol.pdf)**。

> ⚠️ 上位机必须接在**独立的 CAN 接口**上（启动时输入接口名，如 `can4`），**不能**与电机总线 `can0~can3` 共用——否则上位机发的 ID 1~4 会被腿 1 的真电机直接当作位置命令执行，且 ID 5~16 在电机总线上无接收者。程序启动时会主动拒绝共用。

**② 关节级语义命令**（键盘，本机调试用）：输入 腿号 + 关节 + 方向 + 角度，程序自动完成 读当前角 → 软件限位 → 五次多项式平滑轨迹 → 5ms 周期下发 → 到位确认，对应"腿1的J1顺时针转90°"这类指令。流模式进行中暂缓，急停锁定期间拒绝。

- 构型：4 腿 × 4 关节 YPPY（腿 i ↔ `can{i-1}`，电机 CAN ID 1~4）；角度按**关节端**计算，ccw = 正方向；
- 软件限位（关节端）：J1 ±90°、J2 ±120°、J3 -30°~+180°、J4 ±90°（电机端等效：J2 ±240°、J3 −60°~+360°，J1/J4 直驱同为 ±90°）。**上位机目标越限 → 钳位到限位值下发**（关节走到限位并停住）；**实际角到达限位 → 整机软限位锁定 + `0x162` 报警**；键盘 `move`/`goto` 越界仍直接拒绝。解除方式：键盘 `clear` 或上位机 `[0x03]`；
- 键盘命令：`move`（相对）/ `goto`（绝对）/ `pos`（查询，读缓存零总线流量）/ `stat`（流状态、软限位锁定与报警关节、各腿收帧/丢弃/电机在线/使能）/ `arm`（使能 = 解抱闸，启动时已自动执行）/ `disarm`（去使能 = 抱闸并锁定）/ `estop`（急停抱闸并**锁定**）/ `resume`（解除锁定并自动重新使能）/ `clear`（清除软限位报警锁定）/ `quit`（退出抱闸）；
- **使能语义**：协议无独立使能命令——`setTargetCurrent(0)`（`0x1C` 写 0）= 使能 + 零转矩 = 解抱闸，`0x02` = 去使能 + 抱闸。电机上电默认去使能，未使能时 `0x1E` 位置帧会被接收但不产生运动；本 demo 已在 `initLeg`（起 busThread 前）与 `busThread`（消费 `arm_req`）两处内置使能；
- 线程模型：主线程只做键盘路由，**4 个总线线程各独占一条 `can0~can3`**，**1 个上位机线程独占上位机接口**（`SocketCanInterface` 无内部锁，故每条 CAN 接口只能由一个线程访问）；线程间只通过原子缓存交换数据。急停以标志位形式由各总线线程在自己的 tick 内消费，响应 ≤5ms；
- 断流保护：流模式中 1s 无新样点自动退出流模式（电机保持不抱闸）；某电机 `0x41` 读取连续 3 次超时则该关节读操作退避 1s，避免阻塞 5ms 控制环。

> 从零上手的完整指南（启动流程、命令参数与取值范围、终端反馈格式、执行流水线、定制常量、首上机建议与常见问题）见独立章节 **[walk_control_V1 使用指南](#walk_control_v1-使用指南)**。

### `taihu_keyboardctl`（键盘交互式控制单腿四电机）

```bash
./build/taihu_keyboardctl
```

用键盘实时控制**一条腿的四个电机**（ID 1~4，减速比 101/81/81/101）：

- 启动后依次输入 CAN 接口（`can0` 或通道号 `0`）与初始转速（rpm，输出端，默认 10）；
- 命令行底部始终显示**当前控制的电机 ID**与状态；
- **↑ 按住 = 逆时针转，↓ 按住 = 顺时针转，松手立即停止**（速度模式，控制线程每 10ms 重发速度指令作心跳）；
- 快捷键：

| 快捷键 | 功能 |
|--------|------|
| `Ctrl+V` | 重新输入转速（rpm，范围 0.1~200） |
| `Ctrl+1`~`Ctrl+4`（或直接按 `1`~`4`） | 切换控制电机 ID 1~4 |
| `↑` / `↓` | 按住逆时针 / 顺时针转动，松手即停 |
| `Ctrl+P` | 打印当前电机信息：电压（V）、电流（A）、位置（相对零位，度）、错误状态 |
| `h` | 显示帮助 |
| `q` 或 `Ctrl+C` | 退出（全部电机停止并抱闸） |

**键盘后端（自动选择）**：

- **evdev 后端**（首选）：直接读 `/dev/input/event*`，能拿到真实的按下/松开事件，松手立即停止。需要当前用户在 `input` 组：`sudo usermod -aG input $USER` 后**注销重登**生效；
- **终端 raw 模式**（回退，普通用户默认）：终端收不到「松开」事件，改用**按键自动重复**推断按住状态——按住方向键时终端持续发重复码维持转动，松手后约 150~200ms 内判定停止。功能完全相同，只是停机有轻微延迟。

**安全参数**（在 [examples/taihu_keyboardctl.cpp](examples/taihu_keyboardctl.cpp:1) 文件开头调整）：

- `kApplyVelocityGains=false`：默认使用电机出厂速度环 PID，不额外下发增益（改为 `true` 可下发 `kVelocityKp/Ki/Kd`）；
- `kBrakeOnRelease=false`：松手只把目标速度置 0、保持使能（不抱闸），避免频繁抱闸的机械冲击；改为 `true` 则松手立即 `0x02` 去使能抱闸；
- `kMaxRpm=200`：转速上限，防止误输入飞车。

> 建议首次使用时把腿**悬空**或减小负载，从低转速（如 5 rpm）试起。

### `taihu_keyboardctl_ui`（图形界面版：单腿四电机控制面板）

```bash
./launch_ui.sh          # VS Code（snap 版）终端里用这个
./build/taihu_keyboardctl_ui   # 外部系统终端可直接运行
```

`taihu_keyboardctl` 的 **Qt5 图形界面版本**，功能与命令行版一一对应：

| 命令行版 | 图形界面版 |
|----------|-----------|
| 启动输入 CAN 接口 | 顶部「连接」区下拉框（自动枚举 `can*`，未 up 的接口会给出提示）+ 连接按钮 |
| `Ctrl+V` 改转速 | 「转速设定」数字框 + 滑块 + 快捷预设按钮（1/5/10/30/50/100/200 rpm） |
| `Ctrl+1`~`4` 切电机 | 「电机选择」四个单选按钮（含减速比显示），或快捷键 `Ctrl+1`~`4` |
| `↑`/`↓` 按住转动松手即停 | 「▲ 逆时针 / ▼ 顺时针」大按钮**按住转动**，或方向键 `↑`/`↓` |
| `Ctrl+P` 查看电机信息 | 右侧「当前电机状态」面板：电压/电流/位置/速度/错误位，自动周期刷新 |
| `q`/`Ctrl+C` 退出 | 关闭窗口 / Esc 急停 / 「■ 急停」按钮 |

**线程模型**：所有 CAN 收发集中在独立的 `MotorWorker` 线程（10ms 心跳定时器），UI 线程绝不直接碰总线——运动命令（转速/电机/方向）通过原子变量传递，按下/松手在 10ms 内生效；状态快照经 Qt 信号回传。转动期间默认暂停状态读取（可勾选打开），保证心跳与松手即停的响应稳定。

**安全设计（多重兜底）**：

1. 按钮 `released()` / 键盘 keyRelease → 方向立即置 0；
2. 看门狗定时器（80ms）：丢失 released 事件时按 `qApp->mouseButtons()` 强制停机；
3. **窗口失焦自动停机**（防止用户切走窗口却还按着鼠标）；
4. 急停按钮 / Esc / Ctrl+C → 全部电机速度置 0 并发 `0x02` 抱闸；
5. 关闭窗口 → 全部电机停止并抱闸后线程安全退出；
6. 转速上限 200 rpm，任何确认弹窗弹出前先停止运动。

**snap 版 VS Code 注意**：其终端注入的环境变量会让 Qt 误加载 `/snap/core20` 里的旧 `libpthread` 而报 `symbol lookup error`。请用 [`launch_ui.sh`](launch_ui.sh:1) 启动（自动清理环境），或在外部系统终端直接运行 `./build/taihu_keyboardctl_ui`。

> 构建需要 Qt5 Widgets（`sudo apt install qtbase5-dev`）。未安装时 CMake 自动跳过该目标，不影响其它程序构建。

### `taihu_pd_leg`（单腿四关节纯 PD 位置模式）

```bash
sudo ./build/kcanctl up can0 1000000   # 外部终端先开通道
./build/taihu_pd_leg                   # 默认 can0，可传参指定 canN
```

面向单腿 4 关节（ID 1~4，关节端减速比 101/162/162/101，与 `walk_control_V1` 构型一致）的**纯 PD 位置伺服** demo：电机内部位置环 KP=4000 / KD=60（启动时下发），上位机以 **200Hz** 周期流式发送目标角（`0x1E`），不含重力前馈——悬空时存在重力下垂稳态误差，属预期现象。

| 命令 | 功能 |
|------|------|
| `hold` | 姿态保持：捕获当前实测姿态为目标，解抱闸进入 PD 伺服（验证刚度与下垂） |
| `goto a b c d [s]` | 交互目标姿态：五次多项式平滑过渡到 4 个关节目标角（度），时长 s 默认 2.0s，到位后自动转保持 |
| `pos` | 打印当前关节角 / 目标角 / 电流 |
| `gains kp kd` | 运行时调整电机位置环 KP/KD |
| `mon on\|off` | 1Hz 状态行滚动打印开关 |
| `estop` | 急停：全部电机 `0x02` 抱闸并退出伺服 |
| `help` / `quit`（`q`） | 帮助 / 退出（退出自动抱闸，Ctrl+C 同样抱闸） |

**安全保护**（常量集中在 [examples/taihu_pd_leg.cpp](examples/taihu_pd_leg.cpp:1) 文件头）：

- 目标角下发前钳位到软限位（J1 ±90°、J2 ±120°、J3 −30°~+180°、J4 ±90°；电机端等效 J2 ±240°、J3 −60°~+360°）；
- 实测角越限 +3° 立即急停；电流 >8A 持续 0.5s 急停、>5A 告警；
- 错误位 0.5s 巡检非零急停；连续 20 次读反馈失败（掉线/总线异常）急停；
- 任何退出路径（quit / Ctrl+C / 保护触发）统一 `0x02` 抱闸。

**日志**：`record/pd_leg_log.csv`（100ms 采样，关节端 rad + mA；注意 `plot_motor_log.py` 的步态叠加模型不适用于本 demo，仅用于查看曲线）。

> 建议首次使用时把腿**悬空**：先 `hold` 观察保持刚度，再 `goto 0 30 60 0 3` 等小角度指令验证平滑过渡。

---

## walk_control_V1 使用指南

> 四腿机器人**关节级命令控制** demo 的完整使用手册：上位机给出「哪条腿 · 哪个关节 · 什么方向 · 转多少度 · 用多久」的语义命令，程序自动完成 限位检查 → 五次多项式平滑轨迹 → 5ms 周期下发 → 到位确认。
> 源码：[`examples/walk_control_V1.cpp`](examples/walk_control_V1.cpp)（构型、限位、增益等常量集中在文件头部）。速览见[示例程序](#示例程序)一章。

### 1. 前置条件

1. 鲲弘驱动已安装并加载：`lsmod | grep kcan` 有输出（首次安装见[「鲲弘 CANFD 模块环境配置（首次使用必读）」](#鲲弘-canfd-模块环境配置首次使用必读)）；
2. 电机按构型接好并上电，编码器已标零（`taihu_motor_tools` 菜单 `5`，见[快速开始](#快速开始)）——`goto` 的 0° 与上位机流的 0 rad 均以标零位置为基准；
3. 项目已编译：`cmake -S . -B build && cmake --build build`，产物 `build/walk_control_V1`；
4. （接上位机时）上位机 CAN 卡接到本机**独立**的一路 CAN 上（如 `can4`），不能与 `can0~can3` 共用。

### 2. 构型与约定（程序内置，命令中无需重复指定）

| 项 | 约定 |
|---|---|
| 腿 ↔ CAN 通道 | 腿 1~4 ↔ `can0`~`can3`（4 路独立总线） |
| 每腿关节 | J1~J4，电机 CAN ID 固定：J1=1、J2(P1)=2、J3(P2)=3、J4=4 |
| 上位机帧 ID | ID 1~16 = (腿号−1)×4 + 关节号；ID 1~4 → `can0` 电机 1~4，ID 5~8 → `can1` 电机 1~4，以此类推 |
| 传动 | YPPY：J1/J4 = Y 直驱（减速比 101）；J2/J3 = P 锥齿轮 1:2（关节端总减速比 162） |
| 角度语义 | 一律按**关节端**度数（键盘命令）/ 弧度（上位机协议）；P 关节的 ×2 换算由程序内部完成，**无需预先乘 2** |
| 符号 | **ccw（逆时针）= 正角度**，cw（顺时针）= 负角度；与 `taihu_singleleg_step_xCAN` 的电机正方向一致（该 demo 关键帧 `{0,180,180,0}` 中 ID2/ID3 同为 +180°，说明两个 P 型锥齿轮电机端同向，故 `kJointDirSign = {+1,+1,+1,+1}`） |
| 软件限位（关节端） | J1 ±90°；J2 ±120°；J3 −30°~+180°；J4 ±90°（键盘命令越界直接拒绝；上位机目标越界**钳位到限位值**，实际角触限则整机锁定 + `0x162` 报警） |
| 限位（电机端等效） | = 关节端 × `kMotorRadPerJointRad`（J2/J3 锥齿轮 ×2）：J1 ±90°；J2 ±240°；J3 −60°~+360°；J4 ±90°。判定一律按关节端 |
| 位置环增益 | KP=4000、KD=60（与 xCAN demo 一致）；**PD 在电机固件内执行**，下位机不做软件插值 |

### 3. 启动流程（从零开始）

**第 1 步：打开 CAN 通道**（需 sudo，**必须在外部系统终端**执行——snap 版 VS Code 终端无法提权；通道已打开可跳过）：

```bash
sudo ./build/kcanctl up can0 1000000   # 腿 1（can1/can2/can3 对应腿 2/3/4，按实际接线打开）
sudo ./build/kcanctl up can4 1000000   # 上位机链路（独立接口，接口名按实际填写）
./build/kcanctl list                   # 查看通道状态（普通用户可用）
```

**第 2 步：启动、选择启用的腿、指定上位机接口**：

```bash
./build/walk_control_V1
```

程序打印构型/协议摘要后依次询问：

1. **腿号**：直接回车 = 全部 4 条腿；只接部分腿时可输入如 `1 3`（空格分隔）；
2. **上位机 CAN 接口名**：如 `can4`；**直接回车 = 跳过上位机链路**（仅键盘命令可用）；输入 `can0~can3` 会被拒绝并退出（不能与电机总线共用，否则上位机发的 ID 1~4 会被腿 1 真电机直接执行）。

随后逐腿打开 `can{腿号-1}`、清除电机错误、下发增益，逐电机探测在线性，并**自动使能（解抱闸）**——电机立即以上电时的实测姿态作为保持目标：

```
[腿1] can0 电机在线性：J1 J2 J3 J4（× = 无响应）
[腿1][使能] 抱闸已解除（4/4 电机应答），当前姿态已作为保持目标
[信息] 4 条腿就绪（KP=4000 KD=60，下发周期 5ms，反馈读取 100ms）
[协议] 上位机链路已启动：can4（ID 1~16，10Hz 目标 / 10Hz 反馈 / 200ms 心跳）
```

> ⚠️ **抱闸解除瞬间腿会失去抱闸支撑**，仅靠位置环保持当前姿态。启动前请确认腿已悬空或已可靠支撑，否则可能下坠/顶到地面。

- 个别电机显示 `×` 仅警告、不阻塞启动，但后续涉及该电机的命令会失败；
- 上位机接口打开失败也仅打印告警，键盘命令仍可用；
- **一条腿都没打开则程序直接退出**（检查通道是否 up）。

**第 3 步：命令交互**：`> ` 提示符下输入命令（接口见第 4 节）。`move`/`goto` 按腿路由到对应总线线程，**同一条腿内逐条串行执行**，执行期间可继续输入新命令（排队等待）。

**第 4 步：退出**：输入 `quit`（或 `q`，Ctrl+D 同效）——各总线线程退出 → 全部电机 `0x02` 停止并抱闸 → 关闭总线。

### 4. 命令接口详解

通用规则：参数以空格分隔；关节名 `J1~J4` 大小写均可；`h`/`q` 是 `help`/`quit` 的别名；命令指定的腿须已启用（否则直接提示、不入队）；`estop` 随时可插入并中止当前轨迹；**流模式进行中 `move`/`goto` 暂缓**（上位机流优先），**急停锁定期间被拒绝**。

#### `move` —— 相对转动

```
move <腿1-4> <J1-J4> <cw|ccw> <角度°> [时长s]
```

| 参数 | 取值 | 说明 |
|---|---|---|
| `<腿>` | 1~4 | 目标腿 |
| `<关节>` | J1~J4 | 目标关节 |
| `<cw\|ccw>` | 二选一 | 顺时针 / 逆时针（ccw 为正方向） |
| `<角度°>` | 0.1~360 | 转动幅度（关节端度） |
| `[时长s]`（可选） | 0.1~60，缺省 2.0 | 轨迹时长，越短运动越快 |

以**当前实际角度**为基准叠加。示例：

```
move 1 J1 cw 90        # 腿1 J1 顺时针转 90°（默认 2s）
move 2 J3 ccw 30 1.5   # 腿2 J3 逆时针转 30°，1.5s 完成
move 4 J2 cw 15 0.5    # 腿4 J2 顺时针转 15°，0.5s（较快）
```

#### `goto` —— 绝对关节角

```
goto <腿1-4> <J1-J4> <角度°> [时长s]
```

转到指定**绝对关节角**（0° = 标零位置）：角度带符号（正 = ccw、负 = cw），范围 ±360°，仍受软件限位约束；时长规则同 `move`。示例：

```
goto 1 J1 45      # 腿1 J1 转到 +45°
goto 3 J3 -20     # 腿3 J3 转到 -20°
goto 2 J2 0 1     # 腿2 J2 用 1s 回零位
```

#### `pos` —— 位置查询

```
pos [腿1-4]
```

打印指定腿（缺省 = 全部已启用腿）四个关节的**实际角度**（关节端度）。数据来自 10Hz 反馈缓存，**零总线流量**，流模式中也随时可用；电机离线时该关节标注 `(离线)`。发 `move` 前先 `pos`，可预估相对转动的落点。

#### `stat` —— 状态查询

打印流模式（流模式/空闲）、样点在线、急停锁定状态，以及每条腿的上位机收帧数、丢弃数（越限/锁定）、各电机在线标记（✓/×）与**使能状态**（已解抱闸/抱闸）。联调时用于核对收帧率：流模式下每条腿应 ≈40 帧/s（4 关节 × 10Hz）。

#### `arm` / `disarm` —— 使能（解抱闸）与去使能

> **协议没有独立的「使能」命令**：只有 `setTargetCurrent(0)`（功能码 `0x1C` 写 0）= **使能 + 零转矩 = 解抱闸**；而 `0x02` stop = **去使能 + 抱闸**。电机**上电默认去使能**，此时上位机 `0x1E` 位置帧会被正常接收、通过限位检查、写入目标缓存，但**电机不会动**。

- `arm`：对全部已启用腿执行「读实测位置 → 清错误位 → 逐电机 `setTargetCurrent(0)` 解抱闸 → 切回位置模式保持当前姿态」，顺带解除急停锁定。启动时**已自动执行一次**（在 busThread 起线程前，天然独占 CAN），故正常流程无需手敲。
- `disarm`：全部电机 `0x02` 抱闸并进入锁定状态（等同 `estop`）。

> ⚠️ 抱闸解除瞬间腿会失去保持力矩（位置环接手需要一个周期）；若腿未悬空或负载大，请确认支撑安全。

#### `estop` / `resume` —— 急停与解除

- `estop`：向全部已启用腿的电机发送 `0x02`（去使能并抱闸），同时退出流模式、清 `armed` 并进入**锁定**状态——锁定期间拒绝一切新目标（上位机 `0x1E` 帧计入丢弃、`move`/`goto` 被拒绝）。响应延迟：轨迹执行中 ≤5ms、空闲 ≤50ms。
- `resume`：解除急停锁定**并自动重新使能（解抱闸）**。电机在重新使能后保持当前实测姿态，等待新目标（上位机流或 `move`/`goto`）到达。

#### `help` / `quit`

`help`（或 `h`）重新打印命令格式与协议摘要；`quit`（或 `q`）退出（见第 3 节第 4 步）。

#### 终端反馈格式

| 输出 | 含义 |
|---|---|
| `[腿N][Jn] 相对转动/绝对转角 x° → 目标 y°（限位 …），轨迹 …s，其余关节保持` | 轨迹开始 |
| `[腿N][Jn][拒绝] 目标 …° 超出限位 …` | 限位拒绝，未下发任何运动指令 |
| `[腿N][Jn] 到位确认：目标 … 实际 … 误差 … ✓` | 成功到位（误差 < 0.02 rad，关节端） |
| `[腿N][Jn][警告] 到位超时…` | 3s 内未到位（电机仍由位置环保持目标） |
| `[腿N][Jn][中止] 收到急停，轨迹中止` | 轨迹被 `estop` 打断 |
| `[流] 收到首帧 0x1E：进入流模式` | 上位机开始下发目标 |
| `[流][警告] …s 未收到样点，退出流模式` | 断流保护触发（电机保持不抱闸） |
| `[协议][丢弃] ID… 目标 … 超出限位` | 上位机目标越限，该关节目标被丢弃（每关节仅首次打印） |
| `[格式] …` | 命令语法错误（附具体原因） |

### 5. 执行流水线（内置行为）

**上位机流模式**（主功能）：

1. 上位机线程收到 ID n 的 `[0x1E][int32 = rad × 10000]` 帧 → 解出关节端目标角；
2. 急停锁定中 → 计入丢弃；越限 → 仅该关节计入丢弃（同一样点其余关节不受影响）；
3. 通过检查 → 写入该腿的原子目标缓存（覆盖式，不排队），首帧即进入流模式；
4. 对应总线线程以 5ms 周期把最新目标换算为电机端角/编码器 cnt 并重发 `0x1E` 写帧（纯写不等回复）；
5. 电机固件内部 PD（KP=4000/KD=60）平滑追踪 10Hz 阶梯目标，下位机**不做软件插值**；
6. 每 100ms 逐电机发 `0x41` 读回实际位置写入反馈缓存 → 上位机线程以 10Hz 回发 ID 1~16 反馈帧、200ms 发心跳。

**键盘 move/goto**（本机调试）：

1. **读当前角**：`0x41` 逐关节读回（任一电机无响应 → 命令失败，不下发任何运动）；
2. **算目标**：相对命令 = 当前角 + 角度；绝对命令 = 给定值；
3. **限位检查**：越界直接拒绝，电机保持原位；
4. **平滑轨迹**：五次多项式插值（位置/速度/加速度连续，起停无冲击），5ms 周期下发；**其余 3 个关节每周期重发当前位置**（运动中腿不塌）；
5. **急停/流模式检查**：轨迹循环每 5ms 检查一次急停标志，流模式一旦启动即让出；
6. **到位确认**：轮询实际位置，误差 < 0.02 rad（关节端）即成功；3s 超时仅告警；
7. **到位后不抱闸**：电机内部位置环持续保持目标；仅 `estop`/`quit` 才抱闸。

### 6. 按需定制（源码常量，集中在文件头）

| 常量 | 默认 | 用途 |
|---|---|---|
| `kJointDirSign` | `{1,1,1,1}` | 关节方向符号（+1 = 关节端正方向对应电机端正方向）。基准为 `taihu_singleleg_step_xCAN`（现场已验证）：该 demo 按电机输出端下发关键帧「逆时针为正」，`{0,180,180,0}` 中 ID2/ID3 同为 +180°，说明两个 P 型锥齿轮电机端同向。现场某关节（锥齿轮装反）方向相反时，只改对应元素为 `-1`（限位是关节端语义，不用跟着改） |
| `kJointLimitDeg` | 见第 2 节 | 软件限位（**关节端**度）；判定与丢弃均按关节端 |
| `kMotorRadPerJointRad` | `{1,2,2,1}` | 关节端→电机端位置换算系数（Y 直驱 =1；P 锥齿轮 1:2 =2）。电机端等效限位 = `kJointLimitDeg` × 此系数 |
| `kJointGearRatios` | `{101,162,162,101}` | 关节端总减速比，**只参与速度换算**（位置换算是 `radToCnt/cntToRad`，262144 cnt = 2π 输出端，不含减速比）。故与 xCAN demo 的 `{101,81,81,101}` 差异不影响位置/方向/限位 |
| `kMotorIds` | `{1,2,3,4}` | 每腿电机 CAN ID；现场不符时建议用 `taihu_motor_tools` 菜单 `3` 把 ID 改回 1~4 |
| `kPositionKp` / `kPositionKd` | 4000 / 60 | 位置环增益（过大抖动、过小无力） |
| `kDefaultMoveSec` | 2.0 | 不带时长参数时的默认轨迹时长 |
| `kAngleScale` | 10000.0 | 上位机角度定点标度（1 LSB = 0.0001 rad ≈ 0.0057°）；改动需与上位机同步 |
| `kJointIdBase` | 1 | 上位机帧 ID 起始值（ID = 基址 + (腿−1)×4 + 关节−1） |
| `kHostStreamHz` | 10 | 上位机目标流频率，同时决定反馈频率（`kFeedbackMs = 1000/kHostStreamHz`） |
| `kReadPeriodMs` | 100 | 电机实际位置读取周期（阻塞式往返，不宜过密） |
| `kStreamTimeoutMs` | 1000 | 断流判定门限，超时自动退出流模式 |
| `kHeartbeatMs` | 200 | 心跳周期（上位机建议 600ms 超时判失联） |

修改后 `cmake --build build` 重新编译即可生效。

### 7. 首次上机建议与常见问题

**建议流程**：① 腿悬空、供电正常 → ② 启动时**跳过上位机接口**（直接回车），`pos` 确认全部电机在线且角度合理 → ③ 小角度试方向（`move 1 J1 ccw 10 1`）→ ④ 方向反了改 `kJointDirSign` 对应元素并重编译 → ⑤ 逐腿逐关节验证（重点 J2/J3 锥齿轮方向）→ ⑥ 上位机总线另开一路（如 `sudo ./build/kcanctl up can4 1000000`），下位机重启时上位机接口输 `can4` → ⑦ 另开终端跑上位机调试程序 `python3 tools/up_walk_control_V1.py --can can4`，它会**先只做链路检测**（等心跳 + 主动 ping），在下位机终端用 `stat` 核对收帧、`candump can4` 确认帧到达后，回到上位机终端按回车才开始回放 → ⑧ 先 `--only-mapped` 或大幅裁剪的小幅度轨迹流，再逐步放大（先用 `--dry-run` 确认限位预检查与相邻采样点跳变）。上位机侧参数与四阶段流程详见 [虚拟上位机 `up_walk_control_V1.py` 使用说明](#虚拟上位机-up_walk_control_v1py-使用说明)。

| 现象 | 原因与处理 |
|---|---|
| `[腿N][错误] 打开 canN 失败` | 通道未 up：外部终端 `sudo ./build/kcanctl up canN 1000000` |
| `[协议][错误] 打开 canX 失败，上位机链路不可用` | 上位机通道未 up 或接口名填错；键盘命令仍可用 |
| `[错误] 上位机接口不能与电机总线 canN 共用` | 上位机必须接独立总线（见第 2 节 ID 映射说明） |
| 在线性探测显示 `×` | 电机离线：查供电/接线/终端电阻；用 `taihu_motor_tools` 菜单 `1` 扫描实际 CAN ID |
| 命令被 `[拒绝]` | 目标角超出软件限位：先 `pos` 看当前角，减小幅度或 `goto` 到限位内的绝对角 |
| 命令被 `[暂缓]` | 上位机流模式进行中：让上位机停发 1s 或发 `0x02` 退出流模式 |
| `stat` 收帧数不增长 | 上位机未发送 / 接错总线 / ID 不在 1~16；用 `candump <上位机接口>` 排查 |
| `stat` 丢弃数增长 | 上位机目标越限或处于急停锁定；`pos` 核对当前角，`resume` 解除锁定 |
| 转动方向与命令相反 | 该关节的 `kJointDirSign` 应取 `-1`（见第 6 节） |
| `[警告] 到位超时` | 负载过大 / 增益偏低 / 时长过短；电机仍在位置保持，`pos` 核对实际位置 |
| J2/J3 转角只有一半 | 角度直接按关节端输入即可，程序内置 ×2 换算，勿预先乘 2 |

---

## 上位机关节位置流协议（10Hz CAN）

把本机当作**下位机**使用：上位机（RL 策略/步态规划）通过 CAN 卡接到本机**独立的**一路 CAN 总线，以 10Hz 下发 16 个关节的预期位置，本机按帧 ID 拆分到 4 条电机总线并以 5ms 周期下发到真电机。帧结构与钛虎电机写命令帧**同构**（`DLC=5`，`[功能码][int32 小端]`），上位机发送代码可直接复用控制真电机的写法。

- **物理层**：CAN 2.0A 标准帧，1Mbps；上位机 CAN 卡接本机**独立接口**（如 `can4`，启动时输入），**不可**与 `can0~can3` 共用，两端 120Ω 终端电阻。
- **帧 ID 映射**：ID 1~16 = (腿号−1)×4 + 关节号；ID 1~4 → `can0` 电机 1~4，ID 5~8 → `can1` 电机 1~4，ID 9~12 → `can2`，ID 13~16 → `can3`。
- **上位机 → 下位机**：
  - `0x1E` 单关节目标帧（DLC=5）：`[0x1E][int32 小端 = 关节端 rad × 10000]`，1 LSB = 0.0001 rad ≈ 0.0057°，**关节端绝对角**、ccw 为正，10Hz 周期发满 16 帧；
  - `0x02`（DLC=1，ID 1~16 任一）退出流模式，电机保持不抱闸；
  - `0x03`（DLC=1）**清除软限位报警锁定**，整机恢复接收目标；
  - `0x41`（DLC=1）状态请求 → 下位机立即回同 ID 反馈帧。
- **下位机 → 上位机**：
  - `0x41` 反馈（ID 1~16，DLC=5）：该关节**实际位置** rad × 10000，10Hz 周期主动发送（亦响应请求）；
  - 心跳 `0x161`（200ms 周期，DLC=1）：bit0=流模式中，bit1=最近 1s 有新样点，bit2=**软限位报警锁定中**；
  - 软限位报警 `0x162`（200ms 周期重发，DLC=8）：见下表。ID 取 `0x162` 是为了避开关节 ID 1~16 与心跳 `0x161`。

| 字节 | 内容 | 说明 |
|---|---|---|
| `data[0]` | `0x62` | 报警功能码（与 `0x162` 的低字节呼应，便于日志识别） |
| `data[1]` | 关节 ID | **1~16**（(腿号−1)×4 + 关节号），与上位机帧 ID 同命名空间 |
| `data[2]` | 侧别 | `0x01` = 撞上限，`0x02` = 撞下限 |
| `data[3]` | `0x00` | 保留 |
| `data[4..7]` | `int32` 小端 | 报警时该关节的**当前实际角** rad × 10000（每次重发刷新） |

- **保护**：
  - **越限目标钳位**：上位机目标超出软件限位时，**钳位到限位值**下发（关节走到限位并停住，不会卡住不动），首帧钳位会打印一次提示；
  - **触限报警 + 整机锁定**：流模式中若某关节**实际角**到达限位（进入限位内 `0.5°` 判定），立即整机锁定——丢弃所有新目标、心跳 bit2 置位、以 `0x162` 上报该关节 ID 与侧别（200ms 重发）。需键盘 `clear` 或上位机 `[0x03]` 解除，且需关节**退离限位 2°** 才会再次触发（防抖）；
  - 断流 >1s 自动退出流模式（电机保持）；电机 `0x41` 读取连续 3 次超时则该关节读操作退避 1s（不阻塞 5ms 控制环）；`estop` 立即抱闸、退出流模式并锁定，直到 `resume`。

协议全文（系统架构图、帧格式字节表、时序图、换算公式、限位表、异常处理、上位机 C 参考实现）见 **[docs/step_protocol.pdf](docs/step_protocol.pdf)**（由 [`tools/gen_protocol_pdf.py`](tools/gen_protocol_pdf.py) 生成）。联调用 `stat` 命令查看各腿收帧数/丢弃数/电机在线，用 `candump` 分别观察上位机总线与电机总线。

### 上位机调试程序 `up_walk_control_V1.py`

[`tools/up_walk_control_V1.py`](tools/up_walk_control_V1.py:1) 是配套的**虚拟上位机**：在没有真实策略上位机（RK3588 / RL 策略）的情况下，读轨迹表按 10Hz 发关节目标，用来**先打通两端通讯、再回放固定轨迹**做联调。

安装、完整命令、参数表、轨迹格式、场景速查与排错，见专章 **[虚拟上位机 `up_walk_control_V1.py` 使用说明](#虚拟上位机-up_walk_control_v1py-使用说明)**。

---

## 虚拟上位机 `up_walk_control_V1.py` 使用说明

> **本机目录下运行**：脚本默认轨迹路径是相对路径 `step_position/step_make.xlsx`，请先 `cd` 到项目根目录再执行。

### 1. 它是什么、解决什么问题

真实上位机是 **RK3588（aarch64）** 上的策略/轨迹规划程序，通过 CAN 给下位机 `walk_control_V1` 发 16 个关节的目标位置。在真实上位机没到位、或要单独验证「链路 + 下位机拆分 + 电机执行」这条链时，用这个脚本**顶替真实上位机**：

```
tools/up_walk_control_V1.py    ← 虚拟上位机（读 xlsx 当策略输出）
        │  can4，1Mbps，ID 1~16，DLC=5，[0x1E][int32 LE]，10Hz
        ▼
build/walk_control_V1          ← 下位机（本机），按 ID 拆到 can0~can3
        │  各电机原生位置命令，5ms 重发最新目标
        ▼
16 个钛虎关节模组
```

它只做三件事：**读表 → 打通链路 → 按 10Hz 回放**；不参与控制算法，也不做符号/减速比换算（角一律是**关节端绝对角，rad，ccw 为正**）。

### 2. 依赖与安装

| 依赖 | 说明 | 安装 |
|------|------|------|
| Python 3.7+ | 标准库 `socket/select/struct/threading/argparse` | 系统自带 |
| **原生 SocketCAN** | 用 `socket.PF_CAN + CAN_RAW` 直接发帧 | 内核自带，**不需要 python-can** |
| `openpyxl` | 唯一的第三方依赖，读 `.xlsx` | 见下 |

```bash
# 方式一（Ubuntu 24.04 推荐，走系统包，不污染 pip 环境）
sudo apt install -y python3-openpyxl

# 方式二（pip；Ubuntu 23.04+ 有 PEP 668 外部管理限制，需加 --break-system-packages）
pip3 install --user --break-system-packages openpyxl
```

> 装完可自检：`python3 -c "import openpyxl, socket; print(openpyxl.__version__, hasattr(socket,'AF_CAN'))"`，应输出形如 `3.1.5 True`。
> 缺 openpyxl 时脚本会直接给出这条安装命令并退出，不会静默降级。

### 3. 两个终端逐条命令（最常用路径）

前置：鲲弘驱动已装且 `lsmod | grep kcan` 有输出，设备已插好（`ip -brief link show type can` 能看到 `can0~can5`）。

**终端 A —— 下位机**

```bash
cd /home/sheng/workspaces/Xiaoyuan_2

# ① 手动逐通道打开（必须逐条执行；本项目不做开机自动全开）
sudo ./build/kcanctl up can0 1000000     # 腿 1 电机总线（ID 1~4  → 电机 1~4）
sudo ./build/kcanctl up can1 1000000     # 腿 2（ID 5~8）
sudo ./build/kcanctl up can2 1000000     # 腿 3（ID 9~12）
sudo ./build/kcanctl up can3 1000000     # 腿 4（ID 13~16）
sudo ./build/kcanctl up can4 1000000     # ★ 上位机链路（必须是独立接口，不能与 can0~can3 共用）

./build/kcanctl list                     # 可选：确认 5 路都是 UP / 1Mbps

# ② 启动下位机
./build/walk_control_V1
#   提示「腿号（回车=全部 4 腿，或 1~4）」  → 直接回车
#   提示「上位机接口名（回车=不启用 …）」   → 输入 can4   ★ 必须与终端 B 的 --can 一致
```

**终端 B —— 虚拟上位机**

```bash
cd /home/sheng/workspaces/Xiaoyuan_2

# 建议第一次先做无硬件自测（不打开 CAN，只校验轨迹/限位/节拍）
python3 tools/up_walk_control_V1.py --dry-run

# 正式联调
python3 tools/up_walk_control_V1.py --can can4
```

脚本会依次打印 `[1/4] 加载轨迹` → `[2/4] 打开 CAN 接口` → `[3/4] 链路检测`，**链路通了才出现 `[4/4] 按回车开始发送`**；此时先别急着回车，按下节第 8 条的核对清单确认后再按。

### 4. 命令参数速查

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `--can NAME` | `can4` | 上位机链路接口名。**必须与下位机启动时输入的一致**，且不能是 `can0~can3` |
| `--xlsx PATH` | `step_position/step_make.xlsx` | 轨迹文件路径 |
| `--sheet NAME` | 第一个工作表 | 工作表名（一个 xlsx 里有多个轨迹表时用） |
| `--hz F` | `10.0` | 发送频率。**协议约定 10Hz**，改了要和下位机断流门限（1s）一起评估 |
| `--loop N` | `1` | 回放遍数；`0` = 无限循环直到 Ctrl+C |
| `--only-mapped` | 关 | 只发 xlsx 里**存在对应列**（表头能识别）的关节，缺列的关节完全不发。默认是把 16 个 ID 全发一遍，缺列补 `0.0 rad`（会**运动到零位**）。注意：列存在但单元格为空仍按 `0.0 rad` 发 |
| `--wait-link S` | `10.0` | `[3/4]` 等心跳的最长秒数 |
| `--auto` | 关 | 跳过 `[4/4]` 回车确认，链路通了立即发送（脚本化/无人值守用，慎用） |
| `--dry-run` | 关 | 不打开 CAN、不发帧，只解析 xlsx + 模拟节拍；用于上机前自检 |
| `--keep-on-limit` | 关 | 收到下位机软限位报警（`0x162`）后**仍继续发送**。默认是**立即停发**并打印报警明细——因为下位机已整机锁定，继续发的目标会被丢弃 |
| `-h` / `--help` | — | 打印内置示例 |

**退出码**：`0` 正常完成或用户取消；`1` 加载轨迹失败 / 打开 CAN 接口失败；`2` 链路未打通（此时**一个目标帧都没发**）。

**中断行为**：第一次 `Ctrl+C` → 停止发送并**自动补发 `[0x02]`** 通知下位机退出流模式（电机保持当前目标，**不抱闸**）；第二次 `Ctrl+C` → 立即强制退出。

**软限位报警行为**：回放期间后台监视线程持续解析 `0x162`。**一旦收到报警帧，默认立即停发**（下位机已整机锁定，继续发毫无意义），并打印：

```
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[软限位报警] 下位机上报：ID3  L1_J3    到达下限 -30.0°（实际 -30.02°，越限 -0.02°）
             触发时刻 t=12.30s（第 124/768 点）
             已停止发送（下位机整机锁定，需键盘 clear 或本程序下次启动时发 [0x03] 解除）
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
```

随后 `finally` 仍会补发 `[0x02]` 退流，最终统计里会打印「软限位报警 N 帧」与解除提示。启动时若检测到下位机仍处锁定状态，`[3/4]` 阶段会**自动发一次 `[0x03]`** 尝试解除，避免上一轮的锁挡住本次回放。

### 5. 轨迹文件格式

标准格式（`step_position/step_make.xlsx` 即此结构，单表 `Joint samples`，A1:Q202 = 201 点 @0.1s = 20.0s）：

| 列 | 表头 | 说明 |
|----|------|------|
| A | `Time (s)` | 时间戳，仅用于打印与时长统计；实际发送按 `--hz` 重新定时 |
| B~Q | `L1_J1` … `L4_J4` | 16 个关节的**关节端绝对角，单位 rad，正 = 逆时针** |

- **列→帧 ID 映射由表头名自动识别，不按列序猜**：`L{腿}_J{关节}` → `ID = (腿−1)×4 + 关节`。也兼容 `ID1 … ID16` 写法；带 `(deg)` 后缀的列会被自动跳过（避免把"度"当"弧度"发出去）。
- **表头对不上直接报错退出**，并列出「实际表头」与「期望格式」，不会带着错误映射继续发。
- **缺列的关节**：默认补 `0.0 rad`（会被拉到零位）；加 `--only-mapped` 则这些 ID 完全不发。**注意「列存在但单元格为空」也算 0.0 rad**，不受 `--only-mapped` 影响。
- **限位（关节端）**：J1 ±90°、J2 ±120°、J3 **−30°~+180°**、J4 ±90°（电机端等效 J2 ±240°、J3 −60°~+360°）。脚本在 `[1/4]` 阶段就逐关节报越限点数——下位机对越限帧是**逐帧丢弃**的，关节会直接卡住不动，所以务必先看这条提示。
- **相邻采样点跳变**：脚本会算最大关节变化并打印。超过 **0.035 rad（≈2°/帧）** 会提示——不是错误，但电机 PD 追踪会滞后，建议插值加密或降低 `--hz` 来放宽。

**现有轨迹文件**

| 文件 | 点数 / 时长 | 表 | 说明 |
|------|------------|----|------|
| `step_position/step_make.xlsx` | 201 点 @0.1s / 20.0s | `Joint samples` | 标准格式原始样例，仅腿 1、腿 2 的 J2/J3 有数据 |
| `step_position/step_make_V2.xlsx` | 768 点 @0.1s / 76.7s | `Joint samples` | 由 YPPY 轨迹转换而来：10s 起站立过渡 + 循环步态；J1 已饱和裁剪到 ±90° |

**从 YPPY 轨迹转换**

`step_position/YPPY_joint_trajectories_0p1s_with_10s_transition.xlsx` 是上游生成的 YPPY 命名轨迹（表 `Angles_rad` / `Angles_deg`，表头 `Time_s | LF_Y1_rad … RR_Y4_rad`），**不能直接喂给上位机**（表头认不出、J1 峰值 ±139° 超过 ±90° 限位）。用转换脚本处理：

```bash
python3 tools/convert_yppy_to_step_make.py            # 默认生成 step_position/step_make_V2.xlsx
python3 tools/convert_yppy_to_step_make.py --src <源文件> --dst <输出文件> --sheet Angles_rad
```

转换做三件事，并打印「每列裁剪点数 + 限位核对表」：

1. **改表头**：`LF/RF/LR/RR + Y1/P2/P3/Y4` → `L{腿}_J{关节}`，腿序按现场接线 **LF→L1、RF→L2、LR→L3、RR→L4**（`LEG_MAP` 可改）。
2. **单位统一**：只读 `Angles_rad` 表；`Angles_deg` 表会被拒绝（当成 rad 用会放大 57 倍）。
3. **J1 限位饱和裁剪**：超出 ±90° 的点削平到 ±90°（源轨迹 Y1 峰值到 ±139.19°，不裁剪的话下位机会逐帧丢弃，J1 在 t≈11s 后一直卡在限位上）。裁剪点数会逐列打印，例如 `L1_J1 498 点`。

> ⚠️ 裁剪后 J1 会**贴身贴在 ±90° 限位上**约 65% 的时间——这是为了让轨迹能跑起来的折中方案，姿态与原设计不同。如果机械真的允许更大偏航，应改 `examples/walk_control_V1.cpp` 的 `kJointLimitDeg[0]`、`tools/up_walk_control_V1.py` 的 `JOINT_LIMIT_DEG[1]` 与本脚本三处，而不是依赖裁剪。

### 6. 四阶段运行流程

| 阶段 | 动作 | 失败时 |
|------|------|--------|
| `[1/4] 加载轨迹` | 读 xlsx、报采样点数/时长/平均间隔、最大跳变、**离线限位预检查**、本次将发送的 ID 列表 | 报错 `return 1` |
| `[2/4] 打开 CAN 接口` | `socket(PF_CAN, SOCK_RAW, CAN_RAW)` + `bind` 到 `--can`，并启动后台监视线程 | 打印三条排查建议，`return 1` |
| `[3/4] 链路检测` | ① 被动等心跳 `0x161`（≤ `--wait-link` 秒）② 主动向 16 个 ID 发 `[0x41]` 状态请求，统计回复数 → 打印**实际位置反馈表** ③ 若下位机处于软限位锁定，自动发 `[0x03]` 尝试解除 | 双向都不通则 `return 2`，**不发任何目标帧** |
| `[4/4] 回放` | 等回车（或 `--auto`）后按 `--hz` 逐点发 16 帧；期间监听 `0x162` 报警，**一旦收到立即停发**并打印报警明细（`--keep-on-limit` 可改为继续发）；`finally` 里必发 `[0x02]` 退流；最后打印统计（发/收帧数、心跳次数、软限位报警、反馈关节） | — |

> `[3/4]` 里部分 ID 无回复**不会中止**——常见于这些腿没在下位机启用或电机离线，脚本会明确提示「不影响其余关节」，你可以只用 `--only-mapped` 发能动的那些。

> **`--dry-run` 的行为差异**：`[2/4]`、`[3/4]` 直接跳过，`[4/4]` **不等回车**、也不写 CAN，只按 `--hz` 模拟节拍并每秒打印一次进度。因此 `--dry-run` 只能验证「xlsx 解析 + 限位 + 跳变 + 节拍」，**不能**验证通讯（想快速跑完可以临时加 `--hz 500` 之类加速）。

### 7. 常用场景速查

```bash
# ① 完全无硬件，只想验证 xlsx 有没有问题（限位/跳变/时长）
python3 tools/up_walk_control_V1.py --dry-run

# ② 标准联调（先握手，人工确认后回放一遍）
python3 tools/up_walk_control_V1.py --can can4

# ②' 回放 YPPY 转换来的 76.7s 站立+步态轨迹
python3 tools/up_walk_control_V1.py --can can4 --xlsx step_position/step_make_V2.xlsx

# ③ 只发已经写进 xlsx 的关节（未写列的关节不发，避免被拉回零位）
python3 tools/up_walk_control_V1.py --can can4 --only-mapped

# ④ 反复回放同一段轨迹做疲劳/温升观察（Ctrl+C 停）
python3 tools/up_walk_control_V1.py --can can4 --loop 0

# ⑤ 脚本化 / 无人值守（链路通就发，不等回车）
python3 tools/up_walk_control_V1.py --can can4 --auto

# ⑥ 换成上位机接在 can5 的情况
python3 tools/up_walk_control_V1.py --can can5

# ⑦ 用另一个轨迹文件 / 指定工作表
python3 tools/up_walk_control_V1.py --can can4 \
    --xlsx step_position/step_make.xlsx --sheet "Joint samples"

# ⑧ 轨迹采样率和发送频率不一致时（脚本会提示，按 --hz 重新定时）
python3 tools/up_walk_control_V1.py --can can4 --hz 20
```

> **关于 `--only-mapped` 与当前的 `step_make.xlsx`**：该表 **16 列齐全**，所以 `--only-mapped` 在本文件上**不会减少发送的 ID**（16 个 ID 照样全发），其中没有数据的 12 个关节发的是 `0.0 rad`——若它们当前不在零位，开始发送后会**运动到零位**。想让某条腿完全不动，做法是把该腿对应列从表里**删掉**，再配合 `--only-mapped` 运行。

### 8. 联调核对：怎么确认真的打通了

`[3/4]` 阶段跑完后，**先按下表核对再按回车**：

| 在哪看 | 看什么 | 期望 |
|--------|--------|------|
| 终端 B 脚本输出 | `✓ N/16 个 ID 有回复（双向通讯打通）` | ≥ 你要动的关节数 |
| 终端 B 脚本输出 | 位置反馈表的数值 | 是**真实角度**（不是恒 0、也不是全 0 卡住） |
| 终端 A 下位机 `stat` | 各腿「收帧 / 丢弃 / 电机在线」 | 回车开始发送后**收帧数持续增长**，丢弃数不涨 |
| 第三终端 `candump can4` | 帧流向 | 发送期看到大量 `1E …`；始终有 `161` 心跳（200ms） |
| 第三终端 `candump can0` | 电机总线 | 看到电机原生位置帧在跑（5ms 级） |

```bash
# 第三终端（推荐常开，用于旁路观察，不影响任何一方）
candump -td can4          # 上位机链路：0x161 心跳 + 1~16 的 1E/41 帧
candump -td can0          # 电机总线 1（腿 1）
./build/kcanctl list      # 各通道 UP / 速率 / 错误计数
ip -details link show can4 | grep 'can state'   # 期望 ERROR-ACTIVE
```

> ⚠️ **最容易踩的坑**：把上位机也接到 `can0~can3` 上。那 4 路是**电机总线**，帧 ID 1~4 会被电机固件当自己的命令执行，直接真转。上位机必须占**独立接口**（本项目约定 `can4` 或 `can5`）。

> ⚠️ **未接电机的腿不要在下位机启用**；轨迹里的越限列要提前裁掉；上真机前务必先跑一遍 `--dry-run`。

### 9. 排错对照表

| 现象 | 原因 / 处理 |
|------|-------------|
| `缺少 openpyxl，请先安装：…` | 按第 2 条安装 `python3-openpyxl` |
| `[1/4]` 报「表头无法识别」 | 表头必须含 `Time (s)` 与 `L{腿}_J{关节}`（或 `ID{N}`）；脚本会打印实际表头，对照改表头 |
| `[限位警告] … N/M 点越限` | xlsx 该列超出关节端限位，会被下位机**逐帧丢弃**导致关节卡住；裁到限位内。J3 下限只有 −30°，最容易越限 |
| `[2/4] [错误] 打开 canX 失败` | ① 接口名拼错（`ip -brief link show type can`）② 通道没 up（`sudo ./build/kcanctl up canX 1000000`）③ 与下位机输入的上位机接口名不一致 |
| `[3/4]` 一直等到超时，`✗ 无任何回复` → 退出码 2 | 下位机没启动 / 下位机启动时**没填上位机接口**（填了空回车）/ 两边接口不同 / 波特率不是 1M / `can4` 物理未接线或对端无电。用 `candump can4` 看有没有 `161` |
| 有心跳但 `0/16 个 ID 有回复` | 下位机没启用任何腿（`enabled_count == 0` 会直接 `return -1`）；回到终端 A 用 `stat` 确认电机在线 |
| 发完一遍后关节不动 | 电机**未使能（抱闸未解）**——下位机启动/`arm` 时才解抱闸；`pos` 看角度是否真的在变 |
| 关节只朝一个方向走 / 方向反了 | 关节方向符号问题（下位机 `kJointDirSign`），不是本脚本的事；本脚本不做符号翻转 |
| `stat` 收帧数增长但丢弃数也涨 | 目标越限或处于 `estop` 锁定；`pos` 核对当前角，`resume` 解锁 |
| 关节点动一下就卡住不动 | 下一帧目标越限被丢弃，关节保持上一目标；看 `[1/4]` 的限位警告 |
| `Ctrl+C` 后关节还保持力矩 | 正常设计：`0x02` 只退流模式，**不抱闸**；要抱闸在下位机敲 `estop` |
| 回放结束后 `candump can4` 只剩心跳 | 正常：已发 `[0x02]` 退流，下位机不再发 10Hz 位置帧，但心跳 `0x161` 会继续 |

---

## 日志与绘图

运行示例时，会把各电机的状态（时间戳、位置、速度、电流、电压、错误）以 CSV 写入 [`record/motor_log.csv`](record/motor_log.csv:1)（覆盖模式，不入库）。多总线示例（`taihu_singleleg_step_xCAN`）按总线分文件：`record/motor_log_<接口名>.csv`。

### 采样周期

日志采样周期由 [`config.h`](include/taihu/config.h:14) 配置：

- [`kLoopPeriodMs`](include/taihu/config.h:14) = 5：控制周期（5ms）
- [`kLogSamplePeriodMs`](include/taihu/config.h:21) = 100：日志采样周期（0.1s）

> CAN 总线半双工且为单通道，采样与控制无法物理独立，采样周期拉长为 0.1s 以减少采样对总线的占用、避免争抢控制周期。（鲲弘模块支持 6 路独立 CAN，若电机分散到不同通道，可缩短采样周期）

### 绘制曲线图

```bash
python3 tools/plot_motor_log.py                    # 默认读 record/motor_log.csv，输出到 record/
python3 tools/plot_motor_log.py record/motor_log.csv -o record
python3 tools/plot_motor_log.py record/motor_log_can0.csv --num-steps 3   # xCAN 日志，运行时走了 3 步
```

依赖 `python3 + matplotlib`。生成 4 张图（横轴均为时间，不同电机 ID 用不同颜色区分）：

- `position.png`：位置曲线（叠加各电机同色虚线的**目标位置**做对照）
- `velocity.png`：速度曲线
- `current.png`：电流曲线（含四电机**瞬时电流之和**曲线）
- `error.png`：错误状态曲线

位置图的目标轨迹支持两种步态模型（[`tools/plot_motor_log.py`](tools/plot_motor_log.py:47)）：

| 模型 | 复现自 | 轨迹 | 默认阶段时长 |
|------|--------|------|------------|
| `singleleg` | `examples/taihu_singleleg_step.cpp` | 6 阶段单腿步态（与 `kStepSec` 对应） | 10 s |
| `xcan` | `examples/taihu_singleleg_step_xCAN.cpp` | 一步 3 阶段（零位保持 / ID2,3 0°→+180° / +180°→0°）重复 N 步（与 `kPhaseSec`、运行时 N 对应） | 1 s（cpp 中 `kPhaseSec`；默认「到位即切换」模式下实际阶段 ≥ 1 s，必要时用 `--phase-sec`/`--total-sec` 覆盖） |

相关命令行参数：

- `--gait {auto,singleleg,xcan}`：步态模型，默认 `auto`（文件名含 `_can` → xcan，否则 singleleg）；
- `--num-steps N`：xcan 步数 N，须与运行时输入一致（默认 1）；
- `--phase-sec 秒`：每阶段时长，改了 cpp 的 `kStepSec`/`kPhaseSec` 后同步传入；
- `--total-sec 秒`：直接指定总时长（优先于阶段时长×阶段数）。

---

## 协议要点

依据《电机协议_双编_Can版本_20250623.xlsx》与《钛虎C1关节电机通讯使用说明_0804.pdf》：

- **写命令帧**：`[功能码(1 字节)][数据(4 字节, 小端 Int32)]`，共 5 字节
- **读命令帧**：`[功能码(1 字节)]`，共 1 字节，电机返回 5 字节（第 1 字节功能码回显 + 4 字节数据）
- **三合一读**：`0x41` 返回 8 字节（电流 int16 + 速度 int16 + 位置 int32，无功能码回显）

| 功能 | 功能码 | 长度 |
|------|--------|------|
| 停止电机（去使能/抱闸） | `0x02` | 1 字节 |
| 清除错误 | `0x0B` | 1 字节 |
| 保存参数 | `0x0E` | 1 字节 |
| 恢复出厂 | `0x0F` | 1 字节 |
| 设置目标电流 | `0x1C` | 5 字节 |
| 设置目标位置 | `0x1E` | 5 字节 |
| 修改 CAN ID | `0x2E` | 5 字节 |
| 编码器归零 | `0x50` | 5 字节 |
| 设置位置偏移 | `0x53` | 5 字节 |
| 获取当前位置 | `0x08` | 1 字节（返回 5 字节） |
| 获取电流速度位置 | `0x41` | 1 字节（返回 8 字节） |

- **位置换算**：`cnt = rad / (2π) × 262144`（双编码器，输出端）
- **速度换算**：`编码 = (rad/s) / (2π) × 减速比 × 100`（单位 0.01 Hz，电机端）
- **刹车（抱闸）**：CAN 协议无独立刹车命令，刹车与使能绑定——**使能（进入运行模式）即解除抱闸，去使能（`0x02`）即抱闸**。「打开刹车」用 `0x1C` 置电流 0 实现「使能 + 零转矩」。

---

## 常见问题排查

| 现象 | 可能原因 | 处理 |
|------|---------|------|
| 所有命令无返回 | **CAN ID 不匹配** | 用 `taihu_motor_tools` 菜单选 `1`（扫描 CAN ID）确认实际 ID，再用 `2`（通信诊断）验证 |
| 打开 can0 失败 | 驱动未加载 / 接口未 up | 外部终端 `sudo modprobe kcan` 后用 `sudo ./build/kcanctl up can0 1000000` 打开通道（等价于 `sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up`）；程序本身普通用户即可运行 |
| VS Code 集成终端里 sudo 无效（退出码 1，无密码提示） | snap 版 VS Code 沙箱启用 `NoNewPrivs`，禁止一切提权 | ① 在外部系统终端（Ctrl+Alt+T）用 `sudo ./build/kcanctl up canX <波特率>` 打开通道，之后项目程序在 VS Code 终端可直接运行（普通用户可打开 CAN raw socket）；② 或卸载 snap 版改装 .deb 版 VS Code |
| `modprobe kcan` 报 `Key was rejected by service` | Secure Boot 拒绝未签名模块 | 进 BIOS 关闭 Secure Boot，见[环境配置 4](#4-加载驱动) |
| `modprobe kcan` 报 `Module not found` | 驱动未编译安装 / 内核升级后失效 | 重新执行[环境配置 3](#3-编译--安装)（内核升级后需重新编译） |
| 电机完全不动 | 未供电 / 欠压 | 检查电源，读 `母线电压` 与 `错误状态` |
| 报欠压（错误状态 bit2） | 供电低于额定电压 | 提高电压，调用 `clearError()` |
| 电机抖动 | 位置环 KP 过大 | 降低 KP |
| 响应慢 / 无力 | KP 过小 | 提高 KP |
| 标零后位置未归零 | 用了「编码器归零」而非「设置偏移」 | 用 `taihu_motor_tools` 菜单选 `5`（编码器标零，已封装正确方式） |
| 日志采样偏慢 / 时间戳不到完整时长 | 采样线程读状态耗时过长 | 已用 `0x41` 三合一读 + `drain` 优化；必要时进一步拉大采样周期 |
| 主程序无法正常结束 | 记录线程条件变量未唤醒 | 已在 `running=false` 后 `notify_all`，请确认使用最新代码 |

> `taihu_motor_tools` 菜单选 `2`（通信诊断）是排查的第一手段，可读取位置、偏移、错误状态等关键信息。

---

## ROS2 集成

ROS2 Jazzy 功能包位于 `ros2_ws/src/taihu_ros2/`，**直接编译仓库根目录源码**（不复制代码，`taihu_core` 单一事实来源）。话题/服务/参数速查表与常见问题见 `ros2_ws/src/taihu_ros2/README.md`。

### 节点架构

| 节点 | 职责 |
|------|------|
| `taihu_motor_node` | 一条 CAN 总线 = 一条腿。独立 200Hz 控制线程（不占 ROS 执行器），`0x41` 三合一读电流/速度/位置，200Hz 发布 `sensor_msgs/JointState`；`~/enable` 前不下发任何指令（安全联锁），`~/stop` 或退出即抱闸 |
| `taihu_gait_node` | 复用终端 demo 步态（3 阶段 × 五次多项式插值），发 `command_position` 给电机节点；`pause_after_swing=true` 时每步正摆后暂停，等 `~/resume` 放行（对应原「回车放行」） |

### 使用流程

```bash
# 0. 外部终端打开 CAN（snap VS Code 不能 sudo）
sudo ./build/kcanctl up can0 1000000

# 1. 构建（仓库根目录下）
cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select taihu_ros2

# 2. 启动（每次开新终端都要 source）
source install/setup.bash
ros2 launch taihu_ros2 gait.launch.py can:=can0        # 可选 steps:=3 phase_sec:=3.0 pause:=false

# 3. 另一终端（同样 source）：使能 → 跑步态 → 放行回摆
ros2 service call /taihu_motor_node/enable std_srvs/srv/Trigger {}
ros2 service call /taihu_gait_node/start_gait std_srvs/srv/Trigger {}
ros2 service call /taihu_gait_node/resume std_srvs/srv/Trigger {}     # 正摆暂停时放行
ros2 service call /taihu_gait_node/stop_gait std_srvs/srv/Trigger {}  # 停止步态
ros2 service call /taihu_motor_node/stop std_srvs/srv/Trigger {}      # 紧急抱闸

# 4. 观察
ros2 topic echo /taihu_motor_node/joint_states
ros2 topic echo /taihu_gait_node/gait_state
```

> ⚠️ 电机节点独占 CAN 接口：**不要同时运行终端 demo 和 ROS 节点**（两个进程抢同一 socket）。手动点动：`ros2 topic pub /taihu_motor_node/command_position sensor_msgs/msg/JointState "{position: [0.0, 0.5, 0.5, 0.0]}" -1`（先 `~/enable`）。
> 扭矩传感器 ROS 集成：`ros2 launch taihu_ros2 gait.launch.py torque:=true` 发布 `~/torque` 话题（详见 ros2_ws/src/taihu_ros2/README.md）。

---

## 扭矩传感器（坤维 HIT-V2.0 协议）

传感器与电机共用一条总线（can0，1Mbps），**完全独立、随时启停**：靠双向帧甄别互不干扰——传感器只认自己 Tx_ID（默认 `0x15`）的 7 字节帧，电机驱动只认电机 ID（1~4）的回复。不用传感器时总线零额外流量。

### 协议要点

| 项 | 值 |
|---|---|
| 默认波特率 | 1000kbps（与电机总线一致，直接挂即可） |
| 命令 ID（Rx_ID） | `0x10`；回复 ID（Tx_ID） | `0x15`（不与电机 1~4 冲突） |
| 单次查询 | `49 AA 0D 0A`（发往 Rx_ID） |
| 连续输出 | `48 AA <rate> 0D 0A`，rate：00=100Hz / 01=200Hz / 02=250Hz / 03=400Hz |
| 停止输出 | `43 AA 0D 0A`（工具/节点退出时自动发） |
| 数据帧 | 每次 2 帧 × 7 字节，帧 ID=Tx_ID；第 0 字节同步数（同数两帧配对）；命令码 `0x48`/`0x49`；第 2 字节 `0xAA`→后 4 字节扭矩（float 小端，Nm），`0xBB`→后 4 字节桥压（mV/V） |
| 供电 | 9~30VDC，<2W；**不内置终端电阻**（挂物理末端需外加 120Ω） |

上电初期零点漂移 → 空载稳定后做一次零偏校准。

### 独立终端工具（与任何主控制程序并行使用）

```bash
./build/taihu_torque_sensor can0            # 可选第二、三参数：rx_id tx_id（默认 0x10 0x15）
```

单键操作：`s` 开始 100Hz 连续输出 / `t` 停止 / `z` 零偏校准（100 样本）/ `w` 开关 CSV 日志（`record/torque_log.csv`）/ `q` 退出。运行时实时显示扭矩、原始值、桥压与采样率。

**与主控制程序的关系**：工具是独立进程，Linux 给每个 CAN raw socket 各投一份帧——终端 demo、ROS 节点、工具可同跑在各自进程里互不抢 socket；唯一约束是**别同时用两个"传感器客户端"**（工具 + ROS `torque:=true`），它们都会向传感器发 0x48 启动命令，会互相覆盖输出状态。

### ROS2 集成（opt-in）

```bash
ros2 launch taihu_ros2 gait.launch.py torque:=true
ros2 topic echo /taihu_motor_node/torque                    # std_msgs/Float64，单位 Nm
ros2 service call /taihu_motor_node/torque_zero std_srvs/srv/Trigger {}   # 零偏校准
```

不设 `torque:=true` 时节点行为与之前完全一致（默认 `use_torque_sensor=false`）。

---

## MPT 磁编码器（金钢，RS485 接口）

**关节端外置编码器**，24 位单圈绝对值（±0.01°），走 RS485 总线（`/dev/ttyUSB*`，USB 转 485），**与 CAN 电机总线完全独立**。协议与线序依据《MPT磁编码器数据手册v0.6.pdf》（仓库根目录）。

> **型号示例**（本项目实际采购）：`MPT-15-25H-A-24-R-N-C-M`
> = 定子 15 / 转子 25 正反装 H / 绝对值 / **24 位单圈** / **R = RS485 接口** / 5V 供电 / −40~125°C / **M = 2.5Mbps**。
> R 款**无地址字段，一条总线只能挂 1 台**；两台需各自独立转接头（或换 D 款 BUS 接口才可共 A/B）。

### 协议要点

| 项 | 值 |
|---|---|
| 电气 | 4.5~5.5VDC（约 100mA）、**编码器端 A/B 已内置终端电阻**（控制器侧需外加 120Ω） |
| 波特率代号 | 型号末段：M=2,500,000（**默认**）/ V=5,000,000 / A=115,200 |
| UART 格式 | 8N1，无流控，LSB 优先 |
| 接口型号 | **R（RS485）**：单字节命令，无地址（**总线只能挂 1 台**）；**D（BUS）**：带 5 位地址（出厂 `0x1F`），**多台可共 A/B 总线**（需先逐台改不同地址） |
| 读位置 | R 款 `0x31`（返回 5 字节 `A2 A1 A0 S CRC`）；D 款操作类型 `00` 获取信息（返回 `S A0 A1 A2 CRC`） |
| CRC-8 | 多项式 $x^8+x^7+x^4+x^2+x+1$（手册附 256 项查表） |
| 设零位 | R 款 `0x31`/`0x30` 交替 ≥10 次；D 款操作 00/01 交替。**约 50ms 生效期内编码器不响应任何命令** |
| 角度换算 | `angle = raw / 2^24 * 360`（度） |

状态字节 S：b7=错误（数据不可信）、b6=警告（数据仍有效）、b5~b0 状态位。

### 接线

编码器 8 针连接器（RS485 两线制只需 4 根）：

| 编码器线 | 功能 | 接 USB 转 485 |
|---|---|---|
| 红（Pin8） | +5V | 转接头 5V 输出（多数转接头**不提供**，需独立 5V 供电并共地） |
| 黑（Pin7） | GND | GND |
| 蓝（Pin6） | A(+) | A / D+ |
| 绿（Pin5） | B(−) | B / D− |

- **两台共总线**：两台编码器的 A 并 A、B 并 B（菊花链或星形均可，总长 < 50m），再接到同一个 USB 转 485。
- **终端电阻**：编码器端已内置；控制器侧在 A/B 之间加 120Ω（若转接头板载开关可打开）。
- 屏蔽层在驱动端（控制器侧）单点接地。

### USB 转 485 选型

| 编码器波特率代号 | 波特率 | 转接头要求 |
|---|---|---|
| M（默认） | 2.5 Mbps | **FT232H/FT2232H 类高速款**（支持 2M+ 自定义波特率）；CP2102/CH340 等普通款到不了 2.5M |
| V | 5 Mbps | 同上高速款（接近上限，不稳则降到 M） |
| A | 115200 | 普通款即可 |

> 普通转接头到手后先用 `stty -F /dev/ttyUSB0` 查最大支持；2.5M 波特率需在 Linux 用 `BOTHER` 任意波特率（驱动已实现）。不确定型号时，**到手先用工具自动探测**（见下节），M/A 两种都试。

### 两台共总线操作流程（D 款）

出厂地址都是 `0x1F`，**必须先逐台改地址**（否则两台同时应答会冲突）：

1. **单台**接 USB 转 485（只接第一台），运行工具，按 `d` 改地址 → 输入 `5`；
2. 断电，换**第二台**单接，运行工具，按 `d` → 输入 `6`；
3. 两台并到同一对 A/B，运行工具指定地址列表（或 ROS 节点 `enc_addrs:="[5, 6]"`）即可区分。

R 款两台**不能**共总线（无地址概念），需要两条独立总线/两个转接头。

> **选型警示（多台共总线必看）**：485 物理层的接口有 R / T / D 三种，**只有 D（BUS）带设备地址可多台共 A/B**：
> - `R` RS485：请求无地址字段，单总线仅 1 台；
> - `T` T485（多摩川兼容）：请求 b7~b3 操作类型 + 固定位 010，**同样无地址**，单总线仅 1 台；
> - `D` BUS：请求 b7 奇校验 | b6~b5 操作类型 | **b4~b0 设备地址**（出厂 0x1F，可改 1~31），多台共 A/B，协议自带防“应道涟漪”睡眠机制 T_SUSPEND。
> D 款物理层也是 485 差分 A/B + UART 8N1（默认 2.5Mbps），转接头与 R 款通用；但注意它不是 Modbus-RTU 等标准 485 协议，是金钢私有带地址协议（本项目驱动已完整支持）。

> **R 款上位机改 D 款（实测资料确认可行）**：厂家说法属实。金钢"分析器"上位机（`金钢科技-x64-win v4.7.11.exe`）连接编码器后，在标定模式下有 **读取配置 / 写入配置** 功能，可把 `接口` 从 `RS485` 改成 `BUS`，同时设置 `设备地址`（0~31）与 `波特率`（BUS 支持 115200 / 1M / 2.5M）。写入后编码器重启即按 D 款（BUS 带地址）协议工作。T485 接口软件明确提示"暂时不支持参数修改"，但 RS485 ↔ BUS 双向可改。
> 操作要点：Windows 上位机 + 金钢专用读取器（或 USB-485 转接头，2.5Mbps 需 FT232H 级芯片）；两台分别改成地址 1/2 后即可共 A/B，本仓库驱动/工具（`taihu_mpt_encoder`、`MptEncoder::Protocol::BUS`）无需改动直接识别。改回 RS485 同理（接口选项改回即可）。

### 独立终端工具

```bash
./build/taihu_mpt_encoder [device] [baud] [bus_addr]
# 例：
./build/taihu_mpt_encoder /dev/ttyUSB0 2500000        # 自动探测（R 款或 D 款 0x1F）
./build/taihu_mpt_encoder /dev/ttyUSB0 2500000 5      # D 款已改地址 5
```

单键操作：`z` 设零位 / `d` 改地址（D 款）/ `w` 开关 CSV 日志（`record/encoder_log.csv`）/ `q` 退出。实时显示角度、原始值、状态位、错误/警告、采样率。

### 把编码器接入 CAN 总线（taihu_encoder_can_bridge 网关）

MPT 全系列没有 CAN 输出型号（接口只有 SSI/BiSS-C/RS485/RS422/BUS），且 R 款一条 RS485 总线只能挂 1 台。**软件网关**把编码器读数“翻译”成 CAN 帧发到电机总线，总线上任何程序都能像读电机一样读到关节绝对角度：

```
MPT 编码器 ─RS485(2.5M)─ FT232H USB转485 ─┐
                                            ├─ 同一台 Linux 主机
电机/扭矩传感器 ─CAN─ 鲲弘 canX ───────────┘
```

```bash
./build/taihu_encoder_can_bridge /dev/ttyUSB0 can0           # 默认 200Hz
./build/taihu_encoder_can_bridge /dev/ttyUSB0 can0 100       # 100Hz
```

**CAN 帧协议**（标准帧，与电机/扭矩传感器共总线互不干扰，帧 ID 甄别）：

| 帧 ID | 方向 | 长度 | 格式 |
|---|---|---|---|
| `0x20`（可参数化） | 网关→总线 | 8 | `[0]`=状态 S（b7 错误/b6 警告）；`[1..3]`=24 位原始值大端；`[4..7]`=角度 0.001° int32 大端 |
| `0x21`（可参数化） | 总线→网关 | ≥1 | `[0]=0x01` 触发设零位（`cansend can0 021#01`） |

监听验证：`candump can0,020:7FF`（转动编码器看 `0x20` 帧变化）；第二台编码器再跑一个网关实例，用不同帧 ID（`... can0 200 0x28 0x29`）。网关也可订阅 `0x21` 命令帧远程设零位。与 `taihu_mpt_encoder` 查看工具不要同时读同一台编码器（两个主会重复发 0x31/0x64 争抢应答）。

### ROS2 集成（独立节点）

```bash
ros2 run taihu_ros2 taihu_encoder_node --ros-args -p serial_port:=/dev/ttyUSB0 -p enc_addrs:="[5, 6]"
ros2 topic echo /taihu_encoder_node/angle          # Float64MultiArray，单位度
ros2 service call /taihu_encoder_node/zero std_srvs/srv/Trigger {}
```

话题/参数详见 `ros2_ws/src/taihu_ros2/README.md`。

---

## 参考资料

项目根目录包含以下厂家资料（协议文档与调试截图，供开发参考）：

- `钛虎C1关节电机通讯使用说明_0804.pdf`（EtherCAT + CAN 双通信协议）
- `电机协议_双编_Can版本_20250623.xlsx`（CAN 命令集）
- `位置模式示例.pdf`、`公式换算.docx`、`扭矩系数.xlsx`
- `低压阈值调整.docx`、`电流绝对值阈值.docx`、`产品信息册.xlsx`
- `CAN-2Frames-Torque and Voltage-HIT-V2.0.pdf`（坤维扭矩传感器 CAN 协议 V2.0）
- `MPT磁编码器数据手册v0.6.pdf`（金钢 MPT 磁编码器协议与线序）
- `taihu_breakcontrol1.jpeg`、`taihu_breakcontrol2.jpeg`（刹车控制调试截图）

鲲弘 KH-UCANFDX6-Mini 设备资料（`KH-UCANFDXn-Mini 系列多路CANFD扩展模块配套资料/` 目录内）：

- `鲲弘CAN FD系列产品 Linux驱动安装说明V1.3.pdf`（驱动安装、SocketCAN 配置、测试工具）
- `KH-UCANFDX-Mini Product specification V1.8.pdf`（接口定义、指示灯、Linux 使用说明）

鲲弘 Linux SDK：https://gitee.com/ChengDu-KunHong/KH-UCANFD_Linux_SDK.git
产品文档：https://docs.kunhong-elec.com/zh/products/can-communication/KH-UCANFDX6-Mini.html
SDK 安装说明：https://docs.kunhong-elec.com/zh/tutorials/technical/linuxsdk/introduction/README.html
