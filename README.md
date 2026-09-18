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

- [硬件接线](#硬件接线)
- [鲲弘 CANFD 模块环境配置](#鲲弘-canfd-模块环境配置)
- [鲲弘 CANFD 设备使用与工具指南](#鲲弘-canfd-设备使用与工具指南)
- [编译](#编译)
- [快速开始](#快速开始)
- [项目结构](#项目结构)
- [核心 API](#核心-api)
- [工具函数 API（taihu_tools）](#工具函数-apitaihu_tools)
- [命令行工具](#命令行工具)
- [示例程序](#示例程序)
- [日志与绘图](#日志与绘图)
- [协议要点](#协议要点)
- [常见问题排查](#常见问题排查)
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

驱动安装后附带以下工具，用于排查问题：

| 工具 | 用途 | 用法 |
|------|------|------|
| `lskcan` | 一次性打印所有 CAN 设备的状态、波特率、错误计数 | `lskcan` |
| `kcan_monitor` | 实时总线监控（通信数据统计、总线状态、错误计数） | `kcan_monitor` |
| `kcanfd_test.sh` | 满负载收发压测（需将设备 CAN 通道**两两对接**） | `sudo kcanfd_test.sh -c 2 -b 1000000 -d 5000000 -t 500000` |
| `kcanosdiag.sh` | 生成诊断日志（发给厂家技术支持排查用） | `sudo kcanosdiag.sh` |
| `candump` / `cansend` | Linux 标准 can-utils 工具（抓包/发帧，鲲弘设备完全兼容） | `candump can0`、`cansend can0 123#1122334455667788` |

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
> ⚠️ **通道默认策略**：本项目**不再**在插入 USB 时自动打开所有通道。
> 如果你之前安装过自动配置规则（`deploy/99-kcan-autosetup.rules`），请在外部终端卸载：
>
> ```bash
> bash ~/TaiHu_motor_control/deploy/install_kcan_autosetup.sh --uninstall
> ```
>
> 卸载后重新拔插设备（或 `sudo rmmod kcan && sudo modprobe kcan`），
> 用 `./build/kcanctl list` 确认所有通道为 `DOWN`，需要哪个通道再用 `kcanctl up` 打开。

### 2. 波特率与 CAN FD 配置

```bash
# 经典 CAN：500kbps
sudo ip link set can0 type can bitrate 500000

# CAN FD：仲裁段 500k / 数据段 2M
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on

# 高级：手动指定采样点（仲裁段 500k 采样 75%，数据段 4M 采样 80%）
sudo ip link set can0 type can bitrate 500000 sample-point 0.75 \
    dbitrate 4000000 dsample-point 0.8 fd on

# 查看所有可用配置项
sudo ip link set can0 type can help
```

注意事项（厂家文档强调）：

- **推荐直接设置波特率**，驱动会自动计算较优的时间段参数；手动配 `tq/prop-seg/phase-seg` 等时间参数容易出错，除非有相关基础。
- **不要开启 `berr-reporting`**：部分教程会建议开启错误报告，但鲲弘设备不兼容该功能，开启会报错。
- 设备 **up 之后无法修改波特率**，必须先 `down` 再改。
- 其他可选开关：`loopback`（自环）、`listen-only`（只听不发）、`one-shot`（单次发送不重发）、`restart-ms`（BUS-OFF 自动重启间隔）等，详见 `help`。

### 3. 设备状态检测

#### 3.1 `ip -d link show` / `ifconfig`：查看接口状态

```bash
ip -d link show can0
```

输出中重点字段：

| 字段 | 含义 |
|------|------|
| `state UP / DOWN` | 设备是否已打开 |
| `can state ERROR-ACTIVE` | 正常状态 |
| `can state ERROR-WARNING / ERROR-PASSIVE` | 错误计数偏高，总线质量差或节点不匹配 |
| `can state BUS-OFF` | 总线严重错误已离线，需 down/up 恢复 |
| `berr-counter tx N rx N` | 发送/接收错误计数（**无节点应答时 tx 会涨到 128**） |
| `bitrate / dbitrate` | 当前仲裁段/数据段波特率 |
| `parentdev 1-9.2.3.1:1.0` | 对应的 USB 物理端口（多通道区分用） |

```bash
ifconfig -a        # 查看各接口 RX/TX 包数与错误数
```

#### 3.2 `lskcan`：一次性打印全部设备状态

```bash
lskcan
```

依次打印每个通道（`kcanusbfd32`、`kcanusbfd33`…）的：总线状态（Bus State）、时钟、收发帧数、仲裁段/数据段波特率配置、错误计数（Total Errors / RX / TX Error Counter）。适合快速确认"哪个通道配置对了、哪个通道在报错"。

#### 3.3 `kcan_monitor`：实时总线监控

```bash
kcan_monitor
```

进入交互式界面，实时捕获并统计 CAN 总线通信数据、分析总线状态与错误计数，适合长时间观察总线活跃度。

#### 3.4 `kcan-settings`：查看/设置设备硬件 ID

```bash
kcan-settings -f=/dev/kcanusbfd32          # 查看该通道设备信息
kcan-settings -f=/dev/kcanusbfd32 -d 66    # 设置设备 ID=66（配合通道绑定使用）
```

### 4. 驱动测试与诊断脚本

SDK 的 `KH-socket-can-test/` 目录提供两个脚本（`make install` 后已安装到 `/usr/local/bin`）：

#### 4.1 `kcanfd_test.sh`：满负载收发压测

验证设备在满负载下收发是否丢包。**测试前需将设备的 CAN 通道两两对接**（如 can0↔can1、can2↔can3、can4↔can5 用导线互连）。

```bash
# 默认：6 通道、1M/5M 波特率
sudo kcanfd_test.sh

# 自定义：开 can0+can1 两通道，波特率 500k/4M，收发 50 万包
sudo kcanfd_test.sh -c 2 -b 500000 -d 4000000 -t 500000
```

| 参数 | 含义 |
|------|------|
| `-c N` | 测试通道数（≤12，通道需两两对接） |
| `-b N` | 仲裁段波特率 |
| `-d N` | 数据段波特率（CAN FD） |
| `-t N` | 测试数据包数量 |

脚本自动完成驱动加载、参数设置、收发数据、结果验证；测试日志（帧率、接口收发量）保存在运行目录下。

#### 4.2 `kcanosdiag.sh`：生成诊断日志

设备异常时运行，把生成的日志发给厂家技术支持（support@cdkhdz.com）排查：

```bash
sudo kcanosdiag.sh
# 在当前目录生成 kcanosdiag-<版本>-<日期>-<时间>.log
```

### 5. can-utils 通用工具（抓包/发帧/分析）

can-utils 是 Linux SocketCAN 标准工具集，鲲弘设备完全兼容：

```bash
sudo apt-get install can-utils
```

#### 5.1 `candump`：实时抓包

```bash
candump can0                          # 显示 can0 上所有报文

# 过滤器格式 [can_id]:[can_mask]（ID 与掩码按位与相等才显示）
candump can0,0x123:0x7FF              # 只看 ID=0x123
candump can0,0x123:0x7FF,0x456:0x7FF  # 只看 ID=0x123 或 0x456
```

实用示例（本项目电机协议）：监听电机 ID=2 的所有回复：

```bash
candump can0,0x002:0x7FF
```

#### 5.2 `cansend`：发送单帧

```bash
cansend can0 123#1122334455667788     # 向 can0 发 ID=0x123、8 字节数据的帧（十六进制）
```

实用示例：向 ID=2 的电机发送「读位置」命令（功能码 0x08）：

```bash
cansend can0 002#08
# 另开终端 candump can0 即可看到电机回复，如 002 [5] 08 xx xx xx xx
```

#### 5.3 `cangen`：生成随机报文（压测/仿真）

```bash
cangen can0 -f -g 100 -L 64           # 每 100ms 发一帧 64 位随机 CAN FD 数据
```

#### 5.4 `cansniffer`：只显示数据变化的帧

```bash
cansniffer -c can0                    # 过滤数据不变的帧，变化字节高亮（逆向分析总线协议利器）
```

#### 5.5 `canbusload`：实时总线负载率

```bash
canbusload can0@1000000 -r -t -b -c   # 显示 can0(1M) 的实时负载率、帧率统计
```

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
| `taihu_main` | 示例 | 双电机同步旋转（主 demo） |
| `taihu_singleleg_step` | 示例 | 四电机单腿步态控制 |
| `taihu_single_motorctl` | 示例 | 单电机交互式控制（位置/正弦） |
| `taihu_single_motorctl_xCAN` | 示例 | 多 CAN 总线电机同步控制（每总线一线程，时序对齐） |
| `taihu_motor_tools` | 工具 | **工具总入口（菜单式）**：数字键选总线 + 数字键选工具（扫描/诊断/改ID/恢复出厂/标零/开刹车/低压阈值） |
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

### 2. 运行主 demo（双电机同步旋转）

```bash
./build/taihu_main        # 通道已 up 后普通用户即可运行
```

电机 1（ID=1）逆时针转 90°、电机 2（ID=2）顺时针转 90°，同步运动。

### 3. 标零（首次使用前）

把模组摆到期望零点，运行工具总入口，选择对应总线后按 `5`（编码器标零），再输入电机 CAN ID：

```bash
./build/taihu_motor_tools
```

> 标零后，正弦/旋转等运动以该零点为基准。若需「打开刹车 → 手动旋转到新位置 → 重新标零」，在菜单中选 `6`（打开刹车并重新标零）。

---

## 项目结构

```
TaiHu_motor_control/
├── CMakeLists.txt
├── include/taihu/               # 头文件
│   ├── can_interface.h          #   CAN 抽象接口 CanInterface / CanFrame
│   ├── socket_can.h             #   SocketCAN 驱动（鲲弘 KH-UCANFDX6-Mini，can0~can5）
│   ├── joint_module.h           #   关节模组控制类 JointModule
│   ├── motor_logger.h           #   日志记录器 MotorLogger（CSV）
│   ├── config.h                 #   全局配置（控制周期/采样周期等）
│   └── taihu_tools.h            #   工具函数封装（诊断/标零/旋转等）
├── src/                         # 核心库实现
│   ├── socket_can.cpp
│   ├── joint_module.cpp
│   ├── motor_logger.cpp
│   └── taihu_tools.cpp
├── examples/                    # 示例程序
│   ├── taihu_main.cpp           #   主 demo（双电机同步旋转）
│   ├── taihu_singleleg_step.cpp #   单腿步态（四电机，五次多项式轨迹）
│   ├── taihu_single_motorctl.cpp #  单电机交互式控制（位置/正弦）
│   └── taihu_single_motorctl_xCAN.cpp # 多总线电机同步控制（每总线一线程）
├── tools/                       # 命令行工具入口 + 绘图脚本
│   ├── taihu_motor_tools.cpp    #   工具总入口（菜单式，选总线+选工具）
│   ├── kcanctl.c                #   鲲弘 CAN 通道控制（打开/关闭/波特率）
│   └── plot_motor_log.py        #   日志绘图脚本（Python + matplotlib）
├── deploy/                      # 可选的 udev 自动配置规则（默认不启用）
│   ├── 99-kcan-autosetup.rules  #   插 USB 自动配 1M 并 up（本项目默认不使用）
│   └── install_kcan_autosetup.sh#   规则安装/卸载脚本（--uninstall 卸载）
└── record/                      # 运行时生成的日志与图片（不入库）
    ├── motor_log.csv            #   运行数据日志
    ├── position.png             #   位置曲线
    ├── velocity.png             #   速度曲线
    ├── current.png              #   电流曲线
    └── error.png                #   错误状态曲线
```

核心实现编译为静态库 `taihu_core`，示例与工具统一链接。

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

### `taihu_main`（主 demo，双电机同步旋转）

```bash
./build/taihu_main
```

双电机按分段周期轨迹同步运动：保持零点 → ±90° → 等待 → 回零，循环 3 次。配置见 [`examples/taihu_main.cpp`](examples/taihu_main.cpp:40) 顶部常量。

### `taihu_singleleg_step`（单腿步态，四电机）

```bash
./build/taihu_singleleg_step
```

四个电机（ID=1、4 减速比 101；ID=2、3 减速比 81）按**单腿步态**分段轨迹同步运动（回零 → 膝盖摆动 ±180° → 髋转整圈 ±360° → 膝盖回零 → 回零停顿），相邻阶段用**五次多项式**插值保证位置/速度/加速度连续、轨迹平滑。配置见 [`examples/taihu_singleleg_step.cpp`](examples/taihu_singleleg_step.cpp:44)。

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

---

## 日志与绘图

运行示例时，会把各电机的状态（时间戳、位置、速度、电流、电压、错误）以 CSV 写入 [`record/motor_log.csv`](record/motor_log.csv:1)（覆盖模式，不入库）。

### 采样周期

日志采样周期由 [`config.h`](include/taihu/config.h:14) 配置：

- [`kLoopPeriodMs`](include/taihu/config.h:14) = 5：控制周期（5ms）
- [`kLogSamplePeriodMs`](include/taihu/config.h:21) = 100：日志采样周期（0.1s）

> CAN 总线半双工且为单通道，采样与控制无法物理独立，采样周期拉长为 0.1s 以减少采样对总线的占用、避免争抢控制周期。（鲲弘模块支持 6 路独立 CAN，若电机分散到不同通道，可缩短采样周期）

### 绘制曲线图

```bash
python3 tools/plot_motor_log.py                    # 默认读 record/motor_log.csv，输出到 record/
python3 tools/plot_motor_log.py record/motor_log.csv -o record
```

依赖 `python3 + matplotlib`。生成 4 张图（横轴均为时间，不同电机 ID 用不同颜色区分）：

- `position.png`：位置曲线（叠加各电机同色虚线的**目标位置**做对照，目标轨迹复现自单腿步态）
- `velocity.png`：速度曲线
- `current.png`：电流曲线（含四电机**瞬时电流之和**曲线）
- `error.png`：错误状态曲线

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

## 参考资料

项目根目录包含以下厂家资料（协议文档与调试截图，供开发参考）：

- `钛虎C1关节电机通讯使用说明_0804.pdf`（EtherCAT + CAN 双通信协议）
- `电机协议_双编_Can版本_20250623.xlsx`（CAN 命令集）
- `位置模式示例.pdf`、`公式换算.docx`、`扭矩系数.xlsx`
- `低压阈值调整.docx`、`电流绝对值阈值.docx`、`产品信息册.xlsx`
- `taihu_breakcontrol1.jpeg`、`taihu_breakcontrol2.jpeg`（刹车控制调试截图）

鲲弘 KH-UCANFDX6-Mini 设备资料（`KH-UCANFDXn-Mini 系列多路CANFD扩展模块配套资料/` 目录内）：

- `鲲弘CAN FD系列产品 Linux驱动安装说明V1.3.pdf`（驱动安装、SocketCAN 配置、测试工具）
- `KH-UCANFDX-Mini Product specification V1.8.pdf`（接口定义、指示灯、Linux 使用说明）

鲲弘 Linux SDK：https://gitee.com/ChengDu-KunHong/KH-UCANFD_Linux_SDK.git
产品文档：https://docs.kunhong-elec.com/zh/products/can-communication/KH-UCANFDX6-Mini.html
SDK 安装说明：https://docs.kunhong-elec.com/zh/tutorials/technical/linuxsdk/introduction/README.html
