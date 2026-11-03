# 嵌入式设备（RK3588 / aarch64）部署说明

本目录提供在 **RK3588 + AGX 嵌入式设备**上配置本项目管理运行环境的脚本与说明。

---

## 1. 适用场景

| 项目 | 说明 |
| --- | --- |
| 目标架构 | `aarch64` / `arm64`（RK3588） |
| 目标系统 | Debian 系 Linux（有 `apt`），**不要求是 Ubuntu** |
| 编译方式 | 在板上**本机编译**（不交叉编译） |
| 网络 | 板上**可能不能上网** → 依赖走离线 `.deb` |
| kcan 驱动 | 由**厂商提供已编译好的 ARM64 `.ko`**（板上无内核头文件） |
| 图形界面 | 默认不装 Qt5（`taihu_keyboardctl_ui` 自动跳过，不影响其它程序） |
| ROS 2 | 默认不装（仅 `ros2_ws/` 需要，电机控制主链路不依赖） |

> 本项目核心（CAN 电机控制 + 上位机联调）只依赖 `gcc/g++ + cmake + 标准系统头文件 + iproute2`，
> 编译依赖非常轻，真正的难点只有 **kcan 内核模块**。

---

## 2. 快速开始

### 2.1 先把项目拷到板上

```bash
# 在 x86 开发机上（示例）
scp -r /home/sheng/workspaces/Xiaoyuan_2 user@<板子IP>:~/
```

### 2.2 先做只读自检（不需要 sudo，不改系统）

```bash
cd ~/Xiaoyuan_2
./deploy/install_env.sh --check
```

自检会报告：架构、发行版、包管理器、编译器、内核头文件、联网状态、
当前 CAN 接口、厂商 `kcan.ko` 是否存在且是否为 aarch64。

### 2.3 确认无误后正式安装

```bash
sudo ./deploy/install_env.sh
```

---

## 3. 脚本做了什么（7 个阶段）

| 阶段 | 内容 | 说明 |
| --- | --- | --- |
| 1 | 环境自检 | 架构 / 发行版 / apt / root / 内核头 / 联网 / 现有 CAN 接口 / 厂商 `.ko` |
| 2 | 基础构建依赖 | `build-essential` `cmake` `pkg-config`；并校验 `cmake >= 3.16`、`g++` 支持 C++17 |
| 3 | 运行时诊断工具 | `can-utils`（candump/cansend）、`iproute2`（**kcanctl 运行时硬依赖**）、`usbutils` |
| 4 | Python 依赖 | `openpyxl`（上位机唯一硬依赖）；`matplotlib` / `reportlab` / `fontTools` / `pyserial` 只报告不强制 |
| 5 | 编译项目 | `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release` + `cmake --build build -j$(nproc)` |
| 6 | 安装 kcan 驱动 | **只装 `.ko`，不编译**；校验架构与 `vermagic`，安装到 `/lib/modules/$(uname -r)/misc/` |
| 7 | 最终验收 | 检查产物、CAN 接口、`openpyxl`，并打印后续手动操作 |

---

## 4. 命令行选项

```bash
sudo ./deploy/install_env.sh [选项]

  --skip-deps      跳过阶段 2/3/4
  --skip-build     跳过阶段 5（只装依赖）
  --skip-driver    跳过阶段 6（不装 kcan 驱动）
  --load-driver    装完驱动后立即 modprobe kcan（默认不加载）
  --check          只做环境自检（不需要 root）
  --dry-run        打印将执行的命令但不执行
  -y, --yes        所有确认自动回答 yes
  -h, --help       帮助
```

常用组合：

```bash
# 只装依赖，暂不编译（先把环境备好）
sudo ./deploy/install_env.sh --skip-build --skip-driver

# 驱动还没拿到，先把项目编出来
sudo ./deploy/install_env.sh --skip-driver

# 驱动已就位，只补装驱动
sudo ./deploy/install_env.sh --skip-deps --skip-build --load-driver
```

---

## 5. 离线依赖准备

板上不能上网时见 [`offline-debs/README.md`](offline-debs/README.md)。

核心要点：**`apt-get download` 只下当前机器架构的包**，务必在 aarch64 机器上执行，
或用 `apt-get install --download-only` 把整棵依赖树一起下下来。

---

## 6. kcan 驱动（最关键的一步）

### 6.1 为什么不能自己编译

kcan 是内核模块，编译需要 `/lib/modules/$(uname -r)/build`（内核头文件）。
你的板子没有内核头文件，因此**必须用厂商提供的、与本机内核版本匹配的 ARM64 `kcan.ko`**。

> ⚠️ 厂商 SDK 压缩包里自带的 `driver/kcan.ko` 是 **x86-64** 预编译件，
> 在 RK3588 上**无法加载**。必须向厂商索取 aarch64 版本。

脚本会自动检测并明确报错：

```
[失败] 该 kcan.ko 不是 aarch64 架构，无法在本机加载！
```

### 6.2 把驱动放哪里

脚本按以下顺序查找 `kcan.ko`：

```
deploy/kcan-driver/kcan.ko
.kunhong_sdk/<任意子目录>/kcan.ko
kcan-driver/kcan.ko
```

推荐做法：

```bash
mkdir -p deploy/kcan-driver
# 把厂商给的 ARM64 文件拷进来
cp /path/to/arm64/kcan.ko deploy/kcan-driver/
# 可选：一并带上用户态工具
cp /path/to/kcan.conf /path/to/lskcan /path/to/kcan_monitor deploy/kcan-driver/
```

### 6.3 脚本安装到什么位置

与厂商 `driver/Makefile` 的 `install` 目标保持一致：

| 文件 | 安装位置 |
| --- | --- |
| `kcan.ko` | `/lib/modules/$(uname -r)/misc/kcan.ko` |
| `kcan.conf` | `/etc/modprobe.d/kcan.conf` |
| `lskcan` / `kcan_monitor` / `kcan_make_devices` / `kcanosdiag.sh` | `/usr/local/bin/` |
| `kcan-settings`（需 aarch64） | `/usr/local/bin/` |

安装后执行 `depmod -a` 更新模块依赖表。

### 6.4 加载与排错

```bash
sudo modprobe kcan
lsmod | grep kcan
ip -d link show | grep -E "can[0-9]"
```

常见失败原因：

| 报错 | 原因 | 解决 |
| --- | --- | --- |
| `invalid module format` | `vermagic` 与内核版本不匹配 | 向厂商索取对应内核版本的 `.ko` |
| `Key was rejected by service` | Secure Boot 拒绝未签名模块 | 关闭 Secure Boot，或签名模块 |
| `Exec format error` | `.ko` 架构不对（拿到 x86-64 的了） | 换 aarch64 版本 |

---

## 7. 安装完成后：手动开 CAN 通道

> **本脚本绝不自动 `up` CAN 通道。** 通道必须按现场接线逐个手动打开。

```bash
# 1) 看有哪些通道（免 root）
./build/kcanctl list

# 2) 确认总线健康（berr-counter tx 应为 0）
ip -d link show can4 | grep "can state"

# 3) 手动打开通道并设 1 Mbps
sudo ./build/kcanctl up can4 1000000
```

接口编号随 USB 插入顺序变化，**不要假设一定是 `can4`**，每次插拔都用 `kcanctl list` 确认。

> ⚠️ 悬空（没接设备）的通道不要跑真实流量，20 秒左右就会
> `OSError: [Errno 105] No buffer space available`。

---

## 8. 验证环境是否可用

```bash
# 下位机主程序
sudo ./build/walk_control_V1

# 另一个终端：虚拟上位机（先 dry-run 确认轨迹能被解析）
python3 tools/up_walk_control_V1.py --dry-run --hz 1000 \
    --xlsx step_position/step_make_V2.xlsx

# 总线监视
candump can4
```

---

## 9. 未覆盖的部分（按需自行安装）

| 组件 | 需要时安装 | 说明 |
| --- | --- | --- |
| Qt5 图形界面 | `sudo apt install qtbase5-dev` | 装完重新 `cmake` 即会自动编译 `taihu_keyboardctl_ui` |
| ROS 2 | 按发行版官方文档 | 仅 `ros2_ws/` 需要 |
| `matplotlib` | `sudo apt install python3-matplotlib` | `tools/plot_motor_log.py` 绘图 |
| `reportlab` + `fontTools` | `pip3 install reportlab fonttools` | 仅重新生成 `docs/step_protocol.pdf` 时需要 |
| AGX 的 AI 环境 | JetPack / CUDA / TensorRT | 本脚本不涉及，后续单独处理 |