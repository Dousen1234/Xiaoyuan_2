# 嵌入式设备（RK3588 / aarch64）部署说明

本目录提供在 **RK3588 + AGX 嵌入式设备**上配置本项目管理运行环境的脚本与说明。

| 脚本 | 作用 |
| --- | --- |
| [`install_env.sh`](install_env.sh) | 一次性配好整套环境（依赖 + 编译 + 驱动安装） |
| [`install_kcan.sh`](install_kcan.sh) | **只管 kcan 驱动**：安装 / 热重载 / 卸载 / 开机自加载，可反复执行 |

两个脚本**职责分开**：`install_env.sh` 阶段 6 会把驱动安装**转交**给 `install_kcan.sh`，
不会各自维护一套逻辑。日常驱动排错只跑 `install_kcan.sh` 即可，几秒完成，不碰编译器与依赖。

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

### 2.4 只装驱动（已有环境 / 换驱动 / 驱动排错时用这个）

```bash
sudo ./deploy/install_kcan.sh
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
| 6 | 安装 kcan 驱动 | **只装 `.ko`，不编译**；校验架构与 `vermagic`；**转交 `install_kcan.sh` 执行** |
| 7 | 最终验收 | 检查产物、CAN 接口、`openpyxl`，并打印后续手动操作 |

---

## 4. 命令行选项

```bash
sudo ./deploy/install_env.sh [选项]

  --skip-deps      跳过阶段 2/3/4
  --skip-build     跳过阶段 5（只装依赖）
  --skip-driver    跳过阶段 6（不装 kcan 驱动）
  --load-driver    装完驱动后立即 modprobe kcan（默认不加载）
  --ko <路径>      指定厂商 aarch64 kcan.ko（透传给 install_kcan.sh）
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

**推荐：统一放 `deploy/kcan-driver/`**，脚本会优先从这里找。

```
deploy/kcan-driver/kcan.ko          ← 必需
deploy/kcan-driver/kcan.conf        ← 可选（modprobe 参数）
deploy/kcan-driver/lskcan           ← 可选（厂商工具）
deploy/kcan-driver/kcan-settings    ← 可选（需为 aarch64）
deploy/kcan-driver/kcan.h           ← 可选（头文件）
deploy/kcan-driver/udev/45-kcan.rules ← 可选
```

该目录下有一份详细说明：[`deploy/kcan-driver/README.md`](kcan-driver/README.md)。

脚本搜索顺序（后者只是兜底，通常是 x86-64 文件会被自动排除）：

```
1. --ko <路径> 显式指定
2. deploy/kcan-driver/kcan.ko
3. kcan-driver/kcan.ko（项目根目录）
4. .kunhong_sdk/<任意子目录>/kcan.ko  ← 厂商 SDK，基本是 x86-64，仅作发现
```

```bash
mkdir -p deploy/kcan-driver
cp /path/to/arm64/kcan.ko deploy/kcan-driver/
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

## 7. 专用驱动脚本 `install_kcan.sh`

`install_env.sh` 是"一次性配好整套环境"，而 `install_kcan.sh` 只做**驱动生命周期管理**：

- 驱动换版本了 → 重跑一次即可，不碰编译器、不重装依赖，几秒完成
- 需要开机自加载 → 它负责写 `/etc/modules-load.d/kcan.conf`
- 需要热重载（改了驱动不重启）→ `--reload`
- 需要干净卸载 → `--uninstall`
- 出问题要排查 → `--check` 只读诊断，不需要 root

### 7.1 十个步骤

| 步骤 | 内容 |
| --- | --- |
| 1 | 环境检查：root / 架构 / 内核版本 / 发行版 / 内核头文件 / `depmod` `modprobe` / Secure Boot |
| 2 | 定位 `kcan.ko`（按搜索顺序，自动去重） |
| 3 | **严格校验**：ELF 架构必须 aarch64；`vermagic` 必须与本机 `uname -r` 一致 |
| 4 | 安装 `.ko` 到 `/lib/modules/$(uname -r)/misc/` + `depmod -a` + 写 `/etc/modprobe.d/kcan.conf` |
| 5 | 开机自加载：写 `/etc/modules-load.d/kcan.conf`（内容仅一行 `kcan`） |
| 6 | 安装厂商用户态工具与头文件（有则装，无则跳过） |
| 7 | 可选：安装厂商 udev 规则（`--with-udev` 才装） |
| 8 | 加载模块并验证（`lsmod`、`/sys/module/kcan`、`canN` 接口探测、失败原因对照表） |
| 9 | 现状总览（模块文件 / 加载状态 / 配置 / 接口） |
| 10 | 打印后续手动操作 |

### 7.2 命令行选项

```bash
sudo ./deploy/install_kcan.sh [选项]

  --ko <路径>      指定 kcan.ko（默认自动搜索 deploy/kcan-driver/ 等目录）
  --reload         安装后热重载模块（会先 down 所有 canN 接口）
  --uninstall      完整卸载：rmmod + 删除 .ko / 配置 / 工具
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
```

### 7.3 典型用法

```bash
# ① 第一次装（驱动已放到 deploy/kcan-driver/）
sudo ./deploy/install_kcan.sh

# ② 先看清楚会改什么，再决定是否执行（不需要 root）
./deploy/install_kcan.sh --dry-run --check

# ③ 换了新驱动，想立即生效（不重启）
sudo ./deploy/install_kcan.sh --reload

# ④ 只落盘、暂不加载（等接好线再手动 modprobe）
sudo ./deploy/install_kcan.sh --no-load

# ⑤ 只要模块，不要开机自加载
sudo ./deploy/install_kcan.sh --no-autoload

# ⑥ 需要 /dev/kcan-usb_x6/<adapter>/can<N> 这类稳定路径
sudo ./deploy/install_kcan.sh --with-udev

# ⑦ 出问题只诊断（只读，不改系统）
./deploy/install_kcan.sh --check

# ⑧ 彻底卸载
sudo ./deploy/install_kcan.sh --uninstall
```

### 7.4 退出码

| 退出码 | 含义 |
| --- | --- |
| `0` | 全部步骤完成（可能有警告） |
| `1` | 存在 `[失败]` 项，需按提示逐条处理 |

### 7.5 安全设计

| 行为 | 说明 |
| --- | --- |
| **绝不自动 `up` CAN 通道** | 通道编号随 USB 枚举顺序变化，自动开可能对错误通道发 1 Mbps 流量 |
| `--reload` / `--uninstall` 先 `down` 接口 | `rmmod` 的前置条件，否则 `EBUSY` |
| 开机自加载只加载模块 | `/etc/modules-load.d/kcan.conf` 仅 `modprobe kcan`，不开通道 |
| `--dry-run` 不需要 root | 普通用户可完整预览全流程 |
| `--uninstall` 必须交互确认 | 除非加 `-y` |
| 覆盖已有 `.ko` 自动备份 | 生成 `kcan.ko.bak.<时间戳>`，卸载时不删备份 |

> **为什么不像 `systemd-networkd` 那样开机自动 `up` 通道？**
> USB-CAN 适配器的 `canN` 编号会随插入顺序变化（今天 `can4`，明天可能 `can0`）。
> 自动 `up` 一旦认错通道，就会向一条悬空或错误接线的总线上灌 1 Mbps 流量，
> 轻则 `No buffer space available`，重则影响整条总线。
> 因此**通道必须在确认接线后手动打开**。

---

## 8. 安装完成后：手动开 CAN 通道

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

## 9. 验证环境是否可用

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

## 10. 未覆盖的部分（按需自行安装）

| 组件 | 需要时安装 | 说明 |
| --- | --- | --- |
| Qt5 图形界面 | `sudo apt install qtbase5-dev` | 装完重新 `cmake` 即会自动编译 `taihu_keyboardctl_ui` |
| ROS 2 | 按发行版官方文档 | 仅 `ros2_ws/` 需要 |
| `matplotlib` | `sudo apt install python3-matplotlib` | `tools/plot_motor_log.py` 绘图 |
| `reportlab` + `fontTools` | `pip3 install reportlab fonttools` | 仅重新生成 `docs/step_protocol.pdf` 时需要 |
| AGX 的 AI 环境 | JetPack / CUDA / TensorRT | 本脚本不涉及，后续单独处理 |