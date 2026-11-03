# kcan 驱动文件放置目录

当无法在本机编译时，把**厂商提供的 aarch64 版 `kcan.ko`** 放到本目录，然后执行安装脚本。

> 大多数情况下不需要手动放文件 —— `install_drivers.sh` / `install_kcan.sh` 会
> 优先尝试用 `.kunhong_sdk/` 里的源码直接编译。只有编译路线走不通时才需要用到本目录。

## 目录内容

```
deploy/kcan-driver/
├── kcan.ko          ← 必需：aarch64 内核模块
├── kcan.conf        ← 可选：modprobe 参数（厂商 SDK 的 driver/kcan.conf）
├── lskcan           ← 可选：厂商用户态工具
├── kcan_monitor     ← 可选
├── kcan_make_devices← 可选
├── kcanosdiag.sh    ← 可选
├── kcan-settings    ← 可选（ELF，需为 aarch64）
├── kcan.h           ← 可选：头文件
├── kcanfd.h         ← 可选：头文件
└── udev/
    └── 45-kcan.rules← 可选：udev 规则
```

> 只有 `kcan.ko` 是必需的。其余文件缺失不影响安装，脚本会自动跳过。

## 怎么获取 kcan.ko

**优先自己编译**（厂商 SDK 自带完整源码），编译不出来再向厂商索取预编译件。

### 路线一：自己编译（推荐）

`.kunhong_sdk/KH-UCANFD_Linux_SDK-1.4.2/` 里有完整的 `driver/src/`（50+ 个 `.c/.h`），
`driver/Makefile` 支持 ARM64，`driver/dkms.conf` 也已配好。只要板子上有内核头文件就能编：

```bash
# 先看有没有内核构建树
ls -d /lib/modules/$(uname -r)/build

# 有的话，交给脚本自动编译安装（默认走 DKMS，内核升级后自动重编）
sudo ./deploy/install_drivers.sh
```

脚本内部等价于：

```bash
cd .kunhong_sdk/KH-UCANFD_Linux_SDK-1.4.2
sudo ./build.sh -dkms        # 源码编译 + DKMS 注册 + 安装
```

### 为什么很多时候编不出来

RK3588 板卡上的 Ubuntu 20.04 通常跑**厂商定制内核**（如 `5.10.160`），
而 Ubuntu 20.04 官方内核是 `5.4.x`，apt 源里没有匹配的 `linux-headers` 包：

```bash
apt-cache policy linux-headers-$(uname -r)
# Candidate: (none)      ← 就是这种情况
```

没有内核头文件就无法编译。此时只能走路线二。

### 路线二：向厂商索取预编译件

| 条件 | 要求 |
| --- | --- |
| CPU 架构 | `aarch64`（ARM64），**不是** `x86-64` |
| 内核版本 | 必须与本机 `uname -r` **完全一致**，例如 `5.10.160` |
| 编译方式 | 厂商预编译，不是源码 |

> 厂商在 x86 开发机上交叉编译出的 aarch64 版本也可以，只要能提供。

### ⚠ 厂商 SDK 自带的是 x86-64

仓库里 `.kunhong_sdk/KH-UCANFD_Linux_SDK-*/driver/kcan.ko`
是 **x86-64** 预编译件（给 PC 开发机用的），在 RK3588 上加载会报：

```
insmod: ERROR: could not insert module kcan.ko: Exec format error
```

安装脚本会自动识别并跳过它，不会误装。

## 验证文件是否正确

在**开发机**上就能验证，不需要板卡：

```bash
# 1) 架构必须是 ARM aarch64
file kcan.ko
# 期望输出：ELF 64-bit LSB relocatable, ARM aarch64, ...

# 2) 内核版本必须匹配（在板卡上执行，对比 uname -r）
modinfo kcan.ko | grep vermagic
# 期望输出含：vermagic: 5.10.160 SMP preempt mod_unload aarch64
uname -r
# 期望输出：5.10.160
```

两项都匹配后才能安装。任一不匹配，加载时必然失败：

| 报错 | 含义 |
| --- | --- |
| `Exec format error` | 架构不对（拿到 x86-64 了） |
| `invalid module format` | vermagic 不匹配（内核版本不对） |
| `Key was rejected by service` | Secure Boot 拒绝未签名模块 |

## 安装

```bash
cd ~/Xiaoyuan_2

# 推荐：自动在「源码编译」和「本目录预编译件」之间决策
sudo ./deploy/install_drivers.sh

# 或者显式指定用本目录的文件
sudo ./deploy/install_drivers.sh --kcan=prebuilt

# 只要 kcan 一个驱动
sudo ./deploy/install_kcan.sh
```

脚本会自动在本目录搜索 `kcan.ko` 并完成校验安装。
详细说明见 [`../README.md`](../README.md) 或 `./deploy/install_drivers.sh --help`。

## ⚠ 不要用 `build.sh -rules` / `-dkms-rules`

厂商 `build.sh` 的这两个参数会写入一条 udev 规则：

```
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="kcan", NAME="kcan%n"
```

它会把 `can0` 改名成 `kcan0`，而本项目所有程序都硬编码了 `can0`/`can4`，
改名后全部报 `No such device`。两个安装脚本都**不会**传这两个参数。

如果已经误装：

```bash
sudo rm -f /etc/udev/rules.d/45-kcan.rules
sudo udevadm control --reload-rules
# 拔插一次 USB-CAN 模块，接口名恢复成 canN
```

## 文件权限

从 U 盘 / scp 拷进来后确认权限正常：

```bash
chmod 644 deploy/kcan-driver/kcan.ko
ls -l deploy/kcan-driver/
```