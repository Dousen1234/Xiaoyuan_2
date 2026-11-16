# 项目交接文档：WSL 到原生 Ubuntu 22.04

更新时间：2026-10-28（Asia/Shanghai）

## 1. 当前目标

项目目标是使用 KH-UCANFDX6-Mini 的 Linux `kcan` 驱动提供的 SocketCAN 接口，运行 `claws_control_1` 中的手爪通信程序。当前阶段只做驱动识别、通道映射和被动监听，暂不发送手爪控制报文。

目标硬件链路：

```text
KH-UCANFDX6-Mini -> CAN 总线 -> 手爪驱动板 -> 手爪
```

## 2. 项目与环境位置

- 项目目录：`E:\霄元\二期\软件\claws_control_1`
- Windows/WSL 共享的源代码位于 E 盘，原生 Ubuntu 已确认可以浏览和编辑该分区；切换系统后先确认 E 盘没有被只读挂载。
- WSL Python 虚拟环境：`~/venvs/xy_software`
- 启用命令：

  ```bash
  source ~/venvs/xy_software/bin/activate
  ```

- 原生 Ubuntu 中应重新创建自己的 Python 虚拟环境，不要复制 WSL 的虚拟环境目录。

## 3. 已完成的软件工作

- CAN 协议、命令编码、反馈解析和 CRC 逻辑已存在于项目中。
- 参数保存/加载和参数测试已完成。
- `host_demo/requirements.txt` 要求：`python-can>=4.3,<5`。
- `README.md` 已加入 WSL 虚拟环境启用说明。
- 当前没有为真实硬件发送控制指令，也没有刷写固件。

## 4. WSL 验证记录

WSL 发行版：Ubuntu 22.04.5 LTS，运行于 WSL2。截图中内核为 `6.18.54.1-microsoft-standard-WSL2`。

已完成：

- `vcan0` 创建、启用和 `can-utils` 收发测试通过；测试帧为 `123#1122334455667788`。
- Python `python-can` 的 `socketcan + vcan0` 回环测试通过。
- Python 环境中已安装 `python-can 4.6.1`。
- 项目现有离线单元测试：`44 tests`，结果 `OK`。

这些结果证明应用层 SocketCAN 代码可用，但不证明真实 KH 硬件、USB 转接、驱动和物理通道映射可用。

## 5. KH 设备与官方 SDK 检查结果

Windows 设备枚举结果：

- 看到 6 个 `PCAN-USB FD` 接口；
- USB 标识：`VID_0C72&PID_0012`；
- 当前 Windows 设备状态为错误代码 28（没有安装 Windows 驱动）。

本项目路线不安装 Windows PCAN 驱动。SDK 使用 Linux `kcan` 驱动，提供 SocketCAN 网络接口。

官方 SDK：

- 版本：`1.4.2`
- 官方仓库/发行版：https://gitee.com/ChengDu-KunHong/KH-UCANFD_Linux_SDK
- Windows 下载位置：`C:\Users\admin\Downloads\KH-UCANFD_Linux_SDK.zip`
- SHA256：`0859AFC67DC63F232D64A8815A4C2A7BCF7129298C48748A70174647A4A6508C`
- SDK 源码中的 USB ID：`VID 0x0c72`、`PID 0x0012`，与当前设备枚举结果匹配。
- 压缩包包含 `kcan` 驱动源码、`lskcan`、`kcan_monitor`、`kcanfd_test`、DKMS 和 udev 文件。
- SDK 中的固件升级工具和固件镜像暂不使用。

WSL 中对 SDK 的脚本语法检查通过，但 `make -n -C driver` 因当前 WSL 没有匹配的内核构建目录而停止：

```text
Unable to automatically find the Kernel headers.
These are mandatory to build the kcan driver.
```

## 6. 用户已完成的系统准备

用户已确认：

- Windows 快速启动已关闭；
- Windows 安全启动（Secure Boot）已关闭；
- 原生 Ubuntu 22.04 可以浏览和编辑 E 盘项目文件。

切换到原生 Ubuntu 后仍需用命令确认实际内核、内核头文件和 E 盘挂载状态。

## 7. 切换到原生 Ubuntu 22.04 后的命令顺序

### 7.1 先做只读环境确认

```bash
uname -a
uname -r
lsb_release -ds
ls -ld /lib/modules/$(uname -r)/build
gcc --version
lsusb
ip -details link show type can
```

如果最后一条没有输出，这是预期的：驱动还没有安装或加载。

### 7.2 查找 E 盘项目位置

```bash
lsblk -f
findmnt -t ntfs,ntfs3
find /media/$USER /mnt -maxdepth 6 -type d -name claws_control_1 2>/dev/null
```

不要把 WSL 的 `~/venvs/xy_software` 当作原生 Ubuntu 的虚拟环境使用。

### 7.3 准备原生 Ubuntu 编译环境

确认内核头文件路径存在后，再执行：

```bash
sudo apt update
sudo apt install -y build-essential linux-headers-$(uname -r) libpopt-dev dkms unzip can-utils python3-venv python3-pip
```

### 7.4 准备 Linux SDK

建议把 SDK 解压到原生 Ubuntu 的 Linux 文件系统中编译，例如：

```bash
mkdir -p ~/src
unzip /path/to/KH-UCANFD_Linux_SDK.zip -d ~/src
cd ~/src/KH-UCANFD_Linux_SDK-1.4.2
```

`/path/to/...` 需要替换为原生 Ubuntu 实际可见的 Windows 下载目录，或先把 zip 复制到 `~/src`。

### 7.5 驱动安装前的检查点

先确认：

```bash
test -d /lib/modules/$(uname -r)/build && echo headers-ok
test -f driver/Makefile && echo sdk-ok
```

驱动安装脚本会安装内核模块、udev 规则，并可能屏蔽其他 PCAN 驱动。未确认环境前不要运行 `make install`、`./build.sh -dkms` 或 `./build.sh -dkms-rules`。

### 7.6 驱动安装后的只读检查

驱动安装完成后，先只检查：

```bash
modinfo kcan
lsmod | grep kcan
lsusb -t
ip -details link show type can
```

只有确认接口数量、接口名和驱动绑定关系后，才进行波特率配置和 `candump` 被动监听。暂不运行：

```text
cansend
kcanfd_test
kcanfdtest
项目中的 open/grip/hold/seek/stop 控制命令
```

## 8. 可复用性边界

可以直接复用：源代码、协议参数、Python/C 测试、配置文件、日志和本交接文档。

必须在目标 Linux 上重新完成：内核模块编译、DKMS 注册、udev 规则、USB 驱动绑定、`can0`～`can5` 通道映射和实际波特率配置。不要复制 WSL 的 `.ko` 文件、`/lib/modules` 内容或 `/etc` 驱动配置到原生 Ubuntu。

WSL 并非绝对不能使用真实 KH 硬件：由于 KH 设备是 USB 设备，理论上可以在 Windows 安装 `usbipd-win`，将设备通过 USB/IP 转交给 WSL，再在 WSL 内编译并加载匹配当前 WSL 内核的 `kcan` 驱动。当前 WSL 尚未安装 `usbipd-win`，且没有匹配的内核构建目录，因此本项目暂时只在 WSL 使用 `vcan0`，把真实硬件驱动验证放到原生 Ubuntu 22.04。

## 9. 当前停止点

当前工作停在“已准备原生 Ubuntu 迁移，尚未安装 Linux `kcan` 驱动”。下一次从第 7.1 节开始，先把原生 Ubuntu 的环境输出记录下来，再决定驱动安装命令。

## 9.1 新增跑合命令（待固件编译烧录后验证）

协议新增 `RUN_IN`（opcode `0x0E`）：Byte3 bit0 为原始 DIR 电平，Byte6 为电机掩码，0 表示三路、1/2/4 分别表示电机1/2/3；PWM 固定为 1000‰。运行期间上位机每 200ms 发送 HEARTBEAT，500ms 未收到命令时驱动板停机；STOP、Hall/CAN 故障和通信超时均停止输出。

上位机命令示例：

```bash
python -m host_demo.claw_demo --interface socketcan --channel can0 run-in 1 --dir 0 --confirm
```

该命令必须显式指定 `--confirm`。首次真机验证只能单电机、短时运行，并记录电流、温升、Hall反馈和停止响应。当前 Windows/WSL 只完成 Python 协议编码和语法测试，尚未完成 STM32 固件交叉编译、烧录和真实硬件验证。

## 10. 发给原生 Ubuntu 下 Codex 的首条指令

切换到双系统 Ubuntu 22.04、打开项目目录后，把下面整段发给新的 Codex 对话：

```text
这是 KH-UCANFDX6-Mini 的 Linux SocketCAN 真机测试。请先在当前原生 Ubuntu 22.04 中定位并完整读取项目交接文档：
claws_control_1/docs/PROJECT_HANDOFF_WSL_TO_NATIVE_UBUNTU2204.md

项目位于 Windows 的 E 盘，当前系统可能把它挂载到 /media/<用户名>/... 或 /mnt/e，请先用只读命令定位项目目录和交接文档，不要假定路径。

请严格按交接文档第 7 节从环境检查开始，先执行并报告：
uname -a
uname -r
lsb_release -ds
ls -ld /lib/modules/$(uname -r)/build
gcc --version
lsusb
ip -details link show type can

当前只允许检查内核、内核头文件、USB 设备和驱动条件，不要安装 Windows 驱动，不要刷写固件，不要运行 cansend、kcanfd_test、kcanfdtest，也不要运行 open/grip/hold/seek/stop 等手爪控制命令。没有明确继续指令前，不要加载或安装 kcan 驱动。

把输出和下一步建议写清楚后停下来，等待我确认。
```

如果新对话已经能访问项目目录，也可以直接补充：

```text
请把本次环境输出和后续验证结果继续记录到 docs/PROJECT_HANDOFF_WSL_TO_NATIVE_UBUNTU2204.md；所有真实 CAN 操作先保持被动监听，不发送控制帧。
```

## CAN ID 配置（待固件编译烧录后验证）

手爪默认 CAN ID 已调整为 11~16：命令、位置、状态、诊断、参数、故障依次对应 11~16。上位机支持 `--can-base` 和 `id-set`，可发送新基地址并请求写入 STM32 Flash；修改后需使用新的 `--can-base` 连接。首次使用该功能必须烧录包含 Flash 配置和动态过滤器代码的新固件。
