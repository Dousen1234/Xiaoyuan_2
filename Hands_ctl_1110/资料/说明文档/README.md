# 手爪总线控制接线与使用说明

更新时间：2026-11-04

## 1. 适用范围

本文用于手爪驱动板、三指手爪、KH-UCANFDX6-Mini 或创芯 CANFD 分析仪及上位机的接线、Linux 检查和被动监听。

系统链路：

~~~text
上位机 --USB--> CAN 分析仪 --CANH/CANL/GND--> 手爪驱动板 --三路电机/Hall--> 手爪
~~~

一块驱动板控制一只三指手爪。四只手爪使用四条独立 CAN 总线。

## 2. 安全要求

1. 电脑、分析仪、驱动板和电机电源全部断电后再接线。
2. 手爪周围留出动作空间，手指之间不要放置人员的手或易损物体。
3. 驱动板输入电源12V。
4. 电源正负极按板上标识接入，不能只凭线缆颜色判断。
5. CANH 接 CANH，CANL 接 CANL，GND 接 GND。
6. 现场应准备急停或快速切断电机电源的开关。

## 3. 接线

### 3.1 CAN 分析仪到驱动板

| 分析仪 | 驱动板 | 说明 |
|---|---|---|
| CANH | CANH | CAN 高线 |
| CANL | CANL | CAN 低线 |
| GND | GND | 公共参考地 |

### 3.2 驱动板到手爪

按板上和线束标识连接：

~~~text
驱动板 M1 / HALL1  -> 手指 1
驱动板 M2 / HALL2  -> 手指 2
驱动板 M3 / HALL3  -> 手指 3
~~~

每路电机线和 Hall 线必须保持对应。线束不清晰时先编号、拍照并记录线序，再通电。

### 3.3 终端电阻

CAN 总线两端各需要约 120 欧姆终端电阻。若分析仪或驱动板已经内置终端，应确认拨码或跳线，避免出现三个或更多终端。断电后 CANH 与 CANL 间约 60 欧姆可作为两端终端均接入的参考。

## 4. 总线参数和帧 ID

| 项目 | 设置 |
|---|---|
| 总线类型 | Classical CAN 2.0A |
| 帧格式 | 11 位标准帧 |
| 仲裁速率 | 1,000,000 bit/s |
| CAN-FD | 关闭 |
| 数据长度 | 8 字节 |

| ID | 方向 | 内容 |
|---:|---|---|
| 0x180 | 上位机到驱动板 | 命令和心跳 |
| 0x181 | 驱动板到上位机 | 三路逻辑位置 |
| 0x182 | 驱动板到上位机 | 状态、故障和命令结果 |
| 0x183 | 驱动板到上位机 | Hall、PWM 和计数诊断 |
| 0x184 | 驱动板到上位机 | 参数应答 |
| 0x080 | 驱动板到上位机 | 故障帧 |

四只手爪使用独立总线，相同 ID 可以在不同总线上重复使用。不要把四只手爪直接并到同一条总线上。

## 5. Linux 检查

在原生 Ubuntu 22.04 中先执行以下只读命令：

~~~bash
uname -a
uname -r
lsb_release -ds
ls -ld /lib/modules/$(uname -r)/build
gcc --version
lsusb
ip -details link show type can
~~~

记录 Ubuntu、内核、内核头文件、USB 枚举、can0 至 can5 接口和驱动名称。没有出现 canX 时先停止，不要运行手爪控制程序。

KH 使用官方 Linux SDK 的 kcan 驱动。安装完成后先检查：

~~~bash
modinfo kcan
lsmod | grep kcan
lsusb -t
ip -details link show type can
~~~

创芯设备如果提供标准 SocketCAN 接口，也按本文检查；如果只能通过厂商 DLL、专用 API 或 Windows 工具访问，则不能把创芯专用 API 当成 KH 驱动接口。

## 6. 被动监听

确认接口名称后，以 can0 为例：

~~~bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 fd off
sudo ip link set can0 up
ip -details link show can0
candump -tz can0
~~~

candump 只接收和显示总线帧，不会主动发送手爪动作命令。可能看到 0x181、0x182、0x183 或故障帧 0x080。

没有帧时依次检查：驱动板供电、物理通道、CANH/CANL、GND、终端电阻、1 Mbit/s、CAN-FD 是否关闭，以及 can0 是否实际对应其他通道。

建议保存：

~~~bash
ip -details link show can0
candump -tz -n 100 can0
dmesg --ctime | tail -n 100
~~~

## 7. 上位机软件状态

以解压后的资料包根目录（包含“说明文档”和“总线驱动软件”的目录）为基准，工程目录为：

~~~text
总线驱动软件/claws_control_1
~~~

以下命令需要先进入解压后的 总线驱动软件/claws_control_1 目录。接收方须在自己的 Linux 上创建虚拟环境并安装 host_demo/requirements.txt 中的依赖；原开发机虚拟环境不会随压缩包自动迁移。

WSL 已完成 vcan0 回环测试和离线单元测试，但不等同于真实 KH 或创芯设备测试。Linux 应用层计划使用：

~~~bash
source ~/venvs/xy_software/bin/activate
python -m host_demo.claw_demo --interface socketcan --channel can0 monitor --duration 3
~~~

只有驱动、通道、接线、波特率和被动抓包均确认后，才允许进入上位机测试。当前不要运行 open、grip、hold、seek-open、move、relative、jog、zero 等动作命令。异常时先切断电机电源或使用急停，再保存日志和接线照片。

## 8. 完成情况和未完成工作

已准备或完成：CAN 协议、命令编码、反馈解析、参数文件、Python/C 离线测试、WSL vcan0 和 SocketCAN 应用层验证、KH 官方 Linux SDK 获取和检查，以及 WSL 到原生 Ubuntu 22.04 的交接文档。

尚未完成：

- 原生 Ubuntu 22.04 的 KH kcan 驱动编译、安装和验证；
- KH 真实 USB 绑定及 can0 至 can5 通道映射；
- 真实总线被动抓包和波特率确认；
- Linux SocketCAN 与真实驱动板的完整通信验证；
- 手爪软件与腿臂总线控制软件的合并；
- 合并后的接收分发、心跳、超时和多总线调度验证；
- 真实机械动作、电流、温升、堵转和夹持可靠性验证。

WSL 的 vcan0 测试通过，不代表真实手爪已经可以安全动作。

## 9. 腿臂软件合并约定

手爪作为独立 ClawModule 接入腿臂程序。手爪仍使用 0x180 至 0x184 和 0x080，四只手爪通道以 Linux 实际映射为准。共享一个接收套接字时，必须按 CAN 通道和 ID 统一分发，不要把手爪帧当作腿部关节帧解析。

联调顺序：单总线被动监听；无动作接入腿臂程序；单路小动作；多通道调度；最后进行完整流程和安全超时验证。

## 10. 交接资料

压缩包接收方不能访问原开发计算机。请先将压缩包完整解压到自己的工作目录，再按下面的压缩包相对路径读取交接资料；其他历史交接文档中的 E 盘、C 盘、下载目录及用户名均属于原开发机记录，须替换为接收方的实际路径；SDK 若未随包提供，需要另行获取。`总线驱动软件`目录应与本说明文档所在的`说明文档`目录同级。

后续由 Codex 或其他开发人员继续工作时，优先读取：

~~~text
总线驱动软件\claws_control_1\docs\PROJECT_HANDOFF_WSL_TO_NATIVE_UBUNTU2204.md
~~~

该文档记录工程位置、WSL 测试结果、KH SDK 信息、原生 Ubuntu 检查命令、驱动安装停止点以及不发送控制帧的验证顺序。

相关资料：

~~~text
总线驱动软件/claws_control_1/docs/CAN_PROTOCOL.md
总线驱动软件/claws_control_1/docs/CONTROL.md
总线驱动软件/claws_control_1/docs/COMMANDS.md
总线驱动软件/claws_control_1/docs/INTEGRATION.md
~~~

现场每完成一步，请记录日期、设备型号、Linux 内核、驱动版本、CAN 接口名、波特率、终端电阻状态和抓包文件名。



