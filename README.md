# 钛虎关节模组 CAN 控制库（C++）

基于 **Linux + 达妙 USB2CAN** 的钛虎（Titan）关节模组 C++ 控制库。支持位置控制、单腿步态、编码器标零、刹车控制、CAN ID 管理、通信诊断、多电机同步控制，以及运行数据日志记录与曲线绘图。

- **目标模组**：`CRA-RI50-60-PRO-2-81-B-2E-EC`（双编码器，减速比 81）；亦支持减速比 101 的模组
- **通讯方式**：CAN，波特率 **1M**，出厂默认 CAN ID = 1
- **上位机接口**：达妙 USB2CAN（Linux 下为虚拟串口 `/dev/ttyACM0`，波特率 921600）
- **控制模式**：位置模式（双编码器 `262144 cnt = 360°`）
- **运行数据**：记录到 `record/motor_log.csv`，可用 Python 脚本绘制曲线图

---

## 目录

- [硬件接线](#硬件接线)
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
2. **CAN 接线**：USB2CAN 的 `CAN_H` ↔ 电机 `CAN_H`，`CAN_L` ↔ 电机 `CAN_L`。
3. **终端电阻**：总线两端各接 120Ω 终端电阻（单节点时 USB2CAN 与电机各带一个）。

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
| `taihu_motor_tools_diag` | 工具 | 通信诊断 |
| `taihu_motor_tools_change_id` | 工具 | 修改 CAN ID |
| `taihu_motor_tools_reset_factory` | 工具 | 恢复出厂设置 |
| `taihu_motor_tools_set_zero` | 工具 | 编码器标零 |
| `taihu_motor_tools_openbreak` | 工具 | 打开刹车 + 重新标零 |
| `taihu_motor_tools_set_low_voltage` | 工具 | 设置低压阈值 |
| `taihu_motor_tools_scan` | 工具 | 扫描总线上的 CAN ID |

---

## 快速开始

### 1. 串口权限（二选一）

```bash
# 方式 A：加入 dialout 组（需注销重新登录）
sudo usermod -aG dialout $USER

# 方式 B：直接用 sudo 运行
```

### 2. 运行主 demo（双电机同步旋转）

```bash
sudo ./build/taihu_main
```

电机 1（ID=1）逆时针转 90°、电机 2（ID=2）顺时针转 90°，同步运动。

### 3. 标零（首次使用前）

把模组摆到期望零点，运行：

```bash
sudo ./build/taihu_motor_tools_set_zero 2   # 电机当前 CAN ID（本例已改为 2）
```

> 标零后，正弦/旋转等运动以该零点为基准。若需「打开刹车 → 手动旋转到新位置 → 重新标零」，使用 [`taihu_motor_tools_openbreak`](#taihu_motor_tools_openbreak打开刹车并重新标零)。

---

## 项目结构

```
TaiHu_motor_control/
├── CMakeLists.txt
├── include/taihu/               # 头文件
│   ├── can_interface.h          #   CAN 抽象接口 CanInterface / CanFrame
│   ├── damiao_usb2can.h         #   达妙 USB2CAN 驱动（串口协议，线程安全）
│   ├── socket_can.h             #   SocketCAN 驱动（备用）
│   ├── joint_module.h           #   关节模组控制类 JointModule
│   ├── motor_logger.h           #   日志记录器 MotorLogger（CSV）
│   ├── config.h                 #   全局配置（控制周期/采样周期等）
│   └── taihu_tools.h            #   工具函数封装（诊断/标零/旋转等）
├── src/                         # 核心库实现
│   ├── damiao_usb2can.cpp
│   ├── socket_can.cpp
│   ├── joint_module.cpp
│   ├── motor_logger.cpp
│   └── taihu_tools.cpp
├── examples/                    # 示例程序
│   ├── taihu_main.cpp           #   主 demo（双电机同步旋转）
│   ├── taihu_singleleg_step.cpp #   单腿步态（四电机，五次多项式轨迹）
│   └── taihu_single_motorctl.cpp #  单电机交互式控制（位置/正弦）
├── tools/                       # 命令行工具入口 + 绘图脚本
│   ├── taihu_motor_tools_diag.cpp
│   ├── taihu_motor_tools_change_id.cpp
│   ├── taihu_motor_tools_reset_factory.cpp
│   ├── taihu_motor_tools_set_zero.cpp
│   ├── taihu_motor_tools_openbreak.cpp
│   ├── taihu_motor_tools_set_low_voltage.cpp
│   ├── taihu_motor_tools_scan.cpp
│   └── plot_motor_log.py        #   日志绘图脚本（Python + matplotlib）
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
- [`DamiaoUsb2CanInterface`](include/taihu/damiao_usb2can.h)（默认，达妙 USB2CAN，线程安全）
- [`SocketCanInterface`](include/taihu/socket_can.h)（备用，Linux 原生 SocketCAN）

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
#include "taihu/damiao_usb2can.h"
#include "taihu/taihu_tools.h"

using namespace taihu;

DamiaoUsb2CanInterface can;
can.open("/dev/ttyACM0", 0);

rotateFromZero(can, /*can_id=*/2, /*angle_deg=*/90.0, /*kp=*/5000, /*kd=*/60);
setZeroPosition(can, 2);        // 标零
diagnose(can, 2);               // 诊断

can.close();
```

---

## 命令行工具

每个工具都是对应工具函数的薄封装，负责解析命令行参数。

### `taihu_motor_tools_diag`（诊断）

```bash
sudo ./build/taihu_motor_tools_diag [CAN ID 默认1]
```

对应 [`diagnose()`](include/taihu/taihu_tools.h:15)。打印位置/偏移/错误状态。

### `taihu_motor_tools_change_id`（修改 CAN ID）

```bash
sudo ./build/taihu_motor_tools_change_id <新ID 1~127> [当前ID 默认1]
# 示例：把 ID 从 1 改为 2
sudo ./build/taihu_motor_tools_change_id 2
```

对应 [`changeCanId()`](include/taihu/taihu_tools.h:24)。

### `taihu_motor_tools_reset_factory`（恢复出厂）

```bash
sudo ./build/taihu_motor_tools_reset_factory [当前ID 默认1]
```

对应 [`resetFactory()`](include/taihu/taihu_tools.h:32)。

### `taihu_motor_tools_set_zero`（标零）

```bash
sudo ./build/taihu_motor_tools_set_zero [CAN ID 默认1]
```

对应 [`setZeroPosition()`](include/taihu/taihu_tools.h:40)。运行前先把模组摆到期望零点。

### `taihu_motor_tools_openbreak`（打开刹车并重新标零）

```bash
sudo ./build/taihu_motor_tools_openbreak [CAN ID 默认1]
```

流程：打开刹车（使能 + 目标电流 0，电机可自由旋转）→ 等待你手动旋转到新位置并按回车 → 将当前位置保存为新零位并写入 Flash → 关闭刹车（去使能，重新抱闸）。

### `taihu_motor_tools_set_low_voltage`（设置低压阈值）

```bash
sudo ./build/taihu_motor_tools_set_low_voltage <阈值V> [CAN ID 默认1]
```

读母线电压 → 读旧阈值 → 设置新阈值 → 保存 Flash → 读回确认。设置后需重新上电生效。

### `taihu_motor_tools_scan`（扫描 CAN ID）

```bash
sudo ./build/taihu_motor_tools_scan
```

遍历 CAN ID 1~127，发送「获取当前位置」读命令，打印所有有响应的设备，用于确认总线上各电机的实际 CAN ID。

---

## 示例程序

### `taihu_main`（主 demo，双电机同步旋转）

```bash
sudo ./build/taihu_main
```

双电机按分段周期轨迹同步运动：保持零点 → ±90° → 等待 → 回零，循环 3 次。配置见 [`examples/taihu_main.cpp`](examples/taihu_main.cpp:40) 顶部常量。

### `taihu_singleleg_step`（单腿步态，四电机）

```bash
sudo ./build/taihu_singleleg_step
```

四个电机（ID=1、4 减速比 101；ID=2、3 减速比 81）按**单腿步态**分段轨迹同步运动（回零 → 膝盖摆动 ±180° → 髋转整圈 ±360° → 膝盖回零 → 回零停顿），相邻阶段用**五次多项式**插值保证位置/速度/加速度连续、轨迹平滑。配置见 [`examples/taihu_singleleg_step.cpp`](examples/taihu_singleleg_step.cpp:44)。

### `taihu_single_motorctl`（单电机交互式控制）

```bash
sudo ./build/taihu_single_motorctl
```

交互式输入电机 CAN ID、减速比，然后选择模式：
- **1 位置控制**：再输入目标角度（度）与完成时间（秒），用五次多项式轨迹在指定时间内平滑到达目标角度；
- **2 正弦轨迹测试**：电机做正弦运动。

---

## 日志与绘图

运行示例时，会把各电机的状态（时间戳、位置、速度、电流、电压、错误）以 CSV 写入 [`record/motor_log.csv`](record/motor_log.csv:1)（覆盖模式，不入库）。

### 采样周期

日志采样周期由 [`config.h`](include/taihu/config.h:14) 配置：

- [`kLoopPeriodMs`](include/taihu/config.h:14) = 5：控制周期（5ms）
- [`kLogSamplePeriodMs`](include/taihu/config.h:21) = 100：日志采样周期（0.1s）

> CAN 总线半双工且为单串口，采样与控制无法物理独立，采样周期拉长为 0.1s 以减少采样线程对总线的占用、避免争抢控制周期。

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
| 所有命令无返回 | **CAN ID 不匹配** | 用 `taihu_motor_tools_scan` 扫描实际 ID，或 `taihu_motor_tools_diag <ID>` |
| 电机完全不动 | 未供电 / 欠压 | 检查电源，读 `母线电压` 与 `错误状态` |
| 报欠压（错误状态 bit2） | 供电低于额定电压 | 提高电压，调用 `clearError()` |
| 串口打开失败 | 权限 / 设备未连接 | `sudo` 或加入 dialout 组，确认 `/dev/ttyACM0` |
| 电机抖动 | 位置环 KP 过大 | 降低 KP |
| 响应慢 / 无力 | KP 过小 | 提高 KP |
| 标零后位置未归零 | 用了「编码器归零」而非「设置偏移」 | 用 `taihu_motor_tools_set_zero`（已封装正确方式） |
| 日志采样偏慢 / 时间戳不到完整时长 | 采样线程读状态耗时过长 | 已用 `0x41` 三合一读 + `drain` 超时 1ms 优化；必要时进一步拉大采样周期 |
| 主程序无法正常结束 | 记录线程条件变量未唤醒 | 已在 `running=false` 后 `notify_all`，请确认使用最新代码 |

> 用 `taihu_motor_tools_diag` 是排查的第一手段，可读取位置、偏移、错误状态等关键信息。

---

## 参考资料

项目根目录包含以下厂家资料（协议文档与调试截图，供开发参考）：

- `钛虎C1关节电机通讯使用说明_0804.pdf`（EtherCAT + CAN 双通信协议）
- `电机协议_双编_Can版本_20250623.xlsx`（CAN 命令集）
- `位置模式示例.pdf`、`公式换算.docx`、`扭矩系数.xlsx`
- `低压阈值调整.docx`、`电流绝对值阈值.docx`、`产品信息册.xlsx`
- `taihu_breakcontrol1.jpeg`、`taihu_breakcontrol2.jpeg`（刹车控制调试截图）

达妙 USB2CAN SDK：https://gitee.com/kit-miao/damiao.git（克隆至 `.damiao_sdk/`，不入库）
