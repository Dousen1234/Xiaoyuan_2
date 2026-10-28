# TaiHu Motor Control API

本文面向需要在其他 C++ 项目中复用驱动的开发者。实现使用 C++17，公开头文件位于 `include/taihu/`，核心静态库目标为 `taihu_core`。

## 复用入口

各模块均位于 `taihu` 命名空间。CAN 设备的打开、关闭由应用程序管理；驱动对象通过 `CanInterface&` 使用已打开的接口。

```cpp
#include "taihu/joint_module.h"
#include "taihu/socket_can.h"

int main() {
    taihu::SocketCanInterface can;
    if (!can.open("can0", 1000000)) return 1;

    taihu::JointModule motor(can, 1, 1);
    motor.setGearRatio(81.0);
    motor.clearError();
    motor.setPositionKp(4000);
    motor.setPositionKd(60);

    double position_rad = 0.0;
    const bool ok = motor.readPosition(position_rad);
    motor.stop();
    can.close();
    return ok ? 0 : 1;
}
```

构建本仓库并链接核心库：

```cmake
add_subdirectory(path/to/TaiHu_motor_control taihu-build)
target_link_libraries(my_app PRIVATE taihu_core)
```

本仓库的 CMake 还会定义示例和工具目标。若集成工程不希望生成这些可执行程序，可在自身工程中按根目录 `CMakeLists.txt` 的 `taihu_core` 源文件列表创建库目标，并将 `include/` 设为公开 include 目录。

```bash
cmake -S . -B build
cmake --build build --target taihu_core
```

## CAN 通信层

头文件：`taihu/can_interface.h`、`taihu/socket_can.h`。

| 类型 / 方法 | 用途 |
|---|---|
| `CanFrame` | CAN 帧：`id`、`data[8]`、`dlc`、`is_extended` |
| `CanInterface::open(const char*, int)` | 打开设备；波特率单位 bit/s |
| `CanInterface::close()` | 关闭设备 |
| `CanInterface::send(const CanFrame&)` | 发送 CAN 帧 |
| `CanInterface::receive(CanFrame&, int timeout_ms = 0)` | 接收帧；0 非阻塞，负数无限等待 |
| `CanInterface::drain()` | 清空接收残帧；默认空实现 |
| `SocketCanInterface` | Linux SocketCAN 实现；CAN 速率需预先配置并将接口置为 up |

## 关节电机

头文件：`taihu/joint_module.h`。构造函数为 `JointModule(CanInterface&, uint32_t tx_id = 1, uint32_t rx_id = 1)`。位置使用输出端弧度，速度使用输出端 rad/s，电流使用 mA；默认减速比为 81，可用 `setGearRatio()` 覆盖。

| API | 用途 / 协议 |
|---|---|
| `stop()`、`clearError()` | 停止抱闸（`0x02`）；清除错误（`0x0B`） |
| `setMotorId(id)`、`saveParams()`、`resetFactory()` | 改 ID（`0x2E`）、保存 Flash（`0x0E`）、恢复出厂（`0x0F`） |
| `setZeroPosition()`、`setPositionOffset(cnt)` | 硬件标零（`0x50`）、设置软偏移（`0x53`） |
| `readPositionOffset(cnt)`、`readPositionCnt(cnt)` | 读偏移（`0x54`）、读原始位置计数（`0x08`） |
| `setTargetPosition(rad)`、`setTargetVelocity(rad_s)`、`setTargetCurrent(ma)` | 位置（`0x1E`）、速度（`0x1D`）、电流（`0x1C`）控制 |
| `setPositionKp(kp)`、`setPositionKd(kd)` | 设置位置环增益（`0x2B`、`0x2D`） |
| `setVelocityKp(kp)`、`setVelocityKi(ki)`、`setVelocityKd(kd)` | 设置速度环增益（`0x29`、`0x2A`、`0x33`） |
| `readPosition(rad)`、`readVelocity(rad_s)`、`readCurrent(ma)` | 读位置（`0x08`）、速度（`0x06`）、电流（`0x04`） |
| `readCurrentVelocityPosition(ma, rad_s, rad)` | 三合一状态读取（`0x41`），适合周期采样 |
| `readErrorState(bits)`、`readBusVoltage(v)` | 读错误状态（`0x0A`）、母线电压（`0x14`） |
| `readLowVoltageThreshold(v)`、`setLowVoltageThreshold(v)` | 读/设低压阈值（`0x8C` / `0x89`）；设置后重启生效 |

每个读写 API 返回 `bool` 表示通信与解析是否成功。电机 ID 修改后若需掉电保留，需使用新 ID 调用 `saveParams()`。

## 高层电机工具

头文件：`taihu/taihu_tools.h`。

| API | 用途 |
|---|---|
| `initMotor(CanInterface&, const MotorConfig&)` | 按名称、ID、减速比和位置增益创建配置好的 `JointModule` |
| `readMotorStatus(can, id, gear_ratio, status)` | 读取位置、速度、电流、电压和错误状态 |
| `diagnose(can, id)` | 输出诊断信息 |
| `changeCanId(can, cur_id, new_id)` | 修改并保存 CAN ID（1~127） |
| `resetFactory(can, id)` | 恢复并保存出厂设置 |
| `setZeroPosition(can, id)` | 软标零并保存 |
| `setLowVoltageThreshold(can, id, voltage_v)` | 设置低压保护阈值并读回确认 |
| `rotateFromZero(can, id, angle_deg, kp, kd)` | 回零位后旋转指定角度 |

`MotorStatus` 提供 `can_id`、`position_rad`、`velocity_rad_s`、`current_ma`、`bus_voltage_v`、`error_bits`。

## 扭矩传感器

头文件：`taihu/torque_sensor.h`。构造函数 `TorqueSensor(CanInterface&, uint32_t rx_id = 0x10, uint32_t tx_id = 0x15)` 不会打开 CAN；传感器和 CAN 接口生命周期由调用者管理。

主要 API：`requestSingle()` 单次采样；`startContinuous(OutputRate)` / `stopOutput()` 控制连续输出；`startRxThread()` / `stopRxThread()` 管理后台收帧；`latest(Sample&, max_age_ms = 200)` 读取新鲜的配对样本；`startZeroCalib(n = 100)` 校准零偏，`clearZero()` 清除零偏；`samplesPerSecond()` 查询配对采样率。波特率和 ID 可分别通过 `setBaudRate(BaudRate)`、`setId(new_rx_id, new_tx_id)` 修改。

`Sample` 包含同步号、已扣零偏扭矩 `torque_nm`、原始扭矩 `torque_raw_nm`、桥压 `bridge_mv_per_v` 和时间戳。总线上按回复 ID 与 DLC 过滤，能与电机帧共存。改 ID 后需用新 ID 重新构造对象。协议参考：[HIT 扭矩与电压 CAN 协议](manuals/CAN-2Frames-Torque%20and%20Voltage-HIT-V2.0.pdf)。

## MPT 磁编码器

头文件：`taihu/mpt_encoder.h`。编码器使用独立 RS485 串口，不经过 CAN。先创建并打开 `Rs485Port`，再将其引用传给 `MptEncoder`：

```cpp
taihu::Rs485Port port;
if (!port.open("/dev/ttyUSB0", 2500000)) return 1;
taihu::MptEncoder encoder(port, taihu::MptEncoder::Protocol::RS485);
taihu::MptEncoder::Sample sample;
if (encoder.readPositionStatus(sample)) {
    // sample.angle_deg, sample.raw, sample.status
}
```

协议枚举为 `Protocol::RS485`（R 款）和 `Protocol::BUS`（D 款）；BUS 地址范围 0~31，默认出厂地址 `0x1F`。`probe()` 可探测协议和地址；`readPosition()` / `readPositionStatus()` 读取位置；`setZero()` 设置零位；`setBusAddress(new_addr)` 修改 BUS 地址（RS485 协议不支持）。`Sample` 提供原始值、角度、圈数、错误/警告位、状态字节和时间戳。串口波特率由型号配置，必须与编码器一致。

协议参考：[MPT 磁编码器数据手册 v0.6](manuals/MPT磁编码器数据手册v0.6.pdf)。

## 日志与公共配置

- `taihu/motor_logger.h`：`MotorLogger(filename)` 以覆盖模式创建 CSV；按顺序调用 `writeHeader(motor_ids)` 和 `log(timestamp_s, motors)`，其中 `motors` 顺序必须与表头 ID 顺序一致；可用 `isOpen()` 检查和 `close()` 关闭。
- `taihu/config.h`：公共控制周期与到位判定常量，包括 `kLoopPeriodMs`、`kLogSamplePeriodMs`、`kSettleThresholdRad`、`kSettleTimeoutMs`、`kPollPeriodMs`、`kDegToRad`。调用方可直接引用，不应依赖其值替代每个设备的实际配置。

## ROS 2 接口

ROS 2 节点封装位于 `ros2_ws/src/taihu_ros2/`，话题、服务、参数及构建运行说明以[ROS 2 包文档](../ros2_ws/src/taihu_ros2/README.md)为准。主要节点为 `taihu_motor_node`（电机控制）、`taihu_gait_node`（步态播放）和 `taihu_encoder_node`（MPT 编码器）。

## 注意事项

- CAN 电机控制节点/进程应独占相应 CAN socket，避免多个控制进程同时收发导致命令和响应交错。
- 扭矩传感器可与电机共享 CAN 总线；MPT 编码器走独立串口。
- 标零、修改 ID、恢复出厂和写入 Flash 都会更改设备状态；执行前确认电机已停止，且总线上没有其他同 ID 设备。
- 头文件中的完整签名和协议实现以 `include/taihu/*.h` 与 `src/*.cpp` 为准。