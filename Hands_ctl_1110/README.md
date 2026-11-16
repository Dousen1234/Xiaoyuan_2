# Hands_ctl_1110 — 三指手爪接入工作区

本目录集中存放手爪相关的**厂家原始资料**、**我们自写的上位机程序**，以及**接入文档**。
手爪相关的一切改动都只落在本目录内，**不修改腿臂主工程**。

> 目标：在另一台 Linux 机器上 `git clone` 之后，照本文档走完就能编译、能上总线、能控制手爪。

---

## 0. 30 秒速览

```bash
# 1) 拿到代码
git clone git@github.com:Dousen1234/Xiaoyuan_2.git
cd Xiaoyuan_2/Hands_ctl_1110/testcmd_ssj1110

# 2) 编译 + 跑离线自测（不接硬件也能验证）
./build.sh

# 3) 起 CAN 通道（每次开机后一次，需要 root）
sudo ip link set can0 up type can bitrate 1000000

# 4) 先只看计划，不动电机
./test_positionctrl --motors 123 --dry-run

# 5) 确认无误再真机运行（会真的驱动电机！）
./test_positionctrl --motors 123 --confirm
```

如果你**从没接过这只手爪**，请先读 [`docs/手爪快速上手指南.md`](docs/手爪快速上手指南.md)，
从接线和上电前检查开始，不要跳过第 3 章「安全须知」。

---

## 1. 三句话讲清这个系统

1. **一块驱动板 = 一只手爪 = 一条独立 CAN 总线**。四只手爪必须四条总线，不能并联。
2. **经典 CAN**：1 Mbit/s、11 位标准帧、8 字节、**CAN FD 关闭**。默认基址 `11`（命令帧 ID `0x00B`）。
3. **Hall 计数是减速箱输入侧的相对计数，不是手指角度**。手指角度靠 `SET_ZERO` 人为定义；
   `position_counts` 是「上电以来累计走过的霍尔边沿数」，上报时被 int16 钳在 ±327.67°。

---

## 2. 目录结构

| 路径 | 内容 | 说明 |
|---|---|---|
| **`testcmd_ssj1110/`** | **我们自写的 C++17 上位机 + 离线自测** | ★ 日常真正会用到的东西，见[其 README](testcmd_ssj1110/README.md) |
| `docs/` | **我们写的接入文档** | 随接入手爪持续更新 |
| `资料/说明文档/` | 厂家接线与安全说明、补充说明 | 接线 / 模组编号 / U11 短接的权威出处 |
| `资料/手爪驱动板/` | 上层 / 下层 PCB 工程与接线照片 | 视觉参考；`.eprj2` / `.epro2` 需嘉立创 EDA 打开 |
| `资料/手爪电机/` | 电机型号手册（鸣志 `ECU16024H12-S104-PG16M-0021`） | PDF 为**图片，无文字层**，需人工看 |
| `资料/总线驱动软件/claws_control_1/` | STM32F103 固件源码 + Python 上位机 + 协议文档 | 协议与默认参数的**权威出处**，改固件看这里 |
| `资料 copy/` | `资料/` 的重复副本（约 93 MB） | **不入库**（见 `.gitignore`），避免改错文件 |

### 2.1 `testcmd_ssj1110/` 组成

| 文件 | 作用 |
|---|---|
| `claw_can.h` / `claw_can.cpp` | Linux SocketCAN 封装 + 协议编解码（CRC-8/SAE-J1850、帧构造、反馈解析） |
| `position_ctrl_logic.h` | **纯逻辑**（无 I/O、无时钟），位置控制的安全判定，被下面两个程序共用 |
| `test_rotate.cpp` | 功能 1：三指往复旋转，验证总线与固件通路 |
| `test_positionctrl.cpp` | 功能 2：单路 / 多路位置控制（`m1` / `m12` / `m123`） |
| `selftest_sim.cpp` | 离线自测：CAN 通信流程（10 组 / 57 断言） |
| `selftest_position.cpp` | 离线自测：位置控制安全逻辑（26 组 / 168 断言） |
| `build.sh` | 一键编译 + 跑离线自测，编译完直接可用 |
| `CMakeLists.txt` | 等价的 CMake 构建（含 `ctest` 注册） |
| `firmware_backup/` | 板上固件的原始备份与 README，**刷坏了能回滚** |

---

## 3. 文档索引

| 文档 | 面向 | 内容 |
|---|---|---|
| [`docs/新设备拉取即用指南.md`](docs/新设备拉取即用指南.md) | **换机器 / 重新配环境** | 依赖清单 → clone → 编译 → 起总线 → 排错，纯环境向 |
| [`docs/手爪快速上手指南.md`](docs/手爪快速上手指南.md) | **零基础，第一次上手** | 接线 → 总线打通 → Python 环境 → 第一条命令 → 动作阶梯 → 参数 → 标定 → 接入腿臂 → 排错（949 行，最全） |
| [`testcmd_ssj1110/README.md`](testcmd_ssj1110/README.md) | **日常操作 / 改代码** | 自写上位机的用法、参数、三个固件约束、踩坑记录、协议速查 |
| [`testcmd_ssj1110/firmware_backup/2026-11-12/README.md`](testcmd_ssj1110/firmware_backup/2026-11-12/README.md) | **固件刷写与回滚** | 板上固件的备份说明、刷写方法、回滚步骤 |

**阅读顺序建议**：本文件 → `新设备拉取即用指南.md`（换机器时）→ `手爪快速上手指南.md` 第 3~6 章（第一次接线）→ `testcmd_ssj1110/README.md`（日常操作）。

---

## 4. 硬件速查

| 项目 | 值 |
|---|---|
| 驱动板 MCU | STM32F103C8T6（chipid `0x410`，64 KB Flash @ `0x08000000`，20 KB SRAM，1 KB 页） |
| 电机 | 鸣志 `ECU16024H12-S104-PG16M-0021`，无槽无刷 + **PG16M 减速箱 628.5:1**，1 对极，12 V |
| 预驱 | MS4931 × 3（每指一片） |
| PWM | TIM1 CH1/CH2/CH3，20 kHz（`APP_PWM_PERIOD = 3600` @ 72 MHz），**低有效** |
| 霍尔 | M1 = PB6/PB7/PB8，M2 = PB9/PB10/PB11，M3 = PB12/PB13/PB14 |
| 使能 | `nENABLE` 低有效，PA8 / PA9 / PA10 |
| 抱闸 | `NBRAKE` PA3 / PA4 / PA5；**`CLAW_ACTIVE_BRAKE_ENABLED = 0`（从不施加）** |
| 分辨率 | 1 count = `360/(6×628.5)` = **0.095466°**；3431 counts ≈ 327.67° |
| CAN 分析仪 | PEAK PCAN-USB FD（`0c72:0012`）→ `can0..can5`；或鲲弘 KH-UCANFDX6-Mini |

---

## 5. ⚠ 安全须知（动手前必读）

1. **没有保持力矩。** `CLAW_ACTIVE_BRAKE_ENABLED = 0`，短接制动从不施加，电机断电后
   手指**只靠 628.5:1 减速箱的自锁**撑着。断电瞬间若手指上有负载，会缓慢松开。
2. **MS4931 的 SENSE 没有接到 ADC**，固件**没有电流环、没有软件限流**。堵转就是真堵转。
   所以 `--max-pwm` 建议 **≤ 600**（固件 PID 上限只有 100‰，默认参数下不会跑满）。
3. **上位机 500 ms 看门狗**：连续 500 ms 看不到有效霍尔变化就发 `STOP` 断电。
   这比固件自带的 800 ms 堵转保护更快 ⇒ 一定是上位机先动手 ⇒ **不会锁故障**，不用 `CLEAR_FAULT`。
4. **`SET_ZERO` 只写 RAM**，掉电失效；不改固件参数、不写 Flash，所以每次上电都要重新标零。
5. **`SET_ZERO` 不转电机**，但它一次清三路，并把「当前物理位置」定义为 0°。
   标零后角度区间变成 `[0°, +120°]`，**不能输入负角度**；未标零时才允许 `[-30°, +120°]`。
6. **多路位置控制有三个固件坑**（掩码只有 4 种、目标坐标取平均、先到位的一路会被单独断电）。
   详见 [`testcmd_ssj1110/README.md` 的「多路控制的三个固件约束」](testcmd_ssj1110/README.md#多路控制的三个固件约束)。
7. **先 `--dry-run`，再 `--confirm`。** 程序默认只打印将要下发的 CAN 帧，不会驱动电机。

---

## 6. 当前状态

| 事项 | 状态 |
|---|---|
| `testcmd_ssj1110` 离线自测 | ✅ `selftest_sim` 10 组 57 断言、`selftest_position` 26 组 168 断言，全绿 |
| `testcmd_ssj1110` 编译 | ✅ `-Wall -Wextra` 零 warning，`ctest` 2/2 通过 |
| 单路位置控制（`m1` / `m2` / `m3`） | ✅ 实机验证通过 |
| 三路同时（`m123`，单帧掩码 `0x07`） | ✅ 实机验证通过 |
| 两路组合（`m12` / `m13` / `m23`） | ⚠ 固件无单帧支持，程序拆成**两条顺序执行**；两路之间会有停顿，**不是同时动** |
| 位置帧读数区间 | ⚠ 累计行程超过 ±327.67°（3431 counts）后 int16 **饱和**，程序会拦截并提示重新标零 |
| Linux SocketCAN 迁移 | ✅ `testcmd_ssj1110` 已在 `can0` 上跑通 |
| 腿臂合并 | ❌ **未完成**，`Hands_ctl_1110` 与 `examples/walk_control_V1.cpp` 之间尚无联合代码 |
| 通道缺口 | ⚠ 4 腿 + 4 爪 = 8 路，KH-UCANFDX6-Mini 只有 6 路，需先裁剪或加模块 |

---

## 7. 同步说明（哪些入库、哪些不入库）

本目录约 **188 MB**，但真正入库的只有约 **19 MB / 241 个文件**。这是有意的：

| 不入库 | 原因 | 忽略规则来源 |
|---|---|---|
| `资料 copy/`（93 MB） | 与 `资料/` 内容完全重复 | `Hands_ctl_1110/.gitignore` |
| `*.jsonl` 调试日志（约 71 MB） | 单次抓包日志，无复现价值 | `Hands_ctl_1110/.gitignore` |
| `.venv/`、`__pycache__/`、`.pytest_cache/` | 可重建 | 同上 |
| `MDK-ARM/Objects`、`Listings`、`DebugConfig`、`RTE/`、`*.o/*.d/*.crf/*.lst/*.map` | Keil 编译中间产物 | 同上 |
| `testcmd_ssj1110/build/` | CMake 缓存 | `testcmd_ssj1110/.gitignore` |
| `test_rotate`、`test_positionctrl`、`selftest_sim`、`selftest_position` | Linux ELF 可执行文件，`./build.sh` 可重新生成 | `testcmd_ssj1110/.gitignore` |
| `*.pcap`、`*.pcapng` | 抓包文件 | `Hands_ctl_1110/.gitignore` |

**入库的关键内容**：全部源码、全部文档、`资料/` 里的厂家 PDF / PCB 工程 / 固件源码 / Python 上位机 / 协议文档，
以及 `testcmd_ssj1110/firmware_backup/` 下的**固件烧录镜像**。

所以换机器 clone 之后：**源码、资料、固件镜像都是齐的**，只有编译产物需要重新生成——这正是 `./build.sh` 存在的意义。

### 7.1 固件镜像为何入库

`testcmd_ssj1110/firmware_backup/2026-11-12/` 下的 4 个镜像（共约 237 KB）**特意保留**：

| 文件 | 大小 | 用途 |
|---|---|---|
| `can1_new_claws_control_1.bin` / `.hex` | 27 KB / 74 KB | 当前板上的新固件，刷写用 |
| `can1_old_rollback.hex` | 68 KB | 回滚用的旧固件 |
| `can1_stm32f103c8_old_full.bin` | 64 KB | 整片 Flash 完整备份（含 bootloader） |

`Hands_ctl_1110/.gitignore` 里全局忽略 `*.bin` / `*.hex`（挡掉 Keil 中间产物），
但用 `!testcmd_ssj1110/firmware_backup/**` 开了例外。这样新设备 clone 下来就能直接刷写或回滚，
不必再去旧机器上找镜像。刷写方法见该目录的 `README.md`。

---

## 8. 遇到问题

| 现象 | 去哪儿看 |
|---|---|
| `ip link set can0` 报权限不足 / `sudo` 不可用 | [`docs/新设备拉取即用指南.md`](docs/新设备拉取即用指南.md) 的「sudo 不可用」一节 |
| 收不到任何 CAN 帧 | [`docs/手爪快速上手指南.md`](docs/手爪快速上手指南.md) 第 6.5 节 |
| 电机不转 / 转到一半停 | [`testcmd_ssj1110/README.md`](testcmd_ssj1110/README.md) 的「故障排查」「踩坑记录」 |
| 位置读数卡在 ±327.67° | 累计行程饱和，需重新 `SET_ZERO`，见 `testcmd_ssj1110/README.md` 的「位置坐标的真相」 |
| 多路控制只动了一路 | 两路组合是顺序执行；先到位的一路被固件单独断电，见「多路控制的三个固件约束」 |