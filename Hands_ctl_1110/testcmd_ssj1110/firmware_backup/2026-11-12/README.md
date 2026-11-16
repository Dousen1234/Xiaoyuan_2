# can1 手爪驱动板 固件备份与烧录记录

> 目录约定：每次烧录/刷写前，把当次的原始固件与新固件都归档到
> `testcmd_ssj1110/firmware_backup/<YYYY-MM-DD>/` 下，一个日期一个文件夹。

---

## 2026-11-12 — can1 板固件升级（旧固件 → 与 can0 一致的新固件）

### 背景

can1 通道上的手爪驱动板固件过旧：

| 问题 | 证据 |
|---|---|
| CAN 基地址是 `0x180`（384），不是 `11` | `candump can1` 只看到 `181/182/183` |
| 不支持 `RUN_IN`（opcode `0x0E`） | 发 `RUN_IN` 后状态帧 seq 不更新，诊断帧 duty 恒 `00 00 00` |
| 不支持 `SET_CAN_ID` 持久化 | Flash 配置页 `0x0800FC00` 全为 `0xFF`，基地址是编译期硬编码 |
| 组号 | 实测 `max_pwm`/`startup_pwm` 均为 100 → **group 1**，与 can0 相同 |

新固件（`claws_control_1.hex`）行为：

- 基地址默认 **11**，与 can0 完全一致（两块板在不同物理总线上，ID 相同不冲突）
- 组号 **group 1**
- 支持 `RUN_IN` / `SET_CAN_ID`，`CLAW_CAN_ID_MAX_BASE = 120`

### 文件清单

| 文件 | 大小 | 说明 |
|---|---|---|
| `can1_stm32f103c8_old_full.bin` | 65536 B | **原始完整 Flash 镜像**（0x08000000~0x0800FFFF，含配置页），可用于精确回滚 |
| `can1_old_rollback.hex` | 69891 B | 由上面 bin 的有效区域（0x08000000~0x080060FF）转换而来，可直接 `st-flash write` 烧回 |
| `can1_new_claws_control_1.hex` | 75815 B | 本次写入的新固件，与 `资料/总线驱动软件/claws_control_1/MDK-ARM/claws_control_1/claws_control_1.hex` 一致<br>MD5 `3305de175cb891219d306072842c88d8` |
| `can1_new_claws_control_1.bin` | 26936 B | 由上面的 hex 经 `objcopy -I ihex -O binary` 转换而来，**实际烧进 Flash 的就是这个文件** |

### 烧录前状态（旧固件）

```
ST-Link:  V2J46S7, serial E1007200D0D2139393740544
芯片:     chipid 0x410 = STM32F1xx_MD (STM32F103C8T6, 64 KB Flash, 20 KB SRAM)
代码范围: 0x08000000 ~ 0x080060E7  (24808 B / 24.2 KB)
配置页:   0x0800FC00 全 0xFF（magic 0x43414E49 不存在）
CAN 基地址: 0x180（编译期硬编码）
CAN 总线 ID: 181 / 182 / 183 / 184 / 185
```

### 烧录命令

```bash
cd /home/ssj/TaiHu_motor_control/Hands_ctl_1110/testcmd_ssj1110/firmware_backup/2026-11-12

# 1) 备份（已完成）
st-flash read can1_stm32f103c8_old_full.bin 0x08000000 0x10000

# 2) 整片擦除
st-flash erase

# 3) hex 转 bin（st-flash 不能正确解析 Intel HEX，必须转成 bin 再烧！）
arm-none-eabi-objcopy -I ihex -O binary \
    can1_new_claws_control_1.hex can1_new_claws_control_1.bin

# 4) 写入新固件（--reset 用 AIRCR 软复位，NRST 未接线也能生效）
st-flash --reset write can1_new_claws_control_1.bin 0x08000000

# 5) 验证基地址
candump -n 20 can1     # 应看到 00C / 00D / 00E
```

> ⚠️ **踩坑记录**：直接 `st-flash write XXX.hex` 会报
> `ERROR: The size exceeds the size of the flash (0x00010000 bytes available)`，
> 因为 st-flash 把文件大小 75815 当成原始字节数而没解析 HEX。
> **必须先 `objcopy` 转成 `.bin`（26936 B）再烧。**

### 回滚方法（万一新固件有问题）

```bash
cd /home/ssj/TaiHu_motor_control/Hands_ctl_1110/testcmd_ssj1110/firmware_backup/2026-11-12
st-flash erase
arm-none-eabi-objcopy -I ihex -O binary can1_old_rollback.hex /tmp/old.bin
st-flash --reset write /tmp/old.bin 0x08000000
candump -n 20 can1     # 应恢复为 181 / 182 / 183
```

### 烧录结果（2026-11-12 16:59，已实测验证 ✅）

```
st-flash --reset write can1_new_claws_control_1.bin 0x08000000
  Attempting to write 26936 (0x6938) bytes to stm32 address: 134217728 (0x8000000)
  27/27 pages written
  Starting verification of write complete
  Flash written and verified! jolly good!
  NRST is not connected --> using software reset via AIRCR
  Go to Thumb mode
```

回读 `0x08000000` 前 64 B 与源 bin 逐字节 `cmp` 一致：

```
00000000: 6811 0020 b501 0008 0958 0008 3555 0008   SP=0x20001168  Reset=0x080001B5
00000010: 0158 0008 790e 0008 3158 0008 0000 0000
```

CAN 功能验证（`candump can1`）：

| 检查项 | 结果 |
|---|---|
| 帧 ID | `00C` / `00D` / `00E`（基地址 = **11**）✅ |
| 帧速率 | `00C` 50 Hz、`00D` 10 Hz、`00E` 50 Hz ✅ |
| `CLEAR_FAULT` seq=1 | `00D [8] 01 01 00 00 00 00 00 00`，state=0、fault=0 ✅ |
| `PARAM_GET id=13`（组号） | `00F [8] 01 02 0D 00 01 00 00 FC` → **group 1** ✅ |
| `RUN_IN` seq=0x51 dir=1 mask=7 | `00D` 回显 seq=0x51，state=**1 (MOVING)** ✅ |
| RUN_IN 诊断帧 duty | byte4~6 = `FA FA FA` = 250×4 = **1000‰** 满占空 ✅ |
| 电机实际转动 | 电机1 **136.03°**、电机2 **87.35°**、电机3 **154.94°**（1.2 s 内）✅ |
| 霍尔码 | 出现 **54 种**组合（`010101`→`020203`→`050504`→…），说明确实在转 ✅ |
| 堵转/故障 | 全程无故障帧 `0x010`，STOP 后 state 回 **0 (IDLE)** ✅ |

**结论：can1 板的固件已与 can0 完全一致，`RUN_IN` 跑合功能可用。**

> 💡 之前"霍尔信号冻结"的判断是**旧固件不支持 RUN_IN** 导致的假象 —— 旧固件
> 收到 `0x0E` 直接忽略，PWM 恒 0，电机自然不动、霍尔码不变。新固件烧录后
> 三个电机全部正常旋转，硬件接线没有问题。

### 日常硬件检查（仍然建议保留）

换电机 / 换线之后，仍建议先做一次 20 秒空转确认：

1. 断开 12 V，检查电机 3×UVW 相线 + 3×霍尔线是否接好
2. 确认 `U11` 已短接（总线电阻应≈60 Ω，用万用表量）
3. 上电后 `candump can1 "00E"`，手动拨动电机，观察 byte1~3 的 hall_code 是否变化
4. 首次跑合使用最保守参数：`./build/test_rotate --channel can1 --mask 1 --seconds 5 --confirm`

否则固件会在 `CLAW_STALL_TIMEOUT_MS = 800 ms` 后判堵转，
`latch_fault(MOTOR_x)` 锁死并关闭输出（`state=3`，`fault=0x01|0x02|0x04`）。

### 相关常量（来自固件 `App/Inc/claw_config.h`）

```
CLAW_RUN_IN_DUTY_PERMILLE    1000   RUN_IN 固定占空比（不经过 PID）
CLAW_STALL_TIMEOUT_MS         800   无霍尔变化多久判堵转
CLAW_COMMAND_WATCHDOG_MS      500   命令看门狗，RUN_IN 期间需每 200 ms 心跳
CLAW_CAN_ID_DEFAULT_BASE       11   默认基地址
CLAW_CAN_ID_MAX_BASE          120   基地址上限（0x180 不可能是新固件产生的）
CLAW_CAN_CONFIG_ADDR  0x0800FC00   基地址持久化配置页
```