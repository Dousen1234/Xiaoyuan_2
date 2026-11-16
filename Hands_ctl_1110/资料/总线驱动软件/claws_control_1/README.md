# claws_control_1：MS4931 三指手爪控制

## 项目定位

本工程用于 STM32F103C8T6 + 3×MS4931 手爪驱动板。四只手爪使用同一套软件和协议，由四条独立 CAN 总线分别控制；每块板的 `CLAW_PROFILE_GROUP` 选择对应的三电机位置参数组。

- Classical CAN 2.0A，11位标准帧，1 Mbit/s，DLC=8
- 低有效 EN-PWM；PWM是占空比，不是电流或力矩
- Hall只反馈电机减速箱输入侧相对计数，不能代表手指末端实际位置
- SENSE未接MCU ADC，没有电流或夹持力闭环
- 正常停止使用 `stop`；只有FAULT状态使用 `clear`

当前源码默认组号为1，对应模组1–3；组2的实机参数记录对应模组4–6。

## PowerShell启动

```powershell
Set-Location 'E:\霄元\二期\软件\claws_control_1'
conda activate xy_software
$dll = 'C:\Users\admin\Downloads\创芯科技_CANFD分析仪(Linux版)资料20260818(固件V2.11以上)\二次开发库V1.23\x64\ControlCANFD.dll'
Remove-Module claw -ErrorAction SilentlyContinue
Import-Module .\claw.psm1 -ArgumentList @($dll, 'cxcanfd', '0', 1) -Force
claw p show
```

`ArgumentList`依次为 DLL、接口、通道、组号；`p`必须是`claw`后的第一个参数。每个新终端重新导入模块。

## 四只手爪配置

| 手爪 | Windows通道 | Linux接口 | 组号 | 模组 |
|---|---:|---|---:|---|
| 手爪1 | 0 | `can0` | 1 | 1–3 |
| 手爪2 | 1 | `can1` | 2 | 4–6 |
| 手爪3 | 2 | `can2` | 3 | 7–9 |
| 手爪4 | 3 | `can3` | 4 | 10–12 |

四只手爪暂定共用以下默认PWM（每只手爪的本地模组1/2/3，单位‰）：

| 功能 | 模组1 | 模组2 | 模组3 |
|---|---:|---:|---:|
| 张开 `seek_pwm` | 900 | 1000 | 900 |
| 闭合 `grip_pwm` | 900 | 965 | 900 |
| 夹持 `hold_pwm` | 950 | 950 | 950 |

这些值是实机调试值，尚未完成持续电流、温升、堵转和机械强度验证。

切换到另一只手爪，只需重新导入模块并修改通道、组号。例如手爪2：

```powershell
Remove-Module claw -ErrorAction SilentlyContinue
Import-Module .\claw.psm1 -ArgumentList @($dll, 'cxcanfd', '1', 2) -Force
claw clear
claw p show
```

如果目标板已经烧录匹配组号的固件，换CAN通道或换手爪不需要重新烧录。只有组号、控制逻辑、引脚、默认参数或保护逻辑变化时才需要Rebuild并烧录。

修改组号示例：

```powershell
$p = '.\App\Inc\claw_profiles.h'
$s = Get-Content $p -Raw
$s = $s -replace '#define CLAW_PROFILE_GROUP\s+\d+', '#define CLAW_PROFILE_GROUP 2'
Set-Content $p $s -Encoding utf8
```

修改后在Keil中Rebuild并烧录，之后用对应通道和组号执行 `claw p show`，确认回读组号正确。`move/relative/open`等位置命令必须使用匹配组号；`seek-open/grip/hold_pwm`使用三路独立PWM。

## 常用命令

```powershell
claw monitor --duration 3
claw seek-open                 # 三指寻张开端
claw seek-open 2               # 仅本地通道2
claw grip --clear              # 三指独立PWM闭合并进入HOLD
claw run-in 1 --dir 0 --clear --minutes 20 --confirm  # 1号电机满速跑合20分钟，到时自动STOP，dir 0/1对应正反转
claw run-in --dir 0 --clear --minutes 20 --confirm  # 全部电机...
claw move 10                   # 位置PID到逻辑10°
claw relative 2                # 相对当前三指平均Hall角移动2°
claw open                      # 回逻辑0°，不自动找机械限位
claw stop
claw clear
claw zero
```

`grip`每路连续500ms无有效Hall变化后切换自身`hold_pwm`；全部进入HOLD后持续心跳，并每约1s插入20ms的0% EN脉冲尝试重置MS4931 CLD计时。通信超时、Hall故障、STOP和CAN故障都会关闭输出。

## 参数设置、保存与恢复

在 IDLE 状态下设置参数；若正在运动或夹持，先执行 `claw stop`。以下示例对应已连接的手爪1，1/2/3始终表示该手爪的本地电机通道。

调整九项PWM并保存当前板卡的全部参数：

```powershell
claw p set 1 seek_pwm 900
claw p set 2 seek_pwm 1000
claw p set 3 seek_pwm 900
claw p set 1 grip_pwm 900
claw p set 2 grip_pwm 965
claw p set 3 grip_pwm 900
claw p set 1 hold_pwm 950
claw p set 2 hold_pwm 950
claw p set 3 hold_pwm 950
claw p save .\claw1_params.json --force
```

板卡重启后，在对应通道和组号下重新加载即可，无需重输九条设置命令：

```powershell
claw p load .\claw1_params.json
claw p show
```

| 手爪/组号 | 保存文件 |
|---|---|
| 1 | `claw1_params.json` |
| 2 | `claw2_params.json` |
| 3 | `claw3_params.json` |
| 4 | `claw4_params.json` |

切换手爪时同时更换连接的通道、组号及上述文件名。四份文件分别维护，加载前会校验文件组号和固件组号。

- `save --force`：完整回读并校验板卡参数，再替换指定文件；省略 `--force` 时拒绝覆盖已有文件。保存不写入板卡Flash。
- `load`：恢复文件中的全部参数到RAM并回读核对，不启动动作；板卡重启后需再次加载，不会自动恢复文件中的值。加载中断后先用 `claw p show` 核对状态。
- 文件包含PID、启动、PWM和超时参数，不保存逻辑零点。当前提供的文件与各组固件源码默认值对齐；后续调参并保存后，它们就是各手爪独立的调试值。

四组共用上表九项PWM，但位置PID及启动默认值仍按组保留：组1/3/4的 `max_pwm/startup_pwm/startup_ms` 为 `100/100/400`，组2为 `1000/600/200`；`ki`仅组2本地通道2为20，其余为0。`max_pwm`是位置PID上限，不限制独立的seek/grip/hold PWM；`startup_pwm/startup_ms`也用于grip起步。加载文件会恢复这些值。修改上位机或JSON不需要烧录，修改板卡上电默认值才需要重新编译烧录。

## Linux SocketCAN

默认手爪 CAN ID 为 11~16：命令 11、位置 12、状态 13、诊断 14、参数 15、故障 16。基地址可使用上位机 `id-set` 命令修改并保存到 STM32 Flash；首次加入该功能需要烧录新固件。

```bash
sudo ip link set can0 type can bitrate 1000000 fd off
sudo ip link set can0 up
python -m host_demo.claw_demo --interface socketcan --channel can0 monitor --duration 3
```

## 编译与目录

Keil工程：`MDK-ARM/claws_control_1.uvprojx`；CubeMX配置：`claws_control_1.ioc`。修改控制逻辑、默认值、引脚或组号需要重新编译烧录；在线参数调整不需要烧录。不要使用 `claws_control_0`、DM-MC02或旧MS4931_CAN固件控制本工程。

| 路径 | 内容 |
|---|---|
| `App/` | 控制、PWM/Hall、保护和协议 |
| `Core/`、`Drivers/` | STM32/HAL/CAN/TIM/IWDG |
| `host_demo/` | 创芯SDK、SocketCAN、命令行工具 |
| `MDK-ARM/` | Keil工程 |
| `tests/` | Python与C原生离线测试 |
| `docs/` | 命令、协议、控制、标定和集成说明 |

## 验证边界

Python和C原生测试只验证软件逻辑，不代表固件已烧录、接线正确或机械安全。实机调参需记录电流、温升、动作时间、Hall反馈和机械状态。


## WSL Python 虚拟环境

在 WSL 的 Ubuntu 22.04 中进入项目开发环境：

```bash
source ~/venvs/xy_software/bin/activate
```

命令行前出现 `(xy_software)` 时，表示虚拟环境已启用。

## 项目交接文档

切换到原生 Ubuntu 22.04 前，请先阅读 [WSL 到原生 Ubuntu 22.04 交接文档](docs/PROJECT_HANDOFF_WSL_TO_NATIVE_UBUNTU2204.md)。
