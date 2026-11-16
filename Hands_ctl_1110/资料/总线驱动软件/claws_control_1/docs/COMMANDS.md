# 命令速查

## 连接

PowerShell 工程根目录执行：

```powershell
conda activate xy_software
$dll = 'C:\Users\admin\Downloads\创芯科技_CANFD分析仪(Linux版)资料20260818(固件V2.11以上)\二次开发库V1.23\x64\ControlCANFD.dll'
Import-Module .\claw.psm1 -ArgumentList @($dll, 'cxcanfd', '0', 1) -Force
```

手爪默认使用 CAN ID 11~16。修改并保存基地址（例如改为 23，使用 23~28）：

```bash
python -m host_demo.claw_demo --interface socketcan --channel can0 --can-base 11 id-set 23
```

修改后重新连接时使用 `--can-base 23`。该命令会写入 STM32 Flash，不需要再次烧录；首次加入此功能仍需烧录新固件。

`ArgumentList`依次为 DLL、接口、通道、组号；`p`必须是`claw`后的第一个参数。每条独立总线按 README 的通道和组号表切换。

## 动作

| 命令 | 作用 |
|---|---|
| `claw monitor --duration 3` | 监听反馈 |
| `claw seek-open` / `claw seek-open N` | 三指/单指寻张开端，N=1/2/3 |
| `claw grip --clear` | 三指独立PWM闭合并进入HOLD |
| `claw move A` | 位置PID到绝对角度A |
| `claw relative D` | 相对当前三指平均Hall角移动D |
| `claw open` | 位置PID回逻辑0，不自动找机械限位 |
| `claw f1 A`、`f2`、`f3` | 单指位置PID |
| `claw stop` | 正常停止 |
| `claw clear` | 清故障，不启动动作 |
| `claw zero` | 将当前Hall位置设为逻辑零点 |
| `claw jog N --dir 0/1` | 固定短点动，仅方向观察 |
| `python -m host_demo.claw_demo --interface socketcan --channel can0 run-in all --dir 0 --confirm` | 三路1000‰持续跑合，输入 `stop` 或 Ctrl+C 停止 |

`run-in` 的电机参数可填 `all`、`1`、`2` 或 `3`；方向必须填原始 DIR 电平 `0` 或 `1`。该命令持续发送心跳，驱动板在500ms未收到命令时自动停机。首次实机建议单电机运行，并确认没有机械干涉、过流和过热风险。若此前单独执行过 `clear` 后关闭了 CAN 程序，使用 `--clear` 可在同一 CAN 连接内清故障再开始跑合；使用 `--minutes 20` 可定时跑合 20 分钟，到时自动发送 `STOP`：`python -m host_demo.claw_demo --interface cxcanfd --channel 0 --dll $dll --can-base 11 run-in 1 --dir 0 --clear --minutes 20 --confirm`。

位置命令的角度来自Hall减速箱输入侧计数，不是手指末端传感值。正常停止使用`stop`，只有FAULT才使用`clear`。

## 参数

| 命令 | 作用 |
|---|---|
| `claw p show` | 回读当前板卡的全部参数 |
| `claw p set N 参数名 数值` | 设置RAM参数并核对回读；N=1/2/3为本地通道，N=0为全局参数 |
| `claw p save .\claw1_params.json --force` | 回读并保存全部参数，允许替换已有文件；省略--force则拒绝覆盖 |
| `claw p load .\claw1_params.json` | 检查组号、恢复全部RAM参数并回读核对，不启动动作 |

在IDLE下调参或加载；需要时先执行 `claw stop`。手爪1–4分别使用 `claw1_params.json` 至 `claw4_params.json`。参数未写入板卡Flash，重启后需重新加载；文件不保存逻辑零点。PWM单位为‰，范围0–1000。

完整九条PWM设置命令、默认值与重启恢复流程统一见 [README参数设置、保存与恢复](../README.md#参数设置保存与恢复)。JSON同时包含PID和启动参数，不是只保存九项PWM。

`grip`不接收角度，不使用位置PID；每路连续500ms无Hall变化后转入自身`hold_pwm`。HOLD要求上位机每200ms发送心跳，失联超过500ms关闭输出。

## 四只手爪

每只手爪独立总线，可使用相同CAN ID；切换手爪时重新导入模块并选择正确通道和组号。已烧录匹配组号固件后，换通道不需烧录；组号、控制逻辑、引脚、默认值或保护逻辑变化时需Rebuild并烧录。

