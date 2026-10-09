# taihu_ros2 —— 钛虎单腿电机控制的 ROS2 封装

把仓库的 `taihu_core`（SocketCAN + JointModule）接入 ROS2 Jazzy。

**不复制代码**：包内直接编译仓库根目录的 `src/*.cpp`，`taihu_core` 仍是单一事实来源。

## 节点

### `taihu_motor_node` —— 电机驱动节点（一条总线 = 一条腿）

- 10ms 实时线程（独立于 ROS 执行器），200Hz 发布 `sensor_msgs/JointState`
- 读取用 `0x41` 三合一命令（一次读回电流+速度+位置）

| 接口 | 类型 | 说明 |
|---|---|---|
| `~/joint_states` | topic, JointState | 200Hz 状态（joint1~4：pos/vel/effort） |
| `~/command_position` | topic, JointState | 目标位置（rad，4 元素数组） |
| `~/enable` | service, Trigger | 下发增益并使能控制循环 |
| `~/stop` | service, Trigger | 停止全部电机（抱闸） |
| `~/set_gains` | service, Trigger | 重新下发 KP/KD（运行中改参数后调用） |

参数：`can_ifname`（默认 `can0`）、`position_kp`（4000）、`position_kd`（60）、`control_rate_hz`（200）

### `taihu_gait_node` —— 步态播放节点

复用终端 demo 的单腿步态（一步 = 3 阶段：零位保持 → 正摆 0°→180° → 回摆），轨迹参数可调。

| 接口 | 类型 | 说明 |
|---|---|---|
| `~/start_gait` | service, Trigger | 启动步态线程 |
| `~/stop_gait` | service, Trigger | 停止步态（目标冻结） |
| `~/resume` | service, Trigger | 放行回摆（正摆后暂停时） |
| `~/gait_state` | topic, String | 当前步/阶段/是否暂停 |

参数：`num_steps`（1）、`phase_sec`（5.0）、`pause_after_swing`（true）

## 使用

```bash
# 0. 打开 CAN（外部终端，snap VS Code 不能 sudo）
sudo ./build/kcanctl up can0 1000000

# 1. 构建（仓库根目录）
cd ~/TaiHu_motor_control/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select taihu_ros2

# 2. 启动（每次开新终端都要 source 工作区）
source install/setup.bash
ros2 launch taihu_ros2 gait.launch.py can:=can0

# 3. 使能电机（另一个终端，同样先 source）
ros2 service call /taihu_motor_node/enable std_srvs/srv/Trigger {}

# 4. 启动步态
ros2 service call /taihu_gait_node/start_gait std_srvs/srv/Trigger {}

# 5. 正摆结束后放行回摆（pause_after_swing=true 时）
ros2 service call /taihu_gait_node/resume std_srvs/srv/Trigger {}

# 6. 观察
ros2 topic echo /taihu_motor_node/joint_states
ros2 topic echo /taihu_gait_node/gait_state
# 可视化（plotjuggler 可订阅 JointState）
```

launch 参数：`can`、`kp`、`kd`、`rate`、`steps`、`phase_sec`、`pause`，如：

```bash
ros2 launch taihu_ros2 gait.launch.py can:=can0 steps:=3 phase_sec:=3.0 pause:=false```

## 安全提醒

1. **`~/enable` 之前电机不接收任何位置指令**（demo 里的准备/使能两阶段合一）
2. 紧急停止：`~/stop` 服务，或直接 `Ctrl+C`（节点析构会 stop 全部电机）
3. 电机节点独占 CAN 接口：**不要同时运行终端 demo 和 ROS 节点**（两个进程抢同一 socket，都会乱）
4. 接口名按实际连接改：`can1` 就 `ros2 launch ... can:=can1`

## 常见问题

- **CAN 打不开**：先 `ip link show can0` 确认接口 up；`kcanctl` 要在外部终端 sudo
- **服务无响应**：确认两个终端都 source 过（`source install/setup.bash`）
- **想手动点动**：直接发 `command_position`（先 `~/enable`）：
  `ros2 topic pub /taihu_motor_node/command_position sensor_msgs/msg/JointState "{position: [0.0, 0.5, 0.5, 0.0]}" -1`
