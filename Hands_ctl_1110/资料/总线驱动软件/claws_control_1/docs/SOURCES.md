# 移植来源

板型与引脚：用户确认沿用MS4931_CAN的STM32F103C8T6＋3×MS4931双层板。硬件目录：`E:\霄元\二期\电控设计\手爪驱动板\手爪驱动板（from lqh）`。

参考对话：手爪驱动板测试；腿臂手爪CAN通信控制Demo。原MS4931_CAN和xiaoyuan_2-legs均未修改。

移植以MS4931_CAN的安全GPIO/反相PWM/CAN队列/IWDG为板级基础，叠加本聊天的Hall角度独立PID。旧开环协议、电机状态机及预编译HEX没有混入新工程。

来源文件SHA-256（来源快照，不是新工程构建哈希）：

- `MS4931_CAN\Core\Src\board.c`：`d714a33c4eb9ec8ba199b18bc44cdc328ee84f7842537342838707ac54067c9e`
- `MS4931_CAN\MS4931_CAN.ioc`：`2e54a24b424299b0c5cec6c473feaddcd4860122272a2b0c03df8d60a8efd793`
- `MS4931_CAN\MDK-ARM\MS4931_CAN.uvprojx`：`be8ea6a328ae046065d949083736f9f1ddf6355a5d6d65a5c26063645554b9d1`
