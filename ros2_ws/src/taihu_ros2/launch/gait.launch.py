"""启动钛虎单腿步态：电机驱动节点 + 步态播放节点。

用法示例：
    ros2 launch taihu_ros2 gait.launch.py can:=can0 steps:=3 phase_sec:=3.0 pause:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can = LaunchConfiguration('can')
    kp = LaunchConfiguration('kp')
    kd = LaunchConfiguration('kd')
    rate = LaunchConfiguration('rate')
    steps = LaunchConfiguration('steps')
    phase_sec = LaunchConfiguration('phase_sec')
    pause = LaunchConfiguration('pause')

    return LaunchDescription([
        DeclareLaunchArgument('can', default_value='can0',
                              description='CAN 接口名，如 can0 / can1'),
        DeclareLaunchArgument('kp', default_value='4000',
                              description='电机位置环 KP'),
        DeclareLaunchArgument('kd', default_value='60',
                              description='电机位置环 KD'),
        DeclareLaunchArgument('rate', default_value='200',
                              description='控制循环频率 Hz'),
        DeclareLaunchArgument('steps', default_value='1',
                              description='步态步数'),
        DeclareLaunchArgument('phase_sec', default_value='5.0',
                              description='每阶段时长（秒）'),
        DeclareLaunchArgument('pause', default_value='true',
                              description='正摆后是否暂停等 ~/resume 放行'),

        # 电机驱动节点：CAN + 四电机，200Hz JointState，~/enable 后开始控制
        Node(
            package='taihu_ros2',
            executable='taihu_motor_node',
            name='taihu_motor_node',
            output='screen',
            parameters=[{
                'can_ifname': can,
                'position_kp': kp,
                'position_kd': kd,
                'control_rate_hz': rate,
            }],
        ),

        # 步态播放节点：发 command_position 给电机节点，正摆后暂停等 ~/resume
        Node(
            package='taihu_ros2',
            executable='taihu_gait_node',
            name='taihu_gait_node',
            output='screen',
            parameters=[{
                'num_steps': steps,
                'phase_sec': phase_sec,
                'pause_after_swing': pause,
            }],
        ),
    ])
