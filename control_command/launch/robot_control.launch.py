#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # 声明参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='auto',  # auto表示自动检测
        description='Use simulation clock if true, real clock if false, auto to detect automatically'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='sim',
        choices=['sim', 'real'],
        description='Robot type: sim for simulation, real for real robot'
    )
    
    # 机器人控制节点
    robot_control_node = Node(
        package='control_command',
        executable='robot_control_node',
        name='robot_controller',
        output='screen',
        parameters=[
            # use_sim_time将由节点自动检测，但也可以通过参数覆盖
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_type_arg,
        robot_control_node
    ])
