#!/usr/bin/env python3
"""
简化版 Gazebo Harmonic 启动文件
直接使用 gz sim 命令启动仿真
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_dog = get_package_share_directory('DOG')
    
    # 文件路径
    urdf_file = os.path.join(pkg_dog, 'urdf', 'DOG.urdf')
    world_file = os.path.join(pkg_dog, 'worlds', 'dog_world.sdf')
    
    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 读取 URDF 文件内容并替换 package:// 路径为绝对路径
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    
    # 替换 package://DOG 为实际路径，让 Gazebo 能找到网格文件
    robot_description_content = robot_description_content.replace(
        'package://DOG', 'file://' + pkg_dog
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # 启动 Gazebo Sim - 使用自定义世界文件
    gazebo_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-v', '4'],
        output='screen',
        shell=False
    )
    
    # 在 Gazebo 中生成机器人 - 延迟5秒等待 Gazebo 完全启动
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'DOG',
                    '-x', '0.0',
                    '-y', '0.0', 
                    '-z', '0.5',
                ],
                output='screen'
            )
        ]
    )
    
    # 时钟桥接
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # Joint State Publisher (用于测试，发布关节状态)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        gazebo_sim,
        spawn_entity,
        bridge_clock,
        joint_state_publisher_gui,
    ])
