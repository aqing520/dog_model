#!/usr/bin/env python3
"""
Gazebo Harmonic 仿真启动文件
用于在 Gazebo Sim (gz sim) 中加载和仿真四足机器人
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    pkg_dog = get_package_share_directory('DOG')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # 文件路径
    urdf_file = os.path.join(pkg_dog, 'urdf', 'DOG.urdf')
    gazebo_xacro_file = os.path.join(pkg_dog, 'urdf', 'DOG.xacro')
    controller_config = os.path.join(pkg_dog, 'config', 'controller.yaml')
    
    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Gazebo world file path (empty for default empty world)'
    )
    
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless.'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 alongside Gazebo'
    )
    
    # 读取 URDF 文件内容
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    
    # 替换 package://DOG 为绝对路径，让 Gazebo 能找到网格文件
    robot_description_content = robot_description_content.replace(
        'package://DOG', 'file://' + pkg_dog
    )
    
    # Robot State Publisher - 发布机器人状态
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
    
    # 启动 Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v4 ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'DOG',
            '-z', '0.5',  # 在地面上方0.5米处生成
            '-allow_renaming', 'false'
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # ROS-Gazebo 桥接 - 时钟同步
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # 加载关节状态广播器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    # 加载位置控制器
    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'position_controller'],
        output='screen'
    )
    
    # RViz2 (可选)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('DOG'), 'config', 'rviz', 'gazebo.rviz']
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        # 声明参数
        declare_use_sim_time,
        declare_world,
        declare_gui,
        declare_rviz,
        
        # 启动节点
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge_clock,
        
        # 控制器 (延迟加载)
        # load_joint_state_broadcaster,
        # load_position_controller,
        
        # RViz (可选)
        rviz,
    ])
