import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


from launch.actions import SetEnvironmentVariable


pkg_path = get_package_share_directory('DOG')

set_gz_paths = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=f"{pkg_path}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
)

def generate_launch_description():
    # 包名
    package_name = 'DOG'
    pkg_path = os.path.join(get_package_share_directory(package_name))

    # 模型与世界文件路径
    urdf_file = os.path.join(pkg_path, 'urdf', 'DOG.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.sdf')

    # 读取 URDF 文件（非 xacro）
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    robot_description_config = robot_description_config.replace(
    'package://DOG',
    f'file://{pkg_path}'
)

    # 机器人初始位姿
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.3'
    spawn_yaw_val = '0.0'

    # 启动 Gazebo Harmonic
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + world_file}.items(),
    )

    # 启动 ROS-Gazebo 桥接的模型生成节点
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'dog_model',
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', spawn_z_val,
            '-Y', spawn_yaw_val
        ],
        output='screen'
    )

    # 启动 robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True
        }]
    )

    # ROS-Gazebo 桥
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_path, 'config', 'dog.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # 启动所有节点
    return LaunchDescription([
        set_gz_paths,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        node_robot_state_publisher,
    ])
