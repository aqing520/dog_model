import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取默认路径
    # 找到的是install/DOG/share/DOG
    urdf_tutorial_path = get_package_share_directory('DOG')

    # 锁定在CMakeLists.txt中install(DIRECTORY ...)指定的路径  share/DOG/urdf/DOG.urdf
    default_model_path = urdf_tutorial_path + '/urdf/DOG.urdf'
    default_rviz_config_path = urdf_tutorial_path + '/config/rviz/display_model.rviz'


    # 为 Launch 声明参数 // 方便更改URDF文件路径
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')
    

    # 获取文件内容生成新的参数，将xacro模型文件转换为robot_description参数，提供给robot_state_publisher节点使用
    # xacro可以简化URDF文件
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
    

    # 状态发布节点 启动robot_state_publisher
    # 包名和可执行文件名 参数
    # parameters 是传的字典格式
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 关节状态发布节点 启动joint_state_publisher
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    #添加控制滑钮  如果gui参数为true 则启动joint_state_publisher_gui节点
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
    )


    # RViz 节点    启动rviz2  等价于 ros2 run rviz2 rviz2 -d <path_to_config>
    # arguments 是直接传的格式
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
    )

    # 返回 LaunchDescription 对象 多节点启动
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])