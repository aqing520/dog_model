cmake_minimum_required(VERSION 3.8)
project(DOG)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
# joint_state_publisher 和 rviz2 是纯 Python/可执行包，不需要 find_package
# find_package(joint_state_publisher REQUIRED)
# find_package(joint_state_publisher_gui REQUIRED)
# find_package(rviz2 REQUIRED)
# 暂时注释掉 Gazebo 包，它们可能未安装
# find_package(ros_gz_sim REQUIRED)
# find_package(ros_gz_bridge REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# 安装资源文件（等价于 ROS1 中的 install(DIRECTORY ...)）
install(
  DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)


ament_package()







