cmake_minimum_required(VERSION 3.0.2)
project(pallet_gazebo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rospy
  tf
  std_msgs
  gazebo_msgs
  dsr_msgs
  message_generation
)

catkin_python_setup()

## Declare ROS messages, services, and actions
add_service_files(
  FILES
  verticesnnames.srv  # 서비스 파일 이름
)

## Generate messages and services with dependencies
generate_messages(
  DEPENDENCIES
  std_msgs  # 사용하는 메시지 타입
  geometry_msgs
  gazebo_msgs
)

catkin_package(
  CATKIN_DEPENDS gazebo_ros rospy std_msgs message_runtime
)

## Install Python scripts
catkin_install_python(PROGRAMS
  src/pallet_gazebo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
