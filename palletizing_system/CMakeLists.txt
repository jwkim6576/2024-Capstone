cmake_minimum_required(VERSION 3.0.2)
project(palletizing_system)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
# 필수 패키지 로드
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  dsr_msgs
  cv_bridge
  sensor_msgs
  image_transport
  pcl_ros
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# 서비스 파일 설정
add_service_files(
  FILES
  map_results.srv
  re.srv
  lego_place.srv
  box_info.srv
  obj_info.srv
  vision_vertices.srv #중요
  servoing_posx.srv 
  servoing.srv
  findPickposx.srv
  multi_viewing_pose.srv # 중요
)

# 메시지 파일 설정 (누락된 부분 추가)
add_message_files(
  FILES
  servoing_plot.msg  # 여기에 추가된 메시지 파일
  pick_plot.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES palletizing_system
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
)

# PCL 패키지 설정
find_package(PCL REQUIRED)

# Python 스크립트 설치 설정
catkin_install_python(PROGRAMS
  src/half_sphere.py
  src/collision_detect.py
  src/servoing.py
  src/collision_detect_plot.py
  src/Packing-service3.py
  src/p2planetest2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
