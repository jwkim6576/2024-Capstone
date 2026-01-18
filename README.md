# 2024-Capstone
2024 Capstone
2024.Seoultech.uni.Capstone design project

Introduce
Demonstration

Enviroments
Doosan-Robotics collaborative robot m1013
Intel-RealSense 435i
SCHUNK EGP-64 gripper

Build
This program is implemented at Ubuntu 20.04 - ROS-Noetic

0) ROS-Noetic
http://wiki.ros.org/noetic/Installation/Ubuntu

Our project needs the packages written below.

1) Doosan-Robotics noetic-devel
https://github.com/doosan-robotics/doosan-robot#overview

2) Intel-Realsense
https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy

3) PCL
https://github.com/PointCloudLibrary/pcl

Cuda environment is cuda 12.1+cudnn 8.9.0
Pytorch version is 2.2.2+cu121

4) Yolo v7
https://gitlab.com/Alsrbile/2023-capstone/-/tree/main/catkin_test/src/yolov7-u7/src/seg?ref_type=heads

You should download this package.

How to use?
Create ROS workspace in the place you want.

Clone this repository inside the src directory.

Install requirements.

Feel free to use!

Reference
https://github.com/jerry800416/3dbinpacking
License
APACHE2.0
