cmake_minimum_required(VERSION 2.8.12)
project(yolo)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)
catkin_package()
include_directories()