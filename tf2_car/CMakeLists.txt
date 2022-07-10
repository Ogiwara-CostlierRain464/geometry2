cmake_minimum_required(VERSION 2.8.3)
project(tf2_car)

find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_msgs
  tf2_ros
  rospy
  roscpp
)

catkin_package(
  CATKIN_DEPENDS tf2
  tf2_msgs
  tf2_ros
  roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(subscriber
  src/subscriber.cpp
)

install(PROGRAMS scripts/publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
