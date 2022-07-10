cmake_minimum_required(VERSION 2.8.3)
project(tf2_car)

find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_msgs
  tf2_ros
  rospy
  roscpp
  actionlib
)

catkin_package(
  CATKIN_DEPENDS
  tf2
  tf2_msgs
  tf2_ros
  roscpp
  actionlib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(subscriber
  src/subscriber.cpp
)
add_dependencies(subscriber
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(subscriber
  ${catkin_LIBRARIES}
)

install(PROGRAMS scripts/publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
