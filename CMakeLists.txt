cmake_minimum_required(VERSION 3.0.2)
project(terrain_estimator)

# Dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros
)

# Includes
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
)

# Build

add_executable(terrain_estimator src/terrain_estimator.cpp)
target_link_libraries(terrain_estimator
 ${catkin_LIBRARIES}
)
