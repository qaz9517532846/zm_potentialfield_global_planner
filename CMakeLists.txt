cmake_minimum_required(VERSION 3.0.2)
project(zm_potentialfield_global_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES zm_potentialfield_global_planner
   CATKIN_DEPENDS geometry_msgs nav_core pluginlib roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(zm_potentialfield_global_planner src/zm_potentialfield_global_planner.cpp)