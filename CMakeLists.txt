cmake_minimum_required(VERSION 3.0.2)
project(zm_potentialfield_global_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  dynamic_reconfigure
  nav_core
  pluginlib
  roscpp
  std_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/ZMPotentialfieldGlobalPlanner.cfg
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES zm_potentialfield_global_planner
   CATKIN_DEPENDS geometry_msgs nav_core pluginlib roscpp std_msgs dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(zm_potentialfield_global_planner src/zm_potentialfield_global_planner.cpp)
add_dependencies(zm_potentialfield_global_planner zm_potentialfield_global_planner_gencfg)
target_link_libraries(zm_potentialfield_global_planner ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES zm_potentialfield_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)