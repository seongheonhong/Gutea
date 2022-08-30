##############################################################################
# CMake
##############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(robot_tf)

##############################################################################
# Catkin
##############################################################################
## Compile as C++11, supported in ROS Kinetic and newer
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
  tf2
)


################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_tf
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf tf2
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_tf_node src/robot_tf_node.cpp)
target_link_libraries(robot_tf_node ${catkin_LIBRARIES})
add_dependencies(robot_tf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

################################################################################
# Install
################################################################################
install(TARGETS robot_tf_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(PROGRAMS scripts/create_udev_rules
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(FILES 99-turtlebot3-cdc.rules
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
################################################################################
# Test
################################################################################
