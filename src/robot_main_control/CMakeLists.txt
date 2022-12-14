##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(robot_main_control)

##############################################################################
# Catkin
##############################################################################
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
    qt_build
    roscpp
    std_msgs
    sensor_msgs
    image_transport
    geometry_msgs
    tf
    nav_msgs
    robot_msgs
)
include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
    #INCLUDE_DIRS include
    LIBRARIES robot_main_control
    CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        image_transport
        geometry_msgs
        tf
        nav_msgs
        robot_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)

file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/robot_main_control/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
        src/*.cpp
        include/robot_main_control/*.hpp
)

##############################################################################
# Binaries
##############################################################################

add_executable(robot_main_control ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(robot_main_control ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS robot_main_control RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

