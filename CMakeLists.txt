cmake_minimum_required(VERSION 2.8.3)
project(rescuer_project)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rospy
  rqt_gui
  rqt_gui_cpp
  std_msgs
  ardrone_autonomy
)

## System dependencies are found with CMake's conventions
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()
roslint_cpp()

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy rqt_gui_cpp std_msgs
)

#Ax: my file listing
set(rescuer_project_SRCS 
  src/rescuer_project/main_window.cpp
)
set(rescuer_project_HDRS
  include/rescuer_project/main_window.h
)

qt4_wrap_cpp(rescuer_project_MOCS ${gui_cpp_HDRS})
###########
## Build ##
###########
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${rescuer_project_SRCS}
  ${rescuer_project_MOCS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
)

#############
## Install ##
#############

#############
## Testing ##
#############

