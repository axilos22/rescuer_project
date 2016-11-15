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
  image_transport
  sensor_msgs
  geometry_msgs
  cv_bridge
  ardrone_autonomy
)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  message("USING QT5")
  find_package(Qt5Widgets REQUIRED)
else()
  message("USING QT4")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

#Ax: my file listing
set(rescuer_project_SRCS
  src/rescuer_project/main_window.cpp
  src/rescuer_project/lineeditteleop.cpp
)
set(rescuer_project_HDRS
  include/rescuer_project/main_window.h
  include/rescuer_project/lineeditteleop.h
)
set(rescuer_project_UIS
  src/rescuer_project/main_window.ui
)

#roslint_cpp()

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
set(rescuer_project_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

catkin_package(
  INCLUDE_DIRS ${rescuer_project_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge geometry_msgs
)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#WRAPPING with Qt
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rescuer_project_MOCS ${rescuer_project_HDRS})
else()
  qt4_wrap_cpp(rescuer_project_MOCS ${rescuer_project_HDRS})
endif()
# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(rescuer_project_UIS_H ${rescuer_project_UIS})
else()
  qt4_wrap_ui(rescuer_project_UIS_H ${rescuer_project_UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")
###########
## Build ##
###########
include_directories(
  include
  ${rescuer_project_INCLUDE_DIRECTORIES}
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${rescuer_project_HDRS}
  ${rescuer_project_SRCS}
  ${rescuer_project_MOCS}
  ${rescuer_project_UIS_H}
)

## Specify libraries to link a library or executable target against
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()
## Library linking
#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

#find_package(class_loader)
#class_loader_hide_library_symbols(${PROJECT_NAME})

#install(FILES plugin.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

#install(PROGRAMS scripts/rescuer_project
#  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)
#install(PROGRAMS scripts/rescuer_project
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
#############
## Testing ##
#############

