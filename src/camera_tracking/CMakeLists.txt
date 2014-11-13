cmake_minimum_required(VERSION 2.8.3)
project(camera_tracking)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -std=c++11")
#set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /opt/pylon4/genicam/bin/Linux64_x64/ /opt/pylon4/lib64 /opt/pylon4/genicam)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge sensor_msgs image_transport camera_info_manager driver_base dynamic_reconfigure)
find_package(OpenCV)
generate_dynamic_reconfigure_options(cfg/CameraConfig.cfg)

catkin_package(
    DEPENDS GLIB2 opencv
    CATKIN_DEPENDS roscpp image_transport cv_bridge camera_info_manager sensor_msgs dynamic_reconfigure driver_base
    INCLUDE_DIRS
    LIBRARIES
)

link_directories(${CMAKE_LIBRARY_PATH} ${PROJECT_SOURCE_DIR}/src)
include_directories(
  ${PROJECT_SOURCE_DIR}/cfg/cpp
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  /opt/pylon4/include
  /opt/pylon4/genicam/library/CPP/include/
)

add_library(reconfigure_callback ${PROJECT_SOURCE_DIR}/src/reconfigure_callback.cpp)
add_executable(tracking src/tracking.cpp)

target_link_libraries(tracking 
  ${catkin_LIBRARIES}
  #${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
  pylonbase
  pylonutility
  GenApi_gcc40_v2_3
  GCBase_gcc40_v2_3
  Log_gcc40_v2_3
  MathParser_gcc40_v2_3
  Xerces-C_gcc40_v2_7_1
  log4cpp_gcc40_v2_3
  reconfigure_callback
)
add_dependencies(tracking ${PROJECT_NAME}_gencfg)

