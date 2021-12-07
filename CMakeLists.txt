cmake_minimum_required(VERSION 2.8.3)
project(aruco_markers_detect)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
#SET(CMAKE_BUILD_TYPE "Release")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
#SET(CMAKE_BUILD_TYPE  Debug) 
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  eigen_conversions
  nav_msgs
  vision_msgs
  sensor_msgs
  cv_bridge
  image_transport
  serial
  std_msgs
  visualization_msgs
  nodelet
)


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES minicar_ctrl
  CATKIN_DEPENDS roscpp nodelet
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(ALL_TARGET_LIBRARIES "")

include(cmake/eigen.cmake)
include(cmake/YAML.cmake)
include(cmake/glog.cmake)
include(cmake/global_defination.cmake)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/modbus
  /usr/local/include/aruco
  third_party/eigen3
  third_party/aruco-3.1.12
)


add_library(${PROJECT_NAME}  
  src/aruco_markers/aruco_markers.cpp
  src/aruco_markers/aruco_markers_interface.cpp
  src/apps/aruco_markers_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}   
  ${catkin_LIBRARIES} 
  ${ALL_TARGET_LIBRARIES}
  /usr/local/lib/libaruco.so
  yaml-cpp
)
add_dependencies(${PROJECT_NAME}   ${catkin_EXPORTED_TARGETS}   )

add_executable(aruco_markers_node 
  src/apps/aruco_markers_node.cpp
) 
                                                                                                             
target_link_libraries(aruco_markers_node 
   ${PROJECT_NAME} 
)


add_executable(aruco_single_marker_node
  src/apps/aruco_single_marker_node.cpp
  src/aruco_mark_map_build/aruco_single_mark.cpp
) 
                                                                                                             
target_link_libraries(aruco_single_marker_node 
   ${PROJECT_NAME} 
)




