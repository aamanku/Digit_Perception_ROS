cmake_minimum_required(VERSION 3.0.2)
project(ar_perception_client_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O2)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
find_package(ixwebsocket REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ar_perception_client_pkg
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
        ${IXWEBSOCKET_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/ar_perception_client_pkg_node.cpp
        include/ar_perception_client_pkg/perception_stream_ws.h
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} /usr/local/lib/libixwebsocket.so.11.4.4 
Eigen3::Eigen)
