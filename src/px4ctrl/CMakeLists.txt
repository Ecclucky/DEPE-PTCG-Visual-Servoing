cmake_minimum_required(VERSION 3.0.2)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy quadrotor_msgs geometry_msgs nav_msgs sensor_msgs std_msgs
  uav_utils mavros_msgs
)
find_package(Eigen3 REQUIRED)
catkin_package(CATKIN_DEPENDS
  roscpp rospy quadrotor_msgs geometry_msgs nav_msgs sensor_msgs std_msgs
  uav_utils mavros_msgs
)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_executable(px4ctrl_node
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)
add_dependencies(px4ctrl_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(px4ctrl_node ${catkin_LIBRARIES} Eigen3::Eigen)
install(TARGETS px4ctrl_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
