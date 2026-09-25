cmake_minimum_required(VERSION 3.0.2)
project(uav_utils)

add_compile_options(-std=c++11 -Wall -O3 -g)
find_package(catkin REQUIRED COMPONENTS roscpp rospy nav_msgs geometry_msgs cmake_utils)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy nav_msgs geometry_msgs
)
include_directories(include ${EIGEN_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# This package is header-only; there is no uav_utils library to link.
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test src/${PROJECT_NAME}_test.cpp)
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
  endif()
endif()
