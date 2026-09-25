cmake_minimum_required(VERSION 3.0.2)
project(depe_ptcg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs quadrotor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(depe_ptcg src/depe_ptcg.cpp)
add_dependencies(depe_ptcg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(depe_ptcg ${catkin_LIBRARIES})
