cmake_minimum_required(VERSION 3.0.2)
project(MultiRate_Hybrid_attacks)

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

add_executable(MultiRate_Hybrid_attacks src/MultiRate_Hybrid_attacks.cpp)
add_dependencies(MultiRate_Hybrid_attacks ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(MultiRate_Hybrid_attacks ${catkin_LIBRARIES})
