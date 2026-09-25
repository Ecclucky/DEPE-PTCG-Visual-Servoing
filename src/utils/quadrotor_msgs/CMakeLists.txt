cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  message_generation
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    )

add_message_files(
  FILES
  AuxCommand.msg
  Corrections.msg
  Gains.msg
  GoalSet.msg
  OutputData.msg
  PositionCommand.msg
  PPROutputData.msg
  Serial.msg
  SO3Command.msg
  StatusData.msg
  TRPYCommand.msg
  Odometry.msg
  PolynomialTrajectory.msg
  OptimalTimeAllocator.msg
  TrajectoryMatrix.msg
  ReplanCheck.msg
  SpatialTemporalTrajectory.msg
  Bspline.msg
  SwarmCommand.msg
  Replan.msg
  SwarmOdometry.msg
  SwarmInfo.msg
  Px4ctrlDebug.msg
  TakeoffLand.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES encode_msgs decode_msgs
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_msgs message_runtime
)

add_library(decode_msgs src/decode_msgs.cpp)
add_library(encode_msgs src/encode_msgs.cpp)

add_dependencies(encode_msgs quadrotor_msgs_generate_messages_cpp)
add_dependencies(decode_msgs quadrotor_msgs_generate_messages_cpp)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

target_link_libraries(encode_msgs ${catkin_LIBRARIES})
target_link_libraries(decode_msgs ${catkin_LIBRARIES})
