cmake_minimum_required(VERSION 3.0.2)
project(Relative_Pos_Cal)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs cv_bridge)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(VISP REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR}
)

add_executable(Relative_Pos_Cal src/Relative_Pos_Cal_apriltag.cpp)
target_link_libraries(Relative_Pos_Cal
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
  ${VISP_LIBRARIES}
  ${realsense2_LIBRARY}
)
install(TARGETS Relative_Pos_Cal RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
