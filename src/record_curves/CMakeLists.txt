cmake_minimum_required(VERSION 3.0.2)
project(record_curves)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package(CATKIN_DEPENDS roscpp std_msgs)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(record_curves src/record_y_curves.cpp)
target_link_libraries(record_curves ${catkin_LIBRARIES})
install(TARGETS record_curves RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/plot_observer_effect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
