cmake_minimum_required(VERSION 2.8.3)
project(otter_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs usv_msgs tf2 tf2_ros)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(controller
  include/otter_control/controller.h
  src/controller.cpp
)
add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})
