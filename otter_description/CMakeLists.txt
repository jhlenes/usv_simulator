cmake_minimum_required(VERSION 2.8.3)
project(otter_description)

find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package(CATKIN_DEPENDS xacro)

install(
  DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
