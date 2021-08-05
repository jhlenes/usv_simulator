cmake_minimum_required(VERSION 2.8.3)
project(otter_gazebo)

find_package(catkin REQUIRED COMPONENTS
  otter_description
  usv_gazebo_plugins
  xacro
  roscpp
  message_filters
  robot_localization
)

catkin_package(
  CATKIN_DEPENDS otter_description usv_gazebo_plugins xacro roscpp message_filters robot_localization
)

# Generate urdf files from xacro and install
xacro_add_files(
  urdf/otter_gazebo.urdf.xacro
  INSTALL DESTINATION urdf
)

# Install xacro files / macros
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

include_directories(include ${catkin_INCLUDE_DIRS})
