cmake_minimum_required(VERSION 2.8.3)
project(usv_worlds)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package(
  CATKIN_DEPENDS xacro
)

# Generate world files from xacro and install
xacro_add_files(
  worlds/usv.world.xacro
  worlds/usv_fast.world.xacro
  INSTALL DESTINATION worlds
)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)
