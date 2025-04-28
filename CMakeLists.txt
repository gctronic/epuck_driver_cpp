cmake_minimum_required(VERSION 3.8)
project(epuck_driver_cpp)

## Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

## Declare a cpp executable
ament_auto_add_executable(epuck_driver_cpp src/epuck2_driver_cpp.cpp)

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
    scripts
    urdf
)
