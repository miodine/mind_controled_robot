cmake_minimum_required(VERSION 0.0.1)
project(mcr_robot_controls)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS geometry_msgs
  DEPENDS
)

###########
## Build ##
###########


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  nodes/mcr_robot_controls.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/mcr_robot_controls.py
  DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION)
)