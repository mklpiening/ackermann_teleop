cmake_minimum_required(VERSION 2.8.3)
project(ackermann_teleop)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ackermann-teleop
   CATKIN_DEPENDS ackermann_msgs roscpp sensor_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ackermann_teleop src/ackermann_teleop.cpp)
target_link_libraries(ackermann_teleop ${catkin_LIBRARIES})
add_dependencies(ackermann_teleop ${catkin_EXPORTED_TARGETS})

add_executable(ackermann_teleop_joy src/ackermann_teleop_joy.cpp)
target_link_libraries(ackermann_teleop_joy ackermann_teleop ${catkin_LIBRARIES})
add_dependencies(ackermann_teleop_joy ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS ackermann_teleop_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ackermann_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
