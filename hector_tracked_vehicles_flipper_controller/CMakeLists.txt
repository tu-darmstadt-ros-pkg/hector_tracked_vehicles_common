cmake_minimum_required(VERSION 2.8.3)
project(hector_tracked_vehicles_flipper_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hector_tracked_vehicles_flipper_controller
   CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(hector_tracked_vehicles_flipper_controller_node src/hector_tracked_vehicles_flipper_controller_node.cpp src/flippercontroller.cpp)

target_link_libraries(hector_tracked_vehicles_flipper_controller_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS hector_tracked_vehicles_flipper_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

