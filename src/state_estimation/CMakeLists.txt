cmake_minimum_required(VERSION 3.0.2)
project(state_estimation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS can_msgs geometry_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Build the DeadReckoning library
add_library(${PROJECT_NAME}
  src/BicycleModelSimple.cpp
)


add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


## Add an executable for bicycle node (main node)
add_executable(wheel_odom_node src/bicycle_node.cpp)
add_dependencies(wheel_odom_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheel_odom_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(odom_to_ndt_node src/odom_to_ndt.cpp)
add_dependencies(odom_to_ndt_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_to_ndt_node ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############


install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

install(TARGETS wheel_odom_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS odom_to_ndt_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Python scripts
install(PROGRAMS 
  scripts/test.py
  scripts/ndt_path_publisher.py
  scripts/lla_to_odom.py
  scripts/utils.py
  scripts/gps_heading.py
  scripts/pubpose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


