cmake_minimum_required(VERSION 3.0.2)
project(dead_reckoning)

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
  LIBRARIES dead_reckoning
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

## Add an executable node for testing/using the library
add_executable(${PROJECT_NAME}_node src/dead_reckoning_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

## Add an executable for traj_from_csv
add_executable(traj_from_csv src/traj_from_csv.cpp)
add_dependencies(traj_from_csv ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_from_csv ${PROJECT_NAME} ${catkin_LIBRARIES})

## Add an executable for bicycle node (main node)
add_executable(bicycle_node src/bicycle_node.cpp)
add_dependencies(bicycle_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(bicycle_node ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

## Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install node executable
install(TARGETS ${PROJECT_NAME}_node traj_from_csv
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS bicycle_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

## Install Python scripts
install(PROGRAMS scripts/bag_to_csv.py  
  scripts/lidar_pose_extraction.py
  scripts/imu_mag_converter.py
  scripts/vel_remap.py
  scripts/init_pub_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
