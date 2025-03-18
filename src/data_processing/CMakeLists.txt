cmake_minimum_required(VERSION 3.0.2)
project(data_processing)

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


#############
## Install ##
#############



install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

## Install Python scripts
install(PROGRAMS scripts/bag_to_csv.py 
  scripts/lidar_pose_extraction.py
  scripts/imu_mag_converter.py
  scripts/gt_extractor.py
  scripts/path_logger.py
  scripts/data_drift_logger.py
  scripts/ndt_path_logger.py
  scripts/auto_tune.py
  scripts/offline_plot_trajectory.py
  scripts/temp_ndt_path_logger.py
  scripts/ndt_repub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


