cmake_minimum_required(VERSION 2.8.3)
project(ur5_moveit_config)
#
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  diagnostic_msgs
  geometry_msgs
  manipulation_msgs
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  roscpp
  rospy
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  tf
  trajectory_msgs
)
#
find_package(Boost REQUIRED COMPONENTS system)

catkin_package()
#
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
