cmake_minimum_required(VERSION 3.0.2)
project(mobile_manipulator_full)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  urdf 
  gazebo_ros 
  gazebo_msgs 
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mobile_manipulator_full
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

