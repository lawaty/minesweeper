cmake_minimum_required(VERSION 3.0.2)
project(feedback)

find_package(catkin REQUIRED COMPONENTS
  rospy
  stdmsgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
