cmake_minimum_required(VERSION 3.0.2)
project(feedback)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

# catkin_python_setup()

## Add message files
add_message_files(
  FILES
  Encoder.msg
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

install(DIRECTORY src/control/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)