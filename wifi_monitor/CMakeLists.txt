cmake_minimum_required(VERSION 2.8.3)
project(wifi_monitor)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  message_generation
  rospy
  std_msgs
)

add_message_files(
  FILES
  NetworkStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS diagnostic_msgs message_runtime rospy std_msgs)

install(
  DIRECTORY scripts launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

