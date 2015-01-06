cmake_minimum_required(VERSION 2.8.3)
project(libsensors_monitor)

find_package(catkin REQUIRED COMPONENTS roscpp diagnostic_updater)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(libsensors_monitor src/libsensors_monitor.cpp src/libsensors_chip.cpp)
target_link_libraries(libsensors_monitor sensors ${catkin_LIBRARIES})


install(TARGETS libsensors_monitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
