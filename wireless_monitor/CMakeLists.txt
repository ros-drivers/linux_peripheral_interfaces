cmake_minimum_required(VERSION 2.8.3)
project(wireless_monitor)

#############
##  Build  ##
#############

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

#############
##   Test  ##
#############

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

#############
## Install ##
#############

install(PROGRAMS
  scripts/wireless_monitor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
