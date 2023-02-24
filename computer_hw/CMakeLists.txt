# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(computer_hw)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/parse_test.py)
endif()

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS roscpp std_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES network_detector# TODO
)

catkin_install_python(PROGRAMS
  executables/cpu_monitor.py executables/hd_monitor.py executables/ntp_monitor.py executables/ntp_monitor.py executables/nvidia_temp.py executables/wifi_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(network_detector src/network_detector.cpp)
target_link_libraries(network_detector ${catkin_LIBRARIES})
add_dependencies(network_detector ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS network_detector
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir conf)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

catkin_python_setup()
