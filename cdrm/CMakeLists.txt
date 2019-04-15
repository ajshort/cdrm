cmake_minimum_required(VERSION 2.8.3)
project(cdrm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cdrm_msgs
  moveit_ros_planning
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cdrm
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(cdrm
  src/cdrm.cpp
)

add_dependencies(cdrm ${cdrm_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cdrm ${catkin_LIBRARIES})

install(TARGETS cdrm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/cdrm/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
