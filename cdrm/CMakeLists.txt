cmake_minimum_required(VERSION 2.8.3)
project(cdrm)

add_compile_options(-std=c++11)

find_package(Boost REQUIRED COMPONENTS graph serialization)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cdrm_msgs
  eigen_conversions
  moveit_core
  moveit_ros_planning
  roscpp
)
find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cdrm
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

add_library(cdrm
  src/cdrm.cpp
  src/generator.cpp
  src/voxelise.cpp
)

add_dependencies(cdrm ${cdrm_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cdrm ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(cdrm_server
  src/server.cpp
)

add_dependencies(cdrm_server cdrm)
target_link_libraries(cdrm_server cdrm)

install(TARGETS cdrm cdrm_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/cdrm/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
