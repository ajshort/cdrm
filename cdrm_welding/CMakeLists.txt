cmake_minimum_required(VERSION 2.8.3)
project(cdrm_welding)

add_compile_options(-std=c++11)

find_package(Boost REQUIRED COMPONENTS graph serialization)
find_package(catkin REQUIRED COMPONENTS
  cdrm
  cdrm_msgs
  cdrm_welding_msgs
  eigen_conversions
  moveit_core
  moveit_ros_planning
  roscpp
)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

add_executable(cdrm_welding_node
  src/server.cpp
  src/weld.cpp
  src/weld_planner.cpp
  src/welding_cdrm.cpp
  src/welding_cdrm_generator.cpp
)

add_dependencies(cdrm_welding_node ${cdrm_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cdrm_welding_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

install(TARGETS cdrm_welding_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
