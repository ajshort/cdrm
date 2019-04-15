cmake_minimum_required(VERSION 2.8.3)
project(legplan)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cdrm
  moveit_ros_planning
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES legplan
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(legplan
  src/planner_manager.cpp
)

add_dependencies(legplan ${cdrm_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(legplan ${catkin_LIBRARIES})
