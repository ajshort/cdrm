cmake_minimum_required(VERSION 2.8.3)
project(cdrm_legged)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cdrm
  moveit_ros_planning
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cdrm_legged
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(cdrm_legged
  src/planner_manager.cpp
)

add_dependencies(cdrm_legged ${cdrm_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cdrm_legged ${catkin_LIBRARIES})
