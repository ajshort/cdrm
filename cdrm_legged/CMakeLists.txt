cmake_minimum_required(VERSION 2.8.3)
project(cdrm_legged)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cdrm
  eigen_conversions
  moveit_ros_planning
  roscpp
  swri_profiler
)
find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cdrm_legged
)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(include ${catkin_INCLUDE_DIRS}  ${OMPL_INCLUDE_DIRS})

add_library(cdrm_legged
  src/conversions.cpp
  src/disjoint_sets.cpp
  src/leg_config_generator.cpp
  src/leg_model.cpp
  src/leg_planner.cpp
  src/motion_validator.cpp
  src/planner.cpp
  src/planning_context.cpp
  src/stability_checker.cpp
  src/state_validity_checker.cpp
)

add_dependencies(cdrm_legged ${cdrm_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cdrm_legged ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

add_library(cdrm_legged_moveit_plugin src/planner_manager.cpp)
add_dependencies(cdrm_legged_moveit_plugin cdrm_legged)
target_link_libraries(cdrm_legged_moveit_plugin cdrm_legged ${catkin_LIBRARIES})
set_target_properties(cdrm_legged_moveit_plugin PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")

add_executable(contact_generator_node src/contact_generator_node.cpp)
add_dependencies(contact_generator_node cdrm_legged)
target_link_libraries(contact_generator_node cdrm_legged ${catkin_LIBRARIES})
set_target_properties(contact_generator_node PROPERTIES COMPILE_FLAGS "-std=c++11 -Wall")
