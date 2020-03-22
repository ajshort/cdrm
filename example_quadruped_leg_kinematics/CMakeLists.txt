cmake_minimum_required(VERSION 2.8.3)
project(example_quadruped_leg_kinematics)

set(CATKIN_DEPENDENCIES angles moveit_core moveit_ros_planning roscpp)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

catkin_package(INCLUDE_DIRS include
               LIBRARIES example_quadruped_leg_kinematics
               CATKIN_DEPENDS ${CATKIN_DEPENDENCIES})

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/ikfast.cpp src/leg_kinematics_plugin.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11")

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_executable(kinematics_test test/kinematics_test.cpp)
  target_link_libraries(kinematics_test ${PROJECT_NAME} ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
  set_target_properties(kinematics_test PROPERTIES COMPILE_FLAGS "-std=c++11")

  add_rostest(test/kinematics.test)
endif()
