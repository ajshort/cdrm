cmake_minimum_required(VERSION 2.8.3)
project(cdrm_welding)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cdrm
  cdrm_msgs
  cdrm_welding_msgs
  roscpp
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cdrm_welding
  src/server.cpp
)

add_dependencies(cdrm_welding ${cdrm_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cdrm_welding ${catkin_LIBRARIES})

install(TARGETS cdrm_welding
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
