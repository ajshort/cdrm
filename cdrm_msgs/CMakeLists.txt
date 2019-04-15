cmake_minimum_required(VERSION 2.8.3)
project(cdrm_msgs)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs geometry_msgs message_generation std_msgs)

add_action_files(FILES GenerateCdrm.action)
add_service_files(FILES ProcessFile.srv)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs)

catkin_package()
