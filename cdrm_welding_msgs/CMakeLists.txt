cmake_minimum_required(VERSION 2.8.3)
project(cdrm_welding_msgs)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs geometry_msgs message_generation std_msgs trajectory_msgs)

add_action_files(FILES GenerateWeldingCdrm.action)
add_message_files(FILES Interval.msg)
add_service_files(FILES PlanWeld.srv)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs trajectory_msgs)

catkin_package()
