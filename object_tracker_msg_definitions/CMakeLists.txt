cmake_minimum_required(VERSION 2.8.3)
project(object_tracker_msg_definitions)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs message_generation)

add_message_files(
  FILES
	ObjectInfo.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)
 
catkin_package()

