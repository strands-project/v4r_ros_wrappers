cmake_minimum_required(VERSION 2.8.3)
project(incremental_object_learning_srv_definitions)

find_package(catkin REQUIRED COMPONENTS sensor_msgs std_msgs geometry_msgs message_generation)

 add_service_files(
  FILES
   clear.srv
   learn_object.srv
   learn_object_inc.srv
   save_model.srv
   visualize.srv
   write_debug_images_to_disk.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs  # Or other packages containing msgs
 )
catkin_package( )

