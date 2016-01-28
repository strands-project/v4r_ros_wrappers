cmake_minimum_required(VERSION 2.8.3)
project(object_detection_and_tracking)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  object_tracker
  object_tracker_srv_definitions
  pcl_conversions
  recognition_srv_definitions
  roscpp
  rospy
  sensor_msgs
  singleview_object_recognizer
)


find_package(PCL 1.7 REQUIRED)
find_package( OpenCV 2.4 REQUIRED )
find_package(V4R REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${V4R_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(detect_and_track_object src/detect_and_track_object.cpp)
add_dependencies(detect_and_track_object ${catkin_EXPORTED_TARGETS})
target_link_libraries(detect_and_track_object ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${V4R_LIBS} ${OpenCV_LIBS})

install(TARGETS detect_and_track_object
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
FILES_MATCHING PATTERN "*.launch"
)																																																																																																																
