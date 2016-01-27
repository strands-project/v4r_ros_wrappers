cmake_minimum_required(VERSION 2.8.3)
project(segment_and_classify)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs visualization_msgs pcl_conversions segmentation_srv_definitions classifier_srv_definitions object_perception_msgs segmentation object_classifier)
find_package(PCL 1.7 REQUIRED)
find_package( OpenCV 2.4 REQUIRED )
find_package(V4R REQUIRED)

catkin_package(
   CATKIN_DEPENDS segmentation_srv_definitions classifier_srv_definitions object_classifier segmentation pcl_conversions
)

include_directories(include ${catkin_INCLUDE_DIRS} ${V4R_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(segement_and_classify_test src/test.cpp)
add_dependencies(segement_and_classify_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(segement_and_classify_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${V4R_LIBS} ${OpenCV_LIBS})

install(TARGETS segement_and_classify_test
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
FILES_MATCHING PATTERN "*.launch"
)																																																																																																																
