cmake_minimum_required(VERSION 2.8.3)
project(singleview_object_recognizer)

find_package(catkin REQUIRED COMPONENTS roscpp rospy recognition_srv_definitions
                                         genmsg cv_bridge sensor_msgs image_transport pcl_conversions)

find_package( PCL 1.7 REQUIRED )
find_package( OpenCV 2.4 REQUIRED )
find_package( V4R REQUIRED )

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy recognition_srv_definitions pcl_conversions
)


add_definitions(${PCL_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${V4R_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(incs include/recognizer_ros.h)

add_executable(recognition_service src/recognizer_ros.cpp ${incs})
add_executable(test_single_view_recognition src/test_single_view_recognizer.cpp)

add_dependencies(recognition_service ${catkin_EXPORTED_TARGETS})
add_dependencies(test_single_view_recognition ${catkin_EXPORTED_TARGETS})

target_link_libraries(recognition_service
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${V4R_LIBS}
)

target_link_libraries(test_single_view_recognition
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${V4R_LIBS}
   ${OpenCV_LIBS}
)


install(TARGETS recognition_service
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS test_single_view_recognition
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
FILES_MATCHING PATTERN "*.launch"
)																																																																																																																
