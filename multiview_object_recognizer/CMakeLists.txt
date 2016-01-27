cmake_minimum_required(VERSION 2.8.3)
project(multiview_object_recognizer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy recognition_srv_definitions
                                        cv_bridge sensor_msgs image_transport pcl_conversions)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED )
find_package(V4R REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy recognition_srv_definitions cv_bridge sensor_msgs image_transport pcl_conversions
)

include_directories(include
    ${PCL_INCLUDE_DIRS}
    ${V4R_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)


add_executable(multiview_object_recognizer_node
                src/multiview_object_recognizer_ros.cpp
                include/multiview_object_recognizer_ros.h
)

add_executable(test_multiview_object_recognizer_node
                src/test_multiview_recognizer.cpp
)

add_dependencies(multiview_object_recognizer_node ${catkin_EXPORTED_TARGETS})
add_dependencies(test_multiview_object_recognizer_node ${catkin_EXPORTED_TARGETS}})

target_link_libraries(multiview_object_recognizer_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${V4R_LIBS}
)

target_link_libraries(test_multiview_object_recognizer_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${V4R_LIBS}
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

install(TARGETS 
                multiview_object_recognizer_node
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_multiview_object_recognizer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
