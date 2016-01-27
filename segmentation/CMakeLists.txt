cmake_minimum_required(VERSION 2.8.3)
project(segmentation)
find_package(catkin REQUIRED COMPONENTS roscpp rospy segmentation_srv_definitions
                                        cv_bridge sensor_msgs image_transport pcl_conversions
)
find_package(PCL REQUIRED)
find_package(V4R REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy segmentation_srv_definitions
                                        cv_bridge sensor_msgs image_transport pcl_conversions
   )
   

include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${V4R_INCLUDE_DIRS}
)
set(incs include/pcl_segmentation_ros.h)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(pcl_segmentation_service src/pcl_segmentation_ros.cpp ${incs})
add_executable(test_segmentation src/test_segmentation.cpp)

add_dependencies(pcl_segmentation_service ${catkin_EXPORTED_TARGETS})
add_dependencies(test_segmentation ${catkin_EXPORTED_TARGETS})

target_link_libraries(pcl_segmentation_service
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${V4R_LIBS}
)

target_link_libraries(test_segmentation
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${V4R_LIBS}
)

install(TARGETS pcl_segmentation_service
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS test_segmentation
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)
