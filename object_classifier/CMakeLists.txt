cmake_minimum_required(VERSION 2.8.3)
project(object_classifier)

find_package(catkin REQUIRED COMPONENTS roscpp 
   sensor_msgs std_msgs visualization_msgs 
   segmentation_srv_definitions classifier_srv_definitions object_perception_msgs pcl_conversions)

find_package(PCL 1.7 REQUIRED)
find_package(V4R REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS classifier_srv_definitions pcl_conversions
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${V4R_INCLUDE_DIRS})

set(incs include/global_nn_classifier_ros.h)

add_executable(object_classifier_node src/global_nn_classifier_ros.cpp ${incs})
add_executable(object_classifier_demo_node src/test.cpp)

add_dependencies(object_classifier_demo_node ${catkin_EXPORTED_TARGETS})
add_dependencies(object_classifier_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(object_classifier_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${V4R_LIBS} )
target_link_libraries(object_classifier_demo_node  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${V4R_LIBS} )

install(TARGETS object_classifier_demo_node
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS object_classifier_node
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)
