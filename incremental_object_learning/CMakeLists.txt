cmake_minimum_required(VERSION 2.8.3)
project(incremental_object_learning)

find_package(catkin REQUIRED COMPONENTS roscpp rospy incremental_object_learning_srv_definitions cv_bridge sensor_msgs image_transport)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED )
find_package(V4R REQUIRED)

catkin_package( CATKIN_DEPENDS roscpp rospy tf incremental_object_learning_srv_definitions )

include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${V4R_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(${PCL_DEFINITIONS})

set(incs include/object_learning_ros.h)
add_executable(incremental_object_learning_service src/object_learning_ros.cpp ${incs})
add_dependencies(incremental_object_learning_service ${catkin_EXPORTED_TARGETS})

target_link_libraries(incremental_object_learning_service
   ${catkin_LIBRARIES}
   ${V4R_LIBS}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${CERES_LIBRARIES}
)

add_executable(test_incremental_object_learning_service src/test_dol_from_files.cpp ${incs})
add_dependencies(test_incremental_object_learning_service ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_incremental_object_learning_service
   ${catkin_LIBRARIES}
   ${V4R_LIBS}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${CERES_LIBRARIES}
)

install(TARGETS incremental_object_learning_service
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS test_incremental_object_learning_service
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

