cmake_minimum_required(VERSION 2.8.3)
project(object_tracker)

find_package(catkin REQUIRED COMPONENTS roscpp rospy object_tracker_srv_definitions
                                         genmsg tf2 tf visualization_msgs
                                        cv_bridge image_transport pcl_conversions)

find_package(PCL 1.7 REQUIRED)
find_package( OpenCV 2.4 REQUIRED )
find_package( V4R REQUIRED )
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy object_tracker_srv_definitions tf2
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${V4R_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(incs include/object_tracker.h)

## Declare a cpp executable
add_executable(object_tracker_service src/object_tracker.cpp ${incs})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(object_tracker_service ${catkin_EXPORTED_TARGETS} ${V4R_LIBS})

## Specify libraries to link a library or executable target against

target_link_libraries(object_tracker_service
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${V4R_LIBS}
   ${CERES_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS object_tracker_service
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

#install(DIRECTORY launch/
#DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#FILES_MATCHING PATTERN "*.launch"
#)																																																																																																																

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_grasping_recognition_master.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
