cmake_minimum_required(VERSION 2.8.3)
project(camera_tracker)
find_package(catkin REQUIRED COMPONENTS roscpp rospy camera_srv_definitions
                                        genmsg tf2 tf visualization_msgs
                                        cv_bridge image_transport)
find_package(PCL 1.7 REQUIRED)
find_package( OpenCV 2.4 REQUIRED )
find_package(V4R REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(X11 REQUIRED)
find_package(OpenGL REQUIRED)
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy camera_srv_definitions tf2
)
add_definitions(-std=c++0x)

include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${V4R_INCLUDE_DIRS}
)
set(incs include/pcl_conversions.h)

add_executable(camera_tracker_service src/camera_tracker.cpp ${incs})
add_dependencies(camera_tracker_service ${catkin_EXPORTED_TARGETS})

target_link_libraries(camera_tracker_service
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${V4R_LIBS}
   ${CERES_LIBRARIES}
 GLEW glut IL GL X11
)


install(TARGETS camera_tracker_service
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)
