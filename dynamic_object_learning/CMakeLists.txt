cmake_minimum_required(VERSION 2.8.3)
project(dynamic_object_learning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy do_learning_srv_definitions
                                        cv_bridge sensor_msgs image_transport)

## System dependencies are found with CMake's conventions
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED )
find_package(V4R REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(X11 REQUIRED)
find_package(OpenGL REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_object_learning
  CATKIN_DEPENDS roscpp rospy tf do_learning_srv_definitions
#  DEPENDS system_lib
)

message("*******  ${V4R_INCLUDE_DIRS} ********")
include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${V4R_INCLUDE_DIRS}
)

add_definitions(-std=c++0x)
add_definitions(${PCL_DEFINITIONS})

###########
## Build ##
###########
set(incs include/pcl_conversions.h include/object_learning_ros.h)
add_executable(do_learning_service src/object_learning_ros.cpp ${incs})
add_dependencies(do_learning_service ${catkin_EXPORTED_TARGETS})

target_link_libraries(do_learning_service
   ${catkin_LIBRARIES}
   ${V4R_LIBS}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${CERES_LIBRARIES}
 GLEW glut IL GL X11
)


add_executable(test_do_learning_service src/test_dol_from_files.cpp ${incs})
add_dependencies(test_do_learning_service ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_do_learning_service
   ${catkin_LIBRARIES}
   ${V4R_LIBS}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${CERES_LIBRARIES}
 GLEW glut IL GL X11
)

#############
## Install ##
#############

install(TARGETS do_learning_service
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS test_do_learning_service
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
