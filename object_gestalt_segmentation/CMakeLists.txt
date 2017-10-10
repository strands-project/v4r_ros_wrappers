cmake_minimum_required(VERSION 2.8.3)
project(object_gestalt_segmentation)

find_package(catkin REQUIRED COMPONENTS cv_bridge pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf tf2 segmentation_srv_definitions)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(V4R REQUIRED)

#find_package(rostest REQUIRED)

catkin_package(
 INCLUDE_DIRS  #  LIBRARIES object_segmentation_segmentation
 CATKIN_DEPENDS #  DEPENDS system_lib
 cv_bridge pcl_conversions pcl_ros sensor_msgs std_msgs tf tf2 segmentation_srv_definitions 
)

#message(STATUS "MY PCL_INCLUDE_DIRS" ${PCL_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${V4R_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

#message(STATUS "MY V4R_LIBS" ${V4R_LIBS})
# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     message(STATUS "MY ${_variableName}=${${_variableName}}")
# endforeach()
# link_directories(${V4R_LIB_DIR})

##add_executable(object_gestalt_segmentation_test src/test.cpp)

##main server program
add_executable(object_gestalt_segmentation src/object_gestalt_segmentation.cpp)
target_link_libraries(object_gestalt_segmentation ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${V4R_LIBS}
cholmod
amd
klu
btf
spqr
camd
ccolamd
cxsparse
umfpack
colamd
ldl
csparse
)
add_dependencies(object_gestalt_segmentation v4r)

##test server program
add_executable(test_object_gestalt_segmentation src/test_object_gestalt_segmentation.cpp)
add_dependencies(test_object_gestalt_segmentation ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_object_gestalt_segmentation
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${V4R_LIBS}
)


