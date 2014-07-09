cmake_minimum_required(VERSION 2.8.3)
project(table_object_detector)
#SET (CMAKE_BUILD_TYPE RelWithDebInfo)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

#set(PCL_DIR CACHE PATH "pcl directory")


find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg hobbit_msgs image_geometry pcl_msgs sensor_msgs image_transport pcl_conversions pcl_ros)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tmpdf
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs genmsg)

#add_boost_directories ()
#add_definitions (-Wall -O3 -msse3 -mssse3 -funroll-loops)

#find_package( OpenCV 2.4 REQUIRED )

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


#add_executable (seg src/segmentation.cpp)
#rosbuild_add_executable (seg src/sog.cpp)
add_executable (thor src/CloudSegmenthor.cpp)
#add_executable (shapefeature src/ShapeFunctions.cpp)
target_link_libraries(thor ${catkin_LIBRARIES})
target_link_libraries(thor ${PCL_LIBRARIES})
#target_link_libraries(shapefeature ${PCL_LIBRARIES})  df!!!!!!!

#rosbuild_add_executable (cvtest src/cvtest.cpp)
# Online PointCloud2 Viewer
