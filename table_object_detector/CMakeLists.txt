cmake_minimum_required(VERSION 2.8.3)
project(table_object_detector)
SET (CMAKE_BUILD_TYPE RelWithDebInfo)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

set(PCL_DIR CACHE PATH "pcl directory")
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

include_directories(${PCL_INCLUDE_DIRS})


#add_boost_directories ()
add_definitions (-Wall -O3 -msse3 -mssse3 -funroll-loops)

find_package( OpenCV 2.4 REQUIRED )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

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


#rosbuild_add_executable (seg src/segmentation.cpp)
#rosbuild_add_executable (seg src/sog.cpp)
add_executable (thor src/CloudSegmenthor.cpp)

#add_executable (shapefeature src/ShapeFunctions.cpp) df!!!!!!!

target_link_libraries(thor ${PCL_LIBRARIES})
#target_link_libraries(shapefeature ${PCL_LIBRARIES})  df!!!!!!!

#rosbuild_add_executable (cvtest src/cvtest.cpp)
# Online PointCloud2 Viewer
