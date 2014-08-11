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


find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  message_generation
  genmsg 
  hobbit_msgs 
  image_geometry 
  pcl_msgs 
  sensor_msgs 
  geometry_msgs
  image_transport 
  pcl_conversions 
  pcl_ros
  tf
)
find_package(PCL REQUIRED)



include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
add_service_files(
  FILES 
  CheckFreeSpace.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tmpdf
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

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

#PC_TO_IV:
add_library(iv_io src/iv_io.cpp)
add_executable(pc_to_iv src/pc_to_iv.cpp)
add_executable(iv_to_pc src/iv_to_pc.cpp)
target_link_libraries(pc_to_iv iv_io ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(iv_to_pc iv_io ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#POINT_CLOUD_MERGE
add_executable(pc_merge src/pc_merge.cpp)
target_link_libraries(pc_merge ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#rosbuild_add_executable (cvtest src/cvtest.cpp)
# Online PointCloud2 Viewer
