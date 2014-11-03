cmake_minimum_required(VERSION 2.4.6)

add_definitions(-std=c++0x)

project(virtual_laser2)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs tf rospy pcl_ros)

catkin_package()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(virtual_laser2 src/cVirtualLaser.cpp src/VirtualLaser.cpp)
target_link_libraries(virtual_laser2 ${catkin_LIBRARIES})


