cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
# add_definitions(-std=c++11)
add_definitions(-std=c++0x)

project(pugixml)
find_package(catkin REQUIRED)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
)

#catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp nodelet
#   DEPENDS eigen opencv)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


set(XML_LIBRARY_SOURCE_DIR
        src
)

set(XML_LIBRARY_SOURCE_FILES
        src/pugixml.cpp
)
        
set(XML_LIBRARY_HEADER_DIR
        include
)

set(XML_LIBRARY_HEADER_FILES
        include/pugiconfig.hpp
        include/pugixml.hpp
)




include_directories(${XML_LIBRARY_HEADER_DIR})
add_library(pugixml ${XML_LIBRARY_SOURCE_FILES} ${XML_LIBRARY_HEADER_FILES})
