cmake_minimum_required(VERSION 2.4.6)
project(new_layers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES test
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  
)

#find_package(catkin REQUIRED COMPONENTS  pcl_ros pcl_conversions pcl)

find_package(PCL REQUIRED)
include_directories(... ${PCL_INCLUDE_DIRS})

add_library(nav_layer src/nav_layer.cpp)
add_library(map_layer src/map_layer.cpp)
add_library(nav_layer_global src/nav_layer_global.cpp)

target_link_libraries(nav_layer ${PCL_LIBRARIES})
target_link_libraries(map_layer ${PCL_LIBRARIES})
target_link_libraries(nav_layer_global ${PCL_LIBRARIES})

