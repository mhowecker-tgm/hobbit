cmake_minimum_required(VERSION 2.8.3)
project(localization_monitor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs tf eigen map_server occupancy_grid_utils hobbit_msgs)


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
  INCLUDE_DIRS include
#  LIBRARIES test
  CATKIN_DEPENDS roscpp occupancy_grid_utils
  DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(localization_monitor src/cLocalizationMonitor.cpp src/LocalizationMonitor.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 add_dependencies(localization_monitor hobbit_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(localization_monitor yaml-cpp 
  ${catkin_LIBRARIES}
)


