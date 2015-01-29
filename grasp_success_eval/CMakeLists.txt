cmake_minimum_required(VERSION 2.8.3)
project(grasp_success_eval)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

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
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS} src)
add_definitions(${PCL_DEFINITIONS})

## Declare a cpp library
#add_library(checkGrasping
#   src/checkGrasping.cpp
#)

## Declare a cpp executable
add_executable(graspSuccessEval src/graspSuccessEval.cpp)
add_executable(graspSuccessEvalServer src/graspSuccessEvalServer.cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(checkGrasping
# ${catkin_LIBRARIES}
# ${PCL_LIBRARIES}
# ${OpenCV_LIBRARIES}
#)

target_link_libraries(graspSuccessEval
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
 ${OpenCV_LIBRARIES}
 checkGrasping
)

target_link_libraries(graspSuccessEvalServer
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
 ${OpenCV_LIBRARIES}
 checkGrasping
)
