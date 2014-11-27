
cmake_minimum_required(VERSION 2.8.3)
project(client_test)

set(actionlibs_msgs_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../build)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs tf nav_msgs actionlib_msgs dynamic_reconfigure)


#######################
# DYNAMIC RECONFIGURE
#######################
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  #cfg/ChargerParameters.cfg
  #cfg/EBCParameters.cfg
  #cfg/DisplayParameters.cfg
)

# add dynamic_reconfigure header directory
include_directories(cfg/cpp)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS actionlib_msgs
# LIBRARIES scitos
# CATKIN_DEPENDS geometry_msgs roscpp std_msgs
# DEPENDS system_lib
)

## Declare a cpp library
# add_library(scitos
# src/${PROJECT_NAME}/scitos.cpp
# )

## Declare a cpp executable
#add_executable(scitos_node src/scitos_node.cpp)
add_executable(client_test src/client.cpp)
# make sure configure headers are build before node using them
add_dependencies(client_test ${PROJECT_NAME}_gencfg ) #FIXME

target_link_libraries(client_test ${catkin_LIBRARIES})

