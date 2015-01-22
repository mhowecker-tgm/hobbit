cmake_minimum_required(VERSION 2.8.3)
project(interfaces_mira)

set(actionlibs_msgs_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../build)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs tf nav_msgs actionlib_msgs dynamic_reconfigure pugixml)


# include cmake file for MIRA
include(CMakeLists.mira)
#MIRA_REQUIRE_PACKAGE(SCITOS 0.19.0) #FIXME

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
# FILES
# Message1.msg
# Message2.msg
# )

## Generate services in the 'srv' folder
#add_service_files(
# FILES
# ResetMotorStop.srv
#
#)

add_action_files(
  DIRECTORY action
  FILES MiraSendingGoals.action MiraDocking.action
  MiraDocking.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
 )

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
   INCLUDE_DIRS include
   CATKIN_DEPENDS actionlib_msgs pugixml
# LIBRARIES scitos
# CATKIN_DEPENDS geometry_msgs roscpp std_msgs
# DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${MIRA_ROOT_DIR}/toolboxes/RobotDataTypes/include
  ${MIRA_ROOT_DIR}/toolboxes/Navigation/include
  ${MIRA_ROOT_DIR}/toolboxes/Mapping/include #previous versions
  #${MIRA_ROOT_DIR}/../mira-commercial/toolboxes/Mapping/include #different from previous versions of Mira

)

## Declare a cpp library
# add_library(scitos
# src/${PROJECT_NAME}/scitos.cpp
# )

## Declare a cpp executable
#add_executable(scitos_node src/scitos_node.cpp)
add_executable(interfaces_node src/interfaces_node.cpp
                                                        src/MiraRobotModule.cpp
                                                        src/ModuleFactory.cpp
                                                        src/MiraRobot.cpp
                                                        src/MiraVirtualLaser.cpp
							src/MiraSendingGoals.cpp
						        src/MiraGetPose.cpp
							src/MiraGetPath.cpp
							src/MiraGoRecharge.cpp
							src/MiraRobotDrive.cpp
							src/MiraRobotCharger.cpp
							src/MiraRobotDriveRos.cpp)
# make sure configure headers are build before node using them
add_dependencies(interfaces_node ${PROJECT_NAME}_gencfg mira_msgs_gencpp) #FIXME

target_link_libraries(interfaces_node ${catkin_LIBRARIES} Navigation Mapping Maps)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(interfaces_node mira_msgs_gencpp) #FIXME

## Specify libraries to link a library or executable target against
# target_link_libraries(scitos_node
# ${catkin_LIBRARIES}
# )



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
# scripts/my_python_script
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS scitos scitos_node
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h"
# PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
# # myfile1
# # myfile2
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_scitos.cpp)
# if(TARGET ${PROJECT_NAME}-test)
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


##########
## MIRA ##
##########

# for mira support (auto linking, enabling C++0x support,...)
catkin_add_mira(interfaces_node)
