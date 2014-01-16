cmake_minimum_required(VERSION 2.8.3)
project(decision_service)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AppendRules.srv				    
  GetProfileNumberAttribute.srv
  CreateEngine.srv			    
  GetProfileStringAttribute.srv
  DisposeEngine.srv			    
  EngineExists.srv			    
  LoadProfile.srv
  Evaluate.srv				    
  SaveProfile.srv
  GetAllGlobalProfileAttributesWithPrefix.srv 
  SetBoolConditional.srv
  GetError.srv				    
  SetGlobalBoolConditional.srv
  GetGlobalProfileBoolAttribute.srv	    
  SetGlobalNumberConditional.srv
  GetGlobalProfileNumberAttribute.srv	    
  SetGlobalStringConditional.srv
  GetGlobalProfileStringAttribute.srv	    
  SetNumberConditional.srv
  GetProfileAttributesWithPrefix.srv	    
  SetStringConditional.srv
  GetProfileBoolAttribute.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages()

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
  DEPENDS message_runtime
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

