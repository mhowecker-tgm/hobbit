cmake_minimum_required(VERSION 2.4.6)

project(head)

find_package(catkin REQUIRED COMPONENTS std_msgs tf)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages

add_service_files(
  DIRECTORY srv
  FILES
  HeadState.srv
 

)

generate_messages(DEPENDENCIES std_msgs tf)

catkin_package(

  CATKIN_DEPENDS message_runtime
)
