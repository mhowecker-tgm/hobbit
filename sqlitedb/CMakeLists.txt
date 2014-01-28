cmake_minimum_required(VERSION 2.4.6)

project(sqlitedb)

find_package(catkin REQUIRED COMPONENTS std_msgs)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages


catkin_package(

  CATKIN_DEPENDS message_runtime
)
