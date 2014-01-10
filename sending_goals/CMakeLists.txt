cmake_minimum_required(VERSION 2.4.6)
add_definitions(-std=c++0x)

project(sending_goals)
find_package(catkin REQUIRED COMPONENTS roscpp hobbit_msgs move_base_msgs tf actionlib)

catkin_package()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sending_goals src/sendingGoals2.cpp)
target_link_libraries(sending_goals ${catkin_LIBRARIES})




