cmake_minimum_required(VERSION 2.8.3)
project(fitness_coordinator)

find_package(catkin REQUIRED COMPONENTS roscpp roslib sensor_msgs OpenCV cv_bridge roscpp std_msgs genmsg image_transport geometry_msgs tf)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

#This needs to be after generate messages
catkin_package()
 
include_directories(${catkin_INCLUDEDIR} 
		    ${OpenCV_INCLUDEDIR})

add_executable(fitness_coordinator src/main.cpp src/process.cpp )
 
target_link_libraries(fitness_coordinator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

#install(TARGETS fitness_coordinator RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(FILES fitness_coordinator.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(TARGETS fitness_coordinator
        ARCHIVE DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin
        LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin
        RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin
)
install(FILES fitness_coordinator.xml DESTINATION ${CMAKE_CURRENT_SOURCE_DIR} )

