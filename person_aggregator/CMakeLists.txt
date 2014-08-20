cmake_minimum_required(VERSION 2.8.3)
project(person_aggregator)

find_package(catkin REQUIRED COMPONENTS roscpp roslib sensor_msgs OpenCV cv_bridge roscpp std_msgs genmsg image_transport geometry_msgs tf)


#Messages -----------------------------------
add_message_files(DIRECTORY msg FILES Person.msg )


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

#This needs to be after generate messages
catkin_package()
 
include_directories(${catkin_INCLUDEDIR} 
		    ${OpenCV_INCLUDEDIR})

add_executable(person_aggregator src/main.cpp )
 
target_link_libraries(person_aggregator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#install(TARGETS person_aggregator RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(FILES person_aggregator.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(TARGETS person_aggregator
        ARCHIVE DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin
        LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin
        RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin
)
install(FILES person_aggregator.xml DESTINATION ${CMAKE_CURRENT_SOURCE_DIR} )

