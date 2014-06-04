cmake_minimum_required(VERSION 2.8.3)
project(web_interface)

find_package(catkin REQUIRED COMPONENTS sensor_msgs roscpp std_msgs image_transport geometry_msgs roslib tf)
 
     
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

#This needs to be after generate messages
catkin_package()

include_directories(${catkin_INCLUDEDIR} )


add_executable(web_interface src/main.cpp )

target_link_libraries(web_interface ${CMAKE_CURRENT_SOURCE_DIR}/bin/libAmmarServer.a ${catkin_LIBRARIES} )

#install(TARGETS web_interface RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(FILES web_interface.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(TARGETS web_interface
        ARCHIVE DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin
        LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin
        RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin
)
install(FILES web_interface.xml DESTINATION ${CMAKE_CURRENT_SOURCE_DIR} )

