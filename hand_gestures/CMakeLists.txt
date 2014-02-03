cmake_minimum_required(VERSION 2.8.3)
project(hand_gestures)

find_package(catkin REQUIRED COMPONENTS sensor_msgs OpenCV cv_bridge roscpp std_msgs genmsg image_transport geometry_msgs roslib tf)
 
   
#Messages -----------------------------------
add_message_files(DIRECTORY msg FILES HandGesture.msg)

#Services -----------------------------------
add_service_files(DIRECTORY srv FILES SetQuality.srv)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

#This needs to be after generate messages
catkin_package()

find_package(Boost COMPONENTS system filesystem thread serialization program_options) 
#LINK_DIRECTORIES(${BOOST_LIBRARY_DIRS})


include_directories(${catkin_INCLUDEDIR} 
		    ${OpenCV_INCLUDEDIR})


add_executable(hand_gestures src/main.cpp src/services.cpp)

target_link_libraries(hand_gestures ${CMAKE_CURRENT_SOURCE_DIR}/bin/libHobbitTrackerLib.a ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}  ${Boost_LIBRARIES}   )

install(TARGETS hand_gestures RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES hand_gestures.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
