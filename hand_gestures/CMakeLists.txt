cmake_minimum_required(VERSION 2.8.3)
project(hand_gestures)


if(NOT OpenCV_DIR)
 find_path(OpenCV_DIR "OpenCVConfig.cmake" DOC "Root directory of OpenCV" HINTS    "/opt/ros/hydro/share/OpenCV/")
endif(NOT OpenCV_DIR)


find_package(catkin REQUIRED COMPONENTS sensor_msgs OpenCV cv_bridge roscpp std_msgs genmsg image_transport geometry_msgs roslib tf)
 
   
#Messages -----------------------------------
add_message_files(DIRECTORY msg FILES HandGesture.msg)

#Services -----------------------------------
add_service_files(DIRECTORY srv FILES SetQuality.srv)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs hand_gestures)

#This needs to be after generate messages
catkin_package()

find_package(Boost REQUIRED COMPONENTS system filesystem thread serialization program_options) 
#LINK_DIRECTORIES(${BOOST_LIBRARY_DIRS})


include_directories(include ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDEDIR} 
		    ${OpenCV_INCLUDEDIR})


add_executable(hand_gestures src/main.cpp src/services.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(
                      hand_gestures 
                      ${CMAKE_CURRENT_SOURCE_DIR}/bin/libHobbitGestureTrackerLib.a
                      ${CMAKE_CURRENT_SOURCE_DIR}/../skeleton_detector/bin/libMBVCore.so 
                      ${CMAKE_CURRENT_SOURCE_DIR}/../skeleton_detector/bin/libMBVUtils.so  
                      ${CMAKE_CURRENT_SOURCE_DIR}/../skeleton_detector/bin/libMBVDecoding.so
                      ${CMAKE_CURRENT_SOURCE_DIR}/../skeleton_detector/bin/libglog.so.0 
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${Boost_LIBRARIES}
                     )

install(TARGETS hand_gestures RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES hand_gestures.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
