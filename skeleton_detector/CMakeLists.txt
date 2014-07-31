cmake_minimum_required(VERSION 2.8.3)
project(skeleton_detector)


if(NOT OpenCV_DIR)
 find_path(OpenCV_DIR "OpenCVConfig.cmake" DOC "Root directory of OpenCV" HINTS    "/opt/ros/hydro/share/OpenCV/")
endif(NOT OpenCV_DIR)


find_package(catkin REQUIRED COMPONENTS sensor_msgs OpenCV cv_bridge roscpp std_msgs genmsg image_transport geometry_msgs roslib tf)
 
   
#Messages -----------------------------------
add_message_files(DIRECTORY msg FILES Skeleton2D.msg Person.msg PointEvents.msg )

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


add_executable(skeleton_detector src/main.cpp src/services.cpp src/pose.cpp)

target_link_libraries(skeleton_detector  
                      ${CMAKE_CURRENT_SOURCE_DIR}/bin/libHobbitUpperBodyTrackerLib.a 
                      ${CMAKE_CURRENT_SOURCE_DIR}/bin/libMBVCore.so 
                      ${CMAKE_CURRENT_SOURCE_DIR}/bin/libMBVUtils.so  
                      ${CMAKE_CURRENT_SOURCE_DIR}/bin/libMBVDecoding.so
                      ${CMAKE_CURRENT_SOURCE_DIR}/bin/libglog.so.0
                      ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}  ${Boost_LIBRARIES}   )

install(TARGETS skeleton_detector RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES skeleton_detector.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
