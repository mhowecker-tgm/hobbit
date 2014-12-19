cmake_minimum_required(VERSION 2.8.3)
project(follow_user)


if(NOT OpenCV_DIR)
 find_path(OpenCV_DIR "OpenCVConfig.cmake" DOC "Root directory of OpenCV" HINTS    "/opt/ros/hydro/share/OpenCV/")
endif(NOT OpenCV_DIR)


find_package(catkin REQUIRED COMPONENTS sensor_msgs OpenCV cv_bridge roscpp std_msgs genmsg image_transport geometry_msgs roslib tf message_generation)
 
   
#Messages -----------------------------------
add_message_files(DIRECTORY msg FILES CameraMotion.msg TrackerTarget.msg)

#Services -----------------------------------
add_service_files(DIRECTORY srv FILES SetQuality.srv)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs follow_user)

#This needs to be after generate messages
catkin_package(CATKIN_DEPENDS message_runtime)

find_package(Boost COMPONENTS system filesystem thread serialization program_options) 
#LINK_DIRECTORIES(${BOOST_LIBRARY_DIRS})


include_directories(${catkin_INCLUDEDIR}  ${BOOST_INCLUDE_DIRS} ${OpenCV_INCLUDEDIR})


add_executable(follow_user src/main.cpp src/services.cpp src/pose.cpp)

target_link_libraries(follow_user    
                      ${CMAKE_CURRENT_SOURCE_DIR}/bin/libHobbitPeopleTracker.so
                      ${CMAKE_CURRENT_SOURCE_DIR}/bin/libPeopleTrackerLib.so
                      ${CMAKE_CURRENT_SOURCE_DIR}/bin/libMBVCore.so 
                      ${CMAKE_CURRENT_SOURCE_DIR}/bin/libMBVUtils.so  
                      ${CMAKE_CURRENT_SOURCE_DIR}/bin/libMBVAcquisition.so
                      ${CMAKE_CURRENT_SOURCE_DIR}/bin/libMBVOptimization.so
                    #  ${CMAKE_CURRENT_SOURCE_DIR}/bin/libglog.so.0 
                      ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
                      #${Boost_LIBRARIES}   
                     )


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)


install(TARGETS follow_user RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES follow_user.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
