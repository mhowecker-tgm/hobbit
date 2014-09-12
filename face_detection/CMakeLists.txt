cmake_minimum_required(VERSION 2.8.3)
project(face_detection)


if(NOT OpenCV_DIR)
 find_path(OpenCV_DIR "OpenCVConfig.cmake" DOC "Root directory of OpenCV" HINTS    "/opt/ros/hydro/share/OpenCV/")
endif(NOT OpenCV_DIR)


find_package(catkin REQUIRED COMPONENTS sensor_msgs OpenCV cv_bridge roscpp std_msgs genmsg image_transport geometry_msgs roslib tf)
 
   
#Messages -----------------------------------
add_message_files(DIRECTORY msg FILES Person.msg)

#Services -----------------------------------
add_service_files(DIRECTORY srv FILES SetQuality.srv)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs face_detection)

#This needs to be after generate messages
catkin_package()

find_package(Boost COMPONENTS system filesystem thread serialization program_options) 
#LINK_DIRECTORIES(${BOOST_LIBRARY_DIRS})


include_directories(${catkin_INCLUDEDIR} 
		    ${OpenCV_INCLUDEDIR})


add_executable(face_detection src/main.cpp src/services.cpp src/pose.cpp src/FaceDetection.cpp src/AmMatrix/solveLinearSystemGJ.c src/AmMatrix/matrixCalculations.c src/AmMatrix/matrix4x4Tools.c src/AmMatrix/matrix3x3Tools.c src/calibration.c src/ImageOperations/imageOps.c)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(face_detection  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

install(TARGETS face_detection RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES face_detection.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
