cmake_minimum_required(VERSION 2.8.3)
project(emergency_detector)

find_package(catkin REQUIRED COMPONENTS sensor_msgs OpenCV cv_bridge roscpp std_msgs genmsg image_transport geometry_msgs roslib tf message_generation)
 
   
#Messages -----------------------------------
add_message_files(DIRECTORY msg FILES EmergencyMessage.msg SkeletonBBox.msg Skeleton2D.msg Skeleton3D.msg Person.msg )
 


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs emergency_detector)

#This needs to be after generate messages
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDEDIR} 
		    ${OpenCV_INCLUDEDIR})

add_executable(emergency_detector 
               src/main.cpp
               src/process.cpp
               src/fall_detection.cpp
               src/RGBDAcquisition/acquisitionSegment/AcquisitionSegment.c
               src/RGBDAcquisition/acquisitionSegment/automaticPlaneSegmentation.c
               src/RGBDAcquisition/acquisitionSegment/colorSelector.c
               src/RGBDAcquisition/acquisitionSegment/combineRGBAndDepthOutput.c
               src/RGBDAcquisition/acquisitionSegment/depthSelector.c
               src/RGBDAcquisition/acquisitionSegment/imageProcessing.c 
               src/RGBDAcquisition/tools/Quasirandomness/quasirandomness.c
               src/RGBDAcquisition/tools/Calibration/calibration.c 
               src/RGBDAcquisition/tools/Calibration/calibration.h 
               src/RGBDAcquisition/tools/AmMatrix/matrix3x3Tools.c 
               src/RGBDAcquisition/tools/AmMatrix/matrix3x3Tools.h
               src/RGBDAcquisition/tools/AmMatrix/matrix4x4Tools.c
               src/RGBDAcquisition/tools/AmMatrix/matrix4x4Tools.h
               src/RGBDAcquisition/tools/AmMatrix/matrixCalculations.c
               src/RGBDAcquisition/tools/AmMatrix/matrixCalculations.h
               src/RGBDAcquisition/tools/AmMatrix/solveLinearSystemGJ.c
               src/RGBDAcquisition/tools/AmMatrix/solveLinearSystemGJ.h
               src/RGBDAcquisition/tools/AmMatrix/solveHomography.c
               src/RGBDAcquisition/tools/AmMatrix/solveHomography.h

               src/RGBDAcquisition/processors/ViewpointChange/main.c
               src/RGBDAcquisition/processors/ViewpointChange/ViewpointChange.h
               src/RGBDAcquisition/processors/ViewpointChange/viewpoint_change.c
               src/RGBDAcquisition/processors/ViewpointChange/viewpoint_change.h
              )

target_link_libraries(emergency_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp hobbit_msgs  hobbit_msgs_generate_messages_cpp)

install(TARGETS emergency_detector RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES emergency_detector.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
