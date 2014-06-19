cmake_minimum_required(VERSION 2.8.3)
project(emergency_detector)

find_package(catkin REQUIRED COMPONENTS sensor_msgs OpenCV cv_bridge roscpp std_msgs genmsg image_transport geometry_msgs roslib tf)
 
   
#Messages -----------------------------------
add_message_files(DIRECTORY msg FILES EmergencyMessage.msg SkeletonBBox.msg)

#Services -----------------------------------
add_service_files(DIRECTORY srv FILES TriggerEmergency.srv)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

#This needs to be after generate messages
catkin_package()

include_directories(${catkin_INCLUDEDIR} 
		    ${OpenCV_INCLUDEDIR})

add_executable(emergency_detector 
               src/main.cpp
               src/process.cpp
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
              )

target_link_libraries(emergency_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS emergency_detector RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES emergency_detector.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
