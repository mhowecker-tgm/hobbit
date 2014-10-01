cmake_minimum_required(VERSION 2.8.3)
project(rgbd_acquisition)

if(NOT OpenCV_DIR)
 find_path(OpenCV_DIR "OpenCVConfig.cmake" DOC "Root directory of OpenCV" HINTS    "/opt/ros/hydro/share/OpenCV/")
endif(NOT OpenCV_DIR)


find_package(catkin REQUIRED COMPONENTS 
  sensor_msgs 
  OpenCV 
  sensor_msgs
  cv_bridge 
  roscpp 
  std_msgs 
  genmsg 
  image_transport 
  geometry_msgs 
  roslib 
  tf)
 
  

# OpenNI2 Specific Declarations ---------------
#find_package(PkgConfig)
#pkg_check_modules(OpenNI2 REQUIRED libopenni2)
find_path(OpenNI2_INCLUDEDIR
	  NAMES OpenNI.h
	  HINTS /usr/include/openni2)
find_library(OpenNI2_LIBRARIES
	     NAMES OpenNI2 DummyDevice OniFile PS1090
	     HINTS /usr/lib/ /usr/lib/OpenNI2/Drivers
	     PATH_SUFFIXES lib) 
message(STATUS ${OpenNI2_LIBRARIES})
#--------------------------------------------



# Nite2 Specific Declarations They are now added inside this node ---------------
#message(status $ENV{NITE2_INCLUDE})
#message(status $ENV{NITE2_REDIST64})
#find_path(Nite2_INCLUDEDIR
#	  NAMES NiTE.h
#	  HINTS $ENV{NITE2_INCLUDE})
#find_library(Nite2_LIBRARY
#	     NAMES NiTE2
#	     HINTS $ENV{NITE2_REDIST64}
#	     PATH_SUFFIXES lib) 
#--------------------------------------------

#Messages -----------------------------------
add_message_files(DIRECTORY msg FILES Person.msg PointEvents.msg Skeleton2D.msg Skeleton3D.msg SkeletonBBox.msg )

#Services -----------------------------------
add_service_files(DIRECTORY srv FILES SetQuality.srv)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs rgbd_acquisition)

#This needs to be after generate messages
catkin_package()

include_directories(${catkin_INCLUDEDIR}  
		    ${OpenNI2_INCLUDEDIR}
#		    ${Nite2_INCLUDEDIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/NiTE2Includes
		    ${OpenCV_INCLUDEDIR})


add_executable(rgbd_acquisition  src/main.cpp src/extAcquisition.cpp src/OpenNI2Acquisition.cpp src/Nite2.cpp src/services.cpp src/pose.cpp)

#${Nite2_LIBRARY}
target_link_libraries(rgbd_acquisition ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES}  ${CMAKE_CURRENT_SOURCE_DIR}/bin/libNiTE2.so ${OpenCV_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

install(TARGETS rgbd_acquisition RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES rgbd_acquisition.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
