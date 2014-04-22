cmake_minimum_required(VERSION 2.8.3)
project(places_learning_gui)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hobbit_msgs
  pugixml
  roscpp
  qt_build
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES test
  CATKIN_DEPENDS hobbit_msgs pugixml roscpp qt_build
#  DEPENDS system_lib
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


##############################################################################
# Qt Environment
##############################################################################

#include(qt_build qt-ros)
rosbuild_prepare_qt4(QtCore QtGui QtOpenGL) # Add the appropriate components to the component list here
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)


set(qt_srcs
         src/mainwindow.cpp
         src/myqlabel.cpp
	 src/mypanel.cpp
	 src/qnode.cpp
	 src/mydialog.cpp
    )
set(qt_hdrs
         include/PlacesLearningGui/mainwindow.h
         include/PlacesLearningGui/myqlabel.h
	 include/PlacesLearningGui/mypanel.h
	 include/PlacesLearningGui/qnode.hpp
	 include/PlacesLearningGui/ui_mainwindow.h
	 include/PlacesLearningGui/mydialog.h
    )
qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})

QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

##############################################################################
# Sections
##############################################################################

#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/PlacesLearningGui/*.hpp)

#QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
#QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

SET(QT_USE_QTOPENGL TRUE)


##############################################################################
# Sources
##############################################################################

#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################
include_directories(include ${catkin_INCLUDE_DIRS} )
#add_executable(places_learning_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_executable(places_learning_gui src/PlacesLearningGui.cpp ${qt_srcs} ${qt_moc_srcs} ${QT_FORMS_HPP})
add_dependencies(places_learning_gui hobbit_msgs_generate_messages_cpp)
target_link_libraries(places_learning_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} GL glut)




