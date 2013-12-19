cmake_minimum_required(VERSION 2.4.6)

project(hobbit_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs message_generation)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages

add_message_files(
  DIRECTORY msg
  FILES
  Command.msg
  Event.msg
  Object.msg
  ObjectPosition.msg
  Objects.msg
  Parameter.msg
  Place.msg
  PlacesVector.msg
  Point2D.msg
  Pose2DStamped.msg
  Room.msg
  RoomsVector.msg
  Status.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ClustersOnPlane.srv
  GetName.srv
  GetRooms.srv
  PointCloud2FeatureHistogram.srv
  Request.srv
  SingleShotPC.srv
  Translate.srv

)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(

  CATKIN_DEPENDS message_runtime
)
