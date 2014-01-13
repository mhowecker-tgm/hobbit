cmake_minimum_required(VERSION 2.4.6)

project(hobbit_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs message_generation actionlib)

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
  ObjectLocation.msg
  ObjectLocationVector.msg
  ObjectPosition.msg
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
  GetCoordinates.srv
  GetName.srv
  GetObjectLocations.srv
  GetRobotsCurrentRoom.srv
  GetRoomName.srv
  GetRooms.srv
  GetUsersCurrentRoom.srv
  PointCloud2FeatureHistogram.srv
  Request.srv
  SingleShotPC.srv
  Translate.srv
)

add_action_files(
  DIRECTORY action
  FILES
  ApproachUser.action
  BringObject.action
  CallUser.action
  LocateUser.action
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs actionlib geometry_msgs)

catkin_package(

  CATKIN_DEPENDS message_runtime
)
