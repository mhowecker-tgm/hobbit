//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Paloma de la Puente
// Last update: 10.10.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/GetCurrentRoom/cGetCurrentRoom.h"
#include "ros/package.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{
   
  ros::init(argc, argv, "GetCurrentRoom");
  ros::NodeHandle n;

  std::cout<<"Starting GetCurrentRoom..."<< std::endl;
  
  GetCurrentRoom::cGetCurrentRoom myGetCurrentRoom(argc, argv);
  myGetCurrentRoom.open(n);

  //std::string file_name = ros::package::getPath("GetCurrentRoom") + "/launch/places.xml"; //FIXME, should be given as a parameter

  //std::string file_name = ros::package::getPath("PlacesLearning") + "/launch/places.xml";
  //std::cout << "file_name " << file_name << std::endl;
  //myGetCurrentRoom.LoadFile(file_name);

  std::string file_name(argv[1]);
  std::cout << "file_name " << file_name << std::endl;
  myGetCurrentRoom.LoadFile(file_name);

  ros::ServiceServer service = n.advertiseService("getCurrentRoom", &GetCurrentRoom::cGetCurrentRoom::getCurrentRoom, &myGetCurrentRoom);
  ros::ServiceServer service1 = n.advertiseService("getRoomName", &GetCurrentRoom::cGetCurrentRoom::getNameOfRoom, &myGetCurrentRoom);
  ROS_INFO("Service ready");

  ROS_INFO("Init loop");
    //Loop
  while(ros::ok())
  {
      //Read ros messages
      ros::spinOnce();
      myGetCurrentRoom.Run();
  }

  return 0;
  
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


