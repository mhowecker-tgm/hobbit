//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Paloma de la Puente
// Last update: 11.11.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/PlacesInterpretation/cPlacesInterpretation.h"
#include "ros/package.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{
   
  ros::init(argc, argv, "PlacesInterpretation");
  ros::NodeHandle n;

  std::cout<<"Starting PlacesInterpretation..."<< std::endl;
  
  PlacesInterpretation::cPlacesInterpretation myPlacesInterpretation(argc, argv);
  myPlacesInterpretation.open(n);

  //std::string file_name = ros::package::getPath("PlacesInterpretation") + "/launch/places.xml"; //FIXME, should be given as a parameter
  
  /*std::string file_name = ros::package::getPath("PlacesLearning") + "/launch/places.xml";

  std::cout << "file_name " << file_name << std::endl;*/

  std::string file_name(argv[1]);
  std::cout << "file_name " << file_name << std::endl;

  myPlacesInterpretation.LoadFile(file_name);

  ros::ServiceServer service = n.advertiseService("getRooms", &PlacesInterpretation::cPlacesInterpretation::getRooms, &myPlacesInterpretation);
  ROS_INFO("Service ready");

  ROS_INFO("Init loop");
   //Loop
  while(ros::ok())
  {
      //Read ros messages
      ros::spinOnce();
      myPlacesInterpretation.Run();
  }

  return 0;
  
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


