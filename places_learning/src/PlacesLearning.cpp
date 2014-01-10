//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Paloma de la Puente
// Last update: 2.10.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/PlacesLearning/cPlacesLearning.h"
#include "ros/package.h"
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{
   
  ros::init(argc, argv, "PlacesLearning");
  ros::NodeHandle n;

  cout << "Starting PlacesLearning..." << endl;
  
  PlacesLearning::cPlacesLearning myPlacesLearning(argc, argv);
  myPlacesLearning.open(n);

    //Loop
  while(ros::ok())
  {
      //Read ros messages
      ros::spinOnce();
      myPlacesLearning.Run();
  }

  /*
  std::string file_name = ros::package::getPath("PlacesLearning") + "/launch/places.xml"; //FIXME, should be given as a parameter
  std::cout << "file_name " << file_name << std::endl;*/

  //std::string file_name = ros::package::getPath("Navigation") + "/places.xml";

  std::string file_name(argv[1]);
  std::cout << "file_name " << file_name << std::endl;

  
  if(!myPlacesLearning.savePlaces(file_name))
  		cout << "Places file was not saved " << endl;
  return 0;
  
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


