//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/LocalizationMonitor/cLocalizationMonitor.h"
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{

  ros::init(argc, argv, "localization_monitor");
  ros::NodeHandle n;

  cLocalizationMonitor myLocalizationMonitor(argc, argv);
  
  myLocalizationMonitor.open(n);

  ros::Rate r(5);
  while (ros::ok())
  {
        ros::spinOnce();
	myLocalizationMonitor.Run();
        r.sleep();
  }

  return 0;

}




