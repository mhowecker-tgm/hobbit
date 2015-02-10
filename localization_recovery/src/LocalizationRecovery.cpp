//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/LocalizationRecovery/cLocalizationRecovery.h"
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{

  ros::init(argc, argv, "localization_recovery");

  cLocalizationRecovery myLocalizationRecovery(argc, argv);

  ros::Rate r(10);
  while (ros::ok())
  {
        ros::spinOnce();
        r.sleep();
  }

  return 0;

}
