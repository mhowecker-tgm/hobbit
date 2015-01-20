//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/PersonFollowing/cPersonFollowing.h"
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{

  ros::init(argc, argv, "person_following");

  cPersonFollowing myPersonFollowing(argc, argv);

  ros::Rate r(10);
  while (ros::ok())
  {
        ros::spinOnce();
        r.sleep();
  }

  return 0;

}
