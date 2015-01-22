//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/ComeCloser/cComeCloser.h"
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{

  ros::init(argc, argv, "come_closer");

  cComeCloser myComeCloser(argc, argv);

  ros::Rate r(10);
  while (ros::ok())
  {
        ros::spinOnce();
        r.sleep();
  }

  return 0;

}
