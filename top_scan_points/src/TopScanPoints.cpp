//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/TopScanPoints/cTopScanPoints.h"
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{

  ros::init(argc, argv, "topScan");
  ros::NodeHandle n;

  cTopScanPoints myTopScanPoints(argc, argv);

  myTopScanPoints.open(n);

  tf::TransformBroadcaster obstacle_broadcaster;
  myTopScanPoints.p_obstacle_broadcaster = &obstacle_broadcaster;

  ros::Rate r(10);
  while (ros::ok())
  {
        ros::spinOnce();
	myTopScanPoints.Run();
        r.sleep();
  }


  return 0;
}



