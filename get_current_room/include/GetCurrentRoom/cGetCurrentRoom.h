//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Paloma de la Puente
// Last update: 10.10.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef GET_CURRENT_ROOM_HPP_
#define GET_CURRENT_ROOM_HPP_
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <ros/ros.h>
#include <vector>
#include "hobbit_msgs/Pose2DStamped.h"
#include "hobbit_msgs/Point2D.h"
#include "hobbit_msgs/RoomsVector.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "hobbit_msgs/GetName.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//FIXME check topic names, move to a common interface file?
#define CURRENT_ROOM_NAME	"current_room"
#define CURRENT_POSE	"amcl_pose"
#define ROOM_NAME_REQUESTED "room_name_requested"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace GetCurrentRoom {
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cGetCurrentRoom
{

  public:
    cGetCurrentRoom(int argc, char **argv);
    ~cGetCurrentRoom();
    void open(ros::NodeHandle & n);
    void Run(void);

    bool LoadFile(std::string placesFileName);

    //services
    bool getCurrentRoom(hobbit_msgs::GetName::Request  &req, hobbit_msgs::GetName::Response &res);

  private:
  
    bool bRun;
    bool opened;
	 
    bool publish_room_name;

    float current_x;
    float current_y;

    hobbit_msgs::RoomsVector known_rooms;

    bool isInRoom(float x_pos, float y_pos, std::vector<hobbit_msgs::Point2D> polygon_points);
    
    //subscribers
    
    /*ros::Subscriber getCurrentPoseSubs;
    void getCurrentPoseCallback(const hobbit_msgs::Pose2DStamped::ConstPtr& msg);*/

    ros::Subscriber getCurrentPoseSubs;
    void getCurrentPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    ros::Subscriber roomNameRequestedSubs;
    void roomNameRequestedCallback(const std_msgs::String::ConstPtr& msg);
    
    //publishers
    
    std_msgs::String current_room_name;
    ros::Publisher currentRoomNamePubl;


    
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

