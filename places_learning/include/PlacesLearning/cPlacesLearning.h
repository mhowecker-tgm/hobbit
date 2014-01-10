//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Paloma de la Puente
// Last update: 7.10.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef PLACES_LEARNING_HPP_
#define PLACES_LEARNING_HPP_
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <ros/ros.h>
#include <vector>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "hobbit_msgs/RoomsVector.h"
//#include "Place.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//FIXME check topic names, move to a common interface file?
//#define ROOM_TO_BE_LEARNED "room_to_be_learned"
#define PLACE_TO_BE_LEARNED	"place_to_be_learned"
#define ESTIMATED_POSE	"amcl_pose" //FIXME 
#define CURRENT_ROOM "current_room"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace PlacesLearning {
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cPlacesLearning
{

  public:
    cPlacesLearning(int argc, char **argv);
    ~cPlacesLearning();
    void open(ros::NodeHandle & n);
    void Run(void);
    bool savePlaces(std::string fileName);

  private:
  
    bool bRun;
    bool opened;

    std::string current_room;
	 
    float current_x;
    float current_y;
    float current_theta;
	 
    hobbit_msgs::RoomsVector rooms;
    
    //subscribers
    
    ros::Subscriber currentRoomSubs;
    void currentRoomCallback(const std_msgs::String::ConstPtr& msg);
   
    ros::Subscriber learnPlaceSubs;
    void learnPlaceCallback(const std_msgs::String::ConstPtr& msg);
    
    ros::Subscriber getPoseSubs;
    void getPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    
    //publishers
    //ros::Publisher currentRoomPubl;
    
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

