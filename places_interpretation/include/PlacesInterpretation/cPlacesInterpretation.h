//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Paloma de la Puente
// Last update: 11.11.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef PLACES_INTERPRETATION_HPP_
#define PLACES_INTERPRETATION_HPP_
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <ros/ros.h>
#include <vector>
#include "hobbit_msgs/Pose2DStamped.h"
#include "hobbit_msgs/RoomsVector.h"
#include "hobbit_msgs/PlacesVector.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseArray.h"
#include "hobbit_msgs/GetRooms.h"
//#include "Place.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//FIXME check topic names, move to a common interface file?
#define PLACE_NAME_TARGET	"place_name_target"
#define ROOM_NAME_TARGET	"room_name_target"
#define PLACE_GOAL	        "goal_pose"
#define PLACES_REQUESTED	"places_requested"
#define PLACES_IN_ROOM	        "places_in_room"
#define CORRECT_ROOM_NAME	"correct_room_name"
#define CORRECT_PLACE_NAME	"correct_place_name"
#define ALL_PLACES              "all_places"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace PlacesInterpretation {
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class cPlacesInterpretation
{

  public:
    cPlacesInterpretation(int argc, char **argv);
    ~cPlacesInterpretation();
    void open(ros::NodeHandle & n);
    void Run(void);

    bool LoadFile(std::string placesFileName);

//services
    bool getRooms(hobbit_msgs::GetRooms::Request  &req, hobbit_msgs::GetRooms::Response &res);

  private:
  
    bool bRun;
    bool opened;

    bool new_place;

    std::string current_room;
    std::string current_place;
	 
    bool current_room_found;
	 
    hobbit_msgs::RoomsVector known_rooms;

    geometry_msgs::PoseArray all_places;
    
    //subscribers
    
    ros::Subscriber getRoomNameSubs;
    void getRoomNameCallback(const std_msgs::String::ConstPtr& msg);
    
    ros::Subscriber goToPlaceSubs;
    void goToPlaceCallback(const std_msgs::String::ConstPtr& msg);

    ros::Subscriber getPlacesSubs;
    void getPlacesCallback(const std_msgs::String::ConstPtr& msg);
    
    //publishers
    hobbit_msgs::Pose2DStamped goal;
    ros::Publisher goalPubl;
    int goalPublication(hobbit_msgs::Pose2DStamped g);

    hobbit_msgs::PlacesVector places;
    ros::Publisher placesPubl;
    
    std_msgs::Bool place_isknown;
    ros::Publisher recognizedPlacePubl;
    
    std_msgs::Bool room_isknown;
    ros::Publisher recognizedRoomPubl;
    
    ros::Publisher knownPlacesPubl;
    
    /*hobbit_msgs::RoomsVector known_rooms;
    ros::Publisher knownPlacesPubl;
    int knownPlacesPublication(hobbit_msgs::PlacesVector kp);*/
    
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

