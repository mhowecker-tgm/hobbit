//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Paloma de la Puente
// Last update: 15.4.2014
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef QNODE_HPP_
#define QNODE_HPP_
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <stdint.h>
#include <ros/ros.h>
#include <QThread>

#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "hobbit_msgs/RoomsVector.h"

#define PLACE_TO_BE_LEARNED	"place_to_be_learned"
#define ESTIMATED_POSE	"amcl_pose" //FIXME 
#define CURRENT_ROOM "current_room"
#define CURRENT_ROOM_REQUEST "room_name_requested"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class QNode : public QThread
{
  Q_OBJECT

  public:

    QNode(int argc, char **argv);
    virtual ~QNode();
    bool init();
    void run();

    bool savePlaces(std::string fileName);

    void learnPlace(const std::string place_name, const std::string place_type);

    bool bRun;

    float current_x;
    float current_y;
    float current_theta;

    hobbit_msgs::RoomsVector rooms;

  Q_SIGNALS:
    void rosShutdown();

  private:

    int init_argc;
    char **init_argv;

    bool opened;

    std::string current_room;
    
    //subscribers
    
    ros::Subscriber currentRoomSubs;
    void currentRoomCallback(const std_msgs::String::ConstPtr& msg);
    
    ros::Subscriber getPoseSubs;
    void getPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

   //publisher to request the current room name

    ros::Publisher currentRoomNameRequestPubl;

};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

