//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Paloma de la Puente
// Last update: 15.4.2014
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/PlacesLearningGui/qnode.hpp"

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>
#include "pugixml.hpp"
#include <iostream>
#include <fstream>

using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ROS callback function for the reception of the room name
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::currentRoomCallback(const std_msgs::String::ConstPtr& msg)
{

	// Rooms are added in previous step, by using RoomLearningTool


	//set the current room to the name received
	current_room = msg->data;

        cout << "current_room " << current_room << endl;


}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// function for the reception of new places, update
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::learnPlace(const std::string place_name, const std::string place_type)
{

	cout << "current_x " << current_x << endl;
        cout << "current_y " << current_y << endl;
	cout << "current_th " << current_theta << endl;

        std_msgs::String req_msg;
	req_msg.data = "Request";
	currentRoomNameRequestPubl.publish(req_msg);
        sleep(5);

	hobbit_msgs::Place new_place;
	new_place.x = current_x;
	new_place.y = current_y;
	new_place.theta = current_theta;
	new_place.place_name = place_name.c_str();
	new_place.place_type = place_type.c_str();
	
	bool places_in_room = false;
	
	// add new place to the current room if any other places have already been added to the room
	for (unsigned int i=0;i<rooms.rooms_vector.size();i++)
	{
		std::string room_n = rooms.rooms_vector[i].room_name;
		if (current_room.compare(room_n) == 0)
		{
			std::cout << "Adding place to room " << current_room << std::endl;
			rooms.rooms_vector[i].places_vector.push_back(new_place);
			places_in_room = true;
			break;
		}
	}

	// initialize and add the current room if no other places have already been added to the room
	if(!places_in_room)
	{
		hobbit_msgs::Room new_room;
		new_room.room_name = current_room;
		new_room.places_vector.push_back(new_place);	
		rooms.rooms_vector.push_back(new_room);	
		std::cout << "Adding new room " << current_room << std::endl;
	}

}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ROS callback function for the reception of the pose
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::getPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//should be updated with high frequency
	current_x = msg->pose.pose.position.x;
	current_y = msg->pose.pose.position.y;
	current_theta = tf::getYaw(msg->pose.pose.orientation);


	//cout << "current pose " << current_x << " " << current_y << " " << current_theta*180/M_PI << endl;

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor of the class - allocated and configures resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
QNode::QNode(int argc, char **argv) : init_argc(argc), init_argv(argv)
{
  opened = false;
  bRun = false; 

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Destructor of the class - waits for the ROS node to shut down and
// releases the allocated resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
QNode::~QNode()
{
  printf("cPlacesLearningGui::~cPlacesLearningGui(): shutting down ROS\n");
  usleep(100000);
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  usleep(100000);
  printf(" - done\n");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Function that just starts the thread function.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool QNode::init()
{

// Start the thread function.
  printf("Starting subscriber thread...\n");
  start();
  return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Thread function - this is where the ROS main loop runs.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::run()
{
  if (bRun)
  {

    cout << "here" << endl;

    // Init the ROS stuff and create subscribers and a publisher.
    ros::init(init_argc, init_argv, "PlacesLearningGui");
    ros::NodeHandle n;

    //////subscribers   

    // Subscriber for the current room name
    currentRoomSubs = n.subscribe<std_msgs::String>(CURRENT_ROOM, 1, &QNode::currentRoomCallback, this);
                                                                
    // Subscriber for the pose                                                             
    getPoseSubs = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(ESTIMATED_POSE, 1,
                                                                &QNode::getPoseCallback, this);

   // Publisher to request the current room name
    currentRoomNameRequestPubl = n.advertise<std_msgs::String>(CURRENT_ROOM_REQUEST, 1);

    opened = true;

    // ROS main loop.
    ros::spin();

  }
  else
  {
	printf("cPlacesLearningGui::Run(): ignoring run command due to previous error\n");

  }

// Send the shutdown signal.
  printf("Ros shutdown, proceeding to close the gui.\n");
  Q_EMIT rosShutdown();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Save places to XML file
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool QNode::savePlaces(std::string fileName)
{

    if (rooms.rooms_vector.size()==0)
    {
 		cout << "No places to be saved " << endl;
 		return false;
 
    }
	 

    //open the file

    pugi::xml_document doc;
    std::ifstream nameFile(fileName.c_str());
    pugi::xml_parse_result result = doc.load(nameFile);

    if (!result) //FIXME, pass as argument
    {
        std::cout << "Xml places file missing " << std::endl;
        //assert(0);
        return false;
    
    }

    /*****************************************************************************/

    // the rooms node
    pugi::xml_node rooms_node = doc.child("rooms");
    
    
    for (unsigned int i=0;i<rooms.rooms_vector.size();i++)
    {
	// room name of places to be added
	string room_name = rooms.rooms_vector[i].room_name;

	//cout << "Room name of places to be added " << room_name << endl;
	
	//find node in file
	for(pugi::xml_node room = rooms_node.child("room"); room; room = room.next_sibling("room"))
    	{
		std::string room_name_file;
        	room_name_file=room.child_value("name");
		if(room_name.compare(room_name_file)==0)
		{
			//cout << "Room found " << room_name << endl;
			// add places
			pugi::xml_node places_i = room.append_child();
		 	places_i.set_name("places");

		 
		 	//add place_i_0
		 	pugi::xml_node place_i_0 = places_i.append_child();
		 	place_i_0.set_name("place");
		  	
			// add place name
			pugi::xml_node place_name_i_0 = place_i_0.append_child();
			place_name_i_0.set_name("name");
			string place_i_0_name = rooms.rooms_vector[i].places_vector[0].place_name;
			place_name_i_0.append_child(pugi::node_pcdata).set_value(place_i_0_name.c_str());

			// add place type
			pugi::xml_node place_type_i_0 = place_i_0.append_child();
			place_type_i_0.set_name("type");
			//place_type_i_0.append_child(pugi::node_pcdata).set_value("EDIT, please write the type of place here");
			string place_i_0_type = rooms.rooms_vector[i].places_vector[0].place_type;
			place_type_i_0.append_child(pugi::node_pcdata).set_value(place_i_0_type.c_str());
		 
			//add pose
			pugi::xml_node pose_i_0 = place_i_0.append_child();
		 	pose_i_0.set_name("pose");
			
			// add attributes to pose
		 	float place_i_0_x = rooms.rooms_vector[i].places_vector[0].x;
		 	char place_i_0_x_converted[1000];
		 	snprintf(place_i_0_x_converted,sizeof(place_i_0_x_converted), "%f", place_i_0_x);
		 	pose_i_0.append_attribute("x") = place_i_0_x_converted;
		 
		 	float place_i_0_y = rooms.rooms_vector[i].places_vector[0].y;
		 	char place_i_0_y_converted[1000];
		 	snprintf(place_i_0_y_converted,sizeof(place_i_0_y_converted), "%f", place_i_0_y);
		 	pose_i_0.append_attribute("y") = place_i_0_y_converted;
		 
		 	float place_i_0_theta = rooms.rooms_vector[i].places_vector[0].theta;
		 	char place_i_0_theta_converted[1000];
		 	snprintf(place_i_0_theta_converted,sizeof(place_i_0_theta_converted), "%f", place_i_0_theta);
		 	pose_i_0.append_attribute("theta") = place_i_0_theta_converted;

		 	//add other places
		 	pugi::xml_node prev_place_i = place_i_0;
		 
		 	for (unsigned int j=1;j<rooms.rooms_vector[i].places_vector.size();j++)
		 	{
				pugi::xml_node place_i_j = places_i.insert_child_after(pugi::node_element, prev_place_i);
			 	place_i_j.set_name("place");

				// add place name
				pugi::xml_node place_name_i_j = place_i_j.append_child();
				place_name_i_j.set_name("name");
				string place_i_j_name = rooms.rooms_vector[i].places_vector[j].place_name;
				place_name_i_j.append_child(pugi::node_pcdata).set_value(place_i_j_name.c_str());

				// add place type
				pugi::xml_node place_type_i_j = place_i_j.append_child();
				place_type_i_j.set_name("type");
				//place_type_i_j.append_child(pugi::node_pcdata).set_value("EDIT, please write the type of place here");
    				string place_i_j_type = rooms.rooms_vector[i].places_vector[j].place_type;
				place_type_i_j.append_child(pugi::node_pcdata).set_value(place_i_j_type.c_str());
			 
				//add pose
				pugi::xml_node pose_i_j = place_i_j.append_child();
			 	pose_i_j.set_name("pose");
				
			 	// add attributes to pose
			 	float  places_i_j_x= rooms.rooms_vector[i].places_vector[j].x;
			 	char places_i_j_x_converted[1000];
		 	 	snprintf(places_i_j_x_converted,sizeof(places_i_j_x_converted), "%f", places_i_j_x);
			 	pose_i_j.append_attribute("x") = places_i_j_x_converted;
			 
			 	float  places_i_j_y= rooms.rooms_vector[i].places_vector[j].y;
			 	char places_i_j_y_converted[1000];
		 	 	snprintf(places_i_j_y_converted,sizeof(places_i_j_y_converted), "%f", places_i_j_y);
			 	pose_i_j.append_attribute("y") = places_i_j_y_converted;
			 
			 	float  places_i_j_theta= rooms.rooms_vector[i].places_vector[j].theta;
			 	char places_i_j_theta_converted[1000];
		 	 	snprintf(places_i_j_theta_converted,sizeof(places_i_j_theta_converted), "%f", places_i_j_theta);
			 	pose_i_j.append_attribute("theta") = places_i_j_theta_converted;

			 	prev_place_i = place_i_j;

		 	} //end places in room



		}// end this is the room
	}// end rooms in file

	
    	
    
    }// end rooms with places
    
	 
    /*std::cout << "Saving places: " << doc.save_file(fileName.c_str()) << std::endl;
    return true;*/
    
    return doc.save_file(fileName.c_str());
  
  
}




