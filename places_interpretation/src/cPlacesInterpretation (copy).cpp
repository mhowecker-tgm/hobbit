//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Paloma de la Puente
// Last update: 7.10.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <string.h>
#include "../include/PlacesInterpretation/cPlacesInterpretation.h"
#include "pugixml.hpp"
#include <iostream>
#include <fstream>
#include <string>

//using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace PlacesInterpretation {
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Opens the file with the list of initial known places and stores them.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cPlacesInterpretation::LoadFile(std::string placesFileName)
{

// Open the file.


    pugi::xml_document doc;
    std::ifstream nameFile(placesFileName.c_str());
    pugi::xml_parse_result result = doc.load(nameFile);

   if (!result) //FIXME, pass as argument
   {
        std::cout << "Xml places file missing " << std::endl;
        //assert(0);
        return false;
    
   }
   
   
   pugi::xml_node rooms = doc.child("rooms");

	// known rooms 
	
	 for(pugi::xml_node room = rooms.child("room"); room; room = room.next_sibling("room"))
    {
    
    	  HobbitMsgs::Room room_msg;

		  //room name
        std::string room_name;
        room_name=room.child_value("name");
        room_msg.room_name = room_name;

        //default ref pose
        /*std::string readingValue1;
        readingValue1=room.child_value("x_ref");
        std::istringstream convertValue1(readingValue1);
        convertValue1>>room_msg.x_ref;
        
        std::string readingValue2;
        readingValue2=room.child_value("y_ref");
        std::istringstream convertValue2(readingValue2);
        convertValue2>>room_msg.y_ref;
        
        std::string readingValue3;
        readingValue3=room.child_value("theta_ref");
        std::istringstream convertValue3(readingValue3);
        convertValue3>>room_msg.theta_ref;*/
        

        //Places
        pugi::xml_node places = room.child("places");
        for(pugi::xml_node place = places.child("place"); place; place = place.next_sibling("place"))
        {
        		HobbitMsgs::Place place_msg;
        		
        		place_msg.place_name = place.attribute("place_name").value();
        		place_msg.x = atof(place.attribute("x").value());
        		place_msg.y = atof(place.attribute("y").value());
        		place_msg.theta = atof(place.attribute("theta").value());

            room_msg.places_vector.push_back(place_msg);
        }
        
         //Vertices
   /*     pugi::xml_node vertices = room.child("vertices");
        for(pugi::xml_node vertex = vertices.child("vertex"); vertex; vertex = vertex.next_sibling("vertex"))
        {
        		HobbitMsgs::Point2D vertex_msg;
        		
        		vertex_msg.x = atof(vertex.attribute("x").value());
        		vertex_msg.y = atof(vertex.attribute("y").value());

            room_msg.vertices_vector.push_back(pose_msg);
        }*/
        
        
        known_rooms.rooms_vector.push_back(room_msg);
  }

  std::cout << "num rooms " << known_rooms.rooms_vector.size() << std::endl;

  all_places.header.frame_id = "map";
  all_places.header.stamp = ros::Time::now(); 

  for (unsigned int i=0;i< known_rooms.rooms_vector.size();i++)
  {
	for (unsigned int j=0;j<known_rooms.rooms_vector[i].places_vector.size();j++)
    	{
		HobbitMsgs::Place place_ij = known_rooms.rooms_vector[i].places_vector[j];
		geometry_msgs::Pose p;
		p.position.x = place_ij.x;
		p.position.y = place_ij.y;
		p.orientation.x = 0;
		p.orientation.y = 0;
		p.orientation.z = sin(0.5*place_ij.theta);
		p.orientation.w = cos(0.5*place_ij.theta);

		all_places.poses.push_back(p);
    	}
	
	

  }

  return true;

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Opening and initializing the node
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cPlacesInterpretation::open(ros::NodeHandle & n)
{
        
//// Topics ///

//////subscribers   
        
// Subscriber for going to a place
    goToPlaceSubs = n.subscribe<std_msgs::String>(PLACE_NAME_TARGET, 1,
                                                                &cPlacesInterpretation::goToPlaceCallback, this);
                                                                
// Subscriber for obtaining the room name
    getRoomNameSubs = n.subscribe<std_msgs::String>(ROOM_NAME_TARGET, 1,
                                                                &cPlacesInterpretation::getRoomNameCallback, this);

// Subscriber for the places request
    getPlacesSubs = n.subscribe<std_msgs::String>(PLACES_REQUESTED, 1,
                                                                &cPlacesInterpretation::getPlacesCallback, this);

//////Publishers  

// Publisher for the goal                                                  
    goalPubl = n.advertise<HobbitMsgs::Pose2DStamped>(PLACE_GOAL, 1);
// Publisher for the places in a room 
    placesPubl = n.advertise<HobbitMsgs::PlacesVector>(PLACES_IN_ROOM, 1);
// Publishing if the room name is correct
    recognizedRoomPubl = n.advertise<std_msgs::Bool>(CORRECT_ROOM_NAME, 1);
// Publishing if the place name is correct
    recognizedPlacePubl = n.advertise<std_msgs::Bool>(CORRECT_PLACE_NAME, 1);
// Publisher for all the places
    knownPlacesPubl = n.advertise<geometry_msgs::PoseArray>(ALL_PLACES, 1);

    opened = true;

    return;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ROS callback function for the reception of target place name
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cPlacesInterpretation::goToPlaceCallback(const std_msgs::String::ConstPtr& msg)
{
	// examine the list of available known places and obtain the pose corresponding to the given place name, if it is correct
	
	bool correct_place_name = false;
	bool current_room_found = false;

	std::cout << "go to place " << msg->data << std::endl;
	
	for (unsigned int i=0;i<known_rooms.rooms_vector.size();i++)
	{
		if (known_rooms.rooms_vector[i].room_name.compare(current_room)==0)
		{
			current_room_found = true;
			std::cout << "room found " << current_room << std::endl;
			for (unsigned int j=0;j<known_rooms.rooms_vector[i].places_vector.size();j++)
			{
				if(known_rooms.rooms_vector[i].places_vector[j].place_name == msg->data)
				{
					//goal.time = ros::Time::now() - ros::Time(0,0); //FIXME

					current_place = msg->data;

					goal.x = known_rooms.rooms_vector[i].places_vector[j].x;
					goal.y = known_rooms.rooms_vector[i].places_vector[j].y;
					goal.theta = known_rooms.rooms_vector[i].places_vector[j].theta;

					std::cout << "goal pose " << goal.x << " " << goal.y << " " << goal.theta << std::endl;
				
					correct_place_name = true;
					new_place = true;
		
				}
			}
		}
	
	}
	
	room_isknown.data = current_room_found;
	recognizedRoomPubl.publish(room_isknown);
	
	place_isknown.data = correct_place_name;
	recognizedPlacePubl.publish(place_isknown);
	
	//TODO give nice output?

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ROS callback function for the reception of the room name
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cPlacesInterpretation::getRoomNameCallback(const std_msgs::String::ConstPtr& msg)
{

	// examine the list of available known places and obtain the pose corresponding to the given place name, if it is correct
	
	current_room = msg->data;

	std::cout << "go to room " << current_room << std::endl;
	
	bool current_room_found = false;
	
	for (unsigned int i=0;i<known_rooms.rooms_vector.size();i++)
	{
		if (known_rooms.rooms_vector[i].room_name.compare(current_room)==0)
		{
			current_room_found = true;
		}
	}
	
	room_isknown.data = current_room_found;
	recognizedRoomPubl.publish(room_isknown);
	
	//TODO give nice output?

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ROS callback function for the reception of the places in the current room
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cPlacesInterpretation::getPlacesCallback(const std_msgs::String::ConstPtr& msg)
{
	if(msg->data.compare("Request") == 0)
	{

		bool publish_places = false;
		for (unsigned int i=0;i<known_rooms.rooms_vector.size();i++)
		{
			if (known_rooms.rooms_vector[i].room_name.compare(current_room)==0)
			{
				//room name found
				places.places_vector = known_rooms.rooms_vector[i].places_vector;
				publish_places = true;
				break;

			}

		}
		if (publish_places)
			placesPubl.publish(places);
		
	}
	

}

//SERVICE to get all the rooms
bool cPlacesInterpretation::getRooms(HobbitMsgs::GetRooms::Request  &req, HobbitMsgs::GetRooms::Response &res)
 {
	
	ROS_INFO("Rooms request received");

	res.rooms = known_rooms;

	ROS_INFO("sending back rooms response");
	ROS_INFO("number of rooms: [%d]", res.rooms.rooms_vector.size());


	return true;

 }


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// publication of the next goal corresponding to the given place
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int cPlacesInterpretation::goalPublication(HobbitMsgs::Pose2DStamped g)
{
    if(!opened)
        return 0;


    goalPubl.publish(g);


    return 1;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor. Initialises member attributes and allocates resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cPlacesInterpretation::cPlacesInterpretation(int argc, char **argv)
{
   opened = false;
   bRun = true; //FIXME, set via keyboard to start the node?
   new_place = false;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Destructor. Shuts down ROS, ends the thread and released allocated
// resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cPlacesInterpretation::~cPlacesInterpretation()
{
  printf("cPlacesInterpretation::~cPlacesInterpretation(): shutting down ROS\n");
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
// ROS startup, resource allocation and ROS main loop.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cPlacesInterpretation::Run(void)
{
  if (bRun) 
  {
	if(new_place)
	{
  		goalPublication(goal);
		std::cout << "published new goal pose for place " << current_place << std::endl;
		new_place = false;
	}

	/*geometry_msgs::PoseArray all_places;
	all_places.header.frame_id = "map";
	all_places.header.stamp = ros::Time::now(); 

	for (unsigned int i=0;i< known_rooms.rooms_vector.size();i++)
	{
		for (unsigned int j=0;j<known_rooms.rooms_vector[i].places_vector.size();j++)
		{
			HobbitMsgs::Place place_ij = known_rooms.rooms_vector[i].places_vector[j];
			geometry_msgs::Pose p;
			p.position.x = place_ij.x;
			p.position.y = place_ij.y;
			p.orientation.x = 0;
			p.orientation.y = 0;
			p.orientation.z = sin(0.5*place_ij.theta);
			p.orientation.w = cos(0.5*place_ij.theta);

			all_places.poses.push_back(p);
		}
		
		

	}*/


	knownPlacesPubl.publish(all_places);

  }
  else printf("cPlacesInterpretation::Run(): ignoring run command due to previous error\n");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Publisher for the places
