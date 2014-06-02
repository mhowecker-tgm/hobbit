//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Paloma de la Puente
// Last update: 22.4.2014
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <string.h>
#include "../include/GetCurrentRoom/cGetCurrentRoom.h"
#include "pugixml.hpp"
#include <iostream>
#include <fstream>
#include <string>

//using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace GetCurrentRoom {
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Opens the file with the list of initial known places and stores them.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cGetCurrentRoom::LoadFile(std::string placesFileName)
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
    	  hobbit_msgs::Room room_msg;

	//room name
        std::string room_name;
        room_name=room.child_value("name");
        room_msg.room_name = room_name;
        
         //Vertices
        pugi::xml_node vertices = room.child("vertices");
        for(pugi::xml_node vertex = vertices.child("vertex"); vertex; vertex = vertex.next_sibling("vertex"))
        {
	    hobbit_msgs::Point2D vertex_msg;
	
	    vertex_msg.x = atof(vertex.attribute("x").value());
	    vertex_msg.y = atof(vertex.attribute("y").value());

            room_msg.vertices_vector.push_back(vertex_msg);
        }
        
        
        known_rooms.rooms_vector.push_back(room_msg);
  }

  std::cout << "num rooms " << known_rooms.rooms_vector.size() << std::endl;

  return true;

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Opening and initializing the node
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cGetCurrentRoom::open(ros::NodeHandle & n)
{
        
//// Topics ///

//////subscribers   
        
// Subscriber for the current pose
    getCurrentPoseSubs = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(CURRENT_POSE, 1,
                                                                &cGetCurrentRoom::getCurrentPoseCallback, this);


// Subscriber for requesting the room name
    roomNameRequestedSubs = n.subscribe<std_msgs::String>(ROOM_NAME_REQUESTED, 1,
                                                                &cGetCurrentRoom::roomNameRequestedCallback, this);
                                                              
//////Publishers  

// Publisher for the room name                                                  
    currentRoomNamePubl = n.advertise<std_msgs::String>(CURRENT_ROOM_NAME, 1);

    opened = true;

    return;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ROS callback function for obtaining the current pose
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cGetCurrentRoom::getCurrentPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	current_x = msg->pose.pose.position.x;
	current_y = msg->pose.pose.position.y;

}

//Callback to publish current room name as a TOPIC
void cGetCurrentRoom::roomNameRequestedCallback(const std_msgs::String::ConstPtr& msg)
{

	if(msg->data.compare("Request") == 0)
	{
		//std::cout << "room name requested " << std::endl;
		for (unsigned int i=0;i<known_rooms.rooms_vector.size();i++)
		{
			std::vector<hobbit_msgs::Point2D> room_vertices;
			for (unsigned int j=0; j < known_rooms.rooms_vector[i].vertices_vector.size(); j++)
			{
				hobbit_msgs::Point2D vertex;
				vertex.x = known_rooms.rooms_vector[i].vertices_vector[j].x;
				//std::cout << "vertex_x" << vertex.x << std::endl;
				vertex.y = known_rooms.rooms_vector[i].vertices_vector[j].y;
				//std::cout << "vertex_y" << vertex.y << std::endl;
				room_vertices.push_back(vertex);
				//std::cout << "current_x" << current_x << std::endl;
				//std::cout << "current_y" << current_y << std::endl;
				room_vertices.push_back(vertex);
			}
			if (isInRoom(current_x,current_y,room_vertices)) //The vertices MUST be ordered
			{
				current_room_name.data = known_rooms.rooms_vector[i].room_name;
				//std::cout << "current room " << current_room_name.data << std::endl;
				break;
			}		

		}
		publish_room_name = true;
		
	}

}

//SERVICE to get current room name
bool cGetCurrentRoom::getCurrentRoom(hobbit_msgs::GetName::Request  &req, hobbit_msgs::GetName::Response &res)
 {
	
	ROS_INFO("Room name request received");

	for (unsigned int i=0;i<known_rooms.rooms_vector.size();i++)
	{
		std::vector<hobbit_msgs::Point2D> room_vertices;
		for (unsigned int j=0; j < known_rooms.rooms_vector[i].vertices_vector.size(); j++)
		{
			hobbit_msgs::Point2D vertex;
			vertex.x = known_rooms.rooms_vector[i].vertices_vector[j].x;
			vertex.y = known_rooms.rooms_vector[i].vertices_vector[j].y;
			room_vertices.push_back(vertex);
		}
		if (isInRoom(current_x,current_y,room_vertices)) //The vertices MUST be ordered
		{
			//current_room_name.data = known_rooms.rooms_vector[i].room_name;

			res.name = known_rooms.rooms_vector[i].room_name;
			break;
		}		

	}

	ROS_INFO("sending back response: [%s]", res.name.c_str());
	return true;

 }


//SERVICE to get check for a  room name
bool cGetCurrentRoom::getNameOfRoom(hobbit_msgs::GetNameOfRoom::Request  &req, hobbit_msgs::GetNameOfRoom::Response &res)
 {
	
	ROS_INFO("Room name request received");

	for (unsigned int i=0;i<known_rooms.rooms_vector.size();i++)
	{
		std::vector<hobbit_msgs::Point2D> room_vertices;
		for (unsigned int j=0; j < known_rooms.rooms_vector[i].vertices_vector.size(); j++)
		{
			hobbit_msgs::Point2D vertex;
			vertex.x = known_rooms.rooms_vector[i].vertices_vector[j].x;
			vertex.y = known_rooms.rooms_vector[i].vertices_vector[j].y;
			room_vertices.push_back(vertex);
		}
		if (isInRoom(req.x,req.y,room_vertices)) //The vertices MUST be ordered
		{
			//current_room_name.data = known_rooms.rooms_vector[i].room_name;

			res.name = known_rooms.rooms_vector[i].room_name;
			break;
		}		

	}

	ROS_INFO("sending back response: [%s]", res.name.c_str());
	return true;

 }



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Check if pose is inside polygon
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool cGetCurrentRoom::isInRoom(float x_pos, float y_pos, std::vector<hobbit_msgs::Point2D> polygon_points)
{
	//The points MUST be ordered either clockwise or counterclockwise
	//int num_intersections = 0;

//Implementation of the crossing number algorithm. Inspired by the original article in Communications of the ACM by M. Shimrat and by the pseudocode provided by D. Eppstein, with some inconsistencies fixed. Points laying on the edges or the vertices are also considered.
	bool b = false;
	for (unsigned int i = 0; i < polygon_points.size()-1; i++)
	{
		hobbit_msgs::Point2D vertex = polygon_points[i];
		hobbit_msgs::Point2D next_vertex = polygon_points[i+1];
		if ((vertex.x < x_pos && x_pos < next_vertex.x) || (vertex.x > x_pos && x_pos > next_vertex.x))
		{
		    float xr = x_pos;
		    float yr = ((next_vertex.y-vertex.y)*(xr-vertex.x)/(next_vertex.x-vertex.x))+vertex.y;
		    if (yr == y_pos) 
			return true;
		    else 
			if (yr < y_pos) 
				b = !b; //num_intersections++;
		}
		if (vertex.x == x_pos && vertex.y <= y_pos) 
		{
		    if (vertex.y == y_pos) 
			return true;

		    if (next_vertex.x == x_pos)
		    {
			if (y_pos <= next_vertex.y)
			    return true;
		    } 
		    else 
			if (next_vertex.x > x_pos) b=!b; //num_intersections++;

		}
	}

	//last segment checking
	int n = polygon_points.size();
	if ((polygon_points[n-1].x < x_pos && x_pos < polygon_points[0].x) || (polygon_points[n-1].x > x_pos && x_pos > polygon_points[0].x))
	{
		float yr_last = (polygon_points[0].y-polygon_points[n-1].y)*(x_pos-polygon_points[n-1].x)/(polygon_points[0].x-polygon_points[n-1].x)+polygon_points[n-1].y;

		if (yr_last == y_pos) 
			return true;
		else if (yr_last < y_pos) 
			b = !b; //num_intersections++;
	}

	if (polygon_points[n-1].x == x_pos && polygon_points[n-1].y <= y_pos) 
	{
            //std::cout << "last case " << std::endl;
	    if (polygon_points[n-1].y == y_pos) //point is a vertex of the polygon
		return true;

	    if (polygon_points[0].x == x_pos)  //point lies on one side of the polygon
	    {
		if (y_pos <= polygon_points[0].y)
		    return true;
	    } 
	    else 
		if (polygon_points[0].x < x_pos) b=!b; //num_intersections++;

	}

	//std::cout << "num intersect " << num_intersections << std::endl;

	/*if (num_intersections % 2== 0 )
		return false;
	else 
		return true;*/

	return b;

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor. Initialises member attributes and allocates resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cGetCurrentRoom::cGetCurrentRoom(int argc, char **argv)
{
   opened = false;
   bRun = true; //FIXME, set via keyboard to start the node?

   publish_room_name = false;
   current_x = 0;
   current_y = 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Destructor. Shuts down ROS, ends the thread and released allocated
// resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
cGetCurrentRoom::~cGetCurrentRoom()
{
  printf("cGetCurrentRoom::~cGetCurrentRoom(): shutting down ROS\n");
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
void cGetCurrentRoom::Run(void)
{
  if (bRun)
  {
	if (publish_room_name)
	{
		currentRoomNamePubl.publish(current_room_name);
		publish_room_name = false;
	}
  }
  else printf("cGetCurrentRoom::Run(): ignoring run command due to previous error\n");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

