
/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: GonÃ§alo Cabrita and Pedro Sousa on 17/11/2010
* Modified by: Paloma de la Puente on 31/10/2013
*********************************************************************/
#include <vector>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "hobbit_msgs/Pose2DStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#define GOAL_POSE "goal_pose"
//#define GOAL_SUCCESS "goal_success"
//#define GOAL_ABORTED "goal_aborted"
#define STOP_REQUEST "stop_request"

#define GOAL_STATUS "goal_status"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//std::vector<geometry_msgs::Pose2D> targets;

hobbit_msgs::Pose2DStamped goal_p;

bool new_goal;
//std_msgs::Bool goal_reached;
//std_msgs::Bool goal_aborted;

std_msgs::String goal_status;

bool stop;

ros::Publisher goal_status_pub;

void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
	if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) 
	{
		goal_status.data = "reached";
		std::cout << "Goal reached " << std::endl;
		goal_status_pub.publish(goal_status);

		goal_status.data = "idle";
		goal_status_pub.publish(goal_status);
	}
	if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) 
	{
		goal_status.data = "aborted";
		std::cout << "Goal aborted " << std::endl;
		goal_status_pub.publish(goal_status);
	}
	if(state.state_ == actionlib::SimpleClientGoalState::PREEMPTED) 
	{
		goal_status.data = "preempted";
		std::cout << "Goal preempted " << std::endl;
		goal_status_pub.publish(goal_status);
	}
	if(state.state_ == actionlib::SimpleClientGoalState::RECALLED) 
	{
		goal_status.data = "recalled";
		std::cout << "Goal recalled " << std::endl;
		goal_status_pub.publish(goal_status);
	}
}

void goalActiveCallback()
{
	ROS_INFO("Goal active");
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
	//ROS_INFO("Getting feedback");
}


void goalPoseCallback(const hobbit_msgs::Pose2DStamped::ConstPtr& goal_pose)
{
	goal_p.time = goal_pose->time;
	goal_p.x = goal_pose->x;
	goal_p.y = goal_pose->y;
	goal_p.theta = goal_pose->theta;
	new_goal = true;
}

void stopRequestCallback(const std_msgs::String::ConstPtr& msg)
{
	if (msg->data.compare("stop") || msg->data.compare("Stop") || msg->data.compare("STOP"))
	{
		stop = true;
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sending_goals");
	
	ros::NodeHandle n;
	
	goal_status_pub  = n.advertise<std_msgs::String>(GOAL_STATUS, 20);
	
	ros::Subscriber pose_sub = n.subscribe<hobbit_msgs::Pose2DStamped>(GOAL_POSE, 2, goalPoseCallback);
	ros::Subscriber stop_sub = n.subscribe<std_msgs::String>(STOP_REQUEST, 2, stopRequestCallback);

	
	// Tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	ros::Rate r(10);
	// Wait for the action server to come up
	ROS_INFO("Waiting for the move_base action server to come online...");
	if(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_FATAL("Forgot to launch move_base now did we?");
		ROS_BREAK();
	}
	
	stop = false;

	goal_status.data = "idle";
	
	while(ros::ok())
	{	
		if (stop) 
		{
			std::cout << "canceling goal " << std::endl;
			//ac.cancelGoal();
			ac.cancelAllGoals();
			stop = false;
		}
		if(new_goal)
		{	
			move_base_msgs::MoveBaseGoal goal;
		
			// Send a goal to the robot
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now(); //use this time instead of previous timestamp
		
			goal.target_pose.pose.position.x = goal_p.x;
			goal.target_pose.pose.position.y = goal_p.y;
			goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal_p.theta);
			
			new_goal = false;
			ac.sendGoal(goal, boost::bind(&goalDoneCallback, _1, _2), boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));

			goal_status.data = "new_goal";
			std::cout << "New goal: " << goal_p.x << " " << goal_p.y << " " << std::endl;
			goal_status_pub.publish(goal_status);
		}

		
		
		ros::spinOnce();
		r.sleep();
	}

	ROS_INFO("Exiting sending goal...");

	return 0;
}

