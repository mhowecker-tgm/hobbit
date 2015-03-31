
#include <interfaces_mira/MiraGoRecharge.h>

#include <navigation/Task.h>
#include <navigation/tasks/PreferredDirectionTask.h>
#include <navigation/tasks/DockingTask.h>

#include <rpc/RPCServer.h>

#include <string>

using namespace mira::navigation;

MiraGoRecharge::MiraGoRecharge() : MiraRobotModule(std::string ("GoRecharge")) {
}

void MiraGoRecharge::initialize() 
{
  robot_->getMiraAuthority().subscribe<std::string>("/localization/TemplateLocalizationEvent", &MiraGoRecharge::template_channel_callback, this);
  robot_->getMiraAuthority().subscribe<std::string>("/docking/DockingStatus", &MiraGoRecharge::status_channel_callback, this);

  status.data = "idle";

  as_ = new actionlib::SimpleActionServer<interfaces_mira::MiraDockingAction>(robot_->getRosNode(), "docking_task", boost::bind(&MiraGoRecharge::executeCb, this, _1), false);
  as_->start();

  status_updated = false;

}

void MiraGoRecharge::template_channel_callback(mira::ChannelRead<std::string> data) 
{

        if(data->value().compare("Success") == 0) 
	{
		status.data = "template_found";
		template_found = true;
		std::cout << "template_found " << std::endl;
		ROS_INFO("template_found ");
		timeout = ros::Time::now() + ros::Duration(40); //FIXME, add parameter

	}
	if(data->value().compare("Failure") == 0) 
	{
  		status.data = "template_not_found";
		std::cout << "template_not_found " << std::endl;
		ROS_INFO("template_not_found ");
		template_found = false;
	}

	status_updated = true;

}

void MiraGoRecharge::status_channel_callback(mira::ChannelRead<std::string> data) 
{
	

        if(data->value().compare("Docked") == 0) 
	{
		status.data = "docked";
		std::cout << "docked " << std::endl;
	}
	if(data->value().compare("UnDocked") == 0) 
	{
		status.data = "undocked";
		std::cout << "undocked " << std::endl;

	}
	if(data->value().compare("Failure") == 0) 
	{
  		status.data = "failure";
		std::cout << "failure " << std::endl;

	}

	status_updated = true;
}

void MiraGoRecharge::docking_task_callback(const std_msgs::String::ConstPtr& msg)
{

	//if (msg->data.compare("docking_station") || msg->data.compare("DockingStation") || msg->data.compare("Recharge") || msg->data.compare("recharge"))

	std::cout << "data " << msg->data << std::endl;
	
	if (msg->data.compare("docking_on") == 0)
	{
		unsigned int s = 1; //FIXME

		std::cout << "Docking on" << std::endl;
		ROS_INFO("Docking on ");

		// Get docking service
		auto providers = robot_->getMiraAuthority().queryServicesForInterface("IDockingProcess");
		if(providers.empty()) 
		{
		    std::cout << "no providers for IDockingProcess" << std::endl;
		    ROS_INFO("no providers for IDockingProcess ");
		    return;
		}

		// Assume that our DockingProcess is the first and only available IDockingProcess provider.
		const std::string service = providers.front();

		// Let the robot dock on
		auto rpcFuture = robot_->getMiraAuthority().callService<void>(service, "dockOn", s);

		// Get the result, even though it is void. If an exception has occurred, this will throw as well.
		rpcFuture.get();
	}
	else if (msg->data.compare("docking_off") == 0)
	{
		unsigned int s = 1; //FIXME

		std::cout << "Docking off" << std::endl;
		ROS_INFO("Docking off ");

		// Get docking service
		auto providers = robot_->getMiraAuthority().queryServicesForInterface("IDockingProcess");
		if(providers.empty()) 
		{
		    std::cout << "no providers for IDockingProcess" << std::endl;
		    ROS_INFO("no providers for IDockingProcess ");
		    return;
		}

		// Assume that our DockingProcess is the first and only available IDockingProcess provider.
		const std::string service = providers.front();

		//docking_off
		auto rpcFuture = robot_->getMiraAuthority().callService<void>(service, "dockOff", s);

		rpcFuture.get();

	}

}

void MiraGoRecharge::executeCb(const interfaces_mira::MiraDockingGoalConstPtr& docking_action)
{

        //std_msgs::String task = docking_action->docking_task;
	int task = docking_action->docking_task;

	status_updated = false;
	template_found = false;

	//if (task.data.compare("docking_on") == 0)
	if (task == 0)
	{
		unsigned int s = 1; //FIXME

		std::cout << "Docking on received" << std::endl;
		ROS_INFO("Docking on received");

		mira::RPCFuture<void> r1 = robot_->getMiraAuthority().callService<void>("/robot/Robot#builtin", std::string("setProperty"), std::string("MainControlUnit.Force"), std::string("30"));
        	r1.timedWait(mira::Duration::seconds(1));
        	r1.get();

		// Get docking service
		auto providers = robot_->getMiraAuthority().queryServicesForInterface("IDockingProcess");
		if(providers.empty()) 
		{
		    std::cout << "no providers for IDockingProcess" << std::endl;
		    ROS_INFO("no providers for IDockingProcess ");
		    as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting, no providers");
		    return;
		}

		// Assume that our DockingProcess is the first and only available IDockingProcess provider.
		const std::string service = providers.front();

		// Let the robot dock on
		auto rpcFuture = robot_->getMiraAuthority().callService<void>(service, "dockOn", s);

		// Get the result, even though it is void. If an exception has occurred, this will throw as well.
		rpcFuture.get();


	}
	else if (task == 1)
	{
		unsigned int s = 1; //FIXME

		std::cout << "Docking off received" << std::endl;
		ROS_INFO("Docking off received");

		// Get docking service
		auto providers = robot_->getMiraAuthority().queryServicesForInterface("IDockingProcess");
		if(providers.empty()) 
		{
		    std::cout << "no providers for IDockingProcess" << std::endl;
		    ROS_INFO("no providers for IDockingProcess ");
		    return;
		}

		// Assume that our DockingProcess is the first and only available IDockingProcess provider.
		const std::string service = providers.front();

		//docking_off
		auto rpcFuture = robot_->getMiraAuthority().callService<void>(service, "dockOff", s);

		rpcFuture.get();

	}
	else
	{
	 	as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting because no proper task was received");
		return;
	}

	std::cout << "Docking action started " << std::endl;
 	ros::NodeHandle n = robot_->getRosNode();
    	while(n.ok())
    	{

              //std::cout << "actionlib server executeCb ok" << std::endl;
	      if(as_->isPreemptRequested())
	      {
			std::cout << "preempt requested" << std::endl;
			ROS_INFO("preempt requested ");
			template_found = false;
		
			TaskPtr task(new Task());
			//cancel the task

			if(as_->isNewGoalAvailable())
			{
			  //if we're active and a new docking task is available, we'll accept it, but this should not happen 
			  interfaces_mira::MiraDockingGoal new_task = *as_->acceptNewGoal();
			  std::cout << "new task available, THIS SHOULD NOT HAPPEN? " << new_task.docking_task << std::endl;

			  if(new_task.docking_task != 0 && new_task.docking_task != 1)
			  {
			    as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting because the new task is not valid");
			    return;
			  }


			  //we have a new task
			  if (new_task.docking_task == 0)
			  {
				unsigned int s = 1; //FIXME

				std::cout << "Docking on received" << std::endl;
				ROS_INFO("New docking on received ");

				mira::RPCFuture<void> r1 = robot_->getMiraAuthority().callService<void>("/robot/Robot#builtin", std::string("setProperty"), "MainControlUnit.Force", "30");
        			r1.timedWait(mira::Duration::seconds(1));
        			r1.get();


				// Get docking service
				auto providers = robot_->getMiraAuthority().queryServicesForInterface("IDockingProcess");
				if(providers.empty()) 
				{
				    std::cout << "no providers for IDockingProcess" << std::endl;
				    ROS_INFO("no providers for IDockingProcess ");
				    return;
				}

				// Assume that our DockingProcess is the first and only available IDockingProcess provider.
				const std::string service = providers.front();

				// Let the robot dock on
				auto rpcFuture = robot_->getMiraAuthority().callService<void>(service, "dockOn", s);

				// Get the result, even though it is void. If an exception has occurred, this will throw as well.
				rpcFuture.get();


			  }
			  else if (new_task.docking_task == 1)
			  {
				unsigned int s = 1; //FIXME

				std::cout << "Docking off" << std::endl;
				ROS_INFO("New docking off received ");

				// Get docking service
				auto providers = robot_->getMiraAuthority().queryServicesForInterface("IDockingProcess");
				if(providers.empty()) 
				{
				    std::cout << "no providers for IDockingProcess" << std::endl;
				    ROS_INFO("no providers for IDockingProcess ");
				    return;
				}

				// Assume that our DockingProcess is the first and only available IDockingProcess provider.
				const std::string service = providers.front();

				//docking_off
				auto rpcFuture = robot_->getMiraAuthority().callService<void>(service, "dockOff", s);

				rpcFuture.get();

			  }
			  
			  
	        	} // end of is new task available

			else 
			{

				//notify the ActionServer that we've successfully preempted
				ROS_DEBUG_NAMED("interfaces_mira","preempting the current docking task");
				as_->setPreempted();
				std::cout << "docking task preempted, preemt received" << std::endl;

				//we'll actually return from execute after preempting
				return;
			}


	  	} // end of preempt requested

		if (!status_updated) continue;

		interfaces_mira::MiraDockingFeedback feedback;
		feedback.feedback = status.data.c_str();
		as_->publishFeedback(feedback);

		if (status.data.compare("docked") == 0)
		{

			mira::RPCFuture<void> r2 = robot_->getMiraAuthority().callService<void>("/robot/Robot#builtin", std::string("setProperty"), std::string("MainControlUnit.Force"), std::string("60"));
        		r2.timedWait(mira::Duration::seconds(1));
        		r2.get();			
	
			std::cout << "task succeeded, template found and robot stoppped moving forwards " << std::endl;
			ROS_INFO("task succeeded, template found and robot stoppped moving forwards ");
			as_->setSucceeded(interfaces_mira::MiraDockingResult(), "Task succeeded, robot stoppped moving forwards");
			status_updated = false;
			return;
		}

		if (status.data.compare("undocked") == 0)
		{
			std::cout << "task succeeded, robot undocked " << std::endl;
			ROS_INFO("task succeeded, robot undocked ");
			as_->setSucceeded(interfaces_mira::MiraDockingResult(), "Task succeeded, robot stopped moving backwards");
			status_updated = false;
			return;
		}

		if (status.data.compare("template_found") == 0)
		{
			ros::Duration time_left = timeout - ros::Time::now();
			if (time_left <= ros::Duration(0,0))  //the timeout has been reached
			{
				std::cout << "task succeeded, template found but no feedback received" << std::endl;
				ROS_INFO("task succeeded, template found but no feedback received");
				as_->setSucceeded(interfaces_mira::MiraDockingResult(), "Task succeeded, template found but no feedback received");
				
			}	

		}

		if (status.data.compare("template_not_found") == 0)
		{

			mira::RPCFuture<void> r2 = robot_->getMiraAuthority().callService<void>("/robot/Robot#builtin", std::string("setProperty"), std::string("MainControlUnit.Force"), std::string("60"));
        		r2.timedWait(mira::Duration::seconds(1));
        		r2.get();

			//std::cout << "task preempted, template not found " << std::endl;
			//as_->setPreempted();
			std::cout << "task aborted, template not foundd " << std::endl;
			ROS_INFO("task aborted, template not found");
			as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting, template not found");
			status_updated = false;
			return;
		}  

		if (status.data.compare("failure") == 0)
		{
			mira::RPCFuture<void> r2 = robot_->getMiraAuthority().callService<void>("/robot/Robot#builtin", std::string("setProperty"), std::string("MainControlUnit.Force"), std::string("60"));
        		r2.timedWait(mira::Duration::seconds(1));
        		r2.get();

			if (template_found)
			{
				std::cout << "task succeeded, template found and robot stoppped moving forwards after failure" << std::endl;
				ROS_INFO("task succeeded, template found and robot stoppped moving forwards after failure");
				as_->setSucceeded(interfaces_mira::MiraDockingResult(), "Task succeeded, robot stoppped moving forwards after failure");

			}
			else
			{
				std::cout << "task aborted, failure " << std::endl;
				ROS_INFO("task aborted, failure");
				as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting because task failed");
			}

			status_updated = false;
			return;
		}
		
	} // end of while

	//if the node is killed then we'll abort and return
	as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting because the node is killed");



}

