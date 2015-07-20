
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

  batterySubs = robot_->getRosNode().subscribe<mira_msgs::BatteryState>("battery_state", 1, &MiraGoRecharge::battery_state_callback, this);

  status.data = "idle";

  as_ = new actionlib::SimpleActionServer<hobbit_msgs::MiraDockingAction>(robot_->getRosNode(), "docking_task", boost::bind(&MiraGoRecharge::executeCb, this, _1), false);
  as_->start();

  status_updated = false;
  is_charging = false;

}

MiraGoRecharge::~MiraGoRecharge()
{
  if(as_ != NULL)
  	delete as_;
 
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

void MiraGoRecharge::executeCb(const hobbit_msgs::MiraDockingGoalConstPtr& docking_action)
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

		mira::RPCFuture<void> r1 = robot_->getMiraAuthority().callService<void>("/robot/Robot#builtin", std::string("setProperty"), std::string("MainControlUnit.Force"), std::string("35"));
        	r1.timedWait(mira::Duration::seconds(1));
        	r1.get();

		// Get docking service
		auto providers = robot_->getMiraAuthority().queryServicesForInterface("IDockingProcess");
		if(providers.empty()) 
		{
		    std::cout << "no providers for IDockingProcess" << std::endl;
		    ROS_INFO("no providers for IDockingProcess ");
		    //as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting, no providers");
		    as_->setPreempted();
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
		    //as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting, no providers");
		    as_->setPreempted();
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
	 	//as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting because no proper task was received");
		as_->setPreempted();
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
		
			//cancel the task
			auto providers_c = robot_->getMiraAuthority().queryServicesForInterface("IDockingProcess");
			if(providers_c.empty()) 
			{
			    std::cout << "no providers for IDockingProcess to cancel the task" << std::endl;
			    ROS_INFO("no providers for IDockingProcess ");
			    //as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting, no providers");
			    as_->setPreempted();
			    return;
			}
			const std::string service_c = providers_c.front();
			auto rpcFuture_c = robot_->getMiraAuthority().callService<void>(service_c, "interrupt");
			std::cout << "interrupted " << std::endl;
			rpcFuture_c.get();

			if(as_->isNewGoalAvailable())
			{
			  //if we're active and a new docking task is available, we'll accept it, but this should not happen 
			  hobbit_msgs::MiraDockingGoal new_task = *as_->acceptNewGoal();
			  std::cout << "new task available, THIS SHOULD NOT HAPPEN? " << new_task.docking_task << std::endl;

			  if(new_task.docking_task != 0 && new_task.docking_task != 1)
			  {
			    //as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting because the new task is not valid");
			    as_->setPreempted();
			    return;
			  }


			  //we have a new task
			  if (new_task.docking_task == 0)
			  {
				unsigned int s = 1; //FIXME

				std::cout << "Docking on received" << std::endl;
				ROS_INFO("New docking on received ");

				mira::RPCFuture<void> r1 = robot_->getMiraAuthority().callService<void>("/robot/Robot#builtin", std::string("setProperty"), "MainControlUnit.Force", "35");
        			r1.timedWait(mira::Duration::seconds(1));
        			r1.get();


				// Get docking service
				auto providers = robot_->getMiraAuthority().queryServicesForInterface("IDockingProcess");
				if(providers.empty()) 
				{
				    std::cout << "no providers for IDockingProcess" << std::endl;
				    ROS_INFO("no providers for IDockingProcess ");
				    //as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting, no providers");
				    as_->setPreempted();
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
				    //as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting, no providers");
				    as_->setPreempted();
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
				//interfaces_mira::MiraDockingResult mira_dock_result;
				//mira_dock_result.res = "preempted";
				//as_->setPreempted(mira_dock_result, "Task preempted");
				as_->setPreempted();
				std::cout << "docking task preempted, preemt received" << std::endl;

				//we'll actually return from execute after preempting
				return;
			}


	  	} // end of preempt requested

		if (!status_updated) continue;

		hobbit_msgs::MiraDockingFeedback feedback;
		feedback.feedback = status.data.c_str();
		as_->publishFeedback(feedback);

		if (status.data.compare("docked") == 0)
		{

			mira::RPCFuture<void> r2 = robot_->getMiraAuthority().callService<void>("/robot/Robot#builtin", std::string("setProperty"), std::string("MainControlUnit.Force"), std::string("60"));
        		r2.timedWait(mira::Duration::seconds(1));
        		r2.get();			
	
			std::cout << "task succeeded, template found and robot stoppped moving forwards " << std::endl;
			ROS_INFO("task succeeded, template found and robot stoppped moving forwards ");
			as_->setSucceeded(hobbit_msgs::MiraDockingResult(), "Task succeeded, robot stoppped moving forwards");
			status_updated = false;
			return;
		}

		if (task == 0 && is_charging)
		{

			//cancel the task
			/*auto providers_ = robot_->getMiraAuthority().queryServicesForInterface("IDockingProcess");
			const std::string service_ = providers_.front();
			auto rpcFuture_ = robot_->getMiraAuthority().callService<void>(service_, "interrupt");
			std::cout << "interrupted " << std::endl;
			rpcFuture_.get();*/

			interrupt();

			mira::RPCFuture<void> r2 = robot_->getMiraAuthority().callService<void>("/robot/Robot#builtin", std::string("setProperty"), std::string("MainControlUnit.Force"), std::string("60"));
        		r2.timedWait(mira::Duration::seconds(1));
        		r2.get();			
	
			std::cout << "task succeeded, robot is charging " << std::endl;
			ROS_INFO("task succeeded, robot is charging ");
			as_->setSucceeded(hobbit_msgs::MiraDockingResult(), "Task succeeded, robot is charging");
			status_updated = false;
			return;

		}

		if (status.data.compare("undocked") == 0)
		{
			std::cout << "task succeeded, robot undocked " << std::endl;
			ROS_INFO("task succeeded, robot undocked ");
			as_->setSucceeded(hobbit_msgs::MiraDockingResult(), "Task succeeded, robot stopped moving backwards");
			status_updated = false;
			return;
		}

		if (status.data.compare("template_found") == 0)
		{
			ros::Duration time_left = timeout - ros::Time::now();
			if (time_left <= ros::Duration(0,0))  //the timeout has been reached
			{
				interrupt();

				mira::RPCFuture<void> r2 = robot_->getMiraAuthority().callService<void>("/robot/Robot#builtin", std::string("setProperty"), std::string("MainControlUnit.Force"), std::string("60"));
        			r2.timedWait(mira::Duration::seconds(1));
        			r2.get();

				std::cout << "task succeeded, template found but no feedback received" << std::endl;
				ROS_INFO("task succeeded, template found but no feedback received");
				as_->setSucceeded(hobbit_msgs::MiraDockingResult(), "Task succeeded, template found but no feedback received");
				status_updated = false;
				return;
				
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
			as_->setAborted(hobbit_msgs::MiraDockingResult(), "Aborting, template not found");
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
				as_->setSucceeded(hobbit_msgs::MiraDockingResult(), "Task succeeded, robot stoppped moving forwards after failure");

			}
			else
			{
				std::cout << "task aborted, failure " << std::endl;
				ROS_INFO("task aborted, failure");
				as_->setAborted(hobbit_msgs::MiraDockingResult(), "Aborting because task failed");
			}

			status_updated = false;
			return;
		}
		
	} // end of while

	//if the node is killed then we'll abort and return
	as_->setAborted(hobbit_msgs::MiraDockingResult(), "Aborting because the node is killed");



}

void MiraGoRecharge::interrupt()
{
	auto providers_c = robot_->getMiraAuthority().queryServicesForInterface("IDockingProcess");
	if(providers_c.empty()) 
	{
	    std::cout << "no providers for IDockingProcess to cancel the task" << std::endl;
	    ROS_INFO("no providers for IDockingProcess ");
	    //as_->setAborted(interfaces_mira::MiraDockingResult(), "Aborting, no providers");
	    as_->setPreempted();
	    return;
	}
	const std::string service_c = providers_c.front();
	auto rpcFuture_c = robot_->getMiraAuthority().callService<void>(service_c, "interrupt");
	std::cout << "interrupted " << std::endl;
	rpcFuture_c.get();
}


void MiraGoRecharge::battery_state_callback(const mira_msgs::BatteryState::ConstPtr& msg)
{
  is_charging = (*msg).charging;
    
}

