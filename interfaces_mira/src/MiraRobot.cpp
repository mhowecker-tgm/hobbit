#include <interfaces_mira/MiraRobot.h>

#include <interfaces_mira/ModuleFactory.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>

//Based on code from the Strands project

MiraRobot::MiraRobot(std::vector<std::string> modules) : authority_("/", "interfaces_ros", mira::Authority::INDEPENDENT_SUBSCRIBER_THREAD),node_() {
    ROS_INFO("Creating MiraRobot instance.");

    ModuleFactory *factory = ModuleFactory::Get();
    for (std::vector<std::string>::iterator i = modules.begin(); i!=modules.end(); i++) {
      //ROS_INFO_STREAM("Loading module " << *i);
      if (!factory->CheckForModule(*i)) {
        ROS_ERROR_STREAM("A non existent module was trying to be created. Name=" << *i<<"\n will try to continue without!");
      } else {
        modules_.push_back( factory->CreateModule(*i, this) );
      }
    }

    initialize();
}

void MiraRobot::initialize() {
  // Initialise all of the modules
    for(std::vector<MiraRobotModule*>::iterator it = modules_.begin(); it != modules_.end(); ++it) {
            (*it)->initialize();
    }
}

void MiraRobot::spin() {
  // ros::spin();
  ros::Rate r(5);
  while (ros::ok()) {
          ros::spinOnce();
        for (std::vector< boost::function<void ()> >::iterator i = spin_functions_.begin(); i!=spin_functions_.end(); i++){
         (*i)();
        }
        r.sleep();
  }
}

MiraRobot::~MiraRobot() {
    for(std::vector<MiraRobotModule*>::iterator it = modules_.begin(); it != modules_.end(); ++it) {
         delete (*it);
    }
}

mira::Authority& MiraRobot::getMiraAuthority() {
        return authority_;
}

ros::NodeHandle& MiraRobot::getRosNode() {
        return node_;
}

ros::CallbackQueue& MiraRobot::getMyQueue() {
        return my_queue;
}

tf::TransformBroadcaster& MiraRobot::getTFBroadcaster() {
        return tf_broadcaster_;
}

void MiraRobot::registerSpinFunction(boost::function<void ()> function) {
    spin_functions_.push_back(function);
}
