#include "interfaces_mira/ModuleFactory.h"

#include "interfaces_mira/MiraVirtualLaser.h"
#include "interfaces_mira/MiraSendingGoals.h"
#include "interfaces_mira/MiraGetPose.h"
#include "interfaces_mira/MiraGetPath.h"
#include "interfaces_mira/MiraGoRecharge.h"

#include "interfaces_mira/MiraRobotDrive.h"
#include "interfaces_mira/MiraRobotCharger.h"
#include "interfaces_mira/MiraRobotDriveRos.h"

//based on code from the Strands project

ModuleFactory::ModuleFactory() {
        Register("VirtualLaser", &MiraVirtualLaser::Create);
	Register("SendingGoals", &MiraSendingGoals::Create);
        Register("MiraGetPose", &MiraGetPose::Create);
	Register("MiraGetPath", &MiraGetPath::Create);
	Register("MiraGoRecharge", &MiraGoRecharge::Create);
	Register("MiraRobotDrive", &MiraRobotDrive::Create);
	Register("MiraRobotCharger", &MiraRobotCharger::Create);
	Register("MiraRobotDriveRos", &MiraRobotDriveRos::Create);
}

void ModuleFactory::Register(const std::string &name, ModuleCreator func) {
        modules_[name] = func;
}

MiraRobotModule *ModuleFactory::CreateModule(std::string name, MiraRobot *robot) {
  std::map<std::string, ModuleCreator>::iterator it = modules_.find(name);
  if (it != modules_.end()) {
    MiraRobotModule *mod = it->second();
    mod->setRobot(robot);
    return mod;
  }
  ROS_ERROR("Trying to create unknown Mira robot module.");
  return NULL;
}

bool ModuleFactory::CheckForModule(const std::string &name) {
  std::map<std::string, ModuleCreator>::iterator it = modules_.find(name);
  if (it != modules_.end()) {
    return true;
  }
  return false;
}
