#include <interfaces_mira/MiraRobotModule.h>
#include <string>
#include <interfaces_mira/MiraRobot.h>
#include <rpc/RPCError.h>

//Based on code from the Strands project

MiraRobotModule::MiraRobotModule(std::string name) : name_(name), module_handle_(name_) {
  ROS_INFO("Initialising %s module..",name_.c_str());
}



MiraRobotModule::~MiraRobotModule() {

}


bool MiraRobotModule::set_mira_param_(std::string param_name, std::string value) {
          mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot#builtin", std::string("setProperty"), param_name, value);
        r.timedWait(mira::Duration::seconds(1)); // timeout after a second of trying...
        try {
         r.get();
        } catch (mira::XRPC& e){
         ROS_WARN("Mira RPC error caught when setting parameter: ", e.what() );
         return false;
        }
        return true;
}

std::string MiraRobotModule::get_mira_param_(std::string param_name) {
  mira::RPCFuture<std::string> r = robot_->getMiraAuthority().callService<std::string>("/robot/Robot#builtin", std::string("getProperty"), param_name);
        r.timedWait(mira::Duration::seconds(1)); // timeout after a second of trying...
        return r.get(); // throw excepton if no good...
}
