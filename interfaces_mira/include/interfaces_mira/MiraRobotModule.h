/* MiraRobotModule.h
* Author: chris burbridge
  Modified by Paloma de la Puente
*
* Base class for Mira Robot modules (Head, EBC, Drive etc)
*/

#ifndef MIRAROBOTMODULE_H_
#define MIRAROBOTMODULE_H_

#include <ros/ros.h>
#include <fw/Framework.h>
#include <string>

class MiraRobot;

class MiraRobotModule {
public:
  MiraRobotModule(std::string name);
        void setRobot(MiraRobot *robot) { robot_ = robot; };

        virtual void initialize() = 0;

        virtual ~MiraRobotModule();

protected:
        bool set_mira_param_(std::string param_name, std::string value);
        std::string get_mira_param_(std::string param_name);
  MiraRobot *robot_;
  std::string name_;
  ros::NodeHandle module_handle_;

};

typedef MiraRobotModule* (*ModuleCreator)(void);

#endif /* MIRAROBOTMODULE_H_ */
