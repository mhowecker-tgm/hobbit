
/* MiraRobotCharger.h
* Author: chris burbridge
*
* Module for interfacing all the charger topics, service and params.
* See http://www.mira-project.org/MIRA-doc/domains/robot/SCITOS/index.html#ChargerHG2_Section
*/

//Modified by Paloma de la Puente

#ifndef MIRAROBOTCHARGER_H_
#define MIRAROBOTCHARGER_H_

#include <interfaces_mira/MiraRobotModule.h>

//
//#include <fw/Framework.h>
#include <robot/BatteryState.h>
#include <dynamic_reconfigure/server.h>
//#include <interfaces_mira/ChargerParametersConfig.h>

class MiraRobotCharger: public MiraRobotModule {
public:
static MiraRobotModule* Create() {
return new MiraRobotCharger();
}

void initialize();

void battery_data_callback(mira::ChannelRead<mira::robot::BatteryState> data);
void charger_status_callback(mira::ChannelRead<uint8> data);
//void reconfigure_callback(scitos_mira::ChargerParametersConfig &config, uint32_t level);
private:

MiraRobotCharger();
ros::Publisher battery_pub_;
ros::Publisher charger_pub_;
//dynamic_reconfigure::Server<scitos_mira::ChargerParametersConfig> reconfigure_srv_;
};

#endif /* MiraRobotCharger_H_ */
