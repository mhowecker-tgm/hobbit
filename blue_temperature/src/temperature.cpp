#include "temperature.h"

using namespace std;

int main(int argc, char **argv){
    
    ros::init(argc, argv, "blue_temperature");
    ros::NodeHandle n;
    
    if(!bcm2835_init()){
	ROS_INFO("failed to start blue_temperature.");
	return -1;
    }else{
	ROS_INFO("blue_temperature initialised.");
    }

    bcm2835_i2c_begin();
    bcm2835_i2c_set_baudrate(25000);
    bcm2835_i2c_setSlaveAddress(0x5a);
    bcm2835_i2c_end();

    ros::ServiceServer temperatureService = n.advertiseService("blue_temperature", measureTemperature);
    ros::spin();

    bcm2835_close();
    return 0;
}

bool measureTemperature(blue_temperature::Temperature::Request &req, blue_temperature::Temperature::Response &res){
  starttime = ros::Time::now();
  calcobj=0;
  calcair=0;
  bcm2835_i2c_begin();
  for(i=1;i<=req.average;i++){
    reg=7;
    tempobj = 999;
    while(tempobj < -40 || tempobj > 125){
	bcm2835_i2c_read_register_rs(&reg,&buf[0],3);
	tempobj = (double) (((buf[1]) << 8) + buf[0]);
	tempobj = (tempobj * 0.02)-0.01;
	tempobj = tempobj - 273.15;
    }
    calcobj += tempobj;

    reg = 6;
    tempair = 999;
    while(tempair < -40 || tempair > 125){
	bcm2835_i2c_read_register_rs(&reg,&buf[0],3);
	tempair = (double) (((buf[1]) << 8) + buf[0]);
	tempair = (tempair * 0.02)-0.01;
	tempair = tempair - 273.15;
    }
    calcair += tempair;
    elapsedtime = (ros::Time::now() - starttime).toSec();
    ROS_INFO("temperature ambient: %.2lf, object: %.2lf, sample %ld of %ld, elapsed time: %.2lf", tempair, tempobj, i, req.average, elapsedtime);
  }
  bcm2835_i2c_end();
  res.object = calcobj/req.average;
  res.air = calcair/req.average;
  res.elapsedtime = (ros::Time::now() - starttime).toSec();
  ROS_INFO("temperature ambient: %.2lf, object: %.2lf, averaged over %ld samples, elapsed time: %.2lf", res.air, res.object, req.average, res.elapsedtime);
  return true;
}
