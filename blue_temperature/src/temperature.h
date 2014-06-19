#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <iostream>
#include <cmath>
#include <iomanip>
#include <ros/ros.h>
#include <bcm2835.h>
#include "blue_temperature/Temperature.h"

char buf[6];
long i;
char reg;
double tempobj=0, tempair=0, calcobj=0, calcair=0;
ros::Time starttime;
double elapsedtime=0;

bool measureTemperature(blue_temperature::Temperature::Request &req, blue_temperature::Temperature::Response &res);

#endif
