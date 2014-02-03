#!/bin/bash

FRAMES_TO_PROCESS="10000"
KILLSWITCH_ENGAGED=0 


#INPUT_FROM_PICTURES="Simulated"
 
#killall RGBDAcquisition 

echo "Starting Node"
./run_it.sh&
sleep 1
echo "Relaying Setup"



rosservice list

if [ -z $INPUT_FROM_PICTURES ]
then
 echo "Regular Input From OpenNI"
 rosservice call setAcquisition OPENNI2 "dummy" 640 480 30 0 1 
else
 echo "Using directory : $INPUT_FROM_PICTURES for input "
 rosservice call setAcquisition TEMPLATE $INPUT_FROM_PICTURES 640 480 30 0 1 
fi 

     

sleep 1
echo "Starting Experiment"
 
rosservice call startDrawing


if [ $KILLSWITCH_ENGAGED -gt 0 ]
then
 sleep 30
 echo "Killing Experiment"
 rosservice call stopDrawing


 sleep 3
 echo "Killing Experiment"
killall RGBDAcquisition 
 echo "Killed everything"
fi

exit 0
