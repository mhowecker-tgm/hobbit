#!/bin/bash

#This script is called from ~/.config/autostart/hobbit.desktop
#to make it autostart you can just
#mkdir ~/.config/autostart
#ln -s /opt/ros/hobbit_hydro/src/hobbit_launch/launch/hobbitPT2.desktop ~/.config/autostart/hobbitPT2.desktop
#and ( for now ) brings up all the nodes / etc required for the hobbit robot to function

DELAY_BETWEEN_STEPS="5"

STARTDIR=`pwd`
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"
#cd to the directory where the script lies at

#Make sure we have our stuff sourced
source ~/.bashrc
source /opt/ros/hobbit_hydro/devel/setup.bash

roscore&

#ros core takes ages to start
sleep $DELAY_BETWEEN_STEPS
sleep $DELAY_BETWEEN_STEPS
sleep $DELAY_BETWEEN_STEPS

#/opt/ros/hobbit_hydro/src/rgbd_acquisition/scripts/workAroundUSB.sh
#set HobbitID and Robot-specific parameters
echo "This hobbit is $hobbit_id"
rosparam set hobbit_id "$hobbit_id"
rm /opt/ros/hobbit_hydro/src/configuration/active.yaml
ln -s /opt/ros/hobbit_hydro/src/configuration/$hobbit_id.yaml  /opt/ros/hobbit_hydro/src/configuration/active.yaml
rosparam load /opt/ros/hobbit_hydro/src/configuration/active.yaml


# Start AAL service
ROBOTNUMBER=$( printf '%d' "'${hobbit_id:3:1}" ); let "ROBOTNUMBER -= 96"; echo "copy params$ROBOTNUMBER for PT2$ROBOTNAME"
cd /opt/ros/hobbit_hydro/src/aal_service/launch
cp params"$ROBOTNUMBER".yaml params.yaml
roslaunch aal_service startup.launch&

#Trying to start basecam
#/opt/ros/hobbit_hydro/src/rgbd_acquisition/scripts/startBaseCameraPT2.sh
#If someone would like to start it on his own he could use the following
#roslaunch openni2_launch openni2.launch camera:=basecam depth_registration:=true device_id:="#2"&


#Bring up web interface after we got some things on , so that we can see what is happening ( port 8080 )
#Wtf , it might not be executable ?
sleep $DELAY_BETWEEN_STEPS
chmod +x /opt/ros/hobbit_hydro/src/web_interface/scripts/startWebInterface.sh
/opt/ros/hobbit_hydro/src/web_interface/scripts/startWebInterface.sh&

sleep $DELAY_BETWEEN_STEPS
#Start Head ( and bring it to level )
/opt/ros/hobbit_hydro/src/hobbit_launch/launch/startupBlueDanubeHead.sh&

sleep $DELAY_BETWEEN_STEPS
sleep $DELAY_BETWEEN_STEPS
sleep $DELAY_BETWEEN_STEPS

roslaunch blue_owlpose startup.launch&

sleep $DELAY_BETWEEN_STEPS
#Startup Joystick node , ( it allows webinterface joystick emulation also )
roslaunch joy2twist startup.launch&

sleep $DELAY_BETWEEN_STEPS
#rosrun dynamic_reconfigure dynparam set /basecam/driver data_skip 1&

#Switch Mira stuff to navigation mode ( default )
cd /opt/ros/hobbit_hydro/src/hobbit_launch/launch
./switchMira.sh navigation&

cd $STARTDIR

exit 0

