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
 
THEDATETAG=`date +"%y-%m-%d_%H-%M-%S"`
echo $THEDATETAG > ~/.startup.txt
env >> ~/.startup.txt  
  
#Make sure we have our stuff sourced
source ~/.bashrc
source /opt/ros/hobbit_hydro/devel/setup.bash

roscore&

#ros core takes ages to start
sleep $DELAY_BETWEEN_STEPS
sleep $DELAY_BETWEEN_STEPS


/opt/ros/hobbit_hydro/src/rgbd_acquisition/scripts/workAroundUSB.sh


#Trying to start basecam
/opt/ros/hobbit_hydro/src/rgbd_acquisition/scripts/startBaseCameraPT2.sh
#If someone would like to start it on his own he could use the following
#roslaunch openni2_launch openni2.launch camera:=basecam depth_registration:=true device_id:="#2"&
sleep $DELAY_BETWEEN_STEPS


#Start up vision acquisition etc
/opt/ros/hobbit_hydro/src/rgbd_acquisition/scripts/startFORTHStuffPT2.sh
#In case someone would like to startup the top camera without our integrated node he could do it like following
#roslaunch openni2_launch openni2.launch camera:=headcam depth_registration:=true device_id:="#1"&
sleep $DELAY_BETWEEN_STEPS

#rosrun mobile_platform mobile_platform&
#Bring up web interface ( port 8080 )
/opt/ros/hobbit_hydro/src/web_interface/scripts/startWebInterface.sh&

#Give some time for the first camera to come online
sleep $DELAY_BETWEEN_STEPS
  
#Start Head ( and bring it to level )
/opt/ros/hobbit_hydro/src/hobbit_launch/launch/startupBlueDanubeHead.sh&
#roslaunch head startup.launch&
#Start Base

# Start the virtual laser
cd /opt/ros/hobbit_hydro/src/virtual_laser/launch
roslaunch startup.launch&

sleep $DELAY_BETWEEN_STEPS

# Start AAL service
roslaunch aal_service startup.launch&

# Load Hobbit PT2 parameters
#cd /opt/ros/hobbit_hydro/src/
rosparam load /opt/ros/hobbit_hydro/src/hobbit_params.yaml&

# Start SMACH handling of rooms, places, objects.
rosrun hobbit_smach places_objects.py&
roslaunch hobbit_smach SavePCD.launch&
# Start SMACH handling for the demo on 14,15th July 2014
# Start SMACH handling for the demo on 19th June 2014
rosrun hobbit_smach learn_object.py&
rosrun hobbit_smach away.py&
rosrun hobbit_smach goto.py&
rosrun hobbit_smach emergency_user.py&
rosrun hobbit_smach battery_monitor.py&
rosrun hobbit_smach recharge&
rosrun hobbit_smach sos_monitor.py&

# Start places_interpretation
cd /opt/ros/hobbit_hydro/src/places_interpretation/launch
roslaunch startup.launch&
# Start get room stuff needed for mmui
# bajo: same functionality is provided from places_objects and no other node 
# seems to use it. 
# Disabled unless another node needs it.
# cd /opt/ros/hobbit_hydro/src/get_current_room/launch
# roslaunch startup.launch&

# Start table/floor object detector (for clustering) and trigger for publishing single shot point clouds from headcam
cd /opt/ros/hobbit_hydro/src/table_object_detector/launch
roslaunch startup.launch&

# Start interfaces_mira, which starts the platform driver and the ros-mira interface for virtual lasers
cd /opt/ros/hobbit_hydro/src/interfaces_mira/launch
roslaunch startup.launch&

sleep $DELAY_BETWEEN_STEPS

# Start mira (start mira center with: miracenter -k ipaddress:1234', e.g. 'miracenter -k 127.0.0.1:1234')
cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
#mira mira_config.xml -p1234&
miracenter mira_config.xml

#Startup Joystick node , ( it allows webinterface joystick emulation also )
roslaunch joy2twist startup.launch& 


cd $STARTDIR

exit 0

