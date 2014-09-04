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
env >> ~/.startup.tx

#Make sure we have our stuff sourced
source ~/.bashrc
source /opt/ros/hobbit_hydro/devel/setup.bash

roscore&

#ros core takes ages to start
sleep $DELAY_BETWEEN_STEPS
sleep $DELAY_BETWEEN_STEPS

#Bring up web interface first , so that we can see what is happening ( port 8080 )
#Wtf , it might not be executable ?
chmod +x /opt/ros/hobbit_hydro/src/web_interface/scripts/startWebInterface.sh
/opt/ros/hobbit_hydro/src/web_interface/scripts/startWebInterface.sh&


/opt/ros/hobbit_hydro/src/rgbd_acquisition/scripts/workAroundUSB.sh
sleep $DELAY_BETWEEN_STEPS

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


#Start Head ( and bring it to level )
/opt/ros/hobbit_hydro/src/hobbit_launch/launch/startupBlueDanubeHead.sh&


# Start the virtual laser
cd /opt/ros/hobbit_hydro/src/virtual_laser/launch
roslaunch startup.launch&

sleep $DELAY_BETWEEN_STEPS

# Start AAL service
roslaunch aal_service startup.launch&

# Load Hobbit PT2 parameters
cd /opt/ros/hobbit_hydro/src/ #, web interface will save to this file also when somene clicks Set
rosparam load /opt/ros/hobbit_hydro/src/hobbit_params.yaml&

# Start SMACH handling of rooms, places, objects.
rosrun hobbit_smach places_objects.py&
roslaunch hobbit_smach SavePCD.launch&
rosrun hobbit_smach learn_object.py&
rosrun hobbit_smach away.py&
rosrun hobbit_smach goto.py&
rosrun hobbit_smach follow_me&
rosrun hobbit_smach emergency_user.py&
rosrun hobbit_smach emergency_bathroom.py&
rosrun hobbit_smach battery_monitor.py&
rosrun hobbit_smach recharge&
rosrun hobbit_smach sos_monitor.py&

#sleep $DELAY_BETWEEN_STEPS
rosrun hobbit_smach master.py&

# Start table/floor object detector (for clustering) and trigger for publishing single shot point clouds from headcam
cd /opt/ros/hobbit_hydro/src/table_object_detector/launch
roslaunch startup.launch&

#TODO : REMOVE ALL THESE ..
     # Start interfaces_mira, which starts the platform driver and the ros-mira interface for virtual lasers
     #cd /opt/ros/hobbit_hydro/src/interfaces_mira/launch
     #roslaunch startup.launch&

     #sleep $DELAY_BETWEEN_STEPS
     # Start mira (start mira center with: e.g. 'miracenter mira_vis_config.xml')
     #cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
     #mira mira_config.xml -p1234&
     #sleep $DELAY_BETWEEN_STEPS
     # start new mira visualization
     #cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
     #miracenter mira_vis_config.xml&


# Start the virtual laser from the headcam for obstacle avoidance
cd /opt/ros/hobbit_hydro/src/top_scan_points/launch
roslaunch startup.launch&

sleep $DELAY_BETWEEN_STEPS
#Startup Joystick node , ( it allows webinterface joystick emulation also )
roslaunch joy2twist startup.launch&

sleep $DELAY_BETWEEN_STEPS
#Switch Mira stuff to navigation mode ( default )
cd /opt/ros/hobbit_hydro/src/hobbit_launch/launch
./switchMira.sh navigation

cd $STARTDIR

exit 0

