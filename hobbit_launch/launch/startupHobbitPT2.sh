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
#env > ~/startMsg.txt

#exit 0
  
#Make sure we have our stuff sourced
source /home/hobbit/.bashrc
source /opt/ros/hobbit_hydro/devel/setup.bash

roscore&
sleep $DELAY_BETWEEN_STEPS
  
#Start bottom camera first so that we are as sure as we can that it will work!
#cd /opt/ros/hobbit_hydro/src/hobbit_launch 
roslaunch openni2_launch openni2.launch camera:=basecam depth_registration:=true device_id:="#2"&
#echo "Trying to bring OpenNI2 Camera up with a first try"
#screen -d -m -S "basecam" /bin/bash -c "source ~/.bashrc && source $HOBBITDIR/devel/setup.bash && roslaunch openni2_launch openni2.launch camera:=basecam depth_registration:=true device_id:=\"#2\""

sleep $DELAY_BETWEEN_STEPS
#maybe prompt for failure here!



#Start up vision acquisition etc
/opt/ros/hobbit_hydro/src/rgbd_acquisition/scripts/startFORTHStuffPT2.sh&
#In case someone would like to startup the top camera without our integrated node he could do it like following
#roslaunch openni2_launch openni2.launch camera:=headcam depth_registration:=true device_id:="#1"
sleep $DELAY_BETWEEN_STEPS


#Start Head ( and bring it to level )
#roslaunch head startup.launch&
#Start Base
#rosrun mobile_platform mobile_platform&
#Bring up web interface ( port 8080 )
/opt/ros/hobbit_hydro/src/web_interface/scripts/startWebInterface.sh&

#Give some time for the first camera to come online
sleep $DELAY_BETWEEN_STEPS
  


# Start the virtual laser
cd /opt/ros/hobbit_hydro/src/virtual_laser/launch
roslaunch startup.launch&

sleep $DELAY_BETWEEN_STEPS

# Start interfaces_mira, which starts the platform driver and the ros-mira interface for virtual lasers
cd /opt/ros/hobbit_hydro/src/interfaces_mira/launch
roslaunch startup.launch&

sleep $DELAY_BETWEEN_STEPS

# Start mira center
cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
miracenter mira_config.xml


#After everything is done , bring had to up position
#rostopic pub -1 /HeadMove std_msgs/String 'up'&


cd $STARTDIR

exit 0

