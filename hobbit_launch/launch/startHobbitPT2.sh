#!/bin/bash

#This script is called from ~/.config/autostart/hobbit.desktop 
#and ( for now ) brings up all the nodes / etc required for the hobbit robot to function

STARTDIR=`pwd`
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"
#cd to the directory where the script lies at

#env > ~/startMsg.txt

#Prohibit screen power down after 10mins
xset s off&
#Move mouse out of fov
xdotool mousemove 1024 1024&

#Display knight-rider for head :P
#animate /home/hobbit/Pictures/knight.gif&

#Make sure we have our stuff sourced
source /home/hobbit/.bashrc
source /opt/ros/hobbit_hydro/devel/setup.bash

roscore&
sleep 3
 
#Start up vision acquisition etc
/opt/ros/hobbit_hydro/src/rgbd_acquisition/scripts/startFORTHStuff.sh&
sleep 5


#Start Head ( and bring it to level )
roslaunch head startup.launch&
#Start Base
rosrun mobile_platform mobile_platform&
#Bring up web interface ( port 8080 )
/opt/ros/hobbit_hydro/src/web_interface/scripts/startWebInterface.sh&

#Give some time for the first camera to come online
sleep 5

#Start bottom camera
roslaunch openni2_launch openni2.launch camera:=basecam depth_registration:=true disparity_processing:=true disparity_registered_processing:=true &


#After everything is done , bring had to up position
rostopic pub -1 /HeadMove std_msgs/String 'up'&


cd $STARTDIR

exit 0

