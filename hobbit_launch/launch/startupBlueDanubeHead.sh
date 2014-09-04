#!/bin/bash 

HOBBITDIR="/opt/ros/hobbit_hydro/"
screen -d -m -S "head" /bin/bash -c "source ~/.bashrc && source $HOBBITDIR/devel/setup.bash && nc6 192.168.2.199 8080"
 
#Useful Commands :
 
#Before anything else ( and after the robot is started one must do )
#rostopic pub /head/cmd std_msgs/String "startup" -1

#To Move the head you can use
#rostopic pub /head/move std_msgs/String "center_right" -1
#it can be moved to down_center up center down left center right to_turntable to_grasp

#To restart the servos you can do
#rostopic pub /head/cmd std_msgs/String "restart" -1

#To change the emotion you can do 
#rostopic pub /head/emo std_msgs/String "HAPPY" -1
#Emotions are HAPPY VHAPPY LTIRED VTIRED CONCERNED SAD WONDERING NEUTRAL SLEEPING


exit 0 
