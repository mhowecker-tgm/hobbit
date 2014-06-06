#!/bin/bash
echo "Startup script is beeing invoked"


echo "headStart called again @ "
THEDATETAG=`date +"%y-%m-%d_%H-%M-%S"`
echo  $THEDATETAG

echo "Sourcing bash rc"
source /home/pi/.bashrc
echo "Sourcing ROS bash environment"
source /home/pi/ros/setup.bash

export ROS_PACKAGE_PATH=/opt/ros/fuerte:/opt/ros/fuerte/share/ros:/home/pi/ros:/home/pi/workspace
export ROS_MASTER_URI=http://192.168.2.122:11311/
export ROS_IP=192.168.2.199
export ROS_WORKSPACE=/home/pi/workspace
export ROS_HOSTNAME=192.168.2.199



echo "ROS MASTER IS $ROS_MASTER_URI"
echo "ROS IP IS $ROS_IP"
echo "ROS WORKSPACE IS $ROS_WORKSPACE"
echo "ROS HOSTNAME IS $ROS_HOSTNAME"




env 
echo "We are user" 
whoami



echo "Starting blue_eyes"
#screen -d -m -S "blue_eyes" /bin/bash -c "source /home/pi/.bashrc && source /home/pi/ros/setup.bash && rosrun blue_eyes blue_eyes.py"
rosrun blue_eyes blue_eyes.py&

echo "Starting blue_pose"
#screen -d -m -S "blue_pose" /bin/bash -c "source /home/pi/.bashrc && source /home/pi/ros/setup.bash && rosrun blue_owlpose owlpose.py"
rosrun blue_owlpose owlpose.py&


sleep 2 
rostopic pub /head/cmd std_msgs/String "startup" -1&


#sudo -i
#rosrun blue_temperature blue_temperature&

echo "Starting blue_temperature"
#screen -d -m -S "blue_temperature" /bin/bash -c "source /home/pi/.bashrc && source /home/pi/ros/setup.bash && rosrun blue_temperature blue_temperature"
rosrun blue_temperature blue_temperature&


echo "Waiting so that we still get output on the screen session of xpc"
sleep 40
echo "still waiting"
sleep 40
echo "ok we will now drop the connection"


exit 0

