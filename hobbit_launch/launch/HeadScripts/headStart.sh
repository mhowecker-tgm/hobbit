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

sudo /etc/init.d/chrony stop #starts chrony df 10.7.2014 [sudo needed missing?]
sudo ntpdate 192.168.2.122 # synchronizes time with xpc
sudo /etc/init.d/chrony start #starts chrony df 10.7.2014 [sudo needed missing?]

echo "Starting blue_eyes"
#screen -d -m -S "blue_eyes" /bin/bash -c "source /home/pi/.bashrc && source /home/pi/ros/setup.bash && rosrun blue_eyes blue_eyes.py"
rosrun blue_eyes blue_eyes.py&

echo "Starting blue_pose"
#screen -d -m -S "blue_pose" /bin/bash -c "source /home/pi/.bashrc && source /home/pi/ros/setup.bash && rosrun blue_owlpose owlpose.py"
rosrun blue_owlpose owlpose.py&

MAX_RETRIES=20
SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES=1
i=1

ISHEADUP=`rostopic list | grep /head/cmd`
while [ -z "$ISHEADUP" ]
do
 echo "Head Motor node not started yet , try to wait for it $i/$MAX_RETRIES ( $ISHEADUP )"
 sleep $SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES
 ISHEADUP=`rostopic list | grep /head/cmd`
 ((i++))
 if [ $i -gt $MAX_RETRIES ]
 then 
  echo "Maximum retries reached ,complete failure starting service , todo try to restart a device or something here"
  exit 1
 fi
done

rostopic pub /head/cmd std_msgs/String "startup" -1

echo "Starting blue_temperature"
#screen -d -m -S "blue_temperature" /bin/bash -c "source /home/pi/.bashrc && source /home/pi/ros/setup.bash && rosrun blue_temperature blue_temperature"

# Blue temperature does not work on hobbitpt2c 
#rosrun blue_temperature blue_temperature&


echo "Waiting so that we still get output on the screen session of xpc"
sleep 60
echo "still waiting"
sleep 60
echo "ok we will now drop the connection"


exit 0

