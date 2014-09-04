#!/bin/bash
 
HOBBITDIR="/opt/ros/hobbit_hydro/"
SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES=4
MAX_RETRIES=10

#Script to start a screen session of web_interface

echo "Starting up web_interface"
screen -d -m -S "web_interface" /bin/bash -c "source ~/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch web_interface web_interface.launch"
 
sleep $SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES
 
#We already tried once !
i=2
ISWEBINTFUP=`rosservice list | grep /web_interface/terminate`
while [ -z "$ISWEBINTFUP" ]
do  
 echo "Web Interface node not started yet , try to wait for it $i/$MAX_RETRIES ( $ISWEBINTFUP )"
 screen -d -m -S "web_interface" /bin/bash -c "source ~/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch web_interface web_interface.launch"
 sleep $SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES
   
 ISWEBINTFUP=`rosservice list | grep /web_interface/terminate`
 
 ((i++))
 if [ $i -gt $MAX_RETRIES ]
 then 
  echo "Maximum retries reached ,complete failure starting Web Interface service , todo try to restart a device or something here"
  exit 1
 fi
done







exit 0



