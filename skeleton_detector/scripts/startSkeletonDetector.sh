#!/bin/bash
 
HOBBITDIR="/opt/ros/hobbit_hydro/"
SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES=10
MAX_RETRIES=10

#Script to start a screen session of web_interface

echo "Starting up skeleton_detector"
screen -d -m -S "skeleton_detector" /bin/bash -c "source ~/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch skeleton_detector skeleton_detector.launch"
 

exit 0


sleep $SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES
 
#We already tried once !
i=2
ISWEBINTFUP=`rosservice list | grep /skeleton_detector/terminate`
while [ -z "$ISWEBINTFUP" ]
do  
 echo "Web Interface node not started yet , try to wait for it $i/$MAX_RETRIES ( $ISWEBINTFUP )"
 screen -d -m -S "skeleton_detector" /bin/bash -c "source ~/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch skeleton_detector skeleton_detector.launch"
 sleep $SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES
   
 ISWEBINTFUP=`rosservice list | grep /skeleton_detector/terminate`
 
 ((i++))
 if [ $i -gt $MAX_RETRIES ]
 then 
  echo "Maximum retries reached ,complete failure starting Web Interface service , todo try to restart a device or something here"
  exit 1
 fi
done







exit 0



