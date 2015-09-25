#!/bin/bash
 
HOBBITDIR="/opt/ros/hobbit_hydro/"
SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES=10
MAX_RETRIES=10

#Script to start a screen session of face_detection

echo "Starting up face_detection"
screen -d -m -S "face_detection" /bin/bash -c "source ~/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch face_detection face_detection.launch"
 

exit 0


sleep $SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES
 
#We already tried once !
i=2
ISWEBINTFUP=`rosservice list | grep /face_detection/terminate`
while [ -z "$ISWEBINTFUP" ]
do  
 echo "Face Detector node not started yet , try to wait for it $i/$MAX_RETRIES ( $ISWEBINTFUP )"
 screen -d -m -S "face_detection" /bin/bash -c "source ~/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch face_detection face_detection.launch"
 sleep $SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES
   
 ISWEBINTFUP=`rosservice list | grep /face_detection/terminate`
 
 ((i++))
 if [ $i -gt $MAX_RETRIES ]
 then 
  echo "Maximum retries reached ,complete failure starting Web Interface service , todo try to restart a device or something here"
  exit 1
 fi
done







exit 0



