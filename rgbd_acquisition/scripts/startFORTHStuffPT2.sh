#!/bin/bash

#Quick Settings
HOBBITDIR="/opt/ros/hobbit_hydro/"
SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES=15
MAX_RETRIES=10
 
 
#Ammar's installation at FORTH may override hobbit dir
if [ -d $HOBBITDIR ] 
then
 echo "Hobbit Normal root dir"
elif [ -d /home/ammar/Documents/Programming/FORTH/Hobbit/hobbit_hydro ] 
then
 HOBBITDIR="/home/ammar/Documents/Programming/FORTH/Hobbit/hobbit_hydro/"
fi

echo "Will Use $HOBBITDIR for root"
echo "NewLog" > ~/debug.txt 
#Try to bring up our node for the first time!
echo "Trying to bring RGBD Node up with a first try"
screen -d -m -S "rgbd_acquisition" /bin/bash -c "source ~/.bashrc && source $HOBBITDIR/devel/setup.bash && roslaunch rgbd_acquisition startHobbitTopCameraPT2.sh"
sleep $SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES


#We already tried once !
i=2
ISRGBDUP=`rostopic list | grep /headcam/rgb/image_rect_color/compressed`
while [ -z "$ISRGBDUP" ]
do 
 screen -S "rgbd_acquisition" -X quit
 echo "RGBD node not started yet $i/$MAX_RETRIES ( $ISRGBDUP )"
 sleep $SLEEP_TIME_IN_SECONDS_BETWEEN_RETRIES
 
 #Do Startup Here
  screen -d -m -S "rgbd_acquisition" /bin/bash -c "source ~/.bashrc && source $HOBBITDIR/devel/setup.bash && roslaunch rgbd_acquisition startHobbitTopCameraPT2.sh"
 #Do Startup Here
 
 ISRGBDUP=`rostopic list | grep /headcam/rgb/image_rect_color/compressed`
 ((i++))
 if [ $i -gt $MAX_RETRIES ]
 then 
  echo "Maximum retries reached ,complete failure starting service , todo try to restart a device or something here"
  exit 1
 fi
done

echo "Top Cam - Started succesfully"

 

echo "Starting up hand_gestures"
screen -d -m -S "hand_gestures" /bin/bash -c "source ~/.bashrc && source $HOBBITDIR/devel/setup.bash && roslaunch hand_gestures hand_gestures.launch"
echo "Starting up face_detection"
screen -d -m -S "face_detection" /bin/bash -c "source ~/.bashrc && source $HOBBITDIR/devel/setup.bash && roslaunch face_detection face_detection.launch"
echo "Starting up emergency_detection"
screen -d -m -S "emergency_detector" /bin/bash -c "source ~/.bashrc && source $HOBBITDIR/devel/setup.bash && roslaunch emergency_detector emergency_detector.launch"

echo "Starting up Skeleton Detector"
screen -d -m -S "skeleton_detector" /bin/bash -c "source ~/.bashrc && source $HOBBITDIR/devel/setup.bash && roslaunch skeleton_detector skeleton_detector.launch"
  
echo "Starting up Fitness Function Coordinator"
screen -d -m -S "fitness_coordinator" /bin/bash -c "source ~/.bashrc && source $HOBBITDIR/devel/setup.bash && roslaunch fitness_coordinator fitness_coordinator.launch"
 
echo "Starting up Person Aggregator"
screen -d -m -S "person_aggregator" /bin/bash -c "source ~/.bashrc && source $HOBBITDIR/devel/setup.bash && roslaunch person_aggregator person_aggregator.launch"


exit 0
