#!/bin/bash

sleep 5
echo "TODO : Lets try to see if everything is broadcasting"
rostopic list

ISITUP="grep"

while [ -n `screen -ls | grep rgbd_acquisition` ]
 echo "Trying to start-up rgbd_acquisition"
 screen -d -m -S "rgbd_acquisition" /bin/bash -c "source /home/hobbit/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch rgbd_acquisition startHobbitTopCamera.sh"
 sleep 10
done



echo "Starting up hand_gestures"
screen -d -m -S "hand_gestures" /bin/bash -c "source /home/hobbit/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch hand_gestures hand_gestures.launch"
echo "Starting up face_detection"
screen -d -m -S "face_detection" /bin/bash -c "source /home/hobbit/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch face_detection face_detection.launch"


exit 0
