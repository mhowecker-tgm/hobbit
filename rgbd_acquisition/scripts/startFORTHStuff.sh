#!/bin/bash

echo "Starting up rgbd_acquisition"
screen -d -m -S "rgbd_acquisition" /bin/bash -c "source /home/hobbit/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch rgbd_acquisition startHobbitTopCamera.sh"
echo "Starting up hand_gestures"
screen -d -m -S "hand_gestures" /bin/bash -c "source /home/hobbit/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch hand_gestures hand_gestures.launch"
echo "Starting up face_detection"
screen -d -m -S "face_detection" /bin/bash -c "source /home/hobbit/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch face_detection face_detection.launch"


exit 0
