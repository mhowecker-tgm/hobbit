#!/bin/bash
 
HOBBITDIR="/opt/ros/hobbit_hydro/" 

echo "Starting up skeleton_detector"
screen -d -m -S "skeleton_detector" /bin/bash -c "source ~/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch skeleton_detector skeleton_detector.launch"
 
  
exit 0 
