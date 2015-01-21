#!/bin/bash
 
HOBBITDIR="/opt/ros/hobbit_hydro/" 

echo "Starting up emergency_detector"
screen -d -m -S "emergency_detector" /bin/bash -c "source ~/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch emergency_detector emergency_detector.launch"
 
  
exit 0 
