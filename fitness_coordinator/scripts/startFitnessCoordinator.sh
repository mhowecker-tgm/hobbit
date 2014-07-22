#!/bin/bash
 

echo "Starting up fitness_coordinator"
screen -d -m -S "fitness_coordinator" /bin/bash -c "source ~/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch fitness_coordinator fitness_coordinator.launch"
 
exit 0



