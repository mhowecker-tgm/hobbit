#!/bin/bash

echo "Starting up person_aggregator"
screen -d -m -S "person_aggregator" /bin/bash -c "source ~/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch person_aggregator person_aggregator.launch"
 
exit 0



