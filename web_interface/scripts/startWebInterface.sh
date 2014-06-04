#!/bin/bash
 
#Script to start a screen session of web_interface

echo "Starting up web_interface"
screen -d -m -S "web_interface" /bin/bash -c "source ~/.bashrc && source /opt/ros/hobbit_hydro/devel/setup.bash && roslaunch web_interface web_interface.launch"
 
exit 0



