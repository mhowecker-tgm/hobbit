#!/bin/bash


clear
killall memcheck-amd64-
source ~/.bashrc 
source /opt/ros/hobbit_hydro/devel/setup.bash 


valgrind --tool=memcheck --leak-check=yes --show-reachable=yes --num-callers=20 --track-fds=yes  --track-origins=yes roslaunch web_interface web_interface.launch 2> AmmarServerDebug.log&
exit 0
