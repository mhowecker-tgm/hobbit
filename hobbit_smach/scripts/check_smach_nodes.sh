#!/bin/bash

# author: Markus Bajones
# email: markus.bajones@gmail.com

# This script checks if a given list of ROS nodes are running.

nodes=( "place_handler" "learn_object" "goto" "emergency_user"
"emergency_bathroom" "battery_monitor" "recharge" "sos_monitor"
"follow_me_simple" "pickup" "away" "master" "arm_action_server"
"bring_object" "fitness" )

values=`rosnode list`
for i in "${nodes[@]}"
do
    # pid=`pidof -x ${i}`
    echo ${values}|grep ${i} 1> /dev/null
    if [ $? -ne 0 ]; then
        echo -e "\e[31m${i} is not running\e[0m"
    else
        echo -e "\e[32m${i} is running\e[0m"
    fi
done
