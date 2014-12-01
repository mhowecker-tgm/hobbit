#!/bin/bash

DELAY_BETWEEN_STEPS="5"

if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters, valid modes are reset, default"
fi

if [ "$1" == "reset" ]
then
 echo "Localization reset, docking station"
 killall -9 mira&
 sleep $DELAY_BETWEEN_STEPS
 killall -9 miracenter&
 sleep $DELAY_BETWEEN_STEPS
 rosnode kill interfaces_node&

 # Delete localization files
 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
 rm InitialPose0.xml
 rm InitialPose1.xml

 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/launch
 roslaunch startup.launch&

 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
 mira mira_config2.xml -p1234&

 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
 miracenter mira_vis_config.xml&


elif [ "$1" == "default" ]
then
 echo "Keeping previous localization"
 sleep $DELAY_BETWEEN_STEPS

 killall -9 mira&
 sleep $DELAY_BETWEEN_STEPS
 killall -9 miracenter&
 sleep $DELAY_BETWEEN_STEPS
 rosnode kill interfaces_node&

 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/launch
 roslaunch startup.launch&

 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
 mira mira_config2.xml -p1234&

 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
 miracenter mira_vis_config.xml&

fi


exit 0
