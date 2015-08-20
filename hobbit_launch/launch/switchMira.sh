#!/bin/bash

DELAY_BETWEEN_STEPS="5"

if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters, valid modes are learning,mapping,navigation"
fi

MIRA_PATH=/localhome/demo/mira/mira-base
LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/localhome/demo/mira/mira-base/lib
PATH=${PATH}:/localhome/demo/mira/mira-base/bin
source /localhome/demo/mira/mira-base/scripts/mirabash

MIRA_PATH=${MIRA_PATH}:/localhome/demo/mira/mira-commercial
LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/localhome/demo/mira/mira-commercial/lib
MIRA_PATH=$MIRA_PATH:/localhome/demo/mira/mira-hobbit
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/localhome/demo/mira/mira-hobbit/lib

if [ "$1" == "mapping" ]
then
 echo "Mapping"
 killall -9 interfaces_node
 killall -9 mira&
 killall -9 miracenter&
 
 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/launch
 roslaunch startup_ros_nav.launch&

 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
 mira mira_config_ros_nav.xml -p1234&

 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
 miracenter mira_vis_config.xml&

 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/navigation
 roslaunch gmapping_hobbit.launch&

 sleep $DELAY_BETWEEN_STEPS
 rosrun rviz rviz&

elif [ "$1" == "learning" ]
then
 echo "Learning"
 killall -9 interfaces_node
 killall -9 mira&
 killall -9 miracenter&
 rosnode kill slam_gmapping&
sleep $DELAY_BETWEEN_STEPS
rosnode kill places_objects&
sleep $DELAY_BETWEEN_STEPS
rosnode kill places_object&
sleep $DELAY_BETWEEN_STEPS
rosnode kill localization_monitor&


 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
 mira mira_config.xml -p1234&

 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
 miracenter mira_vis_config.xml&

 sleep $DELAY_BETWEEN_STEPS
sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/launch
 roslaunch startup.launch&
  
 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/places_interpretation/launch
 roslaunch startup.launch&

 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/get_current_room/launch
 roslaunch startup.launch&

 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/places_learning_gui/launch
 roslaunch startup.launch&

elif [ "$1" == "navigation" ]
then
 echo "Regular Startup"
 # bajo: regular startup does not need the following nodes
 # this is why they should be killed with rosnode kill
 #rosnode kill places_interpretation&
 #rosnode kill get_current_room&
 #sleep $DELAY_BETWEEN_STEPS
 #rosnode kill interfaces_node&
 killall -9 interfaces_node
 killall -9 mira&
 #sleep $DELAY_BETWEEN_STEPS
 killall -9 miracenter&
 #sleep $DELAY_BETWEEN_STEPS

 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
 mira mira_config2.xml -p1234&

 sleep $DELAY_BETWEEN_STEPS
 sleep $DELAY_BETWEEN_STEPS
 sleep $DELAY_BETWEEN_STEPS

 cd /opt/ros/hobbit_hydro/src/interfaces_mira/launch
 roslaunch startup.launch&
 
 sleep $DELAY_BETWEEN_STEPS
 cd /opt/ros/hobbit_hydro/src/interfaces_mira/resources
 miracenter mira_vis_config.xml&

 sleep $DELAY_BETWEEN_STEPS
 sleep $DELAY_BETWEEN_STEPS
 rosservice call /obs_nav_mode "{}"&

fi


exit 0
