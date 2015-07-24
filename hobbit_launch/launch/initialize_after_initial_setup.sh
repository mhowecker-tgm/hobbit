#!/bin/bash

DELAY_BETWEEN_STEPS="5"

sleep $DELAY_BETWEEN_STEPS
roslaunch localization_monitor startup.launch&
sleep $DELAY_BETWEEN_STEPS
rosrun hobbit_smach places_objects.py&
sleep $DELAY_BETWEEN_STEPS
rosnode kill distance_to_obstacle&
sleep $DELAY_BETWEEN_STEPS
roslaunch distance_to_obstacle distance_to_obstacle.launch&
sleep $DELAY_BETWEEN_STEPS
rosnode kill /recharge&
sleep $DELAY_BETWEEN_STEPS
rosrun hobbit_smach recharge.py&

exit 0
