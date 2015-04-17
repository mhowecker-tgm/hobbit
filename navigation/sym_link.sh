#!/bin/bash

DELAY_BETWEEN_STEPS="5"

if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters, please provide the file name"
fi

rosnode kill places_objects
cd /opt/ros/hobbit_hydro/src/navigation
rm places.xml
ln -s "$1" places.xml


exit 0