#!/bin/bash

DELAY_BETWEEN_STEPS="5"

cp /opt/ros/hobbit_hydro/src/navigation/share/map.pgm /localhome/demo/apps/MapConverter/map.pgm
cp /opt/ros/hobbit_hydro/src/navigation/share/map.yaml /localhome/demo/apps/MapConverter/map.yaml

cd /localhome/demo/apps/MapConverter

./mapConverter.sh "$1"


exit 0
