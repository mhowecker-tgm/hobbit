#!/bin/bash


#exit 0
echo "We will do a USB work around" 

#Quick Settings
HOBBITDIR="/opt/ros/hobbit_hydro/"  
DEVICE_TOP="1d27/0601@1/3"
DEVICE_BASE="1d27/0601@2/3"

#Ammar's installation at FORTH may override hobbit dir
if [ -d $HOBBITDIR ] 
then
 echo "Hobbit Normal root dir"
elif [ -d ~/Documents/Programming/FORTH/Hobbit/hobbit_hydro ] 
then
 HOBBITDIR="~/Documents/Programming/FORTH/Hobbit/hobbit_hydro/"
fi

cd $HOBBITDIR/src/rgbd_acquisition/bin
 
echo "Doing a camera powercycle workaround for maximum robustness"

LD_LIBRARY_PATH=. timeout 35 ./Grabber -module OPENNI2 -from "$DEVICE_BASE" -to /dev/null -maxFrames 30
sleep 1
LD_LIBRARY_PATH=. timeout 35 ./Grabber -module OPENNI2 -from "$DEVICE_TOP" -to /dev/null -maxFrames 30

echo "Done " 
exit 0
