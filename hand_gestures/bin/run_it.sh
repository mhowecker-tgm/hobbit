#!/bin/bash

STARTDIR=`pwd` 
#Force switch to this directory ( ROS Fuerte seems to go to ~/.ros for some reason :P )
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"


red=$(printf "\033[31m")
green=$(printf "\033[32m") 
normal=$(printf "\033[m")

export GLOG_logtostderr=1
 
echo `pwd`

PATHOFBIN="./hand_gestures" 
  
if [ ! -e $PATHOFBIN ]
then 
  echo "$red Could not find hand_gestures binary! (rosmake RGBDAcquisition) $normal"
else
  echo "$green Found hand_gestures binary! $normal"
fi
 
LD_LIBRARY_PATH=./:$LD_LIBRARY_PATH "$PATHOFBIN" 

cd "$STARTDIR"
exit 0
