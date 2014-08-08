#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" 
cd $DIR  

source ../../../devel/setup.bash
lxterminal -t "Face Detection" -e "roslaunch face_detection face_detection.launch"

lxterminal -t "Hand Gestures" -e "roslaunch hand_gestures hand_gestures.launch"

lxterminal -t "Skeleton Detector" -e "roslaunch skeleton_detector skeleton_detector.launch"

lxterminal -t "RGBDAcquisition" -e "roslaunch rgbd_acquisition rgbd_acquisition.launch"

lxterminal -t "Web Interface" -e "roslaunch web_interface web_interface.launch"

lxterminal -t "Detected Persons" -e "rostopic echo /persons"


exit 0