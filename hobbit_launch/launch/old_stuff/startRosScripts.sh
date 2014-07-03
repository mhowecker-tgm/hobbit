#!/usr/bin/env bash

LONGPAUSE="3"
MEDIUMPAUSE="1"
SHORTPAUSE="0.1"

source /opt/ros/fuerte/setup.bash
export ROS_MASTER_URI="http://192.168.2.122:11311"
export ROS_PACKAGE_PATH=~/hobbit:${ROS_PACKAGE_PATH}

#gnome-terminal -t "First Terminal"
#xdotool sleep $LONGPAUSE

#xdotool type "sudo ptpd –b eth0"
#xdotool sleep $MEDIUMPAUSE
#xdotool key Return
#xdotool sleep $SHORTPAUSE
#xdotool type "hobbit"
#xdotool key Return
#xdotool sleep $MEDIUMPAUSE

echo "Running roscore"
screen -S 'Roscore' -d -m bash -c 'roscore;bash'
sleep $MEDIUMPAUSE

echo "Running decision service"
screen -S 'Decision Service' -d -m bash -c '~/ScreenScripts/decisionService.sh;bash'
sleep $MEDIUMPAUSE

echo "Running coordinator"
screen -S 'Coordinator' -d -m bash -c '~/ScreenScripts/coordinator.sh;bash'
sleep $MEDIUMPAUSE

echo "Running aal service"
screen -S 'AAL Service' -d -m bash -c '~/ScreenScripts/aalService.sh;bash'
sleep $MEDIUMPAUSE

echo "Running a few more services"
screen -S 'Sleep' -d -m bash -c '~/ScreenScripts/sleep.sh;bash'
screen -S 'Help Me' -d -m bash -c '~/ScreenScripts/helpMe.sh;bash'
screen -S 'Reference Arm' -d -m bash -c '~/ScreenScripts/referenceArm.sh;bash'
screen -S 'Surprise Me' -d -m bash -c '~/ScreenScripts/surpriseMe.sh;bash'
screen -S 'Move To Location' -d -m bash -c '~/ScreenScripts/moveToLocation.sh;bash'
screen -S 'Clear Floor' -d -m bash -c '~/ScreenScripts/clearFloor.sh;bash'
screen -S 'Learn Object' -d -m bash -c '~/ScreenScripts/learnObject.sh;bash'
screen -S 'Speak' -d -m bash -c '~/ScreenScripts/speak.sh;bash'
screen -S 'Bring Me' -d -m bash -c '~/ScreenScripts/bringMe.sh;bash'

echo "Running top camera"
screen -S 'Top Camera' -d -m bash -c '~/ScreenScripts/topCamera.sh;bash'

echo "Running hobbit platform"
screen -S 'Hobbit Platform' -d -m bash -c '~/ScreenScripts/hobbitPlatform.sh;bash'

echo "Running armarker"
screen -S 'AR Marker' -d -m bash -c '~/ScreenScripts/arMarker.sh;bash'

echo "Running head"
screen -S 'Head' -d -m bash -c '~/ScreenScripts/head.sh;bash'

echo "Running joystick"
screen -S 'Joystick' -d -m bash -c '~/ScreenScripts/joystick.sh;bash'

gnome-terminal -t "First Terminal"
xdotool sleep $LONGPAUSE

xdotool sleep $LONGPAUSE
xdotool type "sudo bash"
xdotool key Return
xdotool sleep $SHORTPAUSE
xdotool type "hobbit"
xdotool key Return
xdotool sleep $SHORTPAUSE
xdotool type "rosrun ps3joy ps3joy.py"
xdotool key Return
xdotool sleep $SHORTPAUSE

echo "Running tod feature"
screen -S 'TOD Feature' -d -m bash -c '~/ScreenScripts/TODFeature.sh;bash'

echo "Running thor"
screen -S 'THOR' -d -m bash -c '~/ScreenScripts/thor.sh;bash'

echo "Running emotionator"
screen -S 'Emotionator' -d -m bash -c '~/ScreenScripts/emotionator.sh;bash'

echo "Running manual web server"
screen -S 'Manual Web Server' -d -m bash -c '~/ScreenScripts/runWebServer.sh;bash'


echo "Starting keep screen on script"
screen -S 'Keep Screen On' -d -m bash -c '~/ScreenScripts/keepScreenOn.sh;bash'

gnome-terminal -t "Face"
xdotool sleep $LONGPAUSE
xdotool type "roslaunch Face startup.launch "
xdotool key Return
