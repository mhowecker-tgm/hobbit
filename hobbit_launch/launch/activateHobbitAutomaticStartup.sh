#!/bin/bash

red=$(printf "\033[31m")
normal=$(printf "\033[m")

echo "Will now activate automatic hobbit startup script execution on each boot"

if [ ! -e  /opt/ros/hobbit_hydro/src/hobbit_launch/launch/hobbitPT2.desktop ]
then
 echo $red
  echo "Could not find hobbit shortcut file , cannot enable automatic startup"
 echo $normal
 exit 0
fi


cd ~/.config
mkdir autostart
cd autostart
ln -s  /opt/ros/hobbit_hydro/src/hobbit_launch/launch/hobbitPT2.desktop
exit 0
