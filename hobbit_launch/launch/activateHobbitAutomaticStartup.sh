#!/bin/bash
echo "Will now activate automatic hobbit startup script execution on each boot"
cd ~/.config
mkdir autostart
cd autostart
ln -s  /opt/ros/hobbit_hydro/src/hobbit_launch/launch/hobbitPT2.desktop
exit 0
