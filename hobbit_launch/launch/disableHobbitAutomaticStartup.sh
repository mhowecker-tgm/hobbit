#!/bin/bash
echo "Will now disable automatic hobbit startup script execution on , You will need to start everything manual from now on!"
echo "To enable automatic startup again please run  /opt/ros/hobbit_hydro/src/hobbit_launch/launch/activateHobbitAutomaticStartup.sh"
rm /opt/ros/hobbit_hydro/src/hobbit_launch/launch/hobbitPT2.desktop


exit 0
