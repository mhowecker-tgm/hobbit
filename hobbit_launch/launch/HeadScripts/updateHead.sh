#/bin/bash 

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

echo "Trying to make head read write again..!"
nc6 192.168.2.199 8092
sleep 1
echo "Done.."


cd /opt/ros/hobbit_hydro/src/hobbit_launch/launch/HeadScripts/
scp headStart.sh headKickStarter.sh runSystemCommandPi.sh waitForReadWrite.sh waitForShutdown.sh waitForRefresh.sh systemCommandsPi.c checkStatus.sh activateHeadAutoStart.sh headSyncTime.sh pi@192.168.2.199:/home_org/pi

echo "Trying to force refresh"
nc6 192.168.2.199 8084
echo "Done.."


#This is maybe not a good idea , no idea what version of the files is the most recent..
#scp owlpose.py herkulex.py pi@192.168.2.199:/home_org/pi/workspace/blue_owlpose/scripts  

exit 0
