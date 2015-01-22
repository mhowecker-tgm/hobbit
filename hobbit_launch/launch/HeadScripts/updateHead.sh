#/bin/bash 

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

echo "Trying to make head read write again..!"
nc6 192.168.2.199 8092
sleep 1
echo "Done.."


cd /opt/ros/hobbit_hydro/src/hobbit_launch/launch/HeadScripts/
scp headStart.sh headKickStarter.sh runSystemCommandPi.sh waitForShutdown.sh waitForRefresh.sh systemCommandsPi.c checkStatus.sh activateHeadAutoStart.sh headSyncTime.sh pi@192.168.2.199:/home_org/pi

echo "Trying to force refresh"
nc6 192.168.2.199 8084
echo "Done.."

#scp owlpose.py herkulex.py pi@192.168.2.199:/home/pi/workspace/blue_owlpose/scripts


#cd /opt/ros/hobbit_hydro/src/hobbit_launch/launch/HeadScripts/
#scp headStart.sh pi@192.168.2.199:/home/pi/headStart.sh 
#scp headKickStarter.sh pi@192.168.2.199:/home/pi/headKickStarter.sh 
#scp systemCommandsPi.c pi@192.168.2.199:/home/pi/systemCommandsPi.c
#scp systemCommandsPi.c pi@192.168.2.199:/home/pi/systemCommandsPi.c
#scp activateHeadAutoStart.sh pi@192.168.2.199:/home/pi/activateHeadAutoStart.sh
#scp owlpose.py pi@192.168.2.199:/home/pi/workspace/blue_owlpose/scripts/owlpose.py
#scp herkulex.py pi@192.168.2.199:/home/pi/workspace/blue_owlpose/scripts/herkulex.py


exit 0
