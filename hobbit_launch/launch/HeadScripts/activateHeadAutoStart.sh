#!/bin/bash

PI_HOME_DIR="/home/pi"

if [ -d "/home_org/pi" ] 
   then
     echo "It appears we are running on a non writeable rpi.." 
     echo "Making it writeable"
     sudo mount -o remount,rw /
     PI_HOME_DIR="/home_org/pi"
fi
echo "Directory for writing is set to : $PI_HOME_DIR"



echo "Making sudo-enabled ntpdate commands"
gcc systemCommandsPi.c -s -o systemCommandsPi
sudo chmod 777 systemCommandsPi
sudo chown root:root systemCommandsPi
sudo chmod +s systemCommandsPi
#we also have a low privilege wrapper for the suid binary
chmod +x $PI_HOME_DIR/runSystemCommandPi.sh


echo "Adding netcat to the system"
sudo apt-get install -y netcat6

echo "Making Scripts Executable"
chmod +x $PI_HOME_DIR/headStart.sh
chmod +x $PI_HOME_DIR/headKickStarter.sh
chmod +x $PI_HOME_DIR/headSyncTime.sh
chmod +x $PI_HOME_DIR/checkStatus.sh
chmod +x $PI_HOME_DIR/waitForShutdown.sh
chmod +x $PI_HOME_DIR/waitForRefresh.sh
chmod +x $PI_HOME_DIR/waitForReadWrite.sh

echo "Making Blue Temperature not require super user every time"
cd ~/workspace/blue_temperature/bin
sudo chmod 777 blue_temperature
sudo chown root:root blue_temperature
sudo chmod +s blue_temperature

 



if cat /etc/xdg/lxsession/LXDE/autostart  | grep -q "headKickStarter"
then
 echo "Autostart settings are already there!"
else
 echo "Registering AutoStart scripts for head.."
 sudo sh -c 'echo "@/home/pi/headKickStarter.sh" >>/etc/xdg/lxsession/LXDE/autostart '
fi


echo "Head is configured for autostart.."

exit 0
