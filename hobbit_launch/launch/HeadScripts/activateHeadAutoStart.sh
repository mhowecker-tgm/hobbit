#!/bin/bash


echo "Making sudo-enabled ntpdate commands"
gcc systemCommandsPi.c -s -o systemCommandsPi
sudo chmod 777 systemCommandsPi
sudo chown root:root systemCommandsPi
sudo chmod +s systemCommandsPi
#we also have a low privilege wrapper for the suid binary
chmod +x /home/pi/runSystemCommandPi.sh


echo "Adding netcat to the system"
sudo apt-get install -y netcat6

echo "Making Scripts Executable"
chmod +x /home/pi/headStart.sh
chmod +x /home/pi/headKickStarter.sh

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


echo "Head is configured for autostart , please remove all rosrun commands from ~/.bashrc"

exit 0
