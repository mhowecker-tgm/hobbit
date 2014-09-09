#!/bin/bash

source /home/pi/.bashrc

while true; do
 nc6 -l -p 8090 -e "echo \"Head is alive ,`uptime`\""
 echo "Served one client"
 #sleep 1
done

exit 0

