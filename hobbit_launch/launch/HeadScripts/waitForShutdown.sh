#!/bin/bash

source /home/pi/.bashrc

while true; do
 nc6 -l -p 8086 -e "echo \"Shutting down\" && ./runSystemCommandPi.sh shutdown && echo \"kthxbye..\""
 echo "Served one client"
 #sleep 1
done

exit 0

